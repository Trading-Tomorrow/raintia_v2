#!/usr/bin/env python3
"""
Cliente HTTP para enviar as fotos de inspecao do robot para o SIMAGIA.

Sem dependencias de ROS/Nav2 -> pode ser testado isoladamente
(ver test_simagia_client.py).

Endpoint:
    POST {base_url}/claims/{case_id}/robot-inspection
Campos multipart (form):
    mission_id              (texto)
    robot_id                (texto)
    inspection_points_json  (texto, JSON, opcional)
    files                   (campo de FICHEIRO repetido, um por foto)
"""
import os
import json
import time
import uuid

try:
    import requests
except ImportError:
    requests = None

DEFAULT_BASE_URL = "http://127.0.0.1:8000"
DEFAULT_ROBOT_ID = "argus-tiago-lite"


class ConfigError(Exception):
    """Erro de configuracao (ex.: falta o SIMAGIA_CLAIM_ID)."""
    pass


def _cli_get(cli, name):
    if cli is None:
        return None
    if isinstance(cli, dict):
        return cli.get(name)
    return getattr(cli, name, None)


def resolve_config(cli=None, env=None):
    """Resolve a configuracao por prioridade: CLI > env var > default.

    cli: dict ou namespace com (simagia_base_url, simagia_claim_id, robot_id,
         mission_id); valores None/ausentes sao ignorados.
    env: dict de variaveis de ambiente (default: os.environ).

    Devolve dict: {base_url, case_id, robot_id, mission_id}.
    Lanca ConfigError se faltar o case_id (SIMAGIA_CLAIM_ID).
    """
    env = os.environ if env is None else env

    def pick(cli_val, env_key, default=None):
        if cli_val:
            return cli_val
        return env.get(env_key) or default

    base_url = pick(_cli_get(cli, 'simagia_base_url'), 'SIMAGIA_BASE_URL', DEFAULT_BASE_URL).rstrip('/')
    case_id = pick(_cli_get(cli, 'simagia_claim_id'), 'SIMAGIA_CLAIM_ID', None)
    robot_id = pick(_cli_get(cli, 'robot_id'), 'ARGUS_ROBOT_ID', DEFAULT_ROBOT_ID)
    mission_id = pick(_cli_get(cli, 'mission_id'), 'ARGUS_MISSION_ID', None)
    if not mission_id:
        mission_id = "argus-" + uuid.uuid4().hex[:8]

    if not case_id:
        raise ConfigError(
            "SIMAGIA_CLAIM_ID em falta. Define a variavel de ambiente "
            "SIMAGIA_CLAIM_ID ou passa --simagia-claim-id <case_id>."
        )

    return {
        'base_url': base_url,
        'case_id': case_id,
        'robot_id': robot_id,
        'mission_id': mission_id,
    }


def build_endpoint(base_url, case_id):
    return f"{base_url.rstrip('/')}/claims/{case_id}/robot-inspection"


def upload_robot_inspection(base_url, case_id, mission_id, robot_id,
                            photo_paths, inspection_points=None, timeout=30):
    """POST multipart das fotos para o SIMAGIA.

    O campo de ficheiro 'files' e repetido, um por imagem.
    Devolve o objeto Response do requests (NAO levanta por status >=400; o
    chamador decide). Levanta em erros de rede ou ao abrir ficheiros.
    """
    if requests is None:
        raise RuntimeError("biblioteca 'requests' nao esta instalada")

    url = build_endpoint(base_url, case_id)
    data = {
        'mission_id': mission_id,
        'robot_id': robot_id,
    }
    if inspection_points is not None:
        data['inspection_points_json'] = json.dumps(inspection_points)

    files = []
    abertos = []
    try:
        for p in photo_paths:
            fh = open(p, 'rb')
            abertos.append(fh)
            files.append(('files', (os.path.basename(p), fh, 'image/jpeg')))
        return requests.post(url, data=data, files=files, timeout=timeout)
    finally:
        for fh in abertos:
            try:
                fh.close()
            except Exception:
                pass


def write_retry_manifest(directory, base_url, case_id, mission_id, robot_id,
                         photo_paths, inspection_points, error):
    """Escreve um manifest JSON (para re-tentar mais tarde). Devolve o caminho."""
    manifest = {
        'base_url': base_url,
        'case_id': case_id,
        'mission_id': mission_id,
        'robot_id': robot_id,
        'photo_paths': list(photo_paths),
        'inspection_points': inspection_points,
        'error': str(error),
        'created_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    }
    path = os.path.join(directory, f"simagia_retry_{mission_id}.json")
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    return path
