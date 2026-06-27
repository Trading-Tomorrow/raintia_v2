#!/usr/bin/env python3
"""
Testes unitarios para simagia_client (sem ROS, sem robot).
Correr: python3 -m unittest test_simagia_client.py -v
"""
import os
import json
import tempfile
import unittest
from unittest import mock

import simagia_client as sc


class TestResolveConfig(unittest.TestCase):
    def test_defaults_com_env(self):
        cfg = sc.resolve_config(cli=None, env={'SIMAGIA_CLAIM_ID': 'CASE1'})
        self.assertEqual(cfg['base_url'], 'http://127.0.0.1:8000')
        self.assertEqual(cfg['case_id'], 'CASE1')
        self.assertEqual(cfg['robot_id'], 'argus-tiago-lite')
        self.assertTrue(cfg['mission_id'].startswith('argus-'))

    def test_cli_tem_prioridade_sobre_env(self):
        env = {'SIMAGIA_BASE_URL': 'http://env:1', 'SIMAGIA_CLAIM_ID': 'ENVCASE'}
        cli = {'simagia_base_url': 'http://cli:2', 'simagia_claim_id': 'CLICASE',
               'robot_id': 'r2', 'mission_id': 'm9'}
        cfg = sc.resolve_config(cli=cli, env=env)
        self.assertEqual(cfg['base_url'], 'http://cli:2')
        self.assertEqual(cfg['case_id'], 'CLICASE')
        self.assertEqual(cfg['robot_id'], 'r2')
        self.assertEqual(cfg['mission_id'], 'm9')

    def test_env_usado_quando_cli_vazio(self):
        cli = {'simagia_base_url': None, 'simagia_claim_id': None}
        cfg = sc.resolve_config(cli=cli, env={'SIMAGIA_CLAIM_ID': 'X', 'ARGUS_ROBOT_ID': 'rob'})
        self.assertEqual(cfg['case_id'], 'X')
        self.assertEqual(cfg['robot_id'], 'rob')

    def test_falta_claim_id_levanta(self):
        with self.assertRaises(sc.ConfigError):
            sc.resolve_config(cli=None, env={})


class TestEndpoint(unittest.TestCase):
    def test_url(self):
        self.assertEqual(
            sc.build_endpoint('http://h:8000/', 'C9'),
            'http://h:8000/claims/C9/robot-inspection')


class TestUpload(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.photos = []
        for n in ('dir', 'frente', 'esq', 'tras'):
            p = os.path.join(self.tmp, f'foto_carro_{n}.jpg')
            with open(p, 'wb') as f:
                f.write(b'\xff\xd8fakejpeg')
            self.photos.append(p)

    @mock.patch('simagia_client.requests')
    def test_post_bem_formado(self, mreq):
        mreq.post.return_value = mock.Mock(status_code=200, text='ok')
        resp = sc.upload_robot_inspection(
            'http://x:8000', 'CASE9', 'M1', 'R1',
            self.photos, inspection_points=[{'name': 'dir'}])
        self.assertEqual(resp.status_code, 200)
        args, kwargs = mreq.post.call_args
        # URL correto
        self.assertEqual(args[0], 'http://x:8000/claims/CASE9/robot-inspection')
        # campos de texto com os nomes exatos
        self.assertEqual(kwargs['data']['mission_id'], 'M1')
        self.assertEqual(kwargs['data']['robot_id'], 'R1')
        self.assertIn('inspection_points_json', kwargs['data'])
        self.assertEqual(json.loads(kwargs['data']['inspection_points_json']),
                         [{'name': 'dir'}])
        # 4 ficheiros, todos no campo 'files'
        files = kwargs['files']
        self.assertEqual(len(files), 4)
        self.assertTrue(all(f[0] == 'files' for f in files))
        self.assertEqual(files[0][1][0], 'foto_carro_dir.jpg')
        self.assertEqual(files[0][1][2], 'image/jpeg')

    @mock.patch('simagia_client.requests')
    def test_sem_inspection_points(self, mreq):
        mreq.post.return_value = mock.Mock(status_code=201, text='')
        sc.upload_robot_inspection('http://x', 'C', 'M', 'R', self.photos[:1])
        _, kwargs = mreq.post.call_args
        self.assertNotIn('inspection_points_json', kwargs['data'])


class TestManifest(unittest.TestCase):
    def test_escreve_todas_as_chaves(self):
        tmp = tempfile.mkdtemp()
        path = sc.write_retry_manifest(
            tmp, 'http://b', 'C1', 'M1', 'R1',
            ['a.jpg', 'b.jpg'], [{'name': 'dir'}], 'boom')
        self.assertTrue(os.path.exists(path))
        with open(path) as f:
            data = json.load(f)
        for k in ('base_url', 'case_id', 'mission_id', 'robot_id',
                  'photo_paths', 'inspection_points', 'error', 'created_at'):
            self.assertIn(k, data)
        self.assertEqual(data['error'], 'boom')
        self.assertEqual(data['photo_paths'], ['a.jpg', 'b.jpg'])


if __name__ == '__main__':
    unittest.main()
