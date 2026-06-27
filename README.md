# ArgusAI — Inspeção autónoma de um carro (Webots + ROS 2)

Robô TIAGo Lite que mapeia uma arena, navega autonomamente aos **4 lados de um carro** (frente, trás, esquerda, direita), tira uma **fotografia** em cada lado evitando obstáculos, e regressa ao ponto inicial.

- **ROS 2** Humble · **Webots** R2025a · **slam_toolbox** (SLAM) · **Nav2** (navegação)

---

## 0. Antes de começar (em CADA terminal novo)

O `~/.bashrc` já carrega o ROS 2 e o Webots. Confirma:
```bash
echo $WEBOTS_HOME      # deve mostrar /home/nuno/.ros/webotsR2025a/webots
```

> **Regra de ouro:** se algo correr mal ao relançar, limpa primeiro os
> processos antigos (ver secção *Resolução de problemas*).

---

## 1. NAVEGAÇÃO (fluxo principal — o mapa já existe)

O mapa já está feito (`map.pgm` / `map.yaml`). São precisos **2 terminais**.

### Terminal 1 — Simulação + Nav2 + RViz
```bash
ros2 launch /home/nuno/Documents/ArgusAI/argus_nav_launch.py
```
Espera até ao RViz mostrar o mapa e o robot localizado (a pose inicial é
posta automaticamente em 0,0 — **não** é preciso o "2D Pose Estimate").
No RViz vais ver: mapa, costmaps, caminho global (verde), caminho local
(azul), partículas do AMCL (laranja) e o rasto do robot (amarelo).

### Terminal 2 — Executar a missão (quando o Nav2 estiver ativo)
O `SIMAGIA_CLAIM_ID` é **obrigatório** (o case/claim é criado antes da missão):
```bash
export SIMAGIA_CLAIM_ID="CASO-123"            # obrigatorio
export SIMAGIA_BASE_URL="http://127.0.0.1:8000"   # opcional (default)
python3 /home/nuno/Documents/ArgusAI/navegacao.py
```
Ou por CLI: `python3 navegacao.py --simagia-claim-id CASO-123`.

O robô dá a volta ao carro pela ordem otimizada (`dir → frente → esq → trás`),
tira 4 fotos, regressa à base e **envia as fotos para o SIMAGIA** (POST).

#### Upload para o SIMAGIA (fim da missão)
- **Endpoint:** `POST {SIMAGIA_BASE_URL}/claims/{SIMAGIA_CLAIM_ID}/robot-inspection`
- **Envia (multipart):** campos `mission_id`, `robot_id`, `inspection_points_json`
  + as imagens no campo de ficheiro repetido **`files`**.
- **Variáveis/CLI:**
  | Var ambiente | CLI | Default |
  |---|---|---|
  | `SIMAGIA_BASE_URL` | `--simagia-base-url` | `http://127.0.0.1:8000` |
  | `SIMAGIA_CLAIM_ID` | `--simagia-claim-id` | **obrigatório** |
  | `ARGUS_ROBOT_ID` | `--robot-id` | `argus-tiago-lite` |
  | `ARGUS_MISSION_ID` | `--mission-id` | gerado (`argus-<id>`) |
- **Sem `SIMAGIA_CLAIM_ID`** → erro claro e termina **antes** de mexer no robot.
- **Se o upload falhar** → as fotos **não** são apagadas e é escrito um
  `simagia_retry_<mission_id>.json` (manifest para re-enviar mais tarde).
- Lógica de upload isolada em `simagia_client.py` (testável):
  `python3 -m unittest test_simagia_client.py -v`

### Fotos
Guardadas em `/home/nuno/Documents/ArgusAI/`:
```
foto_carro_dir.jpg   foto_carro_frente.jpg   foto_carro_esq.jpg   foto_carro_tras.jpg
```

---

## 2. MAPEAMENTO (só se quiseres refazer o mapa do zero)

### Terminal 1 — Simulação + SLAM + RViz
```bash
ros2 launch /home/nuno/Documents/ArgusAI/argus_mapping_launch.py
```

### Terminal 2 — Conduzir o robot (devagar!)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Teclas: `i` frente · `,` trás · `j`/`l` rodar · `k` parar · `z` baixar velocidade.
Conduz **devagar e com curvas suaves** (a odometria é o que mantém o mapa direito).

### Terminal 3 — Guardar o mapa
```bash
ros2 run nav2_map_server map_saver_cli -f /home/nuno/Documents/ArgusAI/map --ros-args -p save_map_timeout:=20.0
```
> O `save_map_timeout` é obrigatório, senão a gravação falha.

Se mudares o mapa, recalcula os 4 pontos de inspeção:
```bash
python3 /home/nuno/Documents/ArgusAI/detetar_pontos.py   # mostra os pontos novos
python3 /home/nuno/Documents/ArgusAI/tsp_astar.py         # mostra a ordem otima (TSP)
```
e atualiza a lista `self.pontos_inspecao` no `navegacao.py`.

---

## 3. Resolução de problemas

**Limpar processos antigos** (resolve a maioria dos erros: TF "jump back in
time", Webots não arranca, robot não anda):
```bash
pkill -9 -f "webots|slam_toolbox|rviz2|ros2_supervisor|nav2|amcl|controller_server|navegacao"
rm -rf /tmp/webots/nuno
ros2 daemon stop && ros2 daemon start
```
> Limpa **e só depois** relança (em comandos separados — juntos pode falhar).

| Sintoma | O que fazer |
|---|---|
| "Detected jump back in time" | Limpar processos antigos (acima) |
| "sequence size exceeds remaining buffer" | Ruído cosmético do FastDDS — ignorar, não afeta nada |
| Robot não anda no teleop | Limpar processos; usar o tópico `/cmd_vel` normal |
| Mapa não aparece no RViz da navegação | É só a config; relançar recarrega o `nav.rviz` (NÃO carregar Ctrl+S no RViz) |
| Robot anda às voltas no objetivo | Já mitigado (timeout 35s para-o); é intermitente do Nav2 |

---

## 4. Ficheiros do projeto

| Ficheiro | Função |
|---|---|
| `argus_mapping_launch.py` | Launch do mapeamento (SLAM) |
| `argus_nav_launch.py` | Launch da navegação (Nav2) |
| `navegacao.py` | Missão: 4 pontos + fotos + regresso + upload SIMAGIA |
| `simagia_client.py` | Lógica de upload para o SIMAGIA (testável, sem ROS) |
| `test_simagia_client.py` | Testes unitários do `simagia_client` |
| `ros2_control.yml` | Odometria calibrada (rodas) |
| `slam_toolbox_params.yaml` | Parâmetros do SLAM |
| `nav2_params.yaml` | Parâmetros do Nav2 (A*, tolerâncias, inflação) |
| `nav.rviz` | Config do RViz para navegação |
| `detetar_pontos.py` | Calcula os 4 pontos no frame do mapa |
| `tsp_astar.py` | Resolve a ordem ótima (caixeiro-viajante com A*) |
| `calibrar_odom.py` / `calibrar_linear.py` | Calibração da odometria |
| `map.pgm` / `map.yaml` | Mapa final (`map_backup.*` = cópia de segurança) |
| `RELATORIO_erros_e_correcoes.txt` | Erros encontrados e correções (relatório) |

---

## 5. Resumo do fluxo

```
[mapear (1x)]  ->  argus_mapping_launch + teleop + map_saver
                          |
                          v
[navegar]      ->  argus_nav_launch   (Terminal 1)
                   navegacao.py        (Terminal 2)
                          |
                          v
            4 fotos + regresso à base
```
