#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import cv2
import time
import math
import os

# Centro do carro no frame do mapa (detetado por detetar_pontos.py)
CARRO_CENTRO = (6.63, 4.68)
try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None   # MQTT opcional: se nao estiver instalado, a missao corre sem paragem remota
import simagia_client   # upload das fotos para o SIMAGIA (modulo proprio, testavel)

class MissaoArgus(Node):
    def __init__(self, simagia_cfg):
        super().__init__('missao_argus')

        # config do SIMAGIA (base_url, case_id, robot_id, mission_id) resolvida no main
        self.simagia = simagia_cfg
        # caminhos+metadados das fotos efetivamente capturadas durante a missao
        self.fotos_capturadas = []

        # --- CONFIGURAÇÕES DOS DESTINOS (O CARRO) ---
        # Coordenadas no FRAME DO MAPA. Carro detetado em (6.63, 4.68).
        # ORDEM otimizada pelo caixeiro-viajante (tsp_astar.py): anel a volta do
        # carro que minimiza a distancia total (~28m vs ~41m da ordem ingenua) e
        # acaba no ponto mais perto da base (regresso curto).
        # Formato: (X, Y, Theta, nome).
        # Laterais (dir/esq) mais afastados (~3.2m) para apanhar o perfil completo;
        # frente/tras a ~3.5m. A camara e depois apontada ao centro do carro.
        self.pontos_inspecao = [
            (6.79,  1.48,  1.623, 'dir'),     # lado direito (perfil)
            (10.17, 4.86, -3.089, 'frente'),  # frente
            (6.47,  7.88, -1.518, 'esq'),     # lado esquerdo (perfil)
            (3.10,  4.49,  0.053, 'tras'),    # tras (mais perto da base)
        ]
        self.TIMEOUT_PONTO = 55.0   # segundos max por objetivo antes de cancelar

        # --- INICIALIZAÇÃO ROS 2 ---
        self.bridge = CvBridge()
        self.navigator = BasicNavigator()
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # TF para saber a pose do robot e apontar a camara ao carro
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Topico real da camara RGB do Tiago Lite (confirmado: ros2 topic list | grep image)
        self.sub_camera = self.create_subscription(Image, '/Tiago_Lite/Astra_rgb/image_color', self.camera_callback, 10)
        
        # Variáveis de controlo
        self.emergencia_ativada = False
        self.imagem_atual = None
        
        # --- INICIALIZAÇÃO MQTT ---
        self.setup_mqtt()

    def setup_mqtt(self):
        # MQTT nao-fatal: se nao estiver instalado ou sem broker, a missao continua.
        self.mqtt_client = None
        if mqtt is None:
            self.get_logger().warn('paho-mqtt nao instalado - missao sem paragem remota MQTT.')
            return
        try:
            self.get_logger().info('A ligar ao Broker MQTT (hivemq)...')
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_message = self.mqtt_callback
            self.mqtt_client.connect("broker.hivemq.com", 1883, 60)
            self.mqtt_client.subscribe("argusai/emergencia")
            self.mqtt_client.loop_start()
            self.get_logger().info('MQTT Ligado! Topico: argusai/emergencia | Mensagem para travar: STOP')
        except Exception as e:
            self.get_logger().warn(f'MQTT indisponivel ({e}) - missao continua sem paragem remota.')
            self.mqtt_client = None

    def mqtt_callback(self, client, userdata, msg):
        comando = msg.payload.decode('utf-8').upper()
        if comando == "STOP":
            self.get_logger().error('🚨 COMANDO DE EMERGÊNCIA RECEBIDO VIA MQTT! A TRAVAR! 🚨')
            self.emergencia_ativada = True
            # Cancela qualquer navegação em curso
            self.navigator.cancelTask()
            self.travar_robo()

    def travar_robo(self):
        # Envia velocidade ZERO para parar imediatamente
        cmd = Twist()
        self.pub_cmd_vel.publish(cmd)

    def pose_robot(self):
        """Pose atual do robot no frame do mapa (x, y, yaw) via TF; None se falhar."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except Exception:
            return None

    def girar_para_carro(self):
        """Roda no sitio ate a camara apontar ao centro do carro (independente de
        onde o Nav2 parou) -> garante o carro centrado na foto."""
        cx, cy = CARRO_CENTRO
        for _ in range(250):
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose_robot()
            if p is None:
                continue
            x, y, yaw = p
            desejado = math.atan2(cy - y, cx - x)
            err = math.atan2(math.sin(desejado - yaw), math.cos(desejado - yaw))
            if abs(err) < 0.04:
                break
            cmd = Twist()
            cmd.angular.z = max(-0.5, min(0.5, 1.2 * err))
            self.pub_cmd_vel.publish(cmd)
        self.travar_robo()

    def centrar_carro_visual(self):
        """Centra o carro (vermelho) na imagem por servoing visual - robusto a
        erros de localizacao. Se nao ver o carro, roda a procurar."""
        for _ in range(220):
            self.imagem_atual = None
            t = time.time()
            while self.imagem_atual is None and time.time() - t < 2.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.imagem_atual is None:
                break
            img = self.imagem_atual
            W = img.shape[1]
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 70, 30), (14, 255, 255)) | \
                   cv2.inRange(hsv, (166, 70, 30), (180, 255, 255))
            mo = cv2.moments(mask)
            cmd = Twist()
            if mo['m00'] < 800:                 # carro nao visivel -> procurar
                cmd.angular.z = 0.30
            else:
                cx = mo['m10'] / mo['m00']
                err = cx - W / 2.0
                if abs(err) < 20:               # centrado
                    break
                cmd.angular.z = max(-0.4, min(0.4, -0.0022 * err))
            self.pub_cmd_vel.publish(cmd)
            time.sleep(0.05)
        self.travar_robo()

    def enviar_simagia(self):
        """No fim da missao, envia as fotos capturadas para o SIMAGIA.
        Em caso de falha (rede ou HTTP>=400) NAO apaga fotos e escreve um
        retry manifest ao lado delas. Nao-fatal para a missao."""
        cfg = self.simagia
        paths = [p['photo'] for p in self.fotos_capturadas if os.path.exists(p['photo'])]
        if not paths:
            self.get_logger().warn('Sem fotos capturadas - nada para enviar ao SIMAGIA.')
            return

        self.get_logger().info(
            f"A enviar {len(paths)} fotos para o SIMAGIA "
            f"(case={cfg['case_id']}, mission={cfg['mission_id']})...")
        try:
            resp = simagia_client.upload_robot_inspection(
                cfg['base_url'], cfg['case_id'], cfg['mission_id'], cfg['robot_id'],
                paths, inspection_points=self.fotos_capturadas)
            if 200 <= resp.status_code < 300:
                self.get_logger().info(
                    f"SIMAGIA OK (HTTP {resp.status_code}): {resp.text[:300]}")
                return
            # status de erro -> tratar como falha
            raise RuntimeError(f"HTTP {resp.status_code}: {resp.text[:300]}")
        except Exception as e:
            self.get_logger().error(f'Falha no upload para o SIMAGIA: {e}')
            try:
                manifest = simagia_client.write_retry_manifest(
                    '.', cfg['base_url'], cfg['case_id'], cfg['mission_id'],
                    cfg['robot_id'], paths, self.fotos_capturadas, e)
                self.get_logger().warn(
                    f'Retry manifest escrito em {manifest} (fotos preservadas).')
            except Exception as e2:
                self.get_logger().error(f'Falha tambem a escrever o manifest: {e2}')

    def camera_callback(self, msg):
        # Guarda a imagem mais recente sempre na memória
        try:
            self.imagem_atual = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erro ao processar imagem: {e}")

    def criar_pose(self, x, y, theta=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Conversão simples de Theta (radianos) para Quaternion no eixo Z
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        return pose

    def executar_missao(self):
        # 1. Espera que o Nav2 esteja pronto
        self.get_logger().info('A aguardar o sistema de navegação (Nav2)...')
        self.navigator.waitUntilNav2Active()

        # Aquecer a camara (lazy publisher do Webots) ate chegar o 1o frame
        self.get_logger().info('A aquecer a camara...')
        t_warm = time.time()
        while self.imagem_atual is None and time.time() - t_warm < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Camara pronta.' if self.imagem_atual is not None else 'Camara sem frame (continuo).')

        # Guarda a posição inicial onde o robô ligou
        pose_inicial = self.criar_pose(0.0, 0.0, 0.0)

        # 2. Visitar todos os pontos de inspeção
        for index, (px, py, ptheta, nome) in enumerate(self.pontos_inspecao):
            if self.emergencia_ativada: return

            pose_alvo = self.criar_pose(px, py, ptheta)
            self.get_logger().info(f'🚗 Ponto {index + 1}/4 ({nome}) em ({px}, {py})...')
            self.navigator.goToPose(pose_alvo)

            # Loop enquanto viaja — IMPORTANTE: spin_once(self) para o camera_callback
            # disparar e a imagem ficar atualizada (senao as fotos saem vazias).
            # TIMEOUT: se o Nav2 ficar preso a andar as voltas (recuperacao), cancela.
            t_ini = time.time()
            timed_out = False
            while not self.navigator.isTaskComplete():
                if self.emergencia_ativada:
                    self.get_logger().warn('Missão abortada a meio da ida!')
                    return
                if time.time() - t_ini > self.TIMEOUT_PONTO:
                    self.get_logger().warn(f'Ponto {index + 1}: demorou demais (preso?), a cancelar e seguir.')
                    self.navigator.cancelTask()
                    self.travar_robo()
                    timed_out = True
                    break
                rclpy.spin_once(self, timeout_sec=0.2)

            # 3. Chegou ao ponto (ou parou perto)? Tira Foto na mesma.
            resultado = self.navigator.getResult()
            if resultado == TaskResult.SUCCEEDED or timed_out:
                self.get_logger().info(f'🎯 Ponto {index + 1} ({nome}). A apontar ao carro e capturar...')
                # rodar no sitio para a camara ficar centrada no carro
                self.girar_para_carro()          # aponta pela pose (aproximado)
                self.centrar_carro_visual()      # afina pela imagem (robusto)
                # esperar por um frame FRESCO da camara (lazy publisher do Webots)
                self.imagem_atual = None
                t_img = time.time()
                while self.imagem_atual is None and time.time() - t_img < 8.0:
                    rclpy.spin_once(self, timeout_sec=0.1)

                if self.imagem_atual is not None:
                    nome_ficheiro = f'foto_carro_{nome}.jpg'
                    cv2.imwrite(nome_ficheiro, self.imagem_atual)
                    self.get_logger().info(f'📸 Fotografia guardada: {nome_ficheiro}')
                    # registar a foto capturada (caminho + ponto) para o upload SIMAGIA
                    self.fotos_capturadas.append({
                        'name': nome, 'x': px, 'y': py, 'theta': ptheta,
                        'photo': nome_ficheiro,
                    })
                else:
                    self.get_logger().warn('Não há imagem da câmara disponível no momento!')
            else:
                self.get_logger().error(f'❌ Falha ao tentar chegar ao Ponto {index + 1}. A avançar para o próximo...')
                continue # Tenta o próximo ponto mesmo se falhar este

        # 4. Regressar ao ponto inicial (0,0) e parar la.
        if self.emergencia_ativada:
            return
        self.get_logger().info('🏠 Pontos visitados. A regressar ao ponto inicial...')
        self.navigator.goToPose(pose_inicial)
        t_ini = time.time()
        while not self.navigator.isTaskComplete():
            if self.emergencia_ativada:
                self.get_logger().warn('Missão abortada no regresso!')
                return
            if time.time() - t_ini > self.TIMEOUT_PONTO:
                self.get_logger().warn('Regresso demorou demais (preso?), a cancelar e parar.')
                self.navigator.cancelTask()
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            self.get_logger().info('✅ Missão Cumprida! Robô de volta ao ponto inicial.')
        else:
            self.get_logger().info('✅ Missão terminada (parado perto do ponto inicial).')
        # parar o robot de vez
        self.travar_robo()
        # enviar as fotos capturadas para o SIMAGIA
        self.enviar_simagia()


def main(args=None):
    import argparse
    import sys

    parser = argparse.ArgumentParser(description='Missao de inspecao ArgusAI + upload SIMAGIA')
    parser.add_argument('--simagia-base-url', dest='simagia_base_url')
    parser.add_argument('--simagia-claim-id', dest='simagia_claim_id')
    parser.add_argument('--robot-id', dest='robot_id')
    parser.add_argument('--mission-id', dest='mission_id')
    cli, _ = parser.parse_known_args()   # ignora os args do ROS (--ros-args ...)

    # resolver config do SIMAGIA ANTES de mexer no robot (fail-fast e mensagem clara)
    try:
        simagia_cfg = simagia_client.resolve_config(cli=vars(cli))
    except simagia_client.ConfigError as e:
        print(f"[ERRO SIMAGIA] {e}", file=sys.stderr)
        sys.exit(2)
    print(f"[SIMAGIA] base_url={simagia_cfg['base_url']} case_id={simagia_cfg['case_id']} "
          f"robot_id={simagia_cfg['robot_id']} mission_id={simagia_cfg['mission_id']}")

    rclpy.init(args=args)
    missao = MissaoArgus(simagia_cfg)

    try:
        # Inicia a missão em background e mantem os callbacks (câmara) a correr
        missao.executar_missao()
        
        # Mantém o nó vivo caso a missão termine mas queiramos ver as mensagens
        while rclpy.ok():
            rclpy.spin_once(missao, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        missao.get_logger().info('Script interrompido pelo utilizador.')
    finally:
        missao.travar_robo()
        if missao.mqtt_client is not None:
            missao.mqtt_client.loop_stop()
        missao.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
