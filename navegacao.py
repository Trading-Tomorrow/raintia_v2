#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from cv_bridge import CvBridge
import cv2
import time
import math
try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None   # MQTT opcional: se nao estiver instalado, a missao corre sem paragem remota

class MissaoArgus(Node):
    def __init__(self):
        super().__init__('missao_argus')
        
        # --- CONFIGURAÇÕES DOS DESTINOS (O CARRO) ---
        # Coordenadas no FRAME DO MAPA. Carro detetado em (6.63, 4.68).
        # ORDEM otimizada pelo caixeiro-viajante (tsp_astar.py): anel a volta do
        # carro que minimiza a distancia total (~28m vs ~41m da ordem ingenua) e
        # acaba no ponto mais perto da base (regresso curto).
        # Formato: (X, Y, Theta, nome).
        self.pontos_inspecao = [
            (6.77,  2.11,  1.623, 'dir'),     # lado direito
            (10.17, 4.86, -3.089, 'frente'),  # frente
            (6.50,  7.24, -1.518, 'esq'),     # lado esquerdo
            (3.10,  4.49,  0.053, 'tras'),    # tras (mais perto da base)
        ]
        self.TIMEOUT_PONTO = 35.0   # segundos max por objetivo antes de cancelar
        
        # --- INICIALIZAÇÃO ROS 2 ---
        self.bridge = CvBridge()
        self.navigator = BasicNavigator()
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
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
                self.get_logger().info(f'🎯 Ponto {index + 1} ({nome}). A estabilizar e capturar...')
                # spin uns instantes para garantir um frame fresco da camara
                for _ in range(15):
                    rclpy.spin_once(self, timeout_sec=0.2)

                if self.imagem_atual is not None:
                    nome_ficheiro = f'foto_carro_{nome}.jpg'
                    cv2.imwrite(nome_ficheiro, self.imagem_atual)
                    self.get_logger().info(f'📸 Fotografia guardada: {nome_ficheiro}')
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


def main(args=None):
    rclpy.init(args=args)
    missao = MissaoArgus()
    
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
