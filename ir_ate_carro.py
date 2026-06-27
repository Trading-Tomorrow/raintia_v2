#!/usr/bin/env python3
"""
Navega DEVAGAR ate perto do carro, desviando de obstaculos com o lidar,
para mapear o caminho de forma limpa (odometria pura calibrada).
Para quando chega a ~3m do centro do carro ou se ficar encurralado.
"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

CARRO = (6.38, 4.74)        # centro do carro no frame do mapa
CHEGADA = 3.0               # parar a esta distancia do centro (carro e grande)
STOP_OBST = 0.55            # paragem de seguranca a obstaculos
LENTO_LIN = 0.14
LENTO_ANG = 0.30
T_MAX = 150                 # limite de seguranca (s)

class Nav(Node):
    def __init__(self):
        super().__init__('ir_ate_carro')
        self.scan=None; self.amin=None; self.ainc=None
        self.x=None; self.y=None; self.yaw=None
        self.create_subscription(LaserScan,'/scan',self._s,1)
        self.create_subscription(Odometry,'/odom',self._o,1)
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)

    def _s(self,m):
        self.scan=np.array(m.ranges,dtype=np.float32); self.amin=m.angle_min; self.ainc=m.angle_increment
    def _o(self,m):
        p=m.pose.pose.position; q=m.pose.pose.orientation
        self.x,self.y=p.x,p.y
        self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def fresco(self,n=5):
        for _ in range(n): rclpy.spin_once(self,timeout_sec=0.05)

    def setor(self,centro_deg,meia_deg=25):
        # menor alcance num setor angular (graus, 0=frente)
        i=int(round((math.radians(centro_deg)-self.amin)/self.ainc))
        d=int(meia_deg/math.degrees(abs(self.ainc)))
        seg=self.scan[max(0,i-d):i+d]; seg=seg[np.isfinite(seg)&(seg>0.05)]
        return float(seg.min()) if len(seg) else 99.0

    def parar(self):
        for _ in range(5): self.pub.publish(Twist()); time.sleep(0.02)

    def run(self):
        self.fresco(15)
        if self.x is None: print('ERRO: sem odom'); return
        t0=time.time(); passo=0
        while time.time()-t0 < T_MAX:
            self.fresco(3)
            dist=math.hypot(CARRO[0]-self.x, CARRO[1]-self.y)
            if dist < CHEGADA:
                print(f'CHEGOU: a {dist:.2f}m do centro do carro'); break
            ang=math.atan2(CARRO[1]-self.y, CARRO[0]-self.x)
            erro=math.atan2(math.sin(ang-self.yaw),math.cos(ang-self.yaw))
            frente=self.setor(0,25); esq=self.setor(40,25); dir=self.setor(-40,25)
            cmd=Twist()
            if frente < STOP_OBST:
                # obstaculo perto: rodar para o lado mais livre, sem avancar
                cmd.angular.z = LENTO_ANG if esq>dir else -LENTO_ANG
                modo='DESVIO'
            elif abs(erro) > 0.25:
                cmd.angular.z = LENTO_ANG*(1 if erro>0 else -1)
                modo='VIRAR'
            else:
                cmd.linear.x = LENTO_LIN
                # pequena correcao de rumo enquanto anda
                cmd.angular.z = max(-0.2,min(0.2, 0.6*erro))
                # se obstaculo a frente a media distancia, abranda e enviesa
                if frente < 1.0:
                    cmd.linear.x = 0.07
                    cmd.angular.z += (LENTO_ANG if esq>dir else -LENTO_ANG)*0.6
                modo='AVANCAR'
            self.pub.publish(cmd)
            passo+=1
            if passo%12==0:
                self.parar(); time.sleep(1.2)   # pausa p/ SLAM atualizar o mapa
                print(f'  dist={dist:.2f}m erro={math.degrees(erro):+.0f}deg '
                      f'frente={frente:.2f} [{modo}]',flush=True)
            time.sleep(0.1)
        self.parar()
        print(f'fim (dist final={math.hypot(CARRO[0]-self.x,CARRO[1]-self.y):.2f}m)')

def main():
    rclpy.init(); n=Nav()
    try: n.run()
    finally:
        n.parar(); n.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
