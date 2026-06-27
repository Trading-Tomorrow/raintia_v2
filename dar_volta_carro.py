#!/usr/bin/env python3
"""
Da a volta ao carro DEVAGAR por SEGUIMENTO DE CONTORNO: mantem o carro a ESQUERDA
a uma distancia fixa, andando em frente (orbita anti-horaria). Robusto para o carro
(objeto grande) porque nao usa o sensor frontal contra ele. Para apos ~360 graus.
"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

CARRO = (6.38, 4.74)
D_ALVO = 1.6            # distancia desejada ao carro (lado esquerdo)
LIN = 0.13
ANG = 0.40
KP = 0.9               # ganho do seguimento
FRONT_STOP = 0.7       # obstaculo mesmo a frente (parede/barril) -> vira direita
T_MAX = 260

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class Seguir(Node):
    def __init__(self):
        super().__init__('dar_volta_carro')
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
    def fresco(self,n=3):
        for _ in range(n): rclpy.spin_once(self,timeout_sec=0.05)
    def setor(self,centro_deg,meia_deg):
        i=int(round((math.radians(centro_deg)-self.amin)/self.ainc))
        d=int(meia_deg/math.degrees(abs(self.ainc)))
        seg=self.scan[max(0,i-d):i+d]; seg=seg[np.isfinite(seg)&(seg>0.05)]
        return float(seg.min()) if len(seg) else 99.0
    def parar(self):
        for _ in range(5): self.pub.publish(Twist()); time.sleep(0.02)

    def run(self):
        self.fresco(15)
        if self.x is None: print('ERRO: sem odom'); return
        phi_prev=math.atan2(self.y-CARRO[1], self.x-CARRO[0]); cum=0.0
        print(f'inicio: ang={math.degrees(phi_prev):.0f}deg',flush=True)
        t0=time.time(); ult=0.0
        while time.time()-t0 < T_MAX:
            self.fresco(2)
            phi=math.atan2(self.y-CARRO[1], self.x-CARRO[0])
            cum += wrap(phi-phi_prev); phi_prev=phi
            if cum >= 2*math.pi + 0.1:
                print('VOLTA COMPLETA (360 graus)'); break
            esq   = self.setor(90,35)    # carro (a esquerda)
            front = self.setor(0,12)     # obstaculo a frente
            cmd=Twist()
            if front < FRONT_STOP:
                cmd.angular.z = -ANG; cmd.linear.x = 0.03      # parede/barril a frente -> vira direita
            elif esq > 3.0:
                cmd.angular.z = 0.35; cmd.linear.x = 0.08      # perdeu o carro -> curva esquerda p/ reencontrar
            else:
                err = esq - D_ALVO
                cmd.angular.z = max(-ANG,min(ANG, KP*err))     # >0 longe->vira esq; <0 perto->vira dir
                cmd.linear.x = LIN if abs(err) < 0.6 else 0.07
            self.pub.publish(cmd)
            if time.time()-ult > 6:
                ult=time.time()
                print(f'  {math.degrees(cum):.0f}/360deg  pos=({self.x:.1f},{self.y:.1f})  '
                      f'esq={esq:.2f} front={front:.2f}',flush=True)
            time.sleep(0.1)
        self.parar()
        print(f'fim: {math.degrees(cum):.0f}deg percorridos')

def main():
    rclpy.init(); n=Seguir()
    try: n.run()
    finally:
        n.parar(); n.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
