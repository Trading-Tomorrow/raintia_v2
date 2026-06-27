#!/usr/bin/env python3
"""Teste: navega a UM ponto e reporta. Valida o pipeline Nav2 completo."""
import math, time, rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

ALVO = (10.17, 4.86, -3.089)   # ponto 'frente'

def main():
    rclpy.init()
    nav = BasicNavigator()
    print('a aguardar Nav2 ativo...', flush=True)
    nav.waitUntilNav2Active()
    print('Nav2 ativo. a enviar objetivo...', flush=True)
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = ALVO[0]; p.pose.position.y = ALVO[1]
    p.pose.orientation.z = math.sin(ALVO[2]/2); p.pose.orientation.w = math.cos(ALVO[2]/2)
    nav.goToPose(p)
    t0 = time.time()
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb and int(time.time()-t0) % 5 == 0:
            print(f'  restante={fb.distance_remaining:.2f}m', flush=True)
        if time.time()-t0 > 120:
            nav.cancelTask(); print('TIMEOUT'); break
        time.sleep(1)
    r = nav.getResult()
    print(f'RESULTADO: {r}', flush=True)
    nav.lifecycleShutdown() if False else None
    rclpy.shutdown()

if __name__ == '__main__':
    main()
