#!/bin/bash
# Exploracao cuidadosa: move em passos pequenos e mede o skew das paredes
# entre cada passo. Para se as paredes comecarem a entortar.
source /opt/ros/humble/setup.bash

MON=/home/nuno/Documents/ArgusAI/skew_monitor.py

medir() {
  local etapa="$1"
  local j; j=$(timeout 8 python3 "$MON" 2>/dev/null | grep -E "^\{")
  echo "[$etapa] $j"
}

mover() {  # $1=duracao $2=twist
  timeout "$1" ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "$2" --rate 10 >/dev/null 2>&1
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once >/dev/null 2>&1
  sleep 2   # deixar o SLAM atualizar (map_update_interval=2.0)
}

LIN="{linear: {x: 0.2}}"
ROT_E="{angular: {z: 0.3}}"
ROT_D="{angular: {z: -0.3}}"

medir "inicio"
mover 3 "$LIN";   medir "frente-1"
mover 3 "$LIN";   medir "frente-2"
mover 4 "$ROT_E"; medir "rodar-esq-1"
mover 3 "$LIN";   medir "frente-3"
mover 3 "$LIN";   medir "frente-4"
mover 4 "$ROT_E"; medir "rodar-esq-2"
mover 3 "$LIN";   medir "frente-5"
mover 3 "$LIN";   medir "frente-6"
mover 4 "$ROT_E"; medir "rodar-esq-3"
mover 3 "$LIN";   medir "frente-7"
mover 3 "$LIN";   medir "frente-8"
echo "FIM"
