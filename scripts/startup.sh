#!/bin/bash

# SSH到192.168.123.14并关闭电脑
sshpass -p '123' ssh -o StrictHostKeyChecking=no unitree@192.168.123.14 'echo 123 | sudo -S poweroff'

# SSH到192.168.123.13并杀掉wsaudio进程，然后执行restart_camera.sh脚本并保持连接
sshpass -p '123' ssh -o StrictHostKeyChecking=no unitree@192.168.123.13 << 'EOF' &
  while true; do
    # 查找并杀掉wsaudio进程
    pid=$(ps -aux | grep './build/wsaudio' | grep -v 'grep' | awk '{print $2}')
    if [ ! -z "$pid" ]; then
      kill -9 $pid
      echo "Killed wsaudio process with PID $pid"
      break
    fi
    # 等待1秒再重试
    sleep 1
  done
  # 执行restart_camera.sh脚本
  /home/unitree/CAIR/scripts/restart_camera.sh
EOF

# 等待几秒钟以确保 restart_camera.sh 脚本已经启动
sleep 6

# 设置ulimit参数
ulimit -s unlimited

# 在主电脑上执行 main 程序
~/Documents/24SoftCupCodes/24SoftCupFOlder/code/cnsoftbei2024_merge_v0/cnsoftbei2024/build/main

