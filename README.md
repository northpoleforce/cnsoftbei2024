2024 SoftCup Go1机械狗控制代码(新框架)
启动前需要去14板卡执行

```
sudo poweroff
```

然后去13板卡杀掉占用喇叭的进程

```
ps -aux | grep wsaudio
sudo kill -9 PID
```

