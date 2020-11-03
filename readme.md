Link do pobrania rosbaga
https://tulodz-my.sharepoint.com/:f:/g/personal/agnieszka_wegierska_dokt_p_lodz_pl/Eknv9CERc6lNr06leTviCN0BAK7rPHP6-KwFz86BDHp80A?e=YfpuUW
Pobranego rosbaga należy umieścić w katalogu rosbag.
Na poczatku nalezy uruchomic roscore
```
roscore
```
W nowym oknie przechodzimy do katalogu rosbag i uruchamiamy rosbaga z danymi
```
cd /rosbag && rosbag play -l 2020-10-29-13-28-50_10.bag
```

Podglad listy topicow
```
rostopic list
```
podglad wybranego topicu 
```
rostopic echo /nazwa_topicu
```
# Lista wazniejszych topiców
| czujnik      | nazwa topicu |
| ----------- | ----------- |
| imu      |     |
| skaner   | /sick_safetyscanners/scan |
| pozycja z amcl   | /amcl_pose |
| odometria z kol   | /odometry/odom |
| odometria przefiltrowana   | /odom |
| mapa   |         |
| kamera   |         |



