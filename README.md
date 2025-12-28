<!--
 * @Author: skybase
 * @Date: 2025-12-27 23:58:50
 * @LastEditors: skybase
 * @LastEditTime: 2025-12-28 21:50:32
 * @Description:  ᕕ(◠ڼ◠)ᕗ​ 
 * @FilePath: /wheel_legged_sim_ros/README.md
-->
# wheel_legged_sim_ros

## 环境搭建构建
- 构建本地镜像
```
docker compose -f '.devcontainer/docker-compose.yml' up -d --build 'builder' 
``` 
- 启动本地docker容器服务
```
docker compose -f '.devcontainer/docker-compose.yml' up -d --build 'legged-service'
```
如果已经其用过该服务，直接启动dokcer容器即可
```
docker start legged-container
```
允许容器访问宿主机的显示
```
xhost +local:docker
```
- 进入名为“legged-container“的容器
```
docker exec -it  legged-container bash
```
- 构建编译代码
```
colcon build
```

## 仿真的启动步骤
- 启动mujoco仿真器和对应场景模型
```
ros2 run mujoco_sim simulate
# 可选
# 开启foxglove桥连接
ros2 run foxglove_bridge foxglove_bridge
```