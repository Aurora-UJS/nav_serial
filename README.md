# nav_serial

ROS2 串口驱动节点，用于 NUC 与 C-Board 间通信，并发布云台状态。

## 关键话题

- 发布：`/serial/gimbal_joint_state`（类型：`rm_interfaces/msg/GimbalState`）
- 订阅：`/red_standard_robot1/cmd_vel`

## 关于 `rm_interface` / `rm_interfaces`

- 正确包名是：`rm_interfaces`（复数）
- 本项目使用了 `rm_interfaces/msg/GimbalState`
- `CMakeLists.txt` 与 `package.xml` 都依赖 `rm_interfaces`

如果有人问 `rm_interface`，通常是口误或记错包名。若缺少该包，编译会在消息头或 `find_package(rm_interfaces)` 处失败。

## 编译

```zsh
# 在包含 nav_serial 与 rm_interfaces 的 ROS2 工作空间根目录执行
colcon build --packages-select nav_serial --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## 运行

```zsh
source install/setup.zsh
ros2 launch nav_serial serial_driver.launch.py
# 如果你是 bash，请改为 source install/setup.bash
```

## 常见问题

1. 报错 `rm_interfaces/... not found`
   - 确认 `rm_interfaces` 包已在同一工作空间并已被编译
   - 或确认对应安装路径已被 `source install/setup.*` 载入

2. 服务相关头文件缺失（`std_srvs`）
   - 已在 `package.xml` 声明依赖，重新 `colcon build` 即可
