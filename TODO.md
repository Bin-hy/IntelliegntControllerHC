现在是一个QT界面，这个界面根据了每个单独的tab进行区分模块的

我需要你根据实际需求改成“智慧数字大屏”那种风格的样式

因此多个模块根据我如下的要求进行布局

1. 中间第一行类似管理界面的用户模块，右上角是用户头像，点击头像弹出下拉框进行登录和退出登录，预留权限管理的界面。
2. 大屏中间是一个3D视图，3D中展示之前写好的机器臂和机器手的3D实时同步。
3. 大屏左侧是orbbec的相机模块，可以选择不同的相机和多选对应的摄像头进行查看
4. 右侧是一排按钮 包括： 连接设备信息 | 任务模块

5. 连接设备信息：点击之后显示一个弹窗，来显示有哪些设备已经连接，包括设备类型，设备名称，sn码等。
5. 任务模块：点击之后显示之前写好的任务模块的弹窗。


# 新任务-动作捕捉手套
[官方文档](docs/HandDriver/README_HandDriver.md)
[SDK路径](./SDK_Linux/)

## 需求描述
我目前已经部署了宇叠动捕手套的软件部分，设置了target ip localhost, 端口为6321
我希望能够通过动捕手套直接控制机器手，因此我拆分为两个部分进行：

## 第一部分 —— 读取动捕手套的实时数据，发送给上位机
我需要在我的上位机中加一个tab，用来显示动捕手套的实时数据。
阅读官方文档，显示实时数据，目前使用欧拉角发送的数据。

## 第二部分 —— 将实时数据发送给机器手
待定方案，不确定是打算走ros2控制，还是通过Lhand的SDK接口直接控制。


# 遇到问题

## Orbbec相机问题
1. 现在接入了orbbec的215型号的相机用来 测耳朵景深与位置。请你阅读orbbec 215型号的设备帮我实现深度相机的的耳机景深和位置展示。 后续需要记录在每次执行的任务中的。
2. 现在扫描不到215型号的相机命名空间。


## 2026-3-29-1220 情况
1. 现在深度相机可以读取到了，但是深度和点位信息不是很准确。215摄像头拍出来的照片数据，耳机角度深度可以存本地。
2. 我现在的215相机对准的是一个人工假耳，然后我会用上位机控制任务执行给耳朵戴上耳机。我希望能够通过深度相机拍摄到耳机的带入的角度以及深度。帮我架构一下如何实现。

## 回复架构问题
1. 正对着耳朵，215相机对准的是耳朵的中心。从前方拍摄。
2. 入耳式耳机
3. 精度没有具体要求，越精准越好
4. 只需要在佩戴完成后进行显示角度和深度信息

# 1. 启动系统后，先拍基线（假耳无耳机状态）
ros2 service call /earphone_inspector/capture_baseline vision_server/srv/CaptureBaseline

# 2. 机器臂佩戴耳机后，执行测量
ros2 service call /earphone_inspector/measure vision_server/srv/MeasureEarphone "{file_tag: 'test1'}"


## 需求
在任务界面加一个延迟功能，每个步骤后可以添加延迟多久的时间执行下一步，单位设为ms。


1. 我现在需要控制平台的起降，要根据com口，使用modbus rtu协议
先发DO信号，相当于给电机解锁了，但是不能动，要发送报文才能动，详细文档：
/home/hwtws/IntelliegntControllerHC/docs/rs485

2. 之前的IO控制：
```
使用DIO数字接口，发送true为启动设备，发送false为停止设备
DO1:启停清洗机     DO2:启停左侧吹风机    DO3:启停右侧烘干机      DO4:升降平台解除抱闸
DO5:黄灯   DO6:绿灯     DO7:红灯   DO8:蜂鸣器
```

```
三色灯定义：通过红、黄、绿三种颜色灯光指示设备运行状态。
红色：紧急停止、严重故障、安全警报（需立即干预）
黄色：警告、注意、设备待机、预备状态
绿色：设备运行正常、循环完成、安全状态
蜂鸣器与红色灯同步输出
```

问题：
1. '/home/hwtws/IntelliegntControllerHC/ros2_ws/2026-3-29-1625.log' 这是日志。。现在出现了335相机问题。
2. 日志：hwtws@hwtws-O-E-M:~/IntelliegntControllerHC/ros2_ws$ ros2 service call /earphone_inspector/capture_baseline vision_server/srv/CaptureBaseline
waiting for service to become available...
requester: making request: vision_server.srv.CaptureBaseline_Request()

response:
vision_server.srv.CaptureBaseline_Response(success=True, message='Baseline captured (1280x720)')


hwtws@hwtws-O-E-M:~/IntelliegntControllerHC/ros2_ws$ ros2 service call /earphone_inspector/measure vision_server/srv/MeasureEarphone "{file_tag: 'test1'}"
waiting for service to become available...
requester: making request: vision_server.srv.MeasureEarphone_Request(file_tag='test1')

response:
vision_server.srv.MeasureEarphone_Response(success=False, message='No earphone region detected (no depth difference)', angle_deg=0.0, depth_mm=0.0, confidence=0.0, saved_path='')

hwtws@hwtws-O-E-M:~/IntelliegntControllerHC/ros2_ws$ ros2 service call /earphone_inspector/measure vision_server/srv/MeasureEarphone "{file_tag: 'test1'}"
waiting for service to become available...
requester: making request: vision_server.srv.MeasureEarphone_Request(file_tag='test1')

response:
vision_server.srv.MeasureEarphone_Response(success=False, message='No earphone region detected (no depth difference)', angle_deg=0.0, depth_mm=0.0, confidence=0.0, saved_path='')



ros2 service call /lift_server/lift_control lift_server/srv/LiftControl "{command: 'debug', speed_rpm: 0}"


基于RS485总线，标准Modbus RTU协议，选用轻量化协议包，使用异步线程控制伺服电机，发送参数如下:
1、工控机做主站，伺服驱动器是从站（站号1），识别串口信息，正确信息是：波特率19200、数据位8、停止位1、偶校验 EVEN 、流控制NONE
2、选择速度模式，发送HEX报文：01 06 00 01 00 03 98 0B;
3、激活电机内部速度，发送HEX报文：01 06 03 05 03 E8 99 31;
4、控制电机正转：发送HEX报文：01 06 05 1B 00 00 F9 01:
5、使能启动电机，发送HEX报文：01 06 05 14 00 10 C8 CE


ros2 service call /lift_server/lift_control lift_server/srv/LiftControl "{command: 'status', speed_rpm: 0}"

ros2 service call /lift_server/lift_control lift_server/srv/LiftControl "{command: 'move_up', speed_rpm: 1000}"                                       
  然后用 status 命令确认 P529 的值：                                            
  ros2 service call /lift_server/lift_control lift_server/srv/LiftControl "{command: 'status'}"



## 位置模式：
1、基于RS485总线，标准Modbus RTU协议，选用轻量化协议包，使用异步线程控制伺服电机，发送参数如下
2、工控机做主站，伺服驱动器是从站（站号1），识别串口信息，正确信息是：波特率19200、数据位8、停止位1、偶校验 EVEN 、流控制NONE；
3、选择位置模式，发送HEX报文：01 06 00 01 00 05 18 09
4、绝对定位上升沿换步单步执行，发送HEX报文：01 06 04 03 00 11 B8 F6
5、确定有效段数为2步：发送HEX报文：01 06 04 04 00 02 48 7A
第一段参数  //到达目标位置
6、给定10000高位脉冲，发送HEX报文：01 06 04 04 00 02 48 FA
7、给定第一段运行速度，发送HEX报文：01 06 04 0C 03 E8 48 47
8、给定第一段运行加速度，发送HEX报文：01 06 04 0d 03 E8 48 47
9、给定第一段运行减速度，发送HEX报文：01 06 04 0e 03 E8 48 47
10、第一次使能启动电机，发送HEX报文：01 06 05 14 00 10 C8 CE
11、第一次下降沿复位电机，发送HEX报文：01 06 05 23 00 00 C8 CE
12、第二次上升沿置位电机，发送HEX报文：01 06 05 23 00 10 C8 CE

这是位置模式的，我希望在IO控制的tab实现一下可以设置起降的高度，同样同步到task的设置中。



  五、三级角色权限设计建议

┌──────────────────────┬────────┬──────────┬────────┐
│ 功能                 │ 操作员 │ 维护人员 │ 管理员 │
├──────────────────────┼────────┼──────────┼────────┤
│ 上电/下电            │ ✅     │ ✅       │ ✅     │
│ 运行/停止任务        │ ✅     │ ✅       │ ✅     │
│ 查看相机画面         │ ✅     │ ✅       │ ✅     │
│ 相机截图/保存        │ ✅     │ ✅       │ ✅     │
│ CSV导出              │ ✅     │ ✅       │ ✅     │
│ 查看任务记录         │ ✅     │ ✅       │ ✅     │
│ 基础手动控制（限速） │ ✅     │ ✅       │ ✅     │
│ IO基础控制           │ ✅     │ ✅       │ ✅     │
├──────────────────────┼────────┼──────────┼────────┤
│ 全手动控制           │ ❌     │ ✅       │ ✅     │
│ 创建/编辑任务        │ ❌     │ ✅       │ ✅     │
│ 相机参数/标定        │ ❌     │ ✅       │ ✅     │
│ 校准（回原点）       │ ❌     │ ✅       │ ✅     │
│ 查看运行日志         │ ❌     │ ✅       │ ✅     │
├──────────────────────┼────────┼──────────┼────────┤
│ 用户管理             │ ❌     │ ❌       │ ✅     │
│ 删除日志             │ ❌     │ ❌       │ ✅     │
│ 删除照片             │ ❌     │ ❌       │ ✅     │
└──────────────────────┴────────┴──────────┴────────┘