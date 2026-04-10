  完整技术方案    
                                                                           
  1. 系统总体架构 
                                                                           
  数据流                                                                   
                                                                           
  Orbbec Gemini 305 (Eye-in-Hand)                                          
      │                                                                    
      ├─ /camera/color/image_raw (RGB)──→ bottle_detector_node ──→ (u,v)   
  bbox中心                                                                 
      ├─ /camera/depth/image_raw (16UC1 mm)──→ depth_lookup ──→ depth_mm   
      └─ /camera/depth/camera_info ──→ 内参矩阵 K                          
                                            │                              
                                      P_camera = deproject(u, v, depth, K) 
                                            │                              
                                      T_end_camera (标定结果, static TF)   
                                            │                              
                                      T_base_end (从 /tf 实时获取)
                                            │                              
                                      P_base = T_base_end * T_end_camera * 
  P_camera                                                                 
                                            │                              
                                      /duco_robot/robot_move (movel)       
                                            │                              
                                      /lhandpro_service/set_all_position +
  move_motors                                                              
                  
  ROS2 节点设计                                                            
                  
  ┌────────────────────────┬─────────────────────────────┬───────────┐     
  │          节点          │            功能             │ 新增/已有 │
  ├────────────────────────┼─────────────────────────────┼───────────┤     
  │ orbbec_camera_node     │ 发布RGB/Depth/CameraInfo    │ 已有      │
  ├────────────────────────┼─────────────────────────────┼───────────┤
  │ bottle_detector_node   │ YOLOv8推理，输出bbox        │ 新增      │     
  ├────────────────────────┼─────────────────────────────┼───────────┤     
  │ grasp_coordinator_node │ 串联检测→坐标转换→运动→抓取 │ 新增      │     
  ├────────────────────────┼─────────────────────────────┼───────────┤     
  │ duco_ros_driver        │ 机械臂控制                  │ 已有      │
  ├────────────────────────┼─────────────────────────────┼───────────┤     
  │ lhandpro_service       │ 灵巧手控制                  │ 已有      │
  └────────────────────────┴─────────────────────────────┴───────────┘     
                  
  Topic / Service / Action 设计                                            
                  
  # 新增 Topics                                                            
  /bottle_detector/detections        common_msgs/msg/Detection2DArray
  (或自定义)                                                               
  /bottle_detector/debug_image       sensor_msgs/msg/Image
  (标注框可视化)                                                           
                  
  # 新增 Services                                                          
  /grasp_coordinator/trigger_grasp   std_srvs/srv/Trigger
  (触发一次抓取)                                                           
  
  # 复用已有 Services                                                      
  /duco_robot/robot_move             duco_msg/srv/RobotMove
  (movel)                                                                  
  /lhandpro_service/set_all_position lhandpro_interfaces/srv/SetAllPosition
  /lhandpro_service/move_motors      lhandpro_interfaces/srv/MoveMotors    
                                                                           
  2. 相机与数据获取                                                        
                                                                           
  接入方式                                                                 
                  
  你的 vision_system.launch.py 已经通过 OrbbecSDK ROS2                     
  包自动检测相机并启动。Gemini 305 属于 Gemini 330 系列检测逻辑（代码中按
  PID 分类）。                                                             
                  
  关键确认：Gemini 305 的 PID 是否在你 launch 文件的 gemini_330_pids       
  列表中。如果不在，需要添加。查一下 Orbbec 文档确认 PID。
                                                                           
  Topics（相机SN为命名空间，假设为 cam_305）                               
  
  /cam_305/color/image_raw          sensor_msgs/Image (bgr8 或 rgb8)       
  /cam_305/depth/image_raw          sensor_msgs/Image (16UC1, 单位mm)      
  /cam_305/depth/camera_info        sensor_msgs/CameraInfo (含内参K)       
  /cam_305/depth/points             sensor_msgs/PointCloud2 (有序点云)     
                                                                           
  深度与RGB对齐                                                            
                                                                           
  OrbbecSDK ROS2 默认发布对齐后的 depth_to_color 图像（align_depth         
  参数默认为 true）。如果未对齐，在 launch 参数中设置：
                                                                           
  {'align_depth': True}                                                    
  
  对齐后，RGB 的 (u,v) 可直接用于查询 depth 图像的对应位置。               
                  
  3. 目标检测（水瓶）                                                      
                  
  是否需要训练？                                                           
                  
  不需要训练。 COCO 80类中有 bottle (class_id=39)。直接使用 YOLOv8n        
  预训练模型。    
                                                                           
  推理流程        

  输入：RGB图像 (640×480 或 1280×720)                                      
    │
    ├─ 预处理：resize to 640×640, normalize [0,1], NCHW                    
    │                                                                      
    ├─ 推理：ONNX Runtime (yolov8n.onnx) 或 ultralytics Python             
    │                                                                      
    ├─ 后处理：NMS, 过滤 class_id==39 (bottle), confidence > 0.5
    │                                                                      
    └─ 输出：List[(x1,y1,x2,y2,confidence)]
                                                                           
  bbox 中心点计算 
                                                                           
  u_center = (x1 + x2) / 2                                                 
  v_center = (y1 + y2) / 2    # 可下移 1/3 高度，瞄准瓶身中部              
                                                                           
  实现建议                                                                 
                                                                           
  用 Python 节点 + ultralytics 库，原因：                                  
  
  1. 部署最快（pip install ultralytics，3行代码完成推理）                  
  2. 你的系统不需要实时（检测一次即可抓取）
  3. 后续切换模型容易                                                      
                                                                           
  from ultralytics import YOLO                                             
  model = YOLO("yolov8n.pt")                                               
  results = model(cv_image, classes=[39])  # 只检测 bottle                 
  for box in results[0].boxes:                                             
      u_center = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
      v_center = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)                
                  
  4. 3D 坐标计算                                                           
                  
  公式                                                                     
                  
  你的 depth_measure_node.cpp 已经实现了这个。核心公式（pinhole model）：  
  
  给定相机内参 K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]                   
  给定像素 (u, v) 和深度 d (米)                                            
                                                                           
  X_cam = (u - cx) * d / fx                                                
  Y_cam = (v - cy) * d / fy                                                
  Z_cam = d                                                                
                  
  深度滤波策略                                                             
  
  复用你 depth_measure_node 的 cluster 方法：在 bbox 中心点周围取 20×20    
  像素区域，过滤掉 <100mm 和 >2000mm 的无效值，取中位数深度。
                                                                           
  // 伪代码 - 参考你的 depth_measure_node 实现
  roi = depth_image[v-10:v+10, u-10:u+10]                                  
  valid_depths = roi[roi > 100 && roi < 2000]                              
  depth_mm = median(valid_depths)                                          
                                                                           
  是否用点云替代？                                                         
                  
  不建议。 对于单目标抓取，(u,v,depth) 反投影已经足够，且实现更简单。点云增
  加了依赖和计算量，对你的场景没有明显收益。
                                                                           
  5. Eye-in-Hand 标定（核心模块）                                          
  
  方案选择                                                                 
                  
  使用 easy_handeye2（ROS2 版本）+ AprilTag，这是最成熟的方案。            
  
  标定流程                                                                 
                  
  准备工作：                                                               
  
  1. 打印 AprilTag 标定板（建议 tag36h11 族，单个 tag，尺寸                
  80-100mm），固定在桌面不动
  2. 安装依赖：                                                            
  sudo apt install ros-jazzy-apriltag-ros
  pip install transforms3d               
  git clone https://github.com/IRVLUTD/easy_handeye2.git  # 或使用         
  moveit_calibration                                               
                                                                           
  数据采集步骤：  
                                                                           
  1. 将机械臂移到标定板上方，确保相机能看到 AprilTag                       
  2. 在不同位姿下采集 15-20 组数据（每组包含）：                           
    - 机械臂末端位姿 T_base_end（从 /duco_cobot/robot_state 的             
  cart_actual_position 读取）                                              
    - 相机看到的 AprilTag 位姿 T_camera_tag（从 apriltag_ros 获取）        
  3. 运动策略：                                                            
    - 保持标定板始终在视野内                                               
    - 变换平移(x,y,z各方向±50mm) 和旋转(绕z轴±15°, 绕x/y轴±10°)            
    - 避免极端角度（tag检测质量下降）                                      
                                                                           
  # 采集一个数据点的流程：                                                 
  # 1. 手动移动机械臂到新位姿                                              
  # 2. 记录 T_base_end (从 robot_state)                                    
  ros2 topic echo /duco_cobot/robot_state --once                           
  # cart_actual_position: [X, Y, Z, RX, RY, RZ]                            
                                                                           
  # 3. 记录 T_camera_tag (从 apriltag_ros)                                 
  ros2 topic echo /apriltag/detections --once                              
                                                                           
  求解 T_end_camera：                                                      
                                                                           
  这是经典的 AX=XB 问题。有多组：                                          
  - A_i = T_base_end_i^{-1} * T_base_end_j（末端运动）
  - B_i = T_camera_tag_i * T_camera_tag_j^{-1}（相机观测运动）             
                                                              
  求解 X = T_end_camera                                                    
                                                                           
  # 使用 OpenCV 求解                                                       
  import cv2                                                               
  R_end_camera, t_end_camera = cv2.calibrateHandEye(                       
      R_gripper2base_list, t_gripper2base_list,   # T_base_end 的逆        
      R_target2cam_list, t_target2cam_list,        # T_camera_tag          
      method=cv2.CALIB_HAND_EYE_TSAI              # Tsai 方法最稳定        
  )                                                                        
                                                                           
  验证方法：                                                               
                  
  1. 移动到 3-5 个新位姿（不在采集集中）                                   
  2. 用标定结果计算 tag 在基座坐标系下的位置：P_base = T_base_end * 
  T_end_camera * P_camera                                                  
  3. 多个位姿下计算出的 tag 基座坐标应一致，重投影误差 < 5mm 为合格
  4. 实测：让机械臂移动到计算出的目标点上方，目视检查偏差                  
                                                                           
  替代方案（如果不想用 easy_handeye2）                                     
                                                                           
  手动标定法（更快但精度稍低）：                                           
                  
  1. 在机械臂末端法兰上安装相机，手动测量相机光心到法兰中心的偏移          
  2. 用固定的平移+旋转作为 T_end_camera
  3. 用实际抓取测试微调                                                    
                                                                           
  对于2周 Demo，建议先用手动标定快速跑通，后续再用 easy_handeye2 精调。    
                                                                           
  6. 坐标转换（TF系统）                                                    
                  
  TF 树设计                                                                
                  
  world                                                                    
    └── base_link                          (机械臂基座)                    
          └── link_1                        (已有, robot_state_publisher)  
                └── link_2                                                 
                      └── ...                                              
                            └── link_6      (末端法兰)                     
                                  └── camera_link        ← 新增 static TF  
  (标定结果)                                                               
                                        └── camera_optical_frame  ← 新增   
  (x-right, y-down, z-forward)                                             
                  
  发布标定结果                                                             
                  
  # 假设标定结果为: translation (tx, ty, tz), rotation quaternion (qx, qy, 
  qz, qw)                                                                  
  ros2 run tf2_ros static_transform_publisher \                            
      tx ty tz qx qy qz qw \                                               
      link_6 camera_link                                                   
                                                                           
  或者在 launch 文件中添加：                                               
                                                                           
  Node(                                                                    
      package='tf2_ros',
      executable='static_transform_publisher',                             
      arguments=['--x', '0.05', '--y', '0.0', '--z', '0.08',               
                 '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',       
                 '--frame-id', 'link_6', '--child-frame-id',               
  'camera_link'],                                                          
  )                                                                        
                                                                           
  坐标转换计算                                                             
                                                                           
  // 使用 tf2 实现（推荐）                                                 
  #include <tf2_ros/buffer.h>                                              
  #include <tf2_ros/transform_listener.h>                                  
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>                       
                                                                           
  // 在 grasp_coordinator_node 中:                                         
  geometry_msgs::msg::PointStamped p_camera;                               
  p_camera.header.frame_id = "camera_link";                                
  p_camera.point.x = X_cam;  // 来自反投影                                 
  p_camera.point.y = Y_cam;                                                
  p_camera.point.z = Z_cam;                                                
                                                                           
  geometry_msgs::msg::PointStamped p_base;                                 
  tf_buffer_->transform(p_camera, p_base, "base_link");                    
  // p_base 就是目标在机械臂基坐标系下的位置                               
                                                                           
  等价矩阵公式：P_base = T_base_end * T_end_camera * P_camera              
                                                                           
  使用 tf2 的好处是 T_base_end 自动从 /tf 获取（robot_state_publisher      
  发布），T_end_camera 是你发布的 static TF，无需手动矩阵乘法。
                                                                           
  7. 抓取策略（水瓶简化方案）                                              
  
  为什么不需要6D姿态估计                                                   
                  
  1. 水瓶是圆柱体，绕自身轴旋转对称 → 不需要估计 yaw                       
  2. 水瓶放在桌面上 → 轴线垂直（或接近垂直）→ 不需要估计 roll/pitch
  3. 只需要知道 (x, y, z) 位置，从上方垂直抓取即可                         
                                                                           
  抓取点选择                                                               
                                                                           
  抓取点 = 水瓶中心 (x, y) + z方向偏移                                     
                                                                           
  # z 偏移考虑：                                                           
  # - 水瓶高度约 200mm                                                     
  # - 抓取位置选在瓶身中部 (距桌面约 100mm)                                
  # - z_grasp = z_bottle_top - 0.10  (或 z_table + 0.10)                   
                                                                           
  抓取姿态                                                                 
                                                                           
  # 垂直向下抓取 (RPY)                                                     
  RX = 3.14159   # 180° (末端Z轴朝下)                                      
  RY = 0.0                                                                 
  RZ = 0.0       # 任意（圆柱对称）                                        
                                                                           
  # 对应 movel 的 p 参数:                                                  
  p = [x_base, y_base, z_grasp, 3.14159, 0.0, 0.0]                         
                                                                           
  避免碰撞策略    
                                                                           
  分阶段运动：                                                             
  1. pre-grasp: 先到目标正上方 100mm（安全高度）
  2. approach: 垂直下降到抓取高度                                          
  3. grasp: 关闭灵巧手           
  4. lift: 垂直上升 100mm                                                  
  5. retreat: 移到放置位置                                                 
                                                                           
  8. 机械臂控制                                                            
                                                                           
  关键判断：不用 MoveIt2                                                   
                  
  理由：                                                                   
  1. 你的 duco_ros_driver 使用私有 RPC API（DucoCobot），不是标准
  ros2_control 接口                                                        
  2. 配置 MoveIt2  
  需要：SRDF、kinematics.yaml、controllers.yaml、ros2_control 适配 →       
  至少1周工作量                                                            
  3. 水瓶抓取是简单的笛卡尔运动，不需要运动规划
  4. DUCO 控制器内部已有 IK 求解器（movel 命令本身就做 IK）                
                                                                           
  直接使用 RobotMove.srv 的 movel                                          
                                                                           
  auto request = std::make_shared<duco_msg::srv::RobotMove::Request>();    
  request->command = "movel";                                              
  request->arm_num = 0;
  // p: [X(m), Y(m), Z(m), RX(rad), RY(rad), RZ(rad)]                      
  request->p = {x_base, y_base, z_target, 3.14159f, 0.0f, 0.0f};           
  request->v = 0.1f;    // 0.1 m/s (安全速度)                              
  request->a = 0.5f;    // 0.5 m/s²                                        
  request->r = 0.0f;    // 无混合                                          
  request->block = true; // 等待完成                                       
  auto future = client_move_->async_send_request(request);                 
                                                                           
  抓取动作流程                                                             
                                                                           
  ┌─────────────┬──────────────┬────────────────────────────┬─────────┐    
  │    阶段     │     动作     │         movel 目标         │  速度   │
  ├─────────────┼──────────────┼────────────────────────────┼─────────┤    
  │ 1.          │ 移到目标上方 │ (x, y, z+0.10)             │ 0.2 m/s │
  │ pre-grasp   │              │                            │         │    
  ├─────────────┼──────────────┼────────────────────────────┼─────────┤    
  │ 2. approach │ 垂直下降     │ (x, y, z_grasp)            │ 0.05    │    
  │             │              │                            │ m/s     │    
  ├─────────────┼──────────────┼────────────────────────────┼─────────┤    
  │ 3. grasp    │ 关闭手       │ -                          │ -       │
  ├─────────────┼──────────────┼────────────────────────────┼─────────┤    
  │ 4. lift     │ 垂直上升     │ (x, y, z+0.15)             │ 0.1 m/s │
  ├─────────────┼──────────────┼────────────────────────────┼─────────┤    
  │ 5. place    │ 移到放置位   │ (x_place, y_place,         │ 0.2 m/s │
  │             │              │ z_place)                   │         │    
  ├─────────────┼──────────────┼────────────────────────────┼─────────┤
  │ 6. release  │ 打开手       │ -                          │ -       │
  └─────────────┴──────────────┴────────────────────────────┴─────────┘

  9. 灵巧手控制（DH116 简化为夹爪）                                        
  
  你的系统已有完整链路                                                     
                  
  从 system_controller_node.cpp 的 execute_hand_step 可以看到：            
                  
  // 关闭（抓取） - 6个电机设为抓取位置                                    
  set_all_position({800, 800, 800, 800, 800, 800});  // 值需要实测调整     
  move_motors(0);  // 0 = 所有电机                                         
                                                                           
  // 打开（释放）                                                          
  set_all_position({0, 0, 0, 0, 0, 0});              // 全开               
  move_motors(0);                                                          
                  
  具体实现                                                                 
                  
  void close_hand() {                                                      
      auto req =                                                           
  std::make_shared<lhandpro_interfaces::srv::SetAllPosition::Request>();   
      req->positions = {800, 800, 800, 800, 800, 800};  // 抓取位置，需实测
      client_set_pos_->async_send_request(req);                            
      // 等待完成...                                                       
                                                                           
      auto req2 =                                                          
  std::make_shared<lhandpro_interfaces::srv::MoveMotors::Request>();
      req2->joint_id = 0;  // 0 = all                                      
      client_move_motors_->async_send_request(req2);
  }                                                                        
                                                                           
  void open_hand() {                                                       
      auto req =                                                           
  std::make_shared<lhandpro_interfaces::srv::SetAllPosition::Request>();
      req->positions = {0, 0, 0, 0, 0, 0};
      // ... 同上                                                          
  }                                                                        
                                                                           
  建议：positions 值需要实际测试确定。先手动试几个值，找到合适的抓握力度。 
                  
  10. 完整流程伪代码                                                       
                  
  #!/usr/bin/env python3                                                   
  """grasp_coordinator_node.py - 视觉引导抓取协调节点"""                   
                                                                           
  class GraspCoordinator(Node):                                            
      def __init__(self):                                                  
          super().__init__('grasp_coordinator')                            
          # TF    
          self.tf_buffer = Buffer()                                        
          self.tf_listener = TransformListener(self.tf_buffer, self)
          # 订阅                                                           
          self.sub_color = self.create_subscription(Image,                 
  '/cam_305/color/image_raw', self.cb_color, 1)                            
          self.sub_depth = self.create_subscription(Image,                 
  '/cam_305/depth/image_raw', self.cb_depth, 1)                            
          self.sub_info  = self.create_subscription(CameraInfo,
  '/cam_305/depth/camera_info', self.cb_info, 1)                           
          # 客户端
          self.cli_move = self.create_client(RobotMove,                    
  '/duco_robot/robot_move')                                                
          self.cli_hand_pos = self.create_client(SetAllPosition,           
  '/lhandpro_service/set_all_position')                                    
          self.cli_hand_move = self.create_client(MoveMotors,
  '/lhandpro_service/move_motors')                                         
          # 服务
          self.srv_trigger = self.create_service(Trigger,                  
  '/grasp_coordinator/trigger_grasp', self.trigger_cb)                     
          # YOLO
          self.yolo = YOLO("yolov8n.pt")                                   
                                                                           
      def trigger_cb(self, request, response):                             
          """一次完整抓取流程"""                                           
          try:                                                             
              # === Step 1: 检测水瓶 ===
              results = self.yolo(self.latest_color, classes=[39])  #      
  bottle                                                                   
              if len(results[0].boxes) == 0:                               
                  response.success = False                                 
                  response.message = "No bottle detected"
                  return response                                          
                                                                           
              box = results[0].boxes[0]  # 取置信度最高的                  
              u = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)               
              v = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)               
                                                                           
              # === Step 2: 获取深度 ===                                   
              roi = self.latest_depth[v-10:v+10, u-10:u+10]                
              valid = roi[(roi > 100) & (roi < 2000)]                      
              if len(valid) == 0:                                          
                  response.success = False                                 
                  response.message = "No valid depth"                      
                  return response                                          
              depth_m = float(np.median(valid)) / 1000.0
                                                                           
              # === Step 3: 反投影到相机坐标系 ===                         
              fx, fy = self.K[0], self.K[4]                                
              cx, cy = self.K[2], self.K[5]                                
              x_cam = (u - cx) * depth_m / fx                              
              y_cam = (v - cy) * depth_m / fy                              
              z_cam = depth_m                                              
                                                                           
              # === Step 4: TF 转换到基坐标系 ===                          
              p_cam = PointStamped()
              p_cam.header.frame_id = "camera_link"                        
              p_cam.header.stamp = self.get_clock().now().to_msg()         
              p_cam.point = Point(x=x_cam, y=y_cam, z=z_cam)               
              p_base = self.tf_buffer.transform(p_cam, "base_link",        
  timeout=Duration(seconds=1))
                                                                           
              x, y, z = p_base.point.x, p_base.point.y, p_base.point.z     
              self.get_logger().info(f"Target in base: ({x:.3f}, {y:.3f}, 
  {z:.3f})")                                                               
                  
              # === Step 5: 打开手 ===                                     
              self.open_hand()
                                                                           
              # === Step 6: Pre-grasp (目标上方100mm) ===                  
              self.movel(x, y, z + 0.10, rx=3.14159, ry=0.0, rz=0.0, v=0.2)
                                                                           
              # === Step 7: Approach (下降到抓取高度) ===                  
              z_grasp = z + 0.02  # 偏移量需实测（考虑手指长度）           
              self.movel(x, y, z_grasp, rx=3.14159, ry=0.0, rz=0.0, v=0.05)
                                                                           
              # === Step 8: Grasp ===                                      
              self.close_hand()                                            
              time.sleep(1.0)  # 等待手闭合                                
                  
              # === Step 9: Lift ===                                       
              self.movel(x, y, z + 0.15, rx=3.14159, ry=0.0, rz=0.0, v=0.1)
                                                                           
              # === Step 10: Move to place position ===                    
              self.movel(PLACE_X, PLACE_Y, PLACE_Z, rx=3.14159, ry=0.0,    
  rz=0.0, v=0.2)                                                           
                  
              # === Step 11: Release ===                                   
              self.open_hand()
                                                                           
              response.success = True
              response.message = f"Grasped bottle at ({x:.3f}, {y:.3f},    
  {z:.3f})"                                                                
          except Exception as e:
              response.success = False                                     
              response.message = str(e)                                    
          return response
                                                                           
      def movel(self, x, y, z, rx, ry, rz, v=0.1, a=0.5):                  
          req = RobotMove.Request()                                        
          req.command = "movel"                                            
          req.arm_num = 0
          req.p = [float(x), float(y), float(z), float(rx), float(ry),     
  float(rz)]                                                               
          req.v = float(v)
          req.a = float(a)                                                 
          req.block = True
          future = self.cli_move.call_async(req)                           
          rclpy.spin_until_future_complete(self, future, timeout_sec=30)
          result = future.result()                                         
          if result.response != "4":  # 4 = finished
              raise RuntimeError(f"movel failed: {result.response}")       
                                                                           
  11. 时间评估 + 风险点                                                    
                                                                           
  时间评估                                                                 
                  
  ┌────────────────┬──────────┬────────────────────────────────────────┐   
  │      模块      │  工作量  │                  说明                  │
  ├────────────────┼──────────┼────────────────────────────────────────┤
  │ 相机接入确认   │ 0.5天    │ Gemini 305 PID确认，对齐验证           │
  ├────────────────┼──────────┼────────────────────────────────────────┤
  │ YOLOv8检测节点 │ 1-2天    │ Python节点，含调试                     │   
  ├────────────────┼──────────┼────────────────────────────────────────┤   
  │ 手眼标定       │ 3-4天    │ 最耗时：安装依赖、采集数据、求解、验证 │   
  ├────────────────┼──────────┼────────────────────────────────────────┤   
  │ TF发布 +       │ 1天      │ static_transform_publisher + tf2       │
  │ 坐标转换       │          │ lookup                                 │   
  ├────────────────┼──────────┼────────────────────────────────────────┤
  │ 抓取协调节点   │ 2-3天    │ 串联流程 + 参数调试                    │   
  ├────────────────┼──────────┼────────────────────────────────────────┤   
  │ 灵巧手参数调试 │ 1天      │ 找到合适的抓取位置值                   │
  ├────────────────┼──────────┼────────────────────────────────────────┤   
  │ 集成测试 +     │ 2-3天    │ 端到端调试，偏移微调                   │
  │ 调参           │          │                                        │   
  ├────────────────┼──────────┼────────────────────────────────────────┤
  │ 总计           │ ~11-14天 │ 2周勉强可行                            │   
  └────────────────┴──────────┴────────────────────────────────────────┘
                                                                           
  最大风险                                                                 
  
  ┌──────────────────┬──────────┬──────────────────────────────────────┐   
  │       风险       │ 严重程度 │               规避方法               │ 
  ├──────────────────┼──────────┼──────────────────────────────────────┤ 
  │ 手眼标定精度不足 │ ★★★★★    │ 先用手动测量 T_end_camera            │ 
  │                  │          │ 跑通（1小时），再用自动标定精调      │ 
  ├──────────────────┼──────────┼──────────────────────────────────────┤   
  │ Gemini 305       │          │ Gemini 305 最小深度约                │
  │ 近距离深度失效   │ ★★★★     │ 200mm，确保抓取前拍摄距离 >          │   
  │                  │          │ 250mm（先拍照算位置，再下降抓取）    │
  ├──────────────────┼──────────┼──────────────────────────────────────┤   
  │ movel            │ ★★★      │ 限制工作空间范围，避免臂展极限位置   │
  │ IK无解（奇异点） │          │                                      │   
  ├──────────────────┼──────────┼──────────────────────────────────────┤
  │ YOLO误检         │ ★★       │ 加置信度阈值(>0.6) + 面积过滤 +      │   
  │                  │          │ 桌面区域ROI限制                      │   
  ├──────────────────┼──────────┼──────────────────────────────────────┤
  │ 灵巧手抓取力不足 │ ★★       │ 实测调整 position 值，必要时设置     │   
  │                  │          │ max_current                          │   
  └──────────────────┴──────────┴──────────────────────────────────────┘
                                                                           
  2周Demo建议的优先级                                                      
  
  1. 第1天：确认相机出图 + YOLO检测水瓶                                    
  2. 第2-3天：手动测量 T_end_camera（用尺子量），发布 static
  TF，验证坐标转换                                                         
  3. 第4-5天：写抓取协调节点，用固定位置测试 movel + 手开合
  4. 第6-7天：端到端联调                                                   
  5. 第8-10天：自动标定提升精度，调参优化                                  
                                                                           
  ---                                                                      
  总评                                                                     
                  
  ┌────────────┬────────────────────────────────────────────────────────┐
  │    维度    │                          评估                          │  
  ├────────────┼────────────────────────────────────────────────────────┤
  │ 技术可行性 │ 可行 — 所有关键接口已存在                              │  
  ├────────────┼────────────────────────────────────────────────────────┤
  │ 2周Demo    │ 紧但可行 — 前提是先手动标定跑通，不在 MoveIt2 上花时间 │  
  ├────────────┼────────────────────────────────────────────────────────┤  
  │ 最大风险   │ 手眼标定 + Gemini 305 近距离深度盲区                   │  
  ├────────────┼────────────────────────────────────────────────────────┤  
  │ 关键决策   │ 不用 MoveIt2，直接 movel；不用6D姿态，垂直向下抓       │
  └────────────┴────────────────────────────────────────────────────────┘  
                  
  你现有的 system_controller 架构非常好，PoseEstimation.srv                
  接口已经定义好了。最小改动路径是：实现 PoseEstimation 服务 + 在
  system_controller 中加一个 grasp step                                    
  type，这样可以完全复用你的任务编排系统。