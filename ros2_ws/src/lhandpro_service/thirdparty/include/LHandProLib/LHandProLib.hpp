#ifndef LHANDPROLIB_HPP
#define LHANDPROLIB_HPP

/// @brief 定义导出/导入符号
#if defined(_WIN32) || defined(_WIN64)
#ifdef LHandProLib_EXPORTS
#define LHANDPRO_API __declspec(dllexport)
#else
#define LHANDPRO_API __declspec(dllimport)
#endif
#else
#define LHANDPRO_API __attribute__((visibility("default")))
#endif

/// @brief 名字空间
namespace lhplib {
/// @brief 错误码枚举
enum {
  LER_NONE = 0,           ///< 执行成功
  LER_PARAMETER,          ///< 参数错误
  LER_KEY_FUNC_UNINIT,    ///< 关键函数未初始化
  LER_GET_CONFIGURATION,  ///< 读取配置失败
  LER_DATA_ANOMALY,       ///< 数据异常
  LER_COMM_CONNECT,       ///< 通讯连接错误
  LER_COMM_SEND,          ///< 通讯发送错误
  LER_COMM_RECV,          ///< 通讯接收错误
  LER_COMM_DATA_FORMAT,   ///< 通讯数据格式错误
  LER_INVALID_PATH,       ///< 无效的文件路径
  LER_LOG_SAVE_FAIL,      ///< 日志文件保存失败
  LER_NOT_HOME,           ///< 没回零错误
  LER_UNKNOWN = 999,      ///< 未知错误
};

/// @brief 灵巧手类型枚举
enum {
  LAC_DOF_6 = 0,    ///< 6自由度
  LAC_DOF_6_S,      ///< 6自由度(S版本)
  LAC_DOF_15,       ///< 15自由度
  LAC_DOF_6_CUSTOM  ///< 6自由度(自定义)
};

/// @brief 通讯类型枚举
enum {
  LCN_ECAT = 0,  ///< EtherCAT
  LCN_CANFD,     ///< CANFD
  LCN_RS485,     ///< RS485
};

/// @brief 控制模式枚举
enum {
  LCM_POSITION = 0,  ///< 位置控制
  LCM_VELOCITY,      ///< 速度控制
  LCM_TORQUE,        ///< 力矩控制
  LCM_VEL_TOR,       ///< 速度+力矩混合控制
  LCM_POS_TOR,       ///< 位置+力矩混合控制
  LCM_HOME,          ///< 回零
};

/// @brief 力矩到位控制模式
enum {
  LTC_REACHED_HOLD = 0,  ///< 力矩到位后保持
  LTC_REACHED_STOP,      ///< 力矩到位后停止
};


/// @brief 运行状态枚举
enum {
  LST_STOPPED = 0,  ///< 正常停止状态
  LST_RUNNING,      ///< 正常运行状态
  LST_ALARM,        ///< 报警停止状态
  LST_POS_LIMIT,    ///< 正限位状态
  LST_NEG_LIMIT,    ///< 负限位状态
  LST_BOTH_LIMIT,   ///< 正负限位状态
  LST_EMG_STOP,     ///< 急停状态
  LST_HOMING,       ///< 回零运行状态
};

/// @brief 报警类型枚举
enum {
  LAM_NULL = 0,   ///< 无报警
  LAM_POS_ERR,    ///< 位置超差
  LAM_OVER_SPD,   ///< 超速
  LAM_OVER_CUR,   ///< 过流
  LAM_OVER_LOAD,  ///< 过载
  LAM_OVER_VOL,   ///< 过压
  LAM_UNDER_VOL,  ///< 欠压
  LAM_ENC_ERR,    ///< 编码器错误
  LAM_STALL,      ///< 堵转
  LAM_OTHER,      ///< 其他报警
};

/// @brief 传感器id枚举
enum {
  LSS_FINGER_1_1 = 1,  ///< 大拇指指尖
  LSS_FINGER_1_2,      ///< 大拇指指腹
  LSS_FINGER_2_1,      ///< 食指指尖
  LSS_FINGER_2_2,      ///< 食指指腹
  LSS_FINGER_3_1,      ///< 中指指尖
  LSS_FINGER_3_2,      ///< 中指指腹
  LSS_FINGER_4_1,      ///< 无名指指尖
  LSS_FINGER_4_2,      ///< 无名指指腹
  LSS_FINGER_5_1,      ///< 小拇指指尖
  LSS_FINGER_5_2,      ///< 小拇指指腹
  LSS_HAND_PALM,       ///< 手掌
  LSS_MAX_COUNT,       ///< 最大传感器数量
};

/// @brief 左右手枚举
enum {
  LDR_HAND_RIGHT = 0,  ///< 右手
  LDR_HAND_LEFT        ///< 左手
};

/// @brief 函数指针
typedef void (*LogAddCallback)(const char*);
typedef bool (*ECSendDataCallback)(const unsigned char*, unsigned int);
typedef bool (*CANFDSendDataCallback)(const unsigned char*, unsigned int);

#define L_DECLARE_PRIVATE(Class) \
  Class##Private* d_ptr;         \
  friend class Class##Private;

/// @brief 前置声明
class LHandProLibPrivate;

/// @brief LHandProLib类
class LHANDPRO_API LHandProLib {
  L_DECLARE_PRIVATE(LHandProLib)
 public:
  /// @brief 构造函数
  LHandProLib();

  /// @brief 析构函数
  ~LHandProLib();

  /// @brief 初始化灵巧手驱动程序
  /// @param mode 通讯模式 LCN_ECAT/LCN_CANFD/LCN_485
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           [注意] 在调用initial之前, 请先按下方流程进行
  ///           1. 自行初始化通讯对象, 例如EtherCAT
  ///           2. 对通讯对象进行连接
  ///           3. 进行发送数据[回调]函数的处理, 详见set_send_rpdo_callback(ex)
  ///           4. 进行接收数据[回调]函数的处理, 详见set_tpdo_data_decode
  ///           5. 对库进行初始化initial(LCN_ECAT)
  int initial(int mode);

  /// @brief 关闭灵巧手驱动程序
  /// @example
  ///           lhp_lib->close();
  void close();

  /// @brief 发送数据回调函数(EtherCAT)
  /// @param callback 函数指针
  /// @example
  ///           // 1. 定义EtherCAT主站对象
  ///           auto ec_master_ = std::make_shared<EthercatMaster>();
  ///           // 2. 处理EtherCAT发送数据的回调
  ///           lhp_lib_->set_send_rpdo_callback(
  ///               [](const unsigned char* data, unsigned int size) {
  ///                 return ec_master_->setOutputs(data, size);
  ///               });
  void set_send_rpdo_callback(ECSendDataCallback callback);

  /// @brief 发送数据回调函数(EtherCAT)(使用时需使用std::function包装)
  /// @param callback_impl 函数指针(必须保证该 std::function 对象生命周期长于
  /// LHandProLib 实例)
  /// @example
  ///           // 1. 定义EtherCAT主站对象
  ///           auto ec_master_ = std::make_shared<EthercatMaster>();
  ///           // 2. EtherCAT的发送函数
  ///           auto send_func = [ec_master_](const unsigned char* data,
  ///                                    unsigned int size) {
  ///             return ec_master_->setOutputs(data, size);
  ///           };
  ///           // 3. 使用std::function包装(必须保证该 std::function
  ///           对象生命周期长于 LHandProLib 实例) std::function<bool(const
  ///           unsigned char*, unsigned int)> func =
  ///               send_func;
  ///           // 4. 处理EtherCAT发送数据的回调
  ///           lhp_lib_->set_send_rpdo_callback_ex(&func);
  void set_send_rpdo_callback_ex(void* callback_impl);

  /// @brief 获取预发送区的 RPDO
  /// 数据,如设置了set_send_rpdo_callback(ex)回调,则会同时调用发送
  /// @param data_ptr 返回的数据指针（nullptr 时仅获取长度）
  /// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
  /// @return 0 - 成功，其他 - 错误码
  /// @example
  ///           // 1. 第一次调用获取数据长度
  ///           int size = 0;
  ///           lhp_lib_->get_pre_send_rpdo_data(nullptr, &size);
  ///           // 2. 第二次调用获取实际预发送区的字节数组
  ///           std::vector<unsigned char> rpdo_data(size);
  ///           lhp_lib_->get_pre_send_rpdo_data(rpdo_data.data(), &size);
  int get_pre_send_rpdo_data(unsigned char* data_ptr, int* io_size);

  /// @brief 解码传入的 TPDO 数据
  /// @param data_ptr 待解码的数据指针
  /// @param data_size 数据长度
  /// @return 0 成功，其他错误码
  /// @example
  ///           // 1. 使用SDK 对 EtherCAT 获取上来的TPDO数据进行解码
  ///           std::vector<unsigned char> tpdo_data;
  ///           lhp_lib_->set_tpdo_data_decode(tpdo_data.data(),
  ///           tpdo_data.size());
  ///           // 2.
  ///           解码后则可以使用SDK的接口获取目标数据,如get_now_angle,get_finger_pressure等
  int set_tpdo_data_decode(const unsigned char* data_ptr, int data_size);

  /// @brief 发送数据回调函数(CANFD)
  /// @param callback 函数指针
  /// @example
  ///           // 1. 定义CANFD主站对象
  ///           auto canfd_master_ = std::make_shared<CANFDMaster>();
  ///           // 2. 处理CANFD发送数据的回调
  ///           lhp_lib_->set_send_canfd_callback(
  ///               [](const unsigned char* data, unsigned int size) {
  ///                 return canfd_master_->sendData(0x500 + 1, data, size);
  ///               });
  void set_send_canfd_callback(CANFDSendDataCallback callback);

  /// @brief 发送数据回调函数(CANFD)(使用时需使用std::function包装)
  /// @param callback_impl 函数指针(必须保证该 std::function 对象生命周期长于
  /// LHandProLib 实例)
  /// @example
  ///           // 1. 定义CANFD主站对象
  ///           auto canfd_master_ = std::make_shared<CANFDMaster>();
  ///           // 2. CANFD的发送函数
  ///           auto send_func = [canfd_master_](const unsigned char* data,
  ///           unsigned int size) {
  ///             return canfd_master_->sendData(0x500 + 1, data, size);
  ///           };
  ///           // 3. 使用std::function包装(必须保证该 std::function
  ///           对象生命周期长于 LHandProLib 实例) std::function<bool(const
  ///           unsigned char*, unsigned int)> func =
  ///               send_func;
  ///           // 4. 处理CANFD发送数据的回调
  ///           lhp_lib_->set_send_canfd_callback_ex(&func);
  void set_send_canfd_callback_ex(void* callback_impl);

  /// @brief 获取预发送区的 CANFD
  /// 数据,如设置了set_send_canfd_callback(ex)回调,则会同时调用发送
  /// @param data_ptr 返回的数据指针（nullptr 时仅获取长度）
  /// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
  /// @return 0 - 成功，其他 - 错误码
  /// @example
  ///           // 1. 第一次调用获取数据长度
  ///           int size = 0;
  ///           lhp_lib_->get_pre_send_canfd_data(nullptr, &size);
  ///           // 2. 第二次调用获取实际预发送区的字节数组
  ///           std::vector<unsigned char> canfd_data(size);
  ///           lhp_lib_->get_pre_send_canfd_data(canfd_data.data(), &size);
  int get_pre_send_canfd_data(unsigned char* data_ptr, int* io_size);

  /// @brief 解码传入的 CANFD 数据
  /// @param data_ptr 待解码的数据指针
  /// @param data_size 数据长度
  /// @return 0 成功，其他错误码
  /// @example
  ///           // 1. 使用SDK 对 CANFD 获取上来的TPDO数据进行解码
  ///           std::vector<unsigned char> canfd_data;
  ///           lhp_lib_->set_canfd_data_decode(canfd_data.data(),
  ///           canfd_data.size());
  ///           // 2.
  ///           解码后则可以使用SDK的接口获取目标数据,如get_now_angle,get_finger_pressure等
  int set_canfd_data_decode(const unsigned char* data_ptr, int data_size);

  /// @brief 获取当前灵巧手自由度数量
  /// @param total 自由度总数量, 即关节数量
  /// @param active 主动自由度数量, 即电机数量
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int total_dof, active_dof;
  ///           lhp_lib->get_dof(&total_dof, &active_dof);
  int get_dof(int* total, int* active);

  /// @brief 设置灵巧手类型,根据枚举进行设置 默认 LAC_DOF_6
  /// @param hand_type 灵巧手类型
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->set_hand_type(LAC_DOF_6);
  int set_hand_type(int hand_type);

  /// @brief 获取当前灵巧手类型
  /// @param hand_type 灵巧手类型
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int hand_type;
  ///           lhp_lib->get_hand_type(&hand_type);
  int get_hand_type(int* hand_type);

  /// @brief 设置自定义手(LAC_DOF_6_CUSTOM)的手指与轴映射顺序
  ///           按照 大拇指侧摆, 大拇指弯曲, 食指弯曲, 中指弯曲, 无名指弯曲, 小拇指弯曲 的顺序
  /// @param order 指向整数数组的指针
  /// @param size 数组大小，必须为6
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int standard_order[6] = {0, 1, 2, 3, 4, 5};
  ///           lhp_lib->set_finger_order(standard_order, 6);
  int set_finger_order(const int* order, int size);

  /// @brief 设置手的方向(默认即为右手LDR_HAND_RIGHT)
  /// @param dir LDR_HAND_RIGHT/LDR_HAND_LEFT
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->set_hand_direction(LDR_HAND_RIGHT);
  int set_hand_direction(int dir);

  /// @brief 获取手的方向
  /// @param dir LDR_HAND_RIGHT/LDR_HAND_LEFT
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int hand_dir;
  ///           lhp_lib->get_hand_direction(&hand_dir);
  int get_hand_direction(int* dir);

  /// @brief 设置电机控制模式
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///       1~DOF表示单个ID设置
  /// @param mode 控制模式 0:位置控制 1:速度控制 2:力矩控制
  ///       3:速度+力矩混合控制 4:位置+力矩混合控制 5:回零
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->set_control_mode(1, LCM_POSITION);
  int set_control_mode(int id, int mode = LCM_POSITION);

  /// @brief 获取电机控制模式
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param mode 控制模式 0:位置控制 1:速度控制 2:力矩控制
  ///       3:速度+力矩混合控制 4:位置+力矩混合控制 5:回零
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int control_mode;
  ///           lhp_lib->get_control_mode(1, &control_mode);
  int get_control_mode(int id, int* mode);

  /// @brief 设置力矩控制模式
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///       1~DOF表示单个ID设置
  /// @param mode 力矩控制模式 0:力矩到位后保持 1:力矩到位后停止
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->set_torque_control_mode(1, LTC_REACHED_HOLD);
  int set_torque_control_mode(int id, int mode);

  /// @brief 获取力矩控制模式
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param mode 力矩控制模式 0:力矩到位后保持 1:力矩到位后停止
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int torque_mode;
  ///           lhp_lib->get_torque_control_mode(1, &torque_mode);
  int get_torque_control_mode(int id, int* mode);

  /// @brief 设置电机使能/禁止
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///       1~DOF表示单个ID设置
  /// @param enable 使能/禁止 0:禁止 1:使能
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->set_enable(1, 1);  // 使能电机1
  int set_enable(int id, int enable);

  /// @brief 获取电机使能/禁止
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param enable 使能/禁止 0:禁止 1:使能
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int is_enabled;
  ///           lhp_lib->get_enable(1, &is_enabled);
  int get_enable(int id, int* enable);

  /// @brief 获取电机到位信号
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param reached 是否到位, 1:到位 0:运动中
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int is_reached;
  ///           lhp_lib->get_position_reached(1, &is_reached);
  int get_position_reached(int id, int* reached);

  /// @brief 获取力矩到位信号
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param reached 是否到位, 1:到位 0:没到位
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int torque_reached;
  ///           lhp_lib->get_torque_reached(1, &torque_reached);
  int get_torque_reached(int id, int* reached);

  /// @brief 清除电机报警
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///       1~DOF表示单个ID设置
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->set_clear_alarm(1);  // 清除电机1的报警
  ///           lhp_lib->set_clear_alarm();  // 清除所有电机的报警
  int set_clear_alarm(int id = 0);

  /// @brief 获取当前报警状态
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param alarm 报警状态, 参考 LAM_* 报警码枚举
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int alarm_status;
  ///           lhp_lib->get_now_alarm(1, &alarm_status);
  int get_now_alarm(int id, int* alarm);


  /// @brief 是否允许在没回零状态下运动
  /// @param enable 0: 不允许 1: 允许
  /// @return
  int set_move_no_home(int enable = 0);

  /// @brief 启动电机回零
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->home_motors(1);  // 启动电机1回零
  ///           lhp_lib->home_motors(0);  // 启动所有电机回零
  int home_motors(int id);

  /// @brief 获取电机目标角度上下限
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param min_angle 目标角度最小值
  /// @param max_angle 目标角度最大值
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float min_angle, max_angle;
  ///           lhp_lib->get_limit_target_angle(1, &min_angle, &max_angle);
  int get_limit_target_angle(int id, float* min_angle, float* max_angle);

  /// @brief 设置电机目标角度
  /// @param id 电机ID, [0, DOF=自由度数量]
  /// @param angle 目标角度, 范围[0,MAX=电机可达最大角度值],
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 设置电机1的目标角度为45度
  ///           lhp_lib->set_target_angle(1, 45.0f);
  int set_target_angle(int id, float angle);

  /// @brief 获取最近一次设置的电机目标角度
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param angle 目标角度, 范围[0,MAX=电机可达最大角度值]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float target_angle;
  ///           lhp_lib->get_target_angle(1, &target_angle);
  int get_target_angle(int id, float* angle);

  /// @brief 设置电机目标位置（行程当量）
  /// @param id 电机ID, 范围 [0, DOF=自由度数量]
  /// @param position 目标位置, 范围 [0=起始位, 10000=满行程]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 设置电机1的目标位置为5000（中间位置）
  ///           lhp_lib->set_target_position(1, 5000);
  int set_target_position(int id, int position);

  /// @brief 获取最近一次设置的电机目标位置（行程当量）
  /// @param id 电机ID, 范围 [1, DOF=自由度数量]
  /// @param position 目标位置, 范围 [0=起始位, 10000=满行程]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int target_pos;
  ///           lhp_lib->get_target_position(1, &target_pos);
  int get_target_position(int id, int* position);

  /// @brief 设置电机目标速度(默认使用set_angular_velocity)
  /// @param id 电机ID, [0, DOF=自由度数量]
  /// @param velocity 目标速度, 单位: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 设置电机1的目标速度为30度/秒
  ///           lhp_lib->set_velocity(1, 30.0f);
  int set_velocity(int id, float velocity);

  /// @brief 获取最近一次设置的电机目标速度(默认使用get_angular_velocity)
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param velocity 目标速度, 单位: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float velocity;
  ///           lhp_lib->get_velocity(1, &velocity);
  int get_velocity(int id, float* velocity);

  /// @brief 设置电机目标速度
  /// @param id 电机ID, [0, DOF=自由度数量]
  /// @param velocity 目标速度, 单位: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 设置电机1的目标速度为30°/s
  ///           lhp_lib->set_angular_velocity(1, 30.0f);
  int set_angular_velocity(int id, float velocity);

  /// @brief 获取最近一次设置的电机目标速度
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param velocity 目标速度, 单位: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float velocity;
  ///           lhp_lib->get_angular_velocity(1, &velocity);
  int get_angular_velocity(int id, float* velocity);

  /// @brief 设置电机目标速度（行程当量速度）
  /// @param id 电机ID, 范围 [0, DOF=自由度数量]
  /// @param velocity 目标速度, 单位: 当量/秒（即每秒移动的当量数）
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 设置电机1的目标位置速度为1000当量/秒
  ///           lhp_lib->set_position_velocity(1, 1000);
  int set_position_velocity(int id, int velocity);

  /// @brief 获取最近一次设置的电机目标速度（行程当量速度）
  /// @param id 电机ID, 范围 [1, DOF=自由度数量]
  /// @param velocity 输出参数，返回目标位置速度值（单位: 当量/秒）
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int pos_velocity;
  ///           lhp_lib->get_position_velocity(1, &pos_velocity);
  int get_position_velocity(int id, int* velocity);

  /// @brief 设置电机最大电流
  /// @param id 电机ID, [0, DOF=自由度数量]
  /// @param current 最大电流, 单位 ‰(千分比)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 设置电机1的最大电流为500‰
  ///           lhp_lib->set_max_current(1, 500);
  int set_max_current(int id, int current);

  /// @brief 获取最近一次设置的电机最大电流
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param current 最大电流, 单位 ‰(千分比)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int max_current;
  ///           lhp_lib->get_max_current(1, &max_current);
  int get_max_current(int id, int* current);

  /// @brief 驱动电机运动
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///       1~DOF表示单个ID设置
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 驱动电机1运动
  ///           lhp_lib->move_motors(1);
  ///           // 驱动所有电机运动
  ///           lhp_lib->move_motors();
  int move_motors(int id = 0);

  /// @brief 停止电机运动
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///       1~DOF表示单个ID设置
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 停止电机1运动
  ///           lhp_lib->stop_motors(1);
  ///           // 停止所有电机运动
  ///           lhp_lib->stop_motors();
  int stop_motors(int id = 0);

  /// @brief 获取电机当前状态
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param status 运行状态, 0：正常停止状态 1：正常运行状态 2：报警停止状态
  ///       3：正限位状态 4：负限位状态 5：正负限位状态 6：急停状态
  ///       7：回零运行状态
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int motor_status;
  ///           lhp_lib->get_now_status(1, &motor_status);
  int get_now_status(int id, int* status);

  /// @brief 获取电机当前角度位置
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param angle 当前角度位置, 单位: °
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float current_angle;
  ///           lhp_lib->get_now_angle(1, &current_angle);
  int get_now_angle(int id, float* angle);

  /// @brief 获取电机当前行程位置
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param position 当前行程位置, 范围 [0=起始位, 10000=满行程]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int current_position;
  ///           lhp_lib->get_now_position(1, &current_position);
  int get_now_position(int id, int* position);

  /// @brief 获取电机当前速度(单位:度, 默认使用get_now_angular_velocity)
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param velocity 当前速度, 单位: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float current_velocity;
  ///           lhp_lib->get_now_velocity(1, &current_velocity);
  int get_now_velocity(int id, float* velocity);

  /// @brief 获取电机当前速度(单位:度)
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param velocity 当前速度, 单位: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float current_angular_velocity;
  ///           lhp_lib->get_now_angular_velocity(1, &current_angular_velocity);
  int get_now_angular_velocity(int id, float* velocity);

  /// @brief 获取电机当前速度(单位:当量)
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param velocity 当前速度, 单位: 当量/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int current_pos_velocity;
  ///           lhp_lib->get_now_position_velocity(1, &current_pos_velocity);
  int get_now_position_velocity(int id, int* velocity);

  /// @brief 获取电机当前速度
  /// @param id 电机ID, [1, DOF=自由度数量]
  /// @param current 当前电流, 单位 ‰(千分比)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int current_current;
  ///           lhp_lib->get_now_current(1, &current_current);
  int get_now_current(int id, int* current);

  /// @brief 设置是否开启监控传感器
  /// @param enable 是否开启监控传感器
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->set_sensor_enable(1);
  int set_sensor_enable(int enable);

  /// @brief 设置传感器数据格式
  /// @param format 格式类型, 默认为0
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->set_sensor_data_format(0);
  int set_sensor_data_format(int format);

  /// @brief 设置传感器的映射顺序
  ///           按照 大拇指, 食指, 中指, 无名指, 小拇指 的顺序
  /// @param order 指向整数数组的指针
  /// @param size 数组大小，必须为6
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           int standard_order[6] = {0, 1, 2, 3, 4, 5};
  ///           lhp_lib->set_sensor_order(standard_order, 6);
  int set_sensor_order(const int* order, int size);

  /// @brief 获取指定位置的触觉传感器分布位置
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param x_values 返回的X坐标数组指针,位置的范围是[0.0-1.0]
  /// @param y_values 返回的Y坐标数组指针,位置的范围是[0.0-1.0]
  /// @param io_count 返回的数组长度
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 1. 获取count大小
  ///           int count = 0;
  ///           get_finger_sensor_pos(sensor_id, nullptr, nullptr, &count);
  ///           // 2. 获取实际坐标
  ///           std::vector<float> x(count), y(count);
  ///           get_finger_sensor_pos(sensor_id, x.data(), y.data(), &count);
  int get_finger_sensor_pos(int sensor_id, float* x_values, float* y_values,
                            int* io_count);

  /// @brief 获取指定位置的触觉传感器压力值
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param pressure_list 返回的数组指针,力的范围是[0.0-1.0]
  /// @param io_count 返回的数组长度
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///            // 1. 获取count大小
  ///            int count = 0;
  ///            lhp_lib->get_finger_pressure(sensor_id, nullptr, &count);
  ///            // 2. 获取实际压力数组
  ///            std::vector<float> pressures(count);
  ///            lhp_lib->get_finger_pressure(sensor_id, pressures.data(),
  ///            &count);
  int get_finger_pressure(int sensor_id, float* pressure_list, int* io_count);

  /// @brief 设置基准的触觉传感器当前值,即重置所有传感器当前状态为0状态
  ///       get_finger_pressure返回的传感器差值以这个为基准
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->set_finger_pressure_reset();
  int set_finger_pressure_reset();

  /// @brief 获取指定位置的触觉传感器法向力
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param normal_force 返回的法向力,力的范围是[0.0-1.0]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float normal_force;
  ///           lhp_lib->get_finger_normal_force(LSS_FINGER_1_1, &normal_force);
  int get_finger_normal_force(int sensor_id, float* normal_force);

  /// @brief 获取指定位置的触觉传感器法向力数组
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param normal_force_list 返回的法向力数组指针,力的范围是[0.0-1.0]
  /// @param io_count 返回的数组长度
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 1. 获取count大小
  ///           int count = 0;
  ///           get_finger_normal_force_ex(sensor_id, nullptr, &count);
  ///           // 2. 获取实际法向力数组
  ///           std::vector<float> normal_force(count);
  ///           get_finger_normal_force_ex(sensor_id, normal_force.data(),
  ///           &count);
  int get_finger_normal_force_ex(int sensor_id, float* normal_force_list,
                                 int* io_count);

  /// @brief 获取指定位置的触觉传感器切向力
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param tangential_force 返回的切向力,力的范围是[0.0-1.0]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float tangential_force;
  ///           lhp_lib->get_finger_tangential_force(LSS_FINGER_1_1,
  ///           &tangential_force);
  int get_finger_tangential_force(int sensor_id, float* tangential_force);

  /// @brief 获取指定位置的触觉传感器切向力数组
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param tangential_force_list 返回的切向力数组指针,力的范围是[0.0-1.0]
  /// @param io_count 返回的数组长度
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 1. 获取count大小
  ///           int count = 0;
  ///           get_finger_tangential_force_ex(sensor_id, nullptr, &count);
  ///           // 2. 获取实际切向力数组
  ///           std::vector<float> tangential_force(count);
  ///           get_finger_tangential_force_ex(sensor_id,
  ///           tangential_force.data(), &count);
  int get_finger_tangential_force_ex(int sensor_id,
                                     float* tangential_force_list,
                                     int* io_count);

  /// @brief 获取指定位置的触觉传感器的受力方向
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param force_direction 返回的受力方向,方向的范围是[0-360]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float force_direction;
  ///           lhp_lib->get_finger_force_direction(LSS_FINGER_1_1,
  ///           &force_direction);
  int get_finger_force_direction(int sensor_id, float* force_direction);

  /// @brief 获取指定位置的触觉传感器受力方向数组
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param force_direction_list 返回的受力方向数组指针,方向的范围是[0-360]
  /// @param io_count 返回的数组长度
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 1. 获取count大小
  ///           int count = 0;
  ///           get_finger_force_direction_ex(sensor_id, nullptr, &count);
  ///           // 2. 获取实际受力方向数组
  ///           std::vector<float> force_direction(count);
  ///           get_finger_force_direction_ex(sensor_id, force_direction.data(),
  ///           &count);
  int get_finger_force_direction_ex(int sensor_id, float* force_direction_list,
                                    int* io_count);

  /// @brief 获取指定位置的触觉传感器的接近程度
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param proximity 返回的接近程度
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           float proximity;
  ///           lhp_lib->get_finger_proximity(LSS_FINGER_1_1, &proximity);
  int get_finger_proximity(int sensor_id, float* proximity);

  /// @brief 获取指定位置的触觉传感器接近程度数组
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  /// @param proximity_list 返回的接近程度数组指针
  /// @param io_count 返回的数组长度
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 1. 获取count大小
  ///           int count = 0;
  ///           get_finger_proximity_ex(sensor_id, nullptr, &count);
  ///           // 2. 获取实际接近程度数组
  ///           std::vector<float> proximity(count);
  ///           get_finger_proximity_ex(sensor_id, proximity.data(), &count);
  int get_finger_proximity_ex(int sensor_id, float* proximity_list,
                              int* io_count);

  /// @brief 开启/关闭日志打印
  /// @param on 是否开启日志打印, true开启, false关闭(默认)
  /// @param maxsize 最大保存日志数量, 防止内存无限增长
  /// @example
  ///           // 开启日志打印，最大保存10000条日志
  ///           lhp_lib->log_on(true, 10000);
  void log_on(bool on = false, int maxsize = 10000);

  /// @brief 设置需要打印的发送数据地址数组
  /// @param cmd 需要打印的地址数组, 空表示全打印
  /// @param count 指令数组长度
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 设置需要打印的发送数据地址
  ///           int send_cmds[] = {0x00, 0x01, 0x0B};
  ///           lhp_lib->log_send(send_cmds, 3);
  int log_send(int* cmd, int count);

  /// @brief 设置需要打印的接收数据地址数组
  /// @param cmd 需要打印的地址数组, 空表示全打印
  /// @param count 指令数组长度
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           // 设置需要打印的接收数据地址
  ///           int recv_cmds[] = {0x00, 0x40};
  ///           lhp_lib->log_recv(recv_cmds, 2);
  int log_recv(int* cmd, int count);

  /// @brief 清空日志地址数组
  /// @param send 是否清空发送日志地址数组
  /// @param recv 是否清空接收日志地址数组
  /// @example
  ///           // 清空发送和接收日志地址数组
  ///           lhp_lib->log_reset();
  ///           // 仅清空发送日志地址数组
  ///           lhp_lib->log_reset(true, false);
  void log_reset(bool send = true, bool recv = true);

  /// @brief 将日志保存在文件中
  /// @param file_name - 文件地址
  /// @return 0 - 执行成功, 其他 - 参考错误码
  /// @example
  ///           lhp_lib->log_save("hand_log.txt");
  int log_save(const char* file_name);

  /// @brief 清空当前日志记录
  /// @example
  ///           lhp_lib->log_clear();
  void log_clear();

  /// @brief 日志回调函数
  /// @example
  ///           // 定义日志回调函数
  ///           void log_callback(const char* log) {
  ///               std::cout << log << std::endl;
  ///           }
  ///           // 设置日志回调
  ///           lhp_lib->set_log_callback(log_callback);
  void set_log_callback(LogAddCallback callback);
};
}  // namespace lhplib

#endif  // LHANDPROLIB_HPP