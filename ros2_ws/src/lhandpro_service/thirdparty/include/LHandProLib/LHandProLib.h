#ifndef LHANDPROLIB_H
#define LHANDPROLIB_H

#ifdef __cplusplus
extern "C" {
#endif

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

#include <stdbool.h>
// 错误码
#define C_LER_NONE 0               ///< 执行成功
#define C_LER_PARAMETER 1          ///< 参数错误
#define C_LER_KEY_FUNC_UNINIT 2    ///< 关键函数未初始化
#define C_LER_GET_CONFIGURATION 3  ///< 读取配置失败
#define C_LER_DATA_ANOMALY 4       ///< 数据异常
#define C_LER_COMM_CONNECT 5       ///< 通讯连接错误
#define C_LER_COMM_SEND 6          ///< 通讯发送错误
#define C_LER_COMM_RECV 7          ///< 通讯接收错误
#define C_LER_COMM_DATA_FORMAT 8   ///< 通讯数据格式错误
#define C_LER_INVALID_PATH 9       ///< 无效的文件路径
#define C_LER_LOG_SAVE_FAIL 10     ///< 日志文件保存失败
#define C_LER_NOT_HOME 11          ///< 没回零错误
#define C_LER_UNKNOWN 999          ///< 未知错误

// 自由度枚举
#define C_LAC_DOF_6 0       ///< 6自由度
#define C_LAC_DOF_6_S 1     ///< 6自由度(S版本)
#define C_LAC_DOF_15 2      ///< 15自由度
#define C_LAC_DOF_6_CUSTOM  ///< 6自由度(自定义)

// 通讯类型枚举
#define C_LCN_ECAT 0   ///< EtherCAT
#define C_LCN_CANFD 1  ///< CANFD
#define C_LCN_RS485 2  ///< RS485

// 控制模式枚举
#define C_LCM_POSITION 0  ///< 位置控制
#define C_LCM_VELOCITY 1  ///< 速度控制
#define C_LCM_TORQUE 2    ///< 力矩控制
#define C_LCM_VEL_TOR 3   ///< 速度+力矩混合控制
#define C_LCM_POS_TOR 4   ///< 位置+力矩混合控制
#define C_LCM_HOME 5      ///< 回零

// 力矩到位控制模式
#define C_LTC_REACHED_HOLD 0  ///< 力矩到位后保持
#define C_LTC_REACHED_STOP 1  ///< 力矩到位后停止

// 运行状态枚举
#define C_LST_STOPPED 0     ///< 正常停止状态
#define C_LST_RUNNING 1     ///< 正常运行状态
#define C_LST_ALARM 2       ///< 报警停止状态
#define C_LST_POS_LIMIT 3   ///< 正限位状态
#define C_LST_NEG_LIMIT 4   ///< 负限位状态
#define C_LST_BOTH_LIMIT 5  ///< 正负限位状态
#define C_LST_EMG_STOP 6    ///< 急停状态
#define C_LST_HOMING 7      ///< 回零运行状态

// 报警类型枚举
#define C_LAM_NULL 0       ///< 无报警
#define C_LAM_POS_ERR 1    ///< 位置超差
#define C_LAM_OVER_SPD 2   ///< 超速
#define C_LAM_OVER_CUR 3   ///< 过流
#define C_LAM_OVER_LOAD 4  ///< 过载
#define C_LAM_OVER_VOL 5   ///< 过压
#define C_LAM_UNDER_VOL 6  ///< 欠压
#define C_LAM_ENC_ERR 7    ///< 编码器错误
#define C_LAM_STALL 8      ///< 堵转
#define C_LAM_OTHER 9      ///< 其他报警

// 传感器id枚举
#define C_LSS_FINGER_1_1 1   ///< 大拇指指尖
#define C_LSS_FINGER_1_2 2   ///< 大拇指指腹
#define C_LSS_FINGER_2_1 3   ///< 食指指尖
#define C_LSS_FINGER_2_2 4   ///< 食指指腹
#define C_LSS_FINGER_3_1 5   ///< 中指指尖
#define C_LSS_FINGER_3_2 6   ///< 中指指腹
#define C_LSS_FINGER_4_1 7   ///< 无名指指尖
#define C_LSS_FINGER_4_2 8   ///< 无名指指腹
#define C_LSS_FINGER_5_1 9   ///< 小拇指指尖
#define C_LSS_FINGER_5_2 10  ///< 小拇指指腹
#define C_LSS_HAND_PALM 11   ///< 手掌
#define C_LSS_MAX_COUNT 12   ///< 最大传感器数量

// 左右手枚举
#define C_LDR_HAND_RIGHT 0  ///< 右手
#define C_LDR_HAND_LEFT 1   ///< 左手

// 基本类型定义
typedef void* lhandprolib_handle;

// 回调函数类型定义
typedef void (*LogAddCallbackWrapper)(const char*);
typedef bool (*ECSendDataCallbackWrapper)(const unsigned char*, unsigned int);
typedef bool (*CANFDSendDataCallbackWrapper)(const unsigned char*,
                                             unsigned int);

// 创建和销毁
/// @brief 创建LHandProLib实例
/// @return 成功返回实例句柄，失败返回NULL
/// @example
///           lhandprolib_handle handle = lhandprolib_create();
LHANDPRO_API lhandprolib_handle lhandprolib_create();

/// @brief 销毁LHandProLib实例
/// @param handle 实例句柄
/// @example
///           lhandprolib_destroy(handle);
LHANDPRO_API void lhandprolib_destroy(lhandprolib_handle handle);

// 初始化和关闭
/// @brief 初始化灵巧手驱动程序
/// @param handle 实例句柄
/// @param mode 通讯模式 C_LCN_ECAT/C_LCN_CANFD/C_LCN_RS485
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 1. 创建实例
///           lhandprolib_handle handle = lhandprolib_create();
///           // 2. 初始化（使用EtherCAT模式）
///           int result = lhandprolib_initial(handle, C_LCN_ECAT);
LHANDPRO_API int lhandprolib_initial(lhandprolib_handle handle, int mode);

/// @brief 关闭灵巧手驱动程序
/// @param handle 实例句柄
/// @example
///           lhandprolib_close(handle);
LHANDPRO_API void lhandprolib_close(lhandprolib_handle handle);

// 回调设置
/// @brief 设置EtherCAT发送数据回调函数
/// @param handle 实例句柄
/// @param callback 回调函数指针，用于发送RPDO数据
/// @example
///           // 定义回调函数
///           bool send_rpdo_callback(const unsigned char* data, unsigned int size) {
///               // 自行实现EtherCAT数据发送逻辑
///               return true; // 发送成功返回true
///           }
///           // 设置回调
///           lhandprolib_set_send_rpdo_callback(handle, send_rpdo_callback);
LHANDPRO_API void lhandprolib_set_send_rpdo_callback(
    lhandprolib_handle handle, ECSendDataCallbackWrapper callback);

/// @brief 设置CANFD发送数据回调函数
/// @param handle 实例句柄
/// @param callback 回调函数指针，用于发送CANFD数据
/// @example
///           // 定义回调函数
///           bool send_canfd_callback(const unsigned char* data, unsigned int size) {
///               // 自行实现CANFD数据发送逻辑
///               return true; // 发送成功返回true
///           }
///           // 设置回调
///           lhandprolib_set_send_canfd_callback(handle, send_canfd_callback);
LHANDPRO_API void lhandprolib_set_send_canfd_callback(
    lhandprolib_handle handle, CANFDSendDataCallbackWrapper callback);

/// @brief 设置日志回调函数
/// @param handle 实例句柄
/// @param callback 回调函数指针，用于输出日志
/// @example
///           // 定义回调函数
///           void log_callback(const char* log) {
///               printf("%s\n", log);
///           }
///           // 设置回调
///           lhandprolib_set_log_callback(handle, log_callback);
LHANDPRO_API void lhandprolib_set_log_callback(lhandprolib_handle handle,
                                               LogAddCallbackWrapper callback);

// 数据接收处理
/// @brief 解码传入的TPDO数据（EtherCAT）
/// @param handle 实例句柄
/// @param data_ptr 待解码的数据指针
/// @param data_size 数据长度
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 假设已经从EtherCAT获取到TPDO数据
///           unsigned char tpdo_data[128];
///           int tpdo_size = 128;
///           // 解码数据
///           lhandprolib_set_tpdo_data_decode(handle, tpdo_data, tpdo_size);
///           // 解码后可以使用其他接口获取数据
LHANDPRO_API int lhandprolib_set_tpdo_data_decode(lhandprolib_handle handle,
                                                  const unsigned char* data_ptr,
                                                  int data_size);

/// @brief 解码传入的CANFD数据
/// @param handle 实例句柄
/// @param data_ptr 待解码的数据指针
/// @param data_size 数据长度
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 假设已经从CANFD获取到数据
///           unsigned char canfd_data[64];
///           int canfd_size = 64;
///           // 解码数据
///           lhandprolib_set_canfd_data_decode(handle, canfd_data, canfd_size);
///           // 解码后可以使用其他接口获取数据
LHANDPRO_API int lhandprolib_set_canfd_data_decode(
    lhandprolib_handle handle, const unsigned char* data_ptr, int data_size);

// RPDO数据处理
/// @brief 获取预发送区的RPDO数据（EtherCAT）
/// @param handle 实例句柄
/// @param data_ptr 返回的数据指针（nullptr时仅获取长度）
/// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 1. 第一次调用获取数据长度
///           int size = 0;
///           lhandprolib_get_pre_send_rpdo_data(handle, nullptr, &size);
///           // 2. 第二次调用获取实际数据
///           unsigned char* rpdo_data = (unsigned char*)malloc(size);
///           lhandprolib_get_pre_send_rpdo_data(handle, rpdo_data, &size);
///           // 3. 使用数据后释放内存
///           free(rpdo_data);
LHANDPRO_API int lhandprolib_get_pre_send_rpdo_data(lhandprolib_handle handle,
                                                    unsigned char* data_ptr,
                                                    int* io_size);

/// @brief 获取预发送区的CANFD数据
/// @param handle 实例句柄
/// @param data_ptr 返回的数据指针（nullptr时仅获取长度）
/// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 1. 第一次调用获取数据长度
///           int size = 0;
///           lhandprolib_get_pre_send_canfd_data(handle, nullptr, &size);
///           // 2. 第二次调用获取实际数据
///           unsigned char* canfd_data = (unsigned char*)malloc(size);
///           lhandprolib_get_pre_send_canfd_data(handle, canfd_data, &size);
///           // 3. 使用数据后释放内存
///           free(canfd_data);
LHANDPRO_API int lhandprolib_get_pre_send_canfd_data(lhandprolib_handle handle,
                                                     unsigned char* data_ptr,
                                                     int* io_size);

/// @brief 获取当前灵巧手自由度数量
/// @param handle 实例句柄
/// @param total 输出参数，自由度总数量（关节数量）
/// @param active 输出参数，主动自由度数量（电机数量）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int total_dof, active_dof;
///           lhandprolib_get_dof(handle, &total_dof, &active_dof);
LHANDPRO_API int lhandprolib_get_dof(lhandprolib_handle handle, int* total,
                                     int* active);

/// @brief 设置手的类型
/// @param handle 实例句柄
/// @param hand_type 手的类型 C_LAC_DOF_6/C_LAC_DOF_6_S/C_LAC_DOF_15
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           lhandprolib_set_hand_type(handle, C_LAC_DOF_6);
LHANDPRO_API int lhandprolib_set_hand_type(lhandprolib_handle handle,
                                           int hand_type);

/// @brief 获取手的类型
/// @param handle 实例句柄
/// @param hand_type 输出参数，手的类型 C_LAC_DOF_6/C_LAC_DOF_6_S/C_LAC_DOF_15
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int hand_type;
///           lhandprolib_get_hand_type(handle, &hand_type);
LHANDPRO_API int lhandprolib_get_hand_type(lhandprolib_handle handle,
                                           int* hand_type);

/// @brief 设置自定义手(LAC_DOF_6_CUSTOM)的手指与轴映射顺序
///           按照 大拇指侧摆, 大拇指弯曲, 食指弯曲, 中指弯曲, 无名指弯曲, 小拇指弯曲 的顺序
/// @param handle 实例句柄
/// @param order 指向整数数组的指针
/// @param size 数组大小，必须为6
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int standard_order[6] = {0, 1, 2, 3, 4, 5};
///           lhandprolib_set_finger_order(handle, standard_order, 6);
LHANDPRO_API int lhandprolib_set_finger_order(lhandprolib_handle handle,
                                              const int* order, int size);

/// @brief 设置手的方向
/// @param handle 实例句柄
/// @param dir 手的方向 C_LDR_HAND_RIGHT/C_LDR_HAND_LEFT
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           lhandprolib_set_hand_direction(handle, C_LDR_HAND_RIGHT);
LHANDPRO_API int lhandprolib_set_hand_direction(lhandprolib_handle handle,
                                                int dir);

/// @brief 获取手的方向
/// @param handle 实例句柄
/// @param dir 输出参数，手的方向 C_LDR_HAND_RIGHT/C_LDR_HAND_LEFT
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int hand_dir;
///           lhandprolib_get_hand_direction(handle, &hand_dir);
LHANDPRO_API int lhandprolib_get_hand_direction(lhandprolib_handle handle,
                                                int* dir);

// 电机控制
/// @brief 设置电机控制模式
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @param mode 控制模式 C_LCM_POSITION/C_LCM_VELOCITY/C_LCM_TORQUE等
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 设置电机1为位置控制模式
///           lhandprolib_set_control_mode(handle, 1, C_LCM_POSITION);
LHANDPRO_API int lhandprolib_set_control_mode(lhandprolib_handle handle, int id,
                                              int mode);

/// @brief 获取电机控制模式
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param mode 输出参数，控制模式
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int control_mode;
///           lhandprolib_get_control_mode(handle, 1, &control_mode);
LHANDPRO_API int lhandprolib_get_control_mode(lhandprolib_handle handle, int id,
                                              int* mode);

/// @brief 设置力矩控制模式
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @param mode 力矩控制模式 C_LTC_REACHED_HOLD/C_LTC_REACHED_STOP
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 设置电机1力矩到位后停止
///           lhandprolib_set_torque_control_mode(handle, 1, C_LTC_REACHED_STOP);
LHANDPRO_API int lhandprolib_set_torque_control_mode(lhandprolib_handle handle,
                                                     int id, int mode);

/// @brief 获取力矩控制模式
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param mode 输出参数，力矩控制模式
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int torque_mode;
///           lhandprolib_get_torque_control_mode(handle, 1, &torque_mode);
LHANDPRO_API int lhandprolib_get_torque_control_mode(lhandprolib_handle handle,
                                                     int id, int* mode);

/// @brief 设置电机使能状态
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @param enable 使能状态，1-使能，0-失能
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 使能电机1
///           lhandprolib_set_enable(handle, 1, 1);
LHANDPRO_API int lhandprolib_set_enable(lhandprolib_handle handle, int id,
                                        int enable);

/// @brief 获取电机使能状态
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param enable 输出参数，使能状态，1-使能，0-失能
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int enable_status;
///           lhandprolib_get_enable(handle, 1, &enable_status);
LHANDPRO_API int lhandprolib_get_enable(lhandprolib_handle handle, int id,
                                        int* enable);

/// @brief 获取位置到位状态
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param reached 输出参数，到位状态，1-到位，0-未到位
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int pos_reached;
///           lhandprolib_get_position_reached(handle, 1, &pos_reached);
LHANDPRO_API int lhandprolib_get_position_reached(lhandprolib_handle handle,
                                                  int id, int* reached);

/// @brief 获取力矩到位状态
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param reached 输出参数，到位状态，1-到位，0-未到位
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int torque_reached;
///           lhandprolib_get_torque_reached(handle, 1, &torque_reached);
LHANDPRO_API int lhandprolib_get_torque_reached(lhandprolib_handle handle,
                                                int id, int* reached);

/// @brief 清除电机报警
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 清除电机1的报警
///           lhandprolib_set_clear_alarm(handle, 1);
LHANDPRO_API int lhandprolib_set_clear_alarm(lhandprolib_handle handle, int id);

/// @brief 获取电机当前报警状态
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param alarm 输出参数，报警状态，0-无报警
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int alarm_status;
///           lhandprolib_get_now_alarm(handle, 1, &alarm_status);
LHANDPRO_API int lhandprolib_get_now_alarm(lhandprolib_handle handle, int id,
                                           int* alarm);

/// @brief 是否允许在没回零状态下运动
/// @param handle 实例句柄
/// @param enable 0: 不允许 1: 允许
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           lhandprolib_set_move_no_home(handle, 0);
LHANDPRO_API int lhandprolib_set_move_no_home(lhandprolib_handle handle,
                                              int enable);

/// @brief 电机回零
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 电机1回零
///           lhandprolib_home_motors(handle, 1);
LHANDPRO_API int lhandprolib_home_motors(lhandprolib_handle handle, int id);

// 目标设置
/// @brief 设置目标角度
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @param angle 目标角度（单位：度）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 设置电机1目标角度为90度
///           lhandprolib_set_target_angle(handle, 1, 90.0f);
LHANDPRO_API int lhandprolib_set_target_angle(lhandprolib_handle handle, int id,
                                              float angle);

/// @brief 获取目标角度
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param angle 输出参数，目标角度（单位：度）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           float target_angle;
///           lhandprolib_get_target_angle(handle, 1, &target_angle);
LHANDPRO_API int lhandprolib_get_target_angle(lhandprolib_handle handle, int id,
                                              float* angle);

/// @brief 设置目标位置
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @param position 目标位置（编码器计数）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 设置电机1目标位置为10000
///           lhandprolib_set_target_position(handle, 1, 10000);
LHANDPRO_API int lhandprolib_set_target_position(lhandprolib_handle handle,
                                                 int id, int position);

/// @brief 获取目标位置
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param position 输出参数，目标位置（编码器计数）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int target_position;
///           lhandprolib_get_target_position(handle, 1, &target_position);
LHANDPRO_API int lhandprolib_get_target_position(lhandprolib_handle handle,
                                                 int id, int* position);

/// @brief 设置目标角速度
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @param velocity 目标角速度（单位：度/秒）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 设置电机1目标角速度为30度/秒
///           lhandprolib_set_angular_velocity(handle, 1, 30.0f);
LHANDPRO_API int lhandprolib_set_angular_velocity(lhandprolib_handle handle,
                                                  int id, float velocity);

/// @brief 获取目标角速度
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param velocity 输出参数，目标角速度（单位：度/秒）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           float target_velocity;
///           lhandprolib_get_angular_velocity(handle, 1, &target_velocity);
LHANDPRO_API int lhandprolib_get_angular_velocity(lhandprolib_handle handle,
                                                  int id, float* velocity);

/// @brief 设置目标位置速度
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @param velocity 目标位置速度（编码器计数/秒）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 设置电机1目标位置速度为1000
///           lhandprolib_set_position_velocity(handle, 1, 1000);
LHANDPRO_API int lhandprolib_set_position_velocity(lhandprolib_handle handle,
                                                   int id, int velocity);

/// @brief 获取目标位置速度
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param velocity 输出参数，目标位置速度（编码器计数/秒）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int target_pos_velocity;
///           lhandprolib_get_position_velocity(handle, 1, &target_pos_velocity);
LHANDPRO_API int lhandprolib_get_position_velocity(lhandprolib_handle handle,
                                                   int id, int* velocity);

/// @brief 设置最大电流
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @param current 最大电流（mA）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 设置电机1最大电流为1000mA
///           lhandprolib_set_max_current(handle, 1, 1000);
LHANDPRO_API int lhandprolib_set_max_current(lhandprolib_handle handle, int id,
                                             int current);

/// @brief 获取最大电流
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param current 输出参数，最大电流（mA）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int max_current;
///           lhandprolib_get_max_current(handle, 1, &max_current);
LHANDPRO_API int lhandprolib_get_max_current(lhandprolib_handle handle, int id,
                                             int* current);

// 运动控制
/// @brief 电机运动
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 电机1开始运动
///           lhandprolib_move_motors(handle, 1);
LHANDPRO_API int lhandprolib_move_motors(lhandprolib_handle handle, int id);

/// @brief 电机停止
/// @param handle 实例句柄
/// @param id 电机ID, [0, DOF]，0表示广播
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 电机1停止运动
///           lhandprolib_stop_motors(handle, 1);
LHANDPRO_API int lhandprolib_stop_motors(lhandprolib_handle handle, int id);

// 状态获取
/// @brief 获取电机当前状态
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param status 输出参数，运行状态 C_LST_STOPPED/C_LST_RUNNING等
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int motor_status;
///           lhandprolib_get_now_status(handle, 1, &motor_status);
LHANDPRO_API int lhandprolib_get_now_status(lhandprolib_handle handle, int id,
                                            int* status);

/// @brief 获取电机当前角度
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param angle 输出参数，当前角度（单位：度）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           float current_angle;
///           lhandprolib_get_now_angle(handle, 1, &current_angle);
LHANDPRO_API int lhandprolib_get_now_angle(lhandprolib_handle handle, int id,
                                           float* angle);

/// @brief 获取电机当前位置
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param position 输出参数，当前位置（编码器计数）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int current_position;
///           lhandprolib_get_now_position(handle, 1, &current_position);
LHANDPRO_API int lhandprolib_get_now_position(lhandprolib_handle handle, int id,
                                              int* position);

/// @brief 获取电机当前角速度
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param velocity 输出参数，当前角速度（单位：度/秒）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           float current_velocity;
///           lhandprolib_get_now_angular_velocity(handle, 1, &current_velocity);
LHANDPRO_API int lhandprolib_get_now_angular_velocity(lhandprolib_handle handle,
                                                      int id, float* velocity);

/// @brief 获取电机当前位置速度
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param velocity 输出参数，当前位置速度（编码器计数/秒）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int current_pos_velocity;
///           lhandprolib_get_now_position_velocity(handle, 1, &current_pos_velocity);
LHANDPRO_API int lhandprolib_get_now_position_velocity(
    lhandprolib_handle handle, int id, int* velocity);

/// @brief 获取电机当前电流
/// @param handle 实例句柄
/// @param id 电机ID, [1, DOF]
/// @param current 输出参数，当前电流（mA）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int current_current;
///           lhandprolib_get_now_current(handle, 1, &current_current);
LHANDPRO_API int lhandprolib_get_now_current(lhandprolib_handle handle, int id,
                                             int* current);

// 触觉传感器

/// @brief 设置触觉传感器接收功能
/// @param handle 实例句柄
/// @param enable 输入参数，是否启用传感器接收功能
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           lhandprolib_set_sensor_enable(handle, 1);
LHANDPRO_API int lhandprolib_set_sensor_enable(lhandprolib_handle handle,
                                               int enable);
/// @brief 设置传感器数据格式
/// @param handle 实例句柄
/// @param format 格式类型, 默认为0
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           lhandprolib_set_sensor_data_format(handle, 0);
LHANDPRO_API int lhandprolib_set_sensor_data_format(lhandprolib_handle handle,
                                                    int format);
/// @brief 设置传感器的映射顺序
///           按照 大拇指, 食指, 中指, 无名指, 小拇指 的顺序
/// @param handle 实例句柄
/// @param order 指向整数数组的指针
/// @param size 数组大小，必须为6
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           int standard_order[6] = {0, 1, 2, 3, 4, 5};
///           lhandprolib_set_sensor_order(standard_order, 6);
LHANDPRO_API int lhandprolib_set_sensor_order(lhandprolib_handle handle,
                                              const int* order, int size);

/// @brief 获取触觉传感器位置坐标
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param x_values 输出参数，X坐标数组指针（需用户手动free释放）
/// @param y_values 输出参数，Y坐标数组指针（需用户手动free释放）
/// @param count 输出参数，坐标点数量
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 1. 获取传感器1的位置坐标
///           float* x_values = NULL;
///           float* y_values = NULL;
///           int count = 0;
///           lhandprolib_get_finger_sensor_pos(handle, C_LSS_FINGER_1_1, &x_values, &y_values, &count);
///           // 2. 使用坐标数据
///           for (int i = 0; i < count; i++) {
///               printf("Point %d: (%.2f, %.2f)\n", i, x_values[i], y_values[i]);
///           }
///           // 3. 释放内存
///           free(x_values);
///           free(y_values);
LHANDPRO_API int lhandprolib_get_finger_sensor_pos(lhandprolib_handle handle,
                                                   int sensor_id,
                                                   float** x_values,
                                                   float** y_values,
                                                   int* count);

/// @brief 获取触觉传感器压力值
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param pressure_list 输出参数，压力值数组指针（需用户手动free释放）
/// @param count 输出参数，压力值数量
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 1. 获取传感器1的压力值
///           float* pressures = NULL;
///           int count = 0;
///           lhandprolib_get_finger_pressure(handle, C_LSS_FINGER_1_1, &pressures, &count);
///           // 2. 使用压力数据
///           for (int i = 0; i < count; i++) {
///               printf("Pressure %d: %.2f\n", i, pressures[i]);
///           }
///           // 3. 释放内存
///           free(pressures);
LHANDPRO_API int lhandprolib_get_finger_pressure(lhandprolib_handle handle,
                                                 int sensor_id,
                                                 float** pressure_list,
                                                 int* count);

/// @brief 重置触觉传感器压力值
/// @param handle 实例句柄
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           lhandprolib_set_finger_pressure_reset(handle);
LHANDPRO_API int lhandprolib_set_finger_pressure_reset(
    lhandprolib_handle handle);

/// @brief 获取触觉传感器法向力
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param normal_force 输出参数，法向力值（范围0.0-1.0）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           float normal_force;
///           lhandprolib_get_finger_normal_force(handle, C_LSS_FINGER_1_1, &normal_force);
LHANDPRO_API int lhandprolib_get_finger_normal_force(lhandprolib_handle handle,
                                                     int sensor_id,
                                                     float* normal_force);

/// @brief 获取触觉传感器法向力数组
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param normal_force_list 输出参数，法向力数组指针（需用户手动free释放）
/// @param count 输出参数，数组长度
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 1. 获取传感器1的法向力数组
///           float* normal_forces = NULL;
///           int count = 0;
///           lhandprolib_get_finger_normal_force_ex(handle, C_LSS_FINGER_1_1, &normal_forces, &count);
///           // 2. 使用法向力数据
///           for (int i = 0; i < count; i++) {
///               printf("Normal force %d: %.2f\n", i, normal_forces[i]);
///           }
///           // 3. 释放内存
///           free(normal_forces);
LHANDPRO_API int lhandprolib_get_finger_normal_force_ex(
    lhandprolib_handle handle, int sensor_id, float** normal_force_list,
    int* count);

/// @brief 获取触觉传感器切向力
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param tangential_force 输出参数，切向力值
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           float tangential_force;
///           lhandprolib_get_finger_tangential_force(handle, C_LSS_FINGER_1_1, &tangential_force);
LHANDPRO_API int lhandprolib_get_finger_tangential_force(
    lhandprolib_handle handle, int sensor_id, float* tangential_force);

/// @brief 获取触觉传感器切向力数组
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param tangential_force_list 输出参数，切向力数组指针（需用户手动free释放）
/// @param count 输出参数，数组长度
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 1. 获取传感器1的切向力数组
///           float* tangential_forces = NULL;
///           int count = 0;
///           lhandprolib_get_finger_tangential_force_ex(handle, C_LSS_FINGER_1_1, &tangential_forces, &count);
///           // 2. 使用切向力数据
///           for (int i = 0; i < count; i++) {
///               printf("Tangential force %d: %.2f\n", i, tangential_forces[i]);
///           }
///           // 3. 释放内存
///           free(tangential_forces);
LHANDPRO_API int lhandprolib_get_finger_tangential_force_ex(
    lhandprolib_handle handle, int sensor_id, float** tangential_force_list,
    int* count);

/// @brief 获取触觉传感器力方向
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param force_direction 输出参数，力方向（角度）
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           float force_dir;
///           lhandprolib_get_finger_force_direction(handle, C_LSS_FINGER_1_1, &force_dir);
LHANDPRO_API int lhandprolib_get_finger_force_direction(
    lhandprolib_handle handle, int sensor_id, float* force_direction);

/// @brief 获取触觉传感器力方向数组
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param force_direction_list 输出参数，力方向数组指针（需用户手动free释放）
/// @param count 输出参数，数组长度
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 1. 获取传感器1的力方向数组
///           float* force_directions = NULL;
///           int count = 0;
///           lhandprolib_get_finger_force_direction_ex(handle, C_LSS_FINGER_1_1, &force_directions, &count);
///           // 2. 使用力方向数据
///           for (int i = 0; i < count; i++) {
///               printf("Force direction %d: %.2f\n", i, force_directions[i]);
///           }
///           // 3. 释放内存
///           free(force_directions);
LHANDPRO_API int lhandprolib_get_finger_force_direction_ex(
    lhandprolib_handle handle, int sensor_id, float** force_direction_list,
    int* count);

/// @brief 获取触觉传感器接近度
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param proximity 输出参数，接近度值
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           float proximity;
///           lhandprolib_get_finger_proximity(handle, C_LSS_FINGER_1_1, &proximity);
LHANDPRO_API int lhandprolib_get_finger_proximity(lhandprolib_handle handle,
                                                  int sensor_id,
                                                  float* proximity);

/// @brief 获取触觉传感器接近度数组
/// @param handle 实例句柄
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
/// @param proximity_list 输出参数，接近度数组指针（需用户手动free释放）
/// @param count 输出参数，数组长度
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           // 1. 获取传感器1的接近度数组
///           float* proximities = NULL;
///           int count = 0;
///           lhandprolib_get_finger_proximity_ex(handle, C_LSS_FINGER_1_1, &proximities, &count);
///           // 2. 使用接近度数据
///           for (int i = 0; i < count; i++) {
///               printf("Proximity %d: %.2f\n", i, proximities[i]);
///           }
///           // 3. 释放内存
///           free(proximities);
LHANDPRO_API int lhandprolib_get_finger_proximity_ex(lhandprolib_handle handle,
                                                     int sensor_id,
                                                     float** proximity_list,
                                                     int* count);

// 日志管理
/// @brief 开启或关闭日志
/// @param handle 实例句柄
/// @param on true-开启日志，false-关闭日志
/// @param maxsize 日志文件最大大小（KB）
/// @example
///           // 开启日志，最大5MB
///           lhandprolib_log_on(handle, true, 5000);
LHANDPRO_API void lhandprolib_log_on(lhandprolib_handle handle, bool on,
                                     int maxsize);

/// @brief 保存日志到文件
/// @param handle 实例句柄
/// @param file_name 日志文件名
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
/// @example
///           lhandprolib_log_save(handle, "hand_log.txt");
LHANDPRO_API int lhandprolib_log_save(lhandprolib_handle handle,
                                      const char* file_name);

/// @brief 清除日志
/// @param handle 实例句柄
/// @example
///           lhandprolib_log_clear(handle);
LHANDPRO_API void lhandprolib_log_clear(lhandprolib_handle handle);

#ifdef __cplusplus
}
#endif

#endif  // LHANDPROLIB_H