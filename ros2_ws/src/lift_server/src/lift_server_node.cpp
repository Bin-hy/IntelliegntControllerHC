#include <rclcpp/rclcpp.hpp>
#include <lift_server/srv/lift_control.hpp>
#include <duco_msg/srv/robot_io_control.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <mutex>
#include <string>
#include <cstring>
#include <vector>

using namespace std::placeholders;

// ==================== Raw Modbus RTU ====================
class ModbusRTU {
public:
    ModbusRTU(const std::string& port, uint8_t slave)
        : port_(port), slave_(slave) {}
    ~ModbusRTU() { close_port(); }

    bool open_port() {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) return false;

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) { close_port(); return false; }

        cfsetispeed(&tty, B19200);
        cfsetospeed(&tty, B19200);

        tty.c_cflag &= ~(CSIZE | PARODD | CSTOPB | CRTSCTS);
        tty.c_cflag |= CS8 | CREAD | CLOCAL | PARENB;  // 8E1 (even parity)
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_iflag |= INPCK;  // enable parity check
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;  // 500ms read timeout

        tcflush(fd_, TCIOFLUSH);
        return tcsetattr(fd_, TCSANOW, &tty) == 0;
    }
    void close_port() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }

    // Send pre-built 8-byte frame (with CRC already included), read 8-byte echo
    bool send_frame(const uint8_t frame[8]) {
        log_hex("TX", frame, 8);
        tcflush(fd_, TCIOFLUSH);
        if (::write(fd_, frame, 8) != 8) return false;
        uint8_t rx[8]{};
        if (read_bytes(rx, 8) < 8) { log_hex("RX", rx, 0); return false; }
        log_hex("RX", rx, 8);
        // Append to trace log
        trace_ += "TX:" + last_tx_ + "RX:" + last_rx_ + "\n";
        usleep(50000); // 50ms inter-frame delay
        return memcmp(frame, rx, 8) == 0;
    }

    // Build and send 06H write-register frame (for dynamic values like RPM)
    bool write_register(uint16_t reg, uint16_t val) {
        uint8_t f[8] = {
            slave_, 0x06,
            uint8_t(reg >> 8), uint8_t(reg),
            uint8_t(val >> 8), uint8_t(val), 0, 0
        };
        uint16_t c = crc16(f, 6);
        f[6] = c & 0xFF;
        f[7] = (c >> 8) & 0xFF;
        return send_frame(f);
    }

    // 03H: Read holding register
    bool read_register(uint16_t reg, uint16_t& val) {
        uint8_t tx[8] = {
            slave_, 0x03,
            uint8_t(reg >> 8), uint8_t(reg),
            0x00, 0x01, 0, 0
        };
        uint16_t c = crc16(tx, 6);
        tx[6] = c & 0xFF; tx[7] = (c >> 8) & 0xFF;
        log_hex("TX", tx, 8);
        tcflush(fd_, TCIOFLUSH);
        if (::write(fd_, tx, 8) != 8) return false;
        uint8_t rx[7]{};
        if (read_bytes(rx, 7) < 7) return false;
        val = (rx[3] << 8) | rx[4];
        return true;
    }

    void clear_trace() { trace_.clear(); }
    std::string get_trace() const { return trace_; }

    std::string last_tx_, last_rx_, trace_;

private:
    int fd_ = -1;
    std::string port_;
    uint8_t slave_;

    int read_bytes(uint8_t* buf, int need) {
        int got = 0;
        for (int i = 0; got < need && i < 30; ++i) {
            int n = ::read(fd_, buf + got, need - got);
            if (n > 0) { got += n; i = 0; }
            else usleep(5000);
        }
        return got;
    }

    void log_hex(const char* tag, const uint8_t* d, size_t len) {
        std::string s;
        char buf[4];
        for (size_t i = 0; i < len; ++i) { snprintf(buf, 4, "%02X ", d[i]); s += buf; }
        if (tag[0] == 'T') last_tx_ = s; else last_rx_ = s;
    }

    static uint16_t crc16(const uint8_t* d, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i) {
            crc ^= d[i];
            for (int j = 0; j < 8; ++j)
                crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
        return crc;
    }
};

// ==================== Lift Controller (Position Mode Only) ====================
// 位置模式参数(P001=5, P403, P404等)已在伺服面板预设，程序不再重复写入
// 程序只需控制: P520(使能), P535(换步), P411/P410(目标脉冲), P412/P419(速度)
// 脉冲计算: total = P411*10000 + P410  (1cm = 10000脉冲)
//
// 工艺流程 (5步):
//   第1步 enable  → 断使能→写速度→DIO4打开抱闸→使能电机→DIO10触发回零
//   第2步 set_target → 断使能→写速度+目标脉冲→使能电机
//   第3步 step_up  → P535上升沿触发上升动作
//   (等待洗手/烘干时间)
//   第4步 step_down → P535上升沿触发下降动作
//   第5步 disable  → 断使能→DIO4关闭抱闸

class LiftController {
public:
    explicit LiftController(ModbusRTU& bus) : bus_(bus) {}

    // ===== 第1步: 初始化并回零 =====
    // 前端QT输入: 速度RPM + 回原点按钮, 只执行一次
    bool enable(uint16_t speed) {
        // 0. 清除换步信号 P535=0 (电平归零)
        bus_.write_register(0x0523, 0x0000);

        // 1. 先断使能, 才能写入速度参数
        if (!bus_.write_register(0x0514, 0x0000)) return false;

        // (P001=5模式, P403=0x0011定位方式, P404=2段数 已由硬件预设, 不再写入)

        // 2. 段1上升速度 P412(0x040C), 默认500RPM, 页面输入框可调
        if (!bus_.write_register(0x040C, speed)) return false;

        // 3. 段2下降速度 P419(0x0413), 与上升速度一致, 可调
        if (!bus_.write_register(0x0413, speed)) return false;

        // (段2目标P417/P418=0 已由硬件预设, 不再写入)

        // 4. 使能电机 P520: 写0x0010 (上升沿, 触发SI3回原点)
        usleep(100000); // 延时100ms
        if (!bus_.write_register(0x0514, 0x0010)) return false;

        return true;
    }

    // ===== 第2步: 设置目标位置 =====
    // 前端QT输入: 速度RPM + 位移cm + 位置模式按钮
    // 第2、3、4步可封装为一个大流程, 在总测试中调用3次
    bool set_target(int32_t target_pulses, uint16_t speed) {
        // 1. 先断使能, 才能写入位置脉冲值和速度参数
        if (!bus_.write_register(0x0514, 0x0000)) return false;

        // 2. 段1上升速度 P412(0x040C), 默认500RPM, 页面输入框可调
        if (!bus_.write_register(0x040C, speed)) return false;

        // 3. 段2下降速度 P419(0x0413), 与上升速度一致
        if (!bus_.write_register(0x0413, speed)) return false;

        // 4. 设置段1目标脉冲 (脉冲值来源于前端输入, 1cm=10000脉冲)
        //    P411高位先写, P410低位后写, 与厂商一致
        uint16_t pulse_high = (uint16_t)(target_pulses / 10000);
        uint16_t pulse_low  = (uint16_t)(target_pulses % 10000);
        if (!bus_.write_register(0x040B, pulse_high)) return false;  // P411 高位
        if (!bus_.write_register(0x040A, pulse_low))  return false;  // P410 低位

        // 5. 重新使能电机
        usleep(100000); // 延时100ms
        if (!bus_.write_register(0x0514, 0x0010)) return false;

        return true;
    }

    // ===== 第3步: 触发上升动作 =====
    // P535(CHGSTP): 0→0x0010 上升沿执行上升动作
    bool step_up() {
        if (!bus_.write_register(0x0523, 0x0000)) return false;
        usleep(1000000); // 等待1s
        return bus_.write_register(0x0523, 0x0010);
    }

    // ===== 第4步: 触发下降动作 =====
    // P535(CHGSTP): 0→0x0010 上升沿执行下降动作
    bool step_down() {
        if (!bus_.write_register(0x0523, 0x0000)) return false;
        usleep(1000000); // 等待1s
        return bus_.write_register(0x0523, 0x0010);
    }

    // ===== 第5步: 断使能 =====
    bool disable() {
        bus_.write_register(0x0523, 0x0000);  // 复位换步信号
        return bus_.write_register(0x0514, 0x0000); // 复位使能
    }

private:
    ModbusRTU& bus_;
};

// ==================== DIO Helper ====================
// 封装DUCO机械臂DIO控制, 用于抱闸(DIO4)和回零触发(DIO10)
class DioHelper {
public:
    DioHelper(rclcpp::Node* node, rclcpp::CallbackGroup::SharedPtr cb_group)
        : node_(node),
          logger_(node->get_logger())
    {
        // 使用独立的 CallbackGroup, 避免在服务回调内同步调用时死锁
        dio_client_ = node->create_client<duco_msg::srv::RobotIoControl>(
            "/duco_robot/robot_io_control",
            rclcpp::ServicesQoS(),
            cb_group);
    }

    // 同步调用DIO设置, 超时2s
    bool set_dio(int8_t port, bool value) {
        if (!dio_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(logger_, "DIO service not available (port=%d)", port);
            return false;
        }

        auto request = std::make_shared<duco_msg::srv::RobotIoControl::Request>();
        request->command = "setIo";
        request->arm_num = 0;
        request->type = 0;       // 0: Standard IO
        request->port = port;
        request->value = value;
        request->block = true;

        auto future = dio_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            RCLCPP_ERROR(logger_, "DIO timeout (port=%d, value=%d)", port, value);
            return false;
        }

        auto result = future.get();
        RCLCPP_INFO(logger_, "DIO port=%d value=%d response=%s",
                     port, value, result->response.c_str());
        return true;
    }

    // 打开抱闸 (DIO4=true), 第1步使能前调用
    bool brake_release() {
        RCLCPP_INFO(logger_, "Releasing brake (DIO4=true)");
        return set_dio(4, true);
    }

    // 关闭抱闸 (DIO4=false), 第5步断使能后调用
    bool brake_engage() {
        RCLCPP_INFO(logger_, "Engaging brake (DIO4=false)");
        return set_dio(4, false);
    }

    // 触发回零信号 (DIO10: true→等1s→false)
    bool trigger_homing() {
        RCLCPP_INFO(logger_, "Triggering homing signal (DIO10)");
        if (!set_dio(10, true)) return false;
        usleep(1000000); // 等待1s
        if (!set_dio(10, false)) return false;
        return true;
    }

private:
    rclcpp::Node* node_;
    rclcpp::Logger logger_;
    rclcpp::Client<duco_msg::srv::RobotIoControl>::SharedPtr dio_client_;
};

// ==================== ROS2 Node ====================
class LiftServerNode : public rclcpp::Node {
public:
    LiftServerNode() : Node("lift_server"),
        bus_(this->declare_parameter("serial_port", std::string("/dev/ttyUSB0")),
             this->declare_parameter("slave_id", 1)),
        lift_(bus_),
        dio_cb_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
        dio_(this, dio_cb_group_)
    {
        if (!bus_.open_port())
            RCLCPP_ERROR(get_logger(), "Failed to open serial port");
        else
            RCLCPP_INFO(get_logger(), "Serial port opened OK");

        srv_ = create_service<lift_server::srv::LiftControl>(
            "/lift_server/lift_control",
            std::bind(&LiftServerNode::on_request, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Lift Server Node started.");
    }

private:
    ModbusRTU bus_;
    LiftController lift_;
    rclcpp::CallbackGroup::SharedPtr dio_cb_group_;
    DioHelper dio_;
    rclcpp::Service<lift_server::srv::LiftControl>::SharedPtr srv_;
    std::mutex mtx_;

    void on_request(
        const std::shared_ptr<lift_server::srv::LiftControl::Request> req,
        std::shared_ptr<lift_server::srv::LiftControl::Response> res)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        const auto& cmd = req->command;
        int rpm = req->speed_rpm > 0 ? req->speed_rpm : 500; // 默认500RPM
        RCLCPP_INFO(get_logger(), "CMD: %s rpm=%d", cmd.c_str(), rpm);
        bus_.clear_trace();

        bool ok = false;

        // ===== 第1步: enable (回原点) =====
        if (cmd == "enable") {
            uint16_t speed = rpm;
            RCLCPP_INFO(get_logger(), "enable: speed=%d", speed);

            // 1) DIO4=true 打开抱闸
            if (!dio_.brake_release()) {
                res->success = false;
                res->message = "enable FAIL: brake_release DIO4 failed";
                return;
            }

            // 2) 使能电机+写速度+回零
            ok = lift_.enable(speed);

            // 3) DIO10触发回零信号 (true→1s→false)
            if (ok) {
                if (!dio_.trigger_homing()) {
                    RCLCPP_WARN(get_logger(), "DIO10 homing trigger failed, motor enabled but homing may not start");
                }
            }
        }

        // ===== 第2步: set_target (设置目标位置) =====
        else if (cmd == "set_target") {
            int32_t pulses = req->target_pulses;
            uint16_t speed = rpm;
            RCLCPP_INFO(get_logger(), "set_target: pulses=%d speed=%d", pulses, speed);
            ok = lift_.set_target(pulses, speed);
        }

        // ===== 第3步: step_up (触发上升) =====
        else if (cmd == "step_up") {
            RCLCPP_INFO(get_logger(), "step_up: triggering ascend");
            ok = lift_.step_up();
        }

        // ===== 第4步: step_down (触发下降) =====
        else if (cmd == "step_down") {
            RCLCPP_INFO(get_logger(), "step_down: triggering descend");
            ok = lift_.step_down();
        }

        // ===== 第5步: disable (断使能) =====
        else if (cmd == "disable") {
            ok = lift_.disable();

            // 断使能后关闭抱闸 DIO4=false
            if (!dio_.brake_engage()) {
                RCLCPP_WARN(get_logger(), "DIO4 brake_engage failed after disable");
            }
        }

        // ===== 读取编码器位置 =====
        else if (cmd == "get_position") {
            // Read encoder position: 0x1038 (high) + 0x1039 (low) → 32-bit
            uint16_t hi = 0, lo = 0;
            bool ok_hi = bus_.read_register(0x1038, hi);
            bool ok_lo = bus_.read_register(0x1039, lo);
            if (ok_hi && ok_lo) {
                res->encoder_position = (int32_t)((hi << 16) | lo);
                res->success = true;
                res->message = "pos=" + std::to_string(res->encoder_position);
            } else {
                res->success = false;
                res->message = "encoder read fail";
            }
            RCLCPP_DEBUG(get_logger(), "ENCODER: hi=%d lo=%d pos=%d", hi, lo, res->encoder_position);
            return;
        }

        // ===== 读取状态寄存器 =====
        else if (cmd == "status") {
            std::string info;
            uint16_t val = 0;
            struct { uint16_t reg; const char* name; } regs[] = {
                {0x0001, "P001(mode)"}, {0x0305, "P305(speed)"},
                {0x0514, "P520(S-ON)"}, {0x051B, "P527(SPD-D)"},
                {0x051C, "P528(SPD-A)"}, {0x051D, "P529(SPD-B)"}
            };
            for (auto& r : regs) {
                if (bus_.read_register(r.reg, val))
                    info += std::string(r.name) + "=" + std::to_string(val) + " ";
                else
                    info += std::string(r.name) + "=FAIL ";
            }
            res->success = true;
            res->message = info;
            RCLCPP_INFO(get_logger(), "STATUS: %s", info.c_str());
            return;
        }

        // ===== 兼容旧命令 trigger_step =====
        else if (cmd == "trigger_step") {
            RCLCPP_WARN(get_logger(), "trigger_step is deprecated, use step_up/step_down");
            ok = lift_.step_up();
        }

        else {
            res->success = false;
            res->message = "Unknown: " + cmd;
            return;
        }

        res->success = ok;
        res->message = cmd + (ok ? " OK" : " FAIL") + "\n" + bus_.get_trace();
        RCLCPP_INFO(get_logger(), "%s\n%s", cmd.c_str(), bus_.get_trace().c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiftServerNode>();
    // 多线程执行器: 服务回调和DIO客户端在不同线程, 避免死锁
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
