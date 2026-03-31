#include <rclcpp/rclcpp.hpp>
#include <lift_server/srv/lift_control.hpp>
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
// 位置模式参数(P001=5, P403, P404等)已在伺服面板预设
// 程序只需控制: P520(使能), P535(换步), P411/P410(目标脉冲), P412/P419(速度)
// 脉冲计算: total = P411*10000 + P410  (1cm = 10000脉冲)
//
// 工艺流程:
//   enable → 设置速度 + P520使能(触发SI3回原点)
//   set_target → 设置P410/P411目标脉冲
//   trigger_step → P535上升沿触发下一段(移动到目标 或 回原点)
//   disable → P520=0 断使能

class LiftController {
public:
    explicit LiftController(ModbusRTU& bus) : bus_(bus) {}

    // 初始化位置模式并使能(触发SI3回原点)
    // 每次都重新写入关键配置，确保干净状态
    bool enable(uint16_t speed) {
        // 0. 清除换步信号 P535=0
        bus_.write_register(0x0523, 0x0000);
        // 1. 位置模式 P001=5
        static const uint8_t CMD_POS_MODE[] = {0x01,0x06,0x00,0x01,0x00,0x05,0x18,0x09};
        if (!bus_.send_frame(CMD_POS_MODE)) return false;
        // 2. 绝对定位+上升沿换步 P403=0x0011
        if (!bus_.write_register(0x0403, 0x0011)) return false;
        // 3. 有效段数=2 P404=2
        if (!bus_.write_register(0x0404, 2)) return false;
        // 4. 段1速度 P412(0x040C)
        if (!bus_.write_register(0x040C, speed)) return false;
        // 5. 段2速度 P419(0x0413)
        if (!bus_.write_register(0x0413, speed)) return false;
        // 6. 段2目标=0(回原点) P417(0x0411)=0, P418(0x0412)=0
        if (!bus_.write_register(0x0411, 0)) return false;
        if (!bus_.write_register(0x0412, 0)) return false;
        // 7. 使能 P520: 0→0x0010 (上升沿, 触发SI3回原点)
        if (!bus_.write_register(0x0514, 0x0000)) return false;
        usleep(100000); // 100ms
        if (!bus_.write_register(0x0514, 0x0010)) return false;
        return true;
    }

    // 设置段1目标脉冲 (P411高位先写, P410低位后写, 与厂商一致)
    bool set_target(int32_t target_pulses) {
        uint16_t pulse_high = (uint16_t)(target_pulses / 10000);
        uint16_t pulse_low  = (uint16_t)(target_pulses % 10000);
        if (!bus_.write_register(0x040B, pulse_high)) return false;  // P411 高位
        if (!bus_.write_register(0x040A, pulse_low))  return false;  // P410 低位
        return true;
    }

    // 触发换步 P535(CHGSTP): 0→0x0010 上升沿
    bool trigger_step() {
        if (!bus_.write_register(0x0523, 0x0000)) return false;
        usleep(50000);
        return bus_.write_register(0x0523, 0x0010);
    }

    // 断使能 P520=0x0000
    bool disable() {
        bus_.write_register(0x0523, 0x0000);  // 复位CHGSTP
        return bus_.write_register(0x0514, 0x0000);
    }

private:
    ModbusRTU& bus_;
};

// ==================== ROS2 Node ====================
class LiftServerNode : public rclcpp::Node {
public:
    LiftServerNode() : Node("lift_server"),
        bus_(this->declare_parameter("serial_port", std::string("/dev/ttyUSB0")),
             this->declare_parameter("slave_id", 1)),
        lift_(bus_)
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
    rclcpp::Service<lift_server::srv::LiftControl>::SharedPtr srv_;
    std::mutex mtx_;

    void on_request(
        const std::shared_ptr<lift_server::srv::LiftControl::Request> req,
        std::shared_ptr<lift_server::srv::LiftControl::Response> res)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        const auto& cmd = req->command;
        int rpm = req->speed_rpm > 0 ? req->speed_rpm : 1000;
        RCLCPP_INFO(get_logger(), "CMD: %s rpm=%d", cmd.c_str(), rpm);
        bus_.clear_trace();

        bool ok = false;
        if (cmd == "enable") {
            uint16_t speed = rpm;
            RCLCPP_INFO(get_logger(), "enable: speed=%d", speed);
            ok = lift_.enable(speed);
        }
        else if (cmd == "set_target") {
            int32_t pulses = req->target_pulses;
            RCLCPP_INFO(get_logger(), "set_target: pulses=%d", pulses);
            ok = lift_.set_target(pulses);
        }
        else if (cmd == "trigger_step") ok = lift_.trigger_step();
        else if (cmd == "disable") ok = lift_.disable();
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
        else if (cmd == "status") {
            // Read P520(enable), P527(dir), P305(speed), P001(mode) status
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
    rclcpp::spin(std::make_shared<LiftServerNode>());
    rclcpp::shutdown();
    return 0;
}
