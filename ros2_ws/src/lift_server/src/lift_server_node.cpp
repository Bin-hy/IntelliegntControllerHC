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

// ==================== Lift Controller ====================
class LiftController {
public:
    explicit LiftController(ModbusRTU& bus) : bus_(bus) {}

    // 甲方确认的报文序列
    // 关键: 内部速度模式下, SPD-A/SPD-B 信号选择速度段:
    //   SPD-A=0, SPD-B=0 → 0速(不动!)
    //   SPD-A=0, SPD-B=1 → SPEED1 (P305)
    //   SPD-A=1, SPD-B=0 → SPEED2 (P306)
    //   SPD-A=1, SPD-B=1 → SPEED3 (P307)
    // 所以必须设置 P529(/SPD-B)=0x0010(常有效) 来选择 SPEED1

    bool move_up(uint16_t rpm) {
        // 1. 选择速度模式 P001=3
        static const uint8_t CMD_SPEED_MODE[] = {0x01,0x06,0x00,0x01,0x00,0x03,0x98,0x0B};
        if (!bus_.send_frame(CMD_SPEED_MODE)) return false;
        // 2. 设置内部速度1 P305=RPM
        if (!bus_.write_register(0x0305, rpm)) return false;
        // 3. 正转 P527(/SPD-D)=0x0000
        static const uint8_t CMD_FORWARD[] = {0x01,0x06,0x05,0x1B,0x00,0x00,0xF9,0x01};
        if (!bus_.send_frame(CMD_FORWARD)) return false;
        // 4. 选择速度段1: P529(/SPD-B)=0x0010(常有效), SPD-A保持0 → SPEED1
        if (!bus_.write_register(0x051D, 0x0010)) return false;
        // 5. 先关闭使能(确保上升沿) P520=0x0000
        if (!bus_.write_register(0x0514, 0x0000)) return false;
        usleep(50000); // 50ms
        // 6. 使能 P520=0x0010
        static const uint8_t CMD_ENABLE[] = {0x01,0x06,0x05,0x14,0x00,0x10,0xC8,0xCE};
        if (!bus_.send_frame(CMD_ENABLE)) return false;
        return true;
    }

    bool move_down(uint16_t rpm) {
        // 1. 速度模式
        static const uint8_t CMD_SPEED_MODE[] = {0x01,0x06,0x00,0x01,0x00,0x03,0x98,0x0B};
        if (!bus_.send_frame(CMD_SPEED_MODE)) return false;
        // 2. 设置速度
        if (!bus_.write_register(0x0305, rpm)) return false;
        // 3. 反转 P527(/SPD-D)=0x0010
        if (!bus_.write_register(0x051B, 0x0010)) return false;
        // 4. 选择速度段1: P529(/SPD-B)=0x0010
        if (!bus_.write_register(0x051D, 0x0010)) return false;
        // 5. 先关闭使能(确保上升沿)
        if (!bus_.write_register(0x0514, 0x0000)) return false;
        usleep(50000);
        // 6. 使能
        static const uint8_t CMD_ENABLE[] = {0x01,0x06,0x05,0x14,0x00,0x10,0xC8,0xCE};
        if (!bus_.send_frame(CMD_ENABLE)) return false;
        return true;
    }

    bool stop() {
        // 速度段归零: P529(/SPD-B)=0x0000 → 选择0速
        bus_.write_register(0x051D, 0x0000);
        // 方向归零
        static const uint8_t CMD_FWD_ZERO[] = {0x01,0x06,0x05,0x1B,0x00,0x00,0xF9,0x01};
        bus_.send_frame(CMD_FWD_ZERO);
        // 关闭使能 P520=0x0000
        return bus_.write_register(0x0514, 0x0000);
    }

    // ======= Position Mode (内部位置模式 P001=5) =======
    // P403=0x0011: 绝对定位, 上升沿换步
    // P404=2: 2段(段1=目标, 段2=回原点)
    // 脉冲计算: total = high*10000 + low
    // P520=S-ON使能, P535=CHGSTP换步信号

    bool position_move(int32_t target_pulses, uint16_t speed, uint16_t accel_ms, uint16_t decel_ms) {
        // 1. 位置模式 P001=5
        static const uint8_t CMD_POS_MODE[] = {0x01,0x06,0x00,0x01,0x00,0x05,0x18,0x09};
        if (!bus_.send_frame(CMD_POS_MODE)) return false;
        // 2. 绝对定位+上升沿换步 P403=0x0011
        static const uint8_t CMD_ABS_RISING[] = {0x01,0x06,0x04,0x03,0x00,0x11,0xB8,0xF6};
        if (!bus_.send_frame(CMD_ABS_RISING)) return false;
        // 3. 有效段数=2 P404=2
        if (!bus_.write_register(0x0404, 2)) return false;

        // 4. 第一段: 目标位置
        int16_t pulse_high = (int16_t)(target_pulses / 10000);
        int16_t pulse_low  = (int16_t)(target_pulses % 10000);
        if (!bus_.write_register(0x040A, (uint16_t)pulse_low))  return false;  // P4-10 脉冲低位
        if (!bus_.write_register(0x040B, (uint16_t)pulse_high)) return false;  // P4-11 脉冲高位
        if (!bus_.write_register(0x040C, speed))    return false;  // P4-12 转速
        if (!bus_.write_register(0x040D, accel_ms)) return false;  // P4-13 加速时间
        if (!bus_.write_register(0x040E, decel_ms)) return false;  // P4-14 减速时间

        // 5. 第二段: 回原点(位置0)
        if (!bus_.write_register(0x0411, 0))        return false;  // P4-17 脉冲低位
        if (!bus_.write_register(0x0412, 0))        return false;  // P4-18 脉冲高位
        if (!bus_.write_register(0x0413, speed))    return false;  // P4-19 转速
        if (!bus_.write_register(0x0414, accel_ms)) return false;  // P4-20 加速时间
        if (!bus_.write_register(0x0415, decel_ms)) return false;  // P4-21 减速时间

        // 6. 使能伺服 P520: 0→0x0010 (上升沿)
        if (!bus_.write_register(0x0514, 0x0000)) return false;
        usleep(50000);
        if (!bus_.write_register(0x0514, 0x0010)) return false;

        // 7. 触发换步 P535(CHGSTP): 0→0x0010 (上升沿触发第一段)
        if (!bus_.write_register(0x0523, 0x0000)) return false;
        usleep(50000);
        if (!bus_.write_register(0x0523, 0x0010)) return false;

        return true;
    }

    bool position_next() {
        // 触发下一段: CHGSTP上升沿
        if (!bus_.write_register(0x0523, 0x0000)) return false;
        usleep(50000);
        return bus_.write_register(0x0523, 0x0010);
    }

    bool position_stop() {
        bus_.write_register(0x0523, 0x0000);  // 复位CHGSTP
        return bus_.write_register(0x0514, 0x0000);  // 关闭使能
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
            static const uint8_t CMD_SPEED_MODE[] = {0x01,0x06,0x00,0x01,0x00,0x03,0x98,0x0B};
            ok = bus_.send_frame(CMD_SPEED_MODE);
        }
        else if (cmd == "move_up")   ok = lift_.move_up(rpm);
        else if (cmd == "move_down") ok = lift_.move_down(rpm);
        else if (cmd == "stop")      ok = lift_.stop();
        else if (cmd == "position_move") {
            int32_t pulses = req->target_pulses;
            uint16_t speed = req->speed_rpm > 0 ? req->speed_rpm : 1000;
            uint16_t acc = req->accel_ms > 0 ? req->accel_ms : 1000;
            uint16_t dec = req->decel_ms > 0 ? req->decel_ms : 1000;
            RCLCPP_INFO(get_logger(), "position_move: pulses=%d speed=%d acc=%d dec=%d",
                        pulses, speed, acc, dec);
            ok = lift_.position_move(pulses, speed, acc, dec);
        }
        else if (cmd == "position_next") ok = lift_.position_next();
        else if (cmd == "position_stop") ok = lift_.position_stop();
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
