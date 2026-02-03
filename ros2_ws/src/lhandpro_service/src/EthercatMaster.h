// EthercatMaster.h
#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include <soem/soem.h>
}

struct SlaveInfo {
  int index;
  std::string name;
  uint16_t state;
  uint16_t al_status;
  std::string al_status_str;
  bool is_lost;

  std::string toString() const;
};

std::ostream& operator<<(std::ostream& os, const SlaveInfo& info);

class EthercatMaster {
 public:
  enum class EthercatState {
    Disconnected,     // 未初始化或已停止
    Initializing,     // init 成功，但未 start
    SafeOperational,  // 进入 SAFE_OP
    Operational,      // 正常运行（OPERATIONAL）
    Error             // 有从站报错
  };

  EthercatMaster();
  ~EthercatMaster();

  std::vector<std::string> scanNetworkInterfaces();
  bool init(int index);
  bool start();
  void stop();
  void run();  // 启动子线程（非阻塞）

  EthercatState getState() const;
  SlaveInfo getSlaveInfo() const;

  // 单字节操作
  void setOutput(int index, uint8_t value);
  uint8_t getInput(int index);
  // 批量操作
  bool setOutputs(const uint8_t* data, unsigned int len);
  bool getInputs(uint8_t* buffer, unsigned int len);
  bool getOutputs(uint8_t* buffer, unsigned int len);
  int getOutputSize() const;
  int getInputSize() const;

  // SDO 读写: 对象字典 (index, subindex)
  bool sdoRead(uint16_t index, uint8_t subindex, uint32_t* value);
  bool sdoWrite(uint16_t index, uint8_t subindex, uint32_t value);

 private:
  ecx_contextt context_;                 // SOEM context
  std::vector<std::string> interfaces_;  // 扫描到的网卡名
  std::string iface_;                    // 网卡名
  uint8 group_;                          // 组号
  int roundtrip_time_;                   // 往返时间
  uint8 map_[4096];                      // I/O 映射缓冲区
  bool initialized_;
  bool started_;

  // 多线程控制
  std::atomic<bool> running_;
  std::thread worker_thread_;
  std::mutex io_mutex_;  // 保护 outputs/inputs 访问

  // 缓存指针
  uint8* outputs_;
  uint8* inputs_;
  int output_bytes_;
  int input_bytes_;
};
