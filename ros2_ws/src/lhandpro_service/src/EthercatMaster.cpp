// EthercatMaster.cpp
#include "EthercatMaster.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <iostream>

#if ETHERCAT_DEBUG
#define ECOUT std::cout
#else
#define ECOUT \
  if (0) std::cout
#endif

std::string SlaveInfo::toString() const {
  // 状态字符串映射
  const char* state_str;
  switch (state & 0x0F) {
    case EC_STATE_INIT:
      state_str = "INIT";
      break;
    case EC_STATE_PRE_OP:
      state_str = "PRE_OP";
      break;
    case EC_STATE_SAFE_OP:
      state_str = "SAFE_OP";
      break;
    case EC_STATE_OPERATIONAL:
      state_str = "OP";
      break;
    default:
      state_str = "UNKNOWN";
      break;
  }

  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "Slave %2d [%-16s] State: %8s (0x%02X) AL: 0x%04X (%s)%s", index,
           name.substr(0, 16).c_str(), state_str, state, al_status,
           al_status_str.c_str(), is_lost ? " [LOST]" : "");

  return std::string(buffer);
}

std::ostream& operator<<(std::ostream& os, const SlaveInfo& info) {
  return os << info.toString();
}

EthercatMaster::EthercatMaster()
    : group_(0),
      roundtrip_time_(0),
      initialized_(false),
      started_(false),
      running_(false),
      outputs_(nullptr),
      inputs_(nullptr),
      output_bytes_(0),
      input_bytes_(0) {
  memset(&context_, 0, sizeof(context_));
}

EthercatMaster::~EthercatMaster() {
  if (running_) {
    stop();
  }
  if (initialized_) {
    ecx_close(&context_);
  }
}

std::vector<std::string> EthercatMaster::scanNetworkInterfaces() {
  interfaces_.clear();
  std::vector<std::string> descriptions;

  // 排除关键词列表
  static const std::vector<std::string> exclude_keywords = {
      "lo",         // Linux 回环接口
      "docker",     // Docker虚拟网卡
      "veth",       // 虚拟以太网设备
      "br-",        // 网桥接口
      "virbr",      // 虚拟网桥
      "vmnet",      // VMware虚拟网卡
      "tap",        // TAP虚拟设备
      "tun",        // TUN虚拟设备
      "wlan",       // 无线网卡
      "wlp",        // 无线网卡(新命名)
      "wlx",        // 无线网卡
      "wifi",       // WiFi接口
      "wwan",       // 无线广域网
      "bluetooth",  // 蓝牙
      "vboxnet",    // VirtualBox虚拟网卡
      "wintun",     // Windows TUN设备
      "p2p",        // P2P连接
      "loopback",   // 回环(Windows)
      "teredo",     // Teredo隧道
      "isatap"      // ISATAP隧道
  };

  ec_adaptert* adapter = ec_find_adapters();
  ec_adaptert* head = adapter;

  ECOUT << "扫描所有适配器:\n";
  int total_count = 0;
  int filtered_count = 0;

  while (adapter != nullptr) {
    total_count++;
    std::string adapter_name(adapter->name);
    std::string adapter_desc(adapter->desc);

    // 转换为小写以进行不区分大小写的匹配
    std::string desc_lower = adapter_desc;
    std::transform(desc_lower.begin(), desc_lower.end(), desc_lower.begin(),
                   ::tolower);

    // 检查是否应该排除
    bool should_exclude = false;
    for (const auto& keyword : exclude_keywords) {
      // 在描述中搜索关键词
      if (desc_lower.find(keyword) != std::string::npos) {
        ECOUT << "    [排除] " << adapter_name << "  (" << adapter_desc
              << ") - 匹配排除关键词: " << keyword << "\n";
        should_exclude = true;
        filtered_count++;
        break;
      }
    }

    // 如果不排除，则添加到列表
    if (!should_exclude) {
      ECOUT << "    [保留] " << adapter_name << "  (" << adapter_desc << ")\n";
      interfaces_.push_back(adapter_name);
      descriptions.push_back(adapter_desc);
    }

    adapter = adapter->next;
  }

  ec_free_adapters(head);

  ECOUT << "扫描完成: 共 " << total_count << " 个适配器，过滤 "
        << filtered_count << " 个，保留 " << descriptions.size() << " 个\n";

  return descriptions;
}

bool EthercatMaster::init(int index) {
  if (index < 0 || index >= interfaces_.size()) {
    ECOUT << "Index not available\n";
    return false;
  }

  iface_ = interfaces_.at(index);

  ECOUT << "Initializing SOEM on '" << iface_ << "'... " << std::flush;
  if (!ecx_init(&context_, iface_.c_str())) {
    ECOUT << "no socket connection\n";
    return false;
  }
  ECOUT << "done\n";

  ECOUT << "Finding autoconfig slaves... " << std::flush;
  if (ecx_config_init(&context_) <= 0) {
    ECOUT << "no slaves found\n";
    return false;
  }
  ECOUT << context_.slavecount << " slaves found\n";

  ECOUT << "Sequential mapping of I/O... " << std::flush;
  ecx_config_map_group(&context_, map_, group_);
  ec_groupt* grp = context_.grouplist + group_;
  output_bytes_ = grp->Obytes;
  input_bytes_ = grp->Ibytes;
  outputs_ = grp->outputs;
  inputs_ = grp->inputs;

  ECOUT << "mapped " << grp->Obytes << "O+" << grp->Ibytes << "I bytes from "
        << grp->nsegments << " segments";

  if (grp->nsegments > 1) {
    ECOUT << " (";
    for (int i = 0; i < grp->nsegments; ++i) {
      if (i > 0) ECOUT << "+";
      ECOUT << grp->IOsegment[i];
    }
    ECOUT << " slaves)";
  }
  ECOUT << "\n";

  ECOUT << "Configuring distributed clock... " << std::flush;
  ecx_configdc(&context_);
  ECOUT << "done\n";

  initialized_ = true;
  return true;
}

bool EthercatMaster::start() {
  if (!initialized_) return false;

  // ec_groupt* grp = context_.grouplist + group_;
  ec_slavet* slave = context_.slavelist;

  ECOUT << "Waiting for all slaves in safe operational... " << std::flush;
  ecx_statecheck(&context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  ECOUT << "done\n";

  ECOUT << "Send a roundtrip to make outputs in slaves happy... " << std::flush;
  ec_timet start = osal_current_time();
  ecx_send_processdata(&context_);
  ecx_receive_processdata(&context_, EC_TIMEOUTRET);
  ec_timet end = osal_current_time();
  ec_timet diff;
  osal_time_diff(&start, &end, &diff);
  roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);
  ECOUT << "done\n";

  ECOUT << "Setting operational state.." << std::flush;

  slave->state = EC_STATE_OPERATIONAL;
  ecx_writestate(&context_, 0);

  for (int i = 0; i < 10; ++i) {
    ECOUT << "." << std::flush;
    start = osal_current_time();
    ecx_send_processdata(&context_);
    ecx_receive_processdata(&context_, EC_TIMEOUTRET);
    end = osal_current_time();
    osal_time_diff(&start, &end, &diff);
    roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

    ecx_statecheck(&context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 10);
    if (slave->state == EC_STATE_OPERATIONAL) {
      ECOUT << " all slaves are now operational\n";
      started_ = true;
      return true;
    }
  }

  ECOUT << " failed,";
  ecx_readstate(&context_);
  for (int i = 1; i <= context_.slavecount; ++i) {
    slave = context_.slavelist + i;
    if (slave->state != EC_STATE_OPERATIONAL) {
      ECOUT << " slave " << i << " is 0x" << std::hex << std::setfill('0')
            << std::setw(4) << slave->state << " (AL-status=0x" << std::setw(4)
            << slave->ALstatuscode << std::dec << " "
            << ec_ALstatuscode2string(slave->ALstatuscode) << ")";
    }
  }
  ECOUT << "\n";

  return false;
}

void EthercatMaster::run() {
  if (!started_ || running_) {
    return;
  }

  running_ = true;

  worker_thread_ = std::thread([this]() {
    int min_time = 0, max_time = 0;
    int iteration = 1;

    while (running_) {
      ECOUT << "Iteration " << std::setw(4) << std::setfill(' ') << iteration
            << ":";

      ec_groupt* grp = context_.grouplist + group_;

      ec_timet start = osal_current_time();
      ecx_send_processdata(&context_);
      int wkc = ecx_receive_processdata(&context_, EC_TIMEOUTRET);
      ec_timet end = osal_current_time();
      ec_timet diff;
      osal_time_diff(&start, &end, &diff);
      roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

      int expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC;

      ECOUT << std::setw(6) << roundtrip_time_ << " usec  WKC " << wkc;
      if (wkc < expected_wkc) {
        ECOUT << " wrong (expected " << expected_wkc << ")\n";
      } else {
        ECOUT << "  O:";
        for (int n = 0; n < grp->Obytes; ++n) {
          ECOUT << " " << std::hex << std::setw(2) << std::setfill('0')
                << (int)grp->outputs[n] << std::dec;
        }
        ECOUT << "  I:";
        for (int n = 0; n < grp->Ibytes; ++n) {
          ECOUT << " " << std::hex << std::setw(2) << std::setfill('0')
                << (int)grp->inputs[n] << std::dec;
        }
        ECOUT << "  T: " << std::dec << context_.DCtime << "\r" << std::flush;
      }

      if (iteration == 1) {
        min_time = max_time = roundtrip_time_;
      } else {
        if (roundtrip_time_ < min_time) min_time = roundtrip_time_;
        if (roundtrip_time_ > max_time) max_time = roundtrip_time_;
      }

      iteration++;
      osal_usleep(5000);
    }

    ECOUT << "\nRoundtrip time (usec): min " << min_time << " max " << max_time
          << "\n";
  });
}

EthercatMaster::EthercatState EthercatMaster::getState() const {
  // 如果没运行，直接返回断开
  if (!running_) {
    return EthercatState::Disconnected;
  }

  // 如果没初始化
  if (!initialized_) {
    return EthercatState::Disconnected;
  }

  EthercatMaster* mutable_this = const_cast<EthercatMaster*>(this);
  ecx_readstate(&mutable_this->context_);

  // 检查从站状态
  bool all_operational = true;
  bool any_error = false;
  bool any_safe_op = false;

  for (int i = 1; i <= context_.slavecount; ++i) {
    const ec_slavet* slave = context_.slavelist + i;
    if (slave->group != group_) continue;

    if (slave->state == EC_STATE_NONE) {
      return EthercatState::Disconnected;  // 从站丢失
    }

    if (slave->state == EC_STATE_INIT) {
      return EthercatState::Initializing;
    }

    if (slave->state == EC_STATE_PRE_OP) {
      return EthercatState::Initializing;
    }

    if (slave->state == EC_STATE_SAFE_OP) {
      any_safe_op = true;
    }

    if ((slave->state & 0x0F) != EC_STATE_OPERATIONAL) {
      all_operational = false;
    }

    if (slave->state & EC_STATE_ERROR) {
      any_error = true;
    }
  }

  if (any_error) {
    return EthercatState::Error;
  }

  if (all_operational) {
    return EthercatState::Operational;
  }

  if (any_safe_op) {
    return EthercatState::SafeOperational;
  }

  return EthercatState::Initializing;
}

SlaveInfo EthercatMaster::getSlaveInfo() const {
  SlaveInfo info;

  if (context_.slavecount < 1) return info;

  const ec_slavet* slave = context_.slavelist + 1;

  if (slave->group != group_) return info;

  info.index = 1;
  info.name = std::string(slave->name);
  info.state = slave->state;
  info.al_status = slave->ALstatuscode;
  info.al_status_str = std::string(ec_ALstatuscode2string(slave->ALstatuscode));
  info.is_lost = slave->islost;

  return info;
}

void EthercatMaster::stop() {
  if (running_) {
    ECOUT << "Stopping EtherCAT thread... " << std::flush;
    running_ = false;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    ECOUT << "done\n";
  }

  if (started_) {
    ec_slavet* slave = context_.slavelist;
    ECOUT << "Requesting init state on all slaves... " << std::flush;
    slave->state = EC_STATE_INIT;
    ecx_writestate(&context_, 0);
    ECOUT << "done\n";
  }

  if (initialized_) {
    ECOUT << "Close socket... " << std::flush;
    ecx_close(&context_);
    ECOUT << "done\n";
  }

  started_ = false;
  initialized_ = false;
}

void EthercatMaster::setOutput(int index, uint8_t value) {
  if (!started_ || outputs_ == nullptr) return;
  if (index >= 0 && index < output_bytes_) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    outputs_[index] = value;
  }
}

uint8_t EthercatMaster::getInput(int index) {
  if (!started_ || inputs_ == nullptr) return 0;
  if (index >= 0 && index < input_bytes_) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    return inputs_[index];
  }
  return 0;
}
bool EthercatMaster::setOutputs(const uint8_t* data, unsigned int len) {
  if (!started_ || !data || len <= 0) return false;
  if (len > output_bytes_) {
    ECOUT << "Error: setOutputs length " << len
          << " exceeds configured output bytes " << output_bytes_ << "\n";
    return false;
  }

  std::lock_guard<std::mutex> lock(io_mutex_);
  memcpy(outputs_, data, len);
  return true;
}
bool EthercatMaster::getInputs(uint8_t* buffer, unsigned int len) {
  if (!started_ || !buffer || len <= 0) return false;
  if (len > input_bytes_) {
    ECOUT << "Error: getInputs buffer length " << len
          << " exceeds configured input bytes " << input_bytes_ << "\n";
    return false;
  }

  std::lock_guard<std::mutex> lock(io_mutex_);
  memcpy(buffer, inputs_, len);
  return true;
}
bool EthercatMaster::getOutputs(uint8_t* buffer, unsigned int len) {
  if (!started_ || !buffer || len <= 0) return false;
  if (len > output_bytes_) {
    ECOUT << "Error: getOutputs buffer length " << len
          << " exceeds configured input bytes " << output_bytes_ << "\n";
    return false;
  }

  std::lock_guard<std::mutex> lock(io_mutex_);
  memcpy(buffer, outputs_, len);
  return true;
}
int EthercatMaster::getOutputSize() const { return output_bytes_; }

int EthercatMaster::getInputSize() const { return input_bytes_; }

bool EthercatMaster::sdoRead(uint16_t index, uint8_t subindex,
                             uint32_t* value) {
  if (!initialized_) {
    ECOUT << "SOEM not initialized\n";
    return false;
  }

  const int slave_id = 1;
  uint8_t buffer[4];
  int size = sizeof(buffer);

  int result = ecx_SDOread(&context_, slave_id, index, subindex, false, &size,
                           buffer, EC_TIMEOUTRXM * 3);

  // 检查执行结果
  if (result <= 0) {
    ECOUT << "SDO Read [Slave " << slave_id << " 0x" << std::setfill('0')
          << std::setw(4) << std::hex << index << ":" << std::setw(2)
          << static_cast<int>(subindex) << "] failed. Error: " << result
          << "\n";
    return false;
  }
  if (size < sizeof(uint32_t)) {  // 检查实际读取的数据长度
    ECOUT << "SDO data too short (Expected 4, got " << size << ")\n";
    return false;
  }

  // 拷贝并转换字节序（小端→主机序）
  *value = (static_cast<uint32_t>(buffer[3]) << 24) |
           (static_cast<uint32_t>(buffer[2]) << 16) |
           (static_cast<uint32_t>(buffer[1]) << 8) | buffer[0];

  return true;
}

bool EthercatMaster::sdoWrite(uint16_t index, uint8_t subindex,
                              uint32_t value) {
  if (!initialized_) {
    ECOUT << "SOEM not initialized\n";
    return false;
  }

  const int slave_id = 1;
  uint8_t buffer[4];

  // 主机序转小端字节序（适用于EtherCAT设备）
  buffer[0] = static_cast<uint8_t>(value & 0xFF);  // LSB
  buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFF);  // MSB

  int size = sizeof(buffer);
  int result = ecx_SDOwrite(&context_, slave_id, index, subindex, FALSE, size,
                            buffer, EC_TIMEOUTRXM * 3);

  // 检查执行结果（ecx_SDOwrite返回1表示成功）
  if (result != 1) {
    ECOUT << "SDO Write [Slave " << slave_id << " 0x" << std::setfill('0')
          << std::setw(4) << std::hex << index << ":" << std::setw(2)
          << static_cast<int>(subindex) << "] failed. Error: " << result
          << " (Value: 0x" << std::setw(8) << value << ")\n";
    return false;
  }

  return true;
}