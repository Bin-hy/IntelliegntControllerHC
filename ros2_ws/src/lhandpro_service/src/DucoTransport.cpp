// DucoTransport.cpp
#include "DucoTransport.h"

#include <duco_ros_driver/DucoCobot.h>

#include <iostream>

DucoTransport::DucoTransport() = default;

DucoTransport::~DucoTransport() {
  stop();
}

bool DucoTransport::init(const std::string& robot_ip, int port,
                         Protocol protocol) {
  protocol_ = protocol;
  robot_ip_ = robot_ip;
  state_ = State::Connecting;

  duco_ = std::make_shared<DucoRPC::DucoCobot>(robot_ip, port);
  if (duco_->open() != 0) {
    state_ = State::Disconnected;
    return false;
  }

  // Configure baudrate: set_baudrate_485 only affects controller-side RS485 (NOT tool RS485)
  // For ToolRS485, the end-effector port baud rate is fixed by DUCO firmware
  if (protocol == Protocol::RS485) {
    duco_->set_baudrate_485(115200, true);
  } else if (protocol == Protocol::CAN) {
    duco_->set_baudrate_can(1000000, true);
  }

  state_ = State::Operational;
  return true;
}

void DucoTransport::stop() {
  if (state_ == State::Disconnected) return;
  if (duco_) {
    duco_->close();
  }
  state_ = State::Disconnected;
}

DucoTransport::State DucoTransport::getState() const {
  return state_.load();
}

bool DucoTransport::sendRaw(const unsigned char* data, unsigned int len) {
  if (!data || len == 0) return false;
  if (state_.load() != State::Operational) return false;
  std::vector<int8_t> out(data, data + len);
  std::lock_guard<std::mutex> lock(io_mutex_);
  try {
    switch (protocol_) {
      case Protocol::RS485:
        return duco_->write_raw_data_485(out);
      case Protocol::ToolRS485:
        return duco_->tool_write_raw_data_485(out);
      case Protocol::CAN:
        return duco_->write_raw_data_can(0x501, out);
    }
  } catch (const std::exception& e) {
    std::cerr << "[DucoTransport] sendRaw exception: " << e.what()
              << std::endl;
    state_ = State::Error;
  }
  return false;
}

int DucoTransport::sendAndRecv(const unsigned char* send_data,
                               unsigned int send_len,
                               unsigned char* recv_buf,
                               unsigned int recv_max_len) {
  if (!send_data || send_len == 0) return -1;
  if (state_.load() != State::Operational) return -1;

  std::lock_guard<std::mutex> lock(io_mutex_);

  // --- Send ---
  std::vector<int8_t> out(send_data, send_data + send_len);
  bool sent = false;
  try {
    switch (protocol_) {
      case Protocol::RS485:
        sent = duco_->write_raw_data_485(out);
        break;
      case Protocol::ToolRS485:
        sent = duco_->tool_write_raw_data_485(out);
        break;
      case Protocol::CAN:
        sent = duco_->write_raw_data_can(0x501, out);
        break;
    }
  } catch (const std::exception& e) {
    std::cerr << "[DucoTransport] sendAndRecv send exception: " << e.what()
              << std::endl;
    state_ = State::Error;
    return -1;
  }
  if (!sent) {
    std::cerr << "[DucoTransport] write_raw_data_485 returned false (len=" << send_len << ")" << std::endl;
    return -1;
  }

  // --- Receive (immediately after send, same lock) ---
  if (!recv_buf || recv_max_len == 0) return 0;
  std::vector<int8_t> in;
  try {
    switch (protocol_) {
      case Protocol::RS485:
        duco_->read_raw_data_485(in, recv_max_len);
        break;
      case Protocol::ToolRS485:
        duco_->tool_read_raw_data_485(in, recv_max_len);
        break;
      case Protocol::CAN:
        duco_->read_raw_data_can(in);
        break;
    }
  } catch (const std::exception& e) {
    std::cerr << "[DucoTransport] sendAndRecv recv exception: " << e.what()
              << std::endl;
    state_ = State::Error;
    return -1;
  }
  int copy_len =
      std::min(static_cast<int>(in.size()), static_cast<int>(recv_max_len));
  if (copy_len > 0) {
    std::memcpy(recv_buf, in.data(), copy_len);
  }
  return copy_len;
}

int DucoTransport::recvRaw(unsigned char* buffer, unsigned int max_len) {
  if (!buffer || max_len == 0) return 0;
  if (state_.load() != State::Operational) return 0;
  std::vector<int8_t> in;
  std::lock_guard<std::mutex> lock(io_mutex_);
  try {
    switch (protocol_) {
      case Protocol::RS485:
        duco_->read_raw_data_485(in, max_len);
        break;
      case Protocol::ToolRS485:
        duco_->tool_read_raw_data_485(in, max_len);
        break;
      case Protocol::CAN:
        duco_->read_raw_data_can(in);
        break;
    }
  } catch (const std::exception& e) {
    std::cerr << "[DucoTransport] recvRaw exception: " << e.what()
              << std::endl;
    state_ = State::Error;
    return 0;
  }
  int copy_len = std::min(static_cast<int>(in.size()), static_cast<int>(max_len));
  if (copy_len > 0) {
    std::memcpy(buffer, in.data(), copy_len);
  }
  return copy_len;
}
