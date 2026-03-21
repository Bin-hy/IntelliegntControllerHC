// DucoTransport.h
// Transport layer that replaces EthercatMaster, routing LHandProLib data
// through the DUCO controller's RS-485/CAN passthrough API.
#pragma once

#include <atomic>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace DucoRPC {
class DucoCobot;
}

class DucoTransport {
 public:
  enum class Protocol { RS485, ToolRS485, CAN };
  enum class State { Disconnected, Connecting, Operational, Error };

  DucoTransport();
  ~DucoTransport();

  bool init(const std::string& robot_ip, int port, Protocol protocol);
  void stop();
  State getState() const;

  // Synchronous send/receive
  bool sendRaw(const unsigned char* data, unsigned int len);
  int recvRaw(unsigned char* buffer, unsigned int max_len);

  // Atomic send-then-receive (for RS-485 half-duplex: must read immediately after write)
  // Returns bytes received, or -1 on send failure
  int sendAndRecv(const unsigned char* send_data, unsigned int send_len,
                  unsigned char* recv_buf, unsigned int recv_max_len);

 private:
  std::shared_ptr<DucoRPC::DucoCobot> duco_;
  Protocol protocol_;
  std::string robot_ip_;
  std::atomic<State> state_{State::Disconnected};
  mutable std::mutex io_mutex_;  // serializes all Thrift socket access
};
