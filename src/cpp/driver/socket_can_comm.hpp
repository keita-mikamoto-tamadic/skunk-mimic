#pragma once
#include <cstdint>
#include <cstddef>
#include <string>

class SocketCanComm {
  public:
    SocketCanComm();
    ~SocketCanComm();
    
    // Coppy constuctor prohibited
    SocketCanComm(const SocketCanComm&) = delete;
    // Coppy assignment prohibited
    SocketCanComm& operator=(const SocketCanComm&) = delete;
    
    // --Lifecycle--
    bool Open(const std::string& ifname = "can0");
    void Close();
    bool IsOpen() const;

    // -- CAN-FD Frame I/O --
    bool SendFrame(uint32_t arb_id, const uint8_t* data, size_t len);
    bool ReceiveFrame(int device_id, uint8_t* data, size_t* len, int timeout_ms);
    
  private:
   int socket_fd_ = -1;
   std::string ifname_;
    
}; // SocketCanComm