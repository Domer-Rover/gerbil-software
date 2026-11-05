// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

namespace roboclaw_serial
{

class SerialDevice
{
public:
  typedef std::shared_ptr<SerialDevice> SharedPtr;

  SerialDevice() = default;

  explicit SerialDevice(const std::string device) {connect(device);}
  virtual ~SerialDevice() {disconnect();}

  virtual bool connect(const std::string & device)
  {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
    connected_ = fd_ != -1;

    if (connected_) {
      // Claim exclusive access so a second process (e.g. another
      // ros2_control_node) opening this port fails fast with EBUSY instead of
      // silently interleaving its traffic with ours on the wire, which
      // otherwise manifests as CRC mismatches and read timeouts on both sides.
      if (ioctl(fd_, TIOCEXCL) < 0) {
        std::cerr << "Warning: failed to set exclusive access on " << device << std::endl;
      }
      setSerialDeviceOptions();
    } else {
      std::cerr << "Failed to open serial device: " << device << std::endl;
      if (errno == EBUSY) {
        std::cerr << "Device is already open by another process (e.g. another "
                     "ros2_control_node instance). Only one process may talk to "
                     "the RoboClaws at a time." << std::endl;
      }
      perror("Error");
    }

    return connected_;
  }

  virtual void disconnect()
  {
    if (connected_) {
      close(fd_);
      connected_ = false;
    }
  }

  bool connected() const {return connected_;}

  virtual std::size_t write(const std::byte * buffer, std::size_t count)
  {
    ssize_t result = ::write(fd_, buffer, count);
    if (result < 0) {
      // Error writing to device
      throw std::range_error("Error writing to the device!");
    }
    return static_cast<std::size_t>(result) == count;
  }

  virtual std::size_t read(std::byte * buffer, std::size_t count)
  {
    fd_set set;
    struct timeval timeout;

    /* Initialize the file descriptor set. */
    FD_ZERO(&set);
    FD_SET(fd_, &set);

    /* Initialize the timeout data structure. */
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;  // 10ms

    /* select returns 0 if timeout, 1 if input available, -1 if error. */
    int res = select(FD_SETSIZE, &set, NULL, NULL, &timeout);
    if (res < 0) {
      throw std::range_error("Error reading from the serial device!");
    } else if (res == 0) {
      throw std::runtime_error("Read timeout!");
    }
    ssize_t result = ::read(fd_, buffer, count);
    if (result < 0) {
      // Error reading from the device
      throw std::range_error("Error reading from the serial device!");
    }

    return static_cast<std::size_t>(result);
  }

  // Reads exactly `count` bytes, retrying until the full response arrives or
  // the overall deadline elapses. A single read() only guarantees that at
  // least one byte was available, and can return fewer bytes than requested
  // when the response is split across multiple USB frames from the serial
  // adapter. Treating that partial data as a complete response corrupts CRC
  // checks and field parsing, so callers that know the exact expected size
  // should use this instead of read().
  std::size_t read_exact(std::byte * buffer, std::size_t count)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
    std::size_t total_read = 0;

    while (total_read < count) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("Read timeout!");
      }

      // Dispatches to read(), so the per-call timeout and any device-specific
      // behaviour stay in one place
      total_read += read(buffer + total_read, count - total_read);
    }

    return total_read;
  }

protected:
  bool connected_ = false;

private:
  void setSerialDeviceOptions()
  {
    struct termios options;
    tcgetattr(fd_, &options);
    options.c_cflag = CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;

    // CRITICAL FIX: Set baud rate to 38400
    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);

    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &options);

    // Set the file descriptor to non-blocking mode
    fcntl(fd_, F_SETFL, O_NONBLOCK);
  }

  int fd_ = -1;
};

}  // namespace roboclaw_serial
