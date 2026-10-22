// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#ifndef TAUROB_TRACKER_HARDWARE__TEST__MOCK_ECU_COMMON_HPP_
#define TAUROB_TRACKER_HARDWARE__TEST__MOCK_ECU_COMMON_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>
#include <cstdint>

namespace taurob_tracker_hardware_test
{

/// CRC-16 as used by the taurob U-ECU frames (reflected polynomial 0xA001,
/// init 0xffff; matches Arm_frames::Calc_CRC / FlipperFrames::Calc_CRC).
inline uint16_t calc_crc16(const unsigned char * src, int len)
{
  uint16_t crc = 0xffff;
  for (int i = 0; i < len; ++i)
  {
    crc ^= src[i];
    for (int b = 0; b < 8; ++b)
    {
      if (crc & 0x0001)
      {
        crc = (crc >> 1) ^ 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/// Finds a free UDP port on the loopback interface.
inline uint16_t find_free_udp_port()
{
  int probe = socket(AF_INET, SOCK_DGRAM, 0);
  if (probe < 0)
  {
    throw std::runtime_error("socket() failed");
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (::bind(probe, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    ::close(probe);
    throw std::runtime_error("bind() failed");
  }
  sockaddr_in bound{};
  socklen_t len = sizeof(bound);
  getsockname(probe, reinterpret_cast<sockaddr *>(&bound), &len);
  const uint16_t port = ntohs(bound.sin_port);
  ::close(probe);
  return port;
}

}  // namespace taurob_tracker_hardware_test

#endif  // TAUROB_TRACKER_HARDWARE__TEST__MOCK_ECU_COMMON_HPP_
