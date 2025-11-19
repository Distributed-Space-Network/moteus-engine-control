// Copyright 2023 mjbots Robotic Systems, LLC.
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <array>
#include "mbed.h"
#include "mjlib/multiplex/micro_datagram_server.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/base/string_span.h"
#include "fw/stm32g4_async_uart.h"

namespace moteus {

/// A simple micro datagram server that shims the multiplex protocol over
/// a UART. Each datagram is prefaced with a 4‑byte header:
///   byte 0: destination, byte 1: source, byte 2: payload length (0–64),
///   byte 3: flags.  Payload follows immediately.
/// Supports payloads up to 64 bytes.
class UartMicroServer : public mjlib::multiplex::MicroDatagramServer {
 public:
  explicit UartMicroServer(Stm32G4AsyncUart* uart);
  ~UartMicroServer() override = default;

  void AsyncRead(Header* header,
                 const mjlib::base::string_span& data,
                 const mjlib::micro::SizeCallback& callback) override;

  void AsyncWrite(const Header& header,
                  const std::string_view& data,
                  const Header& query_header,
                  const mjlib::micro::SizeCallback& callback) override;

  Properties properties() const override;
  void Poll();

 private:
  void HandleReadHeader(const mjlib::micro::error_code& ec, size_t bytes);
  void HandleReadPayload(const mjlib::micro::error_code& ec, size_t bytes);
  void HandleWriteHeader(const mjlib::micro::error_code& ec, size_t bytes);
  void HandleWritePayload(const mjlib::micro::error_code& ec, size_t bytes);

  Stm32G4AsyncUart* const uart_;
  mjlib::micro::SizeCallback current_read_callback_;
  Header* current_read_header_ = nullptr;
  mjlib::base::string_span current_read_data_{};
  std::array<uint8_t, 4> read_header_buf_{}; 
  size_t read_header_bytes_ = 0;
  size_t read_payload_bytes_ = 0;
  mjlib::micro::SizeCallback current_write_callback_;
  std::array<uint8_t, 4> write_header_buf_{};
  const char* write_payload_ptr_ = nullptr;
  size_t write_payload_size_ = 0;
  size_t write_payload_bytes_ = 0;
};

}  // namespace moteus
