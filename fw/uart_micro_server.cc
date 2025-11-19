// Copyright 2023 mjbots Robotic Systems, LLC.
// Licensed under the Apache License, Version 2.0.
#include "fw/uart_micro_server.h"

#include <cstring>

namespace moteus {

using mjlib::base::string_span;
using mjlib::micro::SizeCallback;

UartMicroServer::UartMicroServer(Stm32G4AsyncUart* uart)
    : uart_(uart) {}

void UartMicroServer::AsyncRead(Header* header,
                                const string_span& data,
                                const SizeCallback& callback) {
  MJ_ASSERT(!current_read_callback_);
  current_read_header_ = header;
  current_read_data_ = data;
  current_read_callback_ = callback;
  read_header_bytes_ = 0;
  read_payload_bytes_ = 0;
  auto header_span = string_span(
      reinterpret_cast<char*>(read_header_buf_.data()), read_header_buf_.size());
  uart_->AsyncReadSome(header_span,
      [this](const mjlib::micro::error_code& ec, size_t bytes) {
        this->HandleReadHeader(ec, bytes);
      });
}

void UartMicroServer::HandleReadHeader(const mjlib::micro::error_code& ec,
                                       size_t bytes) {
  read_header_bytes_ += bytes;
  if (ec) {
    auto cb = current_read_callback_;
    current_read_callback_ = {};
    current_read_header_ = nullptr;
    current_read_data_ = {};
    read_header_bytes_ = 0;
    read_payload_bytes_ = 0;
    cb(ec, 0);
    return;
  }
  if (read_header_bytes_ < read_header_buf_.size()) {
    auto header_span = string_span(
        reinterpret_cast<char*>(read_header_buf_.data()) + read_header_bytes_,
        read_header_buf_.size() - read_header_bytes_);
    uart_->AsyncReadSome(header_span,
        [this](const mjlib::micro::error_code& ec2, size_t bytes2) {
          this->HandleReadHeader(ec2, bytes2);
        });
    return;
  }
  // Parse the 4‑byte header.
  current_read_header_->destination = read_header_buf_[0];
  current_read_header_->source      = read_header_buf_[1];
  current_read_header_->size        = read_header_buf_[2];
  current_read_header_->flags       = read_header_buf_[3];
  // Limit payload to available buffer.
  size_t to_read = current_read_header_->size;
  if (to_read > current_read_data_.size()) {
    to_read = current_read_data_.size();
  }
  if (to_read == 0) {
    auto cb = current_read_callback_;
    current_read_callback_ = {};
    current_read_header_ = nullptr;
    current_read_data_ = {};
    read_header_bytes_ = 0;
    read_payload_bytes_ = 0;
    cb(mjlib::micro::error_code(), 0);
    return;
  }
  uart_->AsyncReadSome(
      string_span(current_read_data_.data(), to_read),
      [this](const mjlib::micro::error_code& ec2, size_t bytes2) {
        this->HandleReadPayload(ec2, bytes2);
      });
}

void UartMicroServer::HandleReadPayload(const mjlib::micro::error_code& ec,
                                        size_t bytes) {
  read_payload_bytes_ += bytes;
  size_t total_expected = current_read_header_->size;
  if (total_expected > current_read_data_.size()) {
    total_expected = current_read_data_.size();
  }
  if (ec) {
    auto cb = current_read_callback_;
    current_read_callback_ = {};
    current_read_header_ = nullptr;
    current_read_data_ = {};
    read_header_bytes_ = 0;
    read_payload_bytes_ = 0;
    cb(ec, read_payload_bytes_);
    return;
  }
  if (read_payload_bytes_ < total_expected) {
    auto span = string_span(
        current_read_data_.data() + read_payload_bytes_,
        total_expected - read_payload_bytes_);
    uart_->AsyncReadSome(span,
        [this](const mjlib::micro::error_code& ec2, size_t bytes2) {
          this->HandleReadPayload(ec2, bytes2);
        });
    return;
  }
  auto cb = current_read_callback_;
  current_read_callback_ = {};
  current_read_header_ = nullptr;
  current_read_data_ = {};
  read_header_bytes_ = 0;
  size_t bytes_read = read_payload_bytes_;
  read_payload_bytes_ = 0;
  cb(mjlib::micro::error_code(), bytes_read);
}

void UartMicroServer::AsyncWrite(const Header& header,
                                 const std::string_view& data,
                                 const Header& /*query_header*/,
                                 const SizeCallback& callback) {
  MJ_ASSERT(!current_write_callback_);
  current_write_callback_ = callback;
  write_header_buf_[0] = static_cast<uint8_t>(header.destination);
  write_header_buf_[1] = static_cast<uint8_t>(header.source);
  write_header_buf_[2] = static_cast<uint8_t>(data.size());
  write_header_buf_[3] = static_cast<uint8_t>(header.flags);
  write_payload_ptr_ = data.data();
  write_payload_size_ = data.size();
  write_payload_bytes_ = 0;
  std::string_view header_view(
      reinterpret_cast<const char*>(write_header_buf_.data()),
      write_header_buf_.size());
  uart_->AsyncWriteSome(header_view,
      [this](const mjlib::micro::error_code& ec, size_t bytes) {
        this->HandleWriteHeader(ec, bytes);
      });
}

void UartMicroServer::HandleWriteHeader(const mjlib::micro::error_code& ec,
                                        size_t /*bytes*/) {
  if (ec) {
    auto cb = current_write_callback_;
    current_write_callback_ = {};
    write_payload_ptr_ = nullptr;
    write_payload_size_ = 0;
    write_payload_bytes_ = 0;
    cb(ec, 0);
    return;
  }
  if (write_payload_size_ == 0) {
    auto cb = current_write_callback_;
    current_write_callback_ = {};
    cb(mjlib::micro::error_code(), 0);
    return;
  }
  size_t remaining = write_payload_size_ - write_payload_bytes_;
  std::string_view payload_view(
      write_payload_ptr_ + write_payload_bytes_, remaining);
  uart_->AsyncWriteSome(payload_view,
      [this](const mjlib::micro::error_code& ec2, size_t bytes2) {
        this->HandleWritePayload(ec2, bytes2);
      });
}

void UartMicroServer::HandleWritePayload(const mjlib::micro::error_code& ec,
                                         size_t bytes) {
  if (ec) {
    auto cb = current_write_callback_;
    current_write_callback_ = {};
    write_payload_ptr_ = nullptr;
    write_payload_size_ = 0;
    write_payload_bytes_ = 0;
    cb(ec, write_payload_bytes_);
    return;
  }
  write_payload_bytes_ += bytes;
  if (write_payload_bytes_ < write_payload_size_) {
    size_t remaining = write_payload_size_ - write_payload_bytes_;
    std::string_view payload_view(
        write_payload_ptr_ + write_payload_bytes_, remaining);
    uart_->AsyncWriteSome(payload_view,
        [this](const mjlib::micro::error_code& ec2, size_t bytes2) {
          this->HandleWritePayload(ec2, bytes2);
        });
    return;
  }
  auto cb = current_write_callback_;
  current_write_callback_ = {};
  write_payload_ptr_ = nullptr;
  write_payload_size_ = 0;
  size_t bytes_written = write_payload_bytes_;
  write_payload_bytes_ = 0;
  cb(mjlib::micro::error_code(), bytes_written);
}

mjlib::multiplex::MicroDatagramServer::Properties
UartMicroServer::properties() const {
  mjlib::multiplex::MicroDatagramServer::Properties props;
  props.max_size = 64;
  return props;
}

void UartMicroServer::Poll() {
  uart_->Poll();
}

}  // namespace moteus
