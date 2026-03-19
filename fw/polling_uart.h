// DMA-free polling UART for USART1 on STM32G4.
// Provides the same AsyncReadSome/AsyncWriteSome/Poll interface as
// Stm32G4AsyncUart but uses pure register-level RXNE/TXE polling,
// avoiding all DMA channel conflicts with aux_port_.
#pragma once

#include <string_view>
#include "mbed.h"
#include "mjlib/base/string_span.h"
#include "mjlib/micro/async_stream.h"
#include "fw/stm32_serial.h"

namespace moteus {

class PollingUart {
 public:
  struct Options {
    PinName tx = NC;
    PinName rx = NC;
    int baud_rate = 115200;
  };

  PollingUart(const Options& options)
      : stm32_serial_([&]() {
          Stm32Serial::Options s;
          s.tx = options.tx;
          s.rx = options.rx;
          s.baud_rate = options.baud_rate;
          return s;
        }()),
        uart_(stm32_serial_.uart()) {
    // Ensure DMA is NOT enabled for RX — we poll RXNE manually.
    uart_->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);
    // Clear any pending error flags from power-on noise.
    uart_->ICR = 0xFFFFFFFF;
    // Ensure receiver + transmitter + USART are enabled.
    uart_->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
  }

  void AsyncReadSome(const mjlib::base::string_span& data,
                     const mjlib::micro::SizeCallback& callback) {
    MJ_ASSERT(!read_callback_);
    read_data_ = data;
    read_callback_ = callback;
  }

  void AsyncWriteSome(const std::string_view& data,
                      const mjlib::micro::SizeCallback& callback) {
    MJ_ASSERT(!write_callback_);
    write_callback_ = callback;
    write_ptr_ = reinterpret_cast<const uint8_t*>(data.data());
    write_remaining_ = data.size();
    write_total_ = data.size();
  }

  void Poll() {
    // Clear UART error flags so the peripheral doesn't lock up.
    if (uart_->ISR & (USART_ISR_ORE | USART_ISR_FE |
                      USART_ISR_NE  | USART_ISR_PE)) {
      uart_->ICR = (USART_ICR_ORECF | USART_ICR_FECF |
                    USART_ICR_PECF  | USART_ICR_NECF);
    }

    // --- RX: read available bytes into the pending read buffer ---
    if (read_callback_) {
      ssize_t count = 0;
      while (count < read_data_.size() && (uart_->ISR & (1 << 5))) {
        read_data_.data()[count++] = uart_->RDR & 0xFF;
      }
      if (count > 0) {
        auto cb = read_callback_;
        read_callback_ = {};
        read_data_ = {};
        cb({}, count);
      }
    } else {
      // Drain and discard if nobody is listening, to prevent ORE.
      while (uart_->ISR & (1 << 5)) {
        (void)uart_->RDR;
      }
    }

    // --- TX: write as many bytes as the hardware will accept ---
    if (write_callback_ && write_remaining_ > 0) {
      while (write_remaining_ > 0 && (uart_->ISR & (1 << 7))) {
        uart_->TDR = *write_ptr_++;
        write_remaining_--;
      }
      if (write_remaining_ == 0) {
        auto cb = write_callback_;
        write_callback_ = {};
        cb({}, write_total_);
      }
    }
  }

 private:
  Stm32Serial stm32_serial_;
  USART_TypeDef* const uart_;

  mjlib::micro::SizeCallback read_callback_;
  mjlib::base::string_span read_data_{};

  mjlib::micro::SizeCallback write_callback_;
  const uint8_t* write_ptr_ = nullptr;
  size_t write_remaining_ = 0;
  size_t write_total_ = 0;
};

}  // namespace moteus
