#include "iec62056.h"
#include "esphome/core/log.h"

namespace esphome {
namespace iec62056 {

static const char *const TAG = "iec62056";

void IEC62056Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IEC62056 component...");
  this->set_next_state_(INIT_HANDSHAKE);
}

void IEC62056Component::loop() {
  switch (this->state_) {
    case INIT_HANDSHAKE:
      this->send_request_();
      break;

    case SET_BAUD_RATE:
      ESP_LOGD(TAG, "Switching to new baud rate %u bps ('%c')", this->new_baudrate_, this->baud_rate_char_);
      this->update_baudrate_(this->new_baudrate_);
      // пауза после переключения скорости
      this->wait_(this->post_baud_switch_ms_, POST_BAUD_SWITCH);
      this->set_next_state_(WAIT_FOR_STX);
      break;

    case WAIT_FOR_STX: {
      ESP_LOGD(TAG, "Waiting STX from meter...");
      const uint32_t started = millis();
      while (millis() - started < this->stx_timeout_ms_) {
        if (this->receive_frame_(1, this->interchar_timeout_ms_) >= 1) {
          if (this->in_buf_[0] == STX) {
            ESP_LOGD(TAG, "Meter started readout transmission");
            this->set_next_state_(READ_DATA_BLOCK);
            return;
          } else {
            // не STX, подчистим
            while (this->uart_->available()) this->uart_->read();
          }
        }
        delay(1);  // имитация yield логгера
      }
      ESP_LOGE(TAG, "No transmission from meter (timeout waiting STX)");
      this->set_next_state_(RETRY_OR_FAIL);
      break;
    }

    case READ_DATA_BLOCK:
      // сюда вставьте ваш разбор данных как раньше
      break;

    default:
      break;
  }
}

void IEC62056Component::send_request_() {
  // очистить вход перед запросом
  while (this->uart_->available()) this->uart_->read();

  this->uart_->write(this->out_buf_, this->data_out_size_);
  this->uart_->flush();

  // пауза разворота
  this->wait_(this->turnaround_delay_ms_, AFTER_TX_TURNAROUND);
}

int IEC62056Component::receive_frame_(size_t min_bytes, uint16_t interchar_timeout_ms) {
  const uint32_t t0 = millis();
  while (this->uart_->available() < (int)min_bytes) {
    if (millis() - t0 > interchar_timeout_ms) return 0;
    delay(1);
  }

  size_t n = 0;
  uint32_t last = millis();
  while (true) {
    while (this->uart_->available()) {
      int c = this->uart_->read();
      if (c < 0) break;
      if (n < sizeof(this->in_buf_)) this->in_buf_[n++] = (uint8_t)c;
      last = millis();
    }
    if (millis() - last >= interchar_timeout_ms) break;
    delay(1);
  }
  return (int)n;
}

// --- дефолты таймингов ---
IEC62056Component::IEC62056Component() {
  this->turnaround_delay_ms_ = 4;     // пауза после send_frame_
  this->post_baud_switch_ms_ = 250;   // после update_baudrate_
  this->stx_timeout_ms_ = 1500;       // ожидание STX
  this->interchar_timeout_ms_ = 30;   // межсимвольный таймаут
}

}  // namespace iec62056
}  // namespace esphome
