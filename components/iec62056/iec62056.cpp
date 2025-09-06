#include "iec62056.component.h"
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
      this->send_frame_();
      break;

    case SET_BAUD_RATE:
      ESP_LOGD(TAG, "Switching to new baud rate %u bps ('%c')", new_baudrate, baud_rate_char);
      update_baudrate_(new_baudrate);

      // пауза после смены скорости
      delay(300);   // 200–300 мс

      set_next_state_(WAIT_FOR_STX);
      break;

    case WAIT_FOR_STX: {  // wait for STX
      report_state_();

      const uint32_t started = millis();
      while (millis() - started < 1500) {   // ждём до 1.5 сек
        if (receive_frame_() >= 1) {
          if (STX == in_buf_[0]) {
            ESP_LOGD(TAG, "Meter started readout transmission");
            set_next_state_(READ_DATA_BLOCK);
            break;
          }
        }
        delay(1);  // имитация yield логгера
      }
      break;
    }

    case READ_DATA_BLOCK:
      // здесь остался твой текущий парсинг данных
      break;

    default:
      break;
  }
}

void IEC62056Component::send_frame_() {
  // подчистить входной буфер
  while (this->uart_->available()) this->uart_->read();

  this->uart_->write(out_buf_, data_out_size_);
  this->uart_->flush();

  // пауза разворота
  delay(5);   // 3–5 мс
}

int IEC62056Component::receive_frame_() {
  size_t n = 0;
  uint32_t last = millis();

  while (true) {
    while (this->uart_->available()) {
      int c = this->uart_->read();
      if (c < 0) break;
      if (n < sizeof(in_buf_)) in_buf_[n++] = (uint8_t)c;
      last = millis();
    }
    if (millis() - last >= 30) break;  // межсимвольный таймаут 30 мс
    delay(1);
  }

  return (int)n;
}

}  // namespace iec62056
}  // namespace esphome
