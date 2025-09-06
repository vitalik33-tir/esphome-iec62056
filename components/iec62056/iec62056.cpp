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
      this->send_frame_();
      break;

    case SET_BAUD_RATE:
      ESP_LOGD(TAG, "Switching to new baud rate %u bps ('%c')", new_baudrate, baud_rate_char);
      update_baudrate_(new_baudrate);
      set_next_state_(WAIT_FOR_STX);
      break;

    case WAIT_FOR_STX:  // wait for STX
      report_state_();

      // If the loop is called not very often, data can be overwritten.
      // In that case just increase UART buffer size
      if (receive_frame_() >= 1) {
        if (STX == in_buf_[0]) {
          ESP_LOGD(TAG, "Meter started readout transmission");
          set_next_state_(READ_DATA_BLOCK);
        }
      }
      break;

    case READ_DATA_BLOCK:
      if (receive_frame_() >= 1) {
        this->process_data_block_();
      }
      break;

    case PROCESS_OBIS:
      this->publish_sensors_();
      break;

    case COMPLETE:
      ESP_LOGD(TAG, "Readout completed successfully");
      set_next_state_(IDLE);
      break;

    case IDLE:
    default:
      break;
  }
}

void IEC62056Component::send_frame_() {
  this->iuart_->write(out_buf_, data_out_size_);
  this->iuart_->flush();
}

size_t IEC62056Component::receive_frame_() {
  size_t n = 0;
  while (this->iuart_->available()) {
    int c = this->iuart_->read();
    if (c < 0) break;
    if (n < sizeof(in_buf_)) in_buf_[n++] = (uint8_t)c;
  }
  return n;
}

}  // namespace iec62056
}  // namespace esphome
