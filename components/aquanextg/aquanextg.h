#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace aquanext {

// ============================================================
// Janus2 Protocol Constants
// ============================================================
#define JANUS_STX        0x02
#define JANUS_ETX        0x03
#define JANUS_CR         0x0D
#define JANUS_MSGT_READ  0xC1
#define JANUS_MSGT_WRITE 0xC2
#define JANUS_TEMP_NC_HI 0xFE
#define JANUS_TEMP_NC_LO 0x7F

// Function IDs
#define FKT_TARGET_TEMP  0x000
#define FKT_ONOFF        0x001
#define FKT_STATUS       0x003
#define FKT_ERRORS       0x004
#define FKT_T_MAX        0x005
#define FKT_T_MIN        0x006
#define FKT_SETTINGS     0x007
#define FKT_SW_MB        0x008
#define FKT_TW1          0x00A
#define FKT_TW2          0x00B
#define FKT_T_AIR        0x00C
#define FKT_T_EVAP       0x00D
#define FKT_TW3          0x00E
#define FKT_HP_H         0x010
#define FKT_HE_H         0x011
#define FKT_T_HP         0x012

// Settings bitmask
#define SETTING_ANTIBACT 0x01
#define SETTING_GREEN    0x02
#define SETTING_VOYAGE   0x04
#define SETTING_DEFROST  0x08
#define SETTING_HP_NC    0x10

class AquaNextComponent : public Component, public uart::UARTDevice {
 public:
  // ---- Sensors ----
  SUB_SENSOR(temperature_target)
  SUB_SENSOR(temperature_dome)
  SUB_SENSOR(temperature_air)
  SUB_SENSOR(temperature_evap)
  SUB_SENSOR(temperature_tw1)
  SUB_SENSOR(temperature_tw2)
  SUB_SENSOR(temperature_tw3)
  SUB_SENSOR(temperature_t_hp)
  SUB_SENSOR(temperature_t_max)
  SUB_SENSOR(temperature_t_min)
  SUB_SENSOR(hp_hours)
  SUB_SENSOR(he_hours)

  // ---- Text sensors ----
  SUB_TEXT_SENSOR(mode)
  SUB_TEXT_SENSOR(fw_version)

  // ---- Binary sensors ----
  SUB_BINARY_SENSOR(power)
  SUB_BINARY_SENSOR(heat_pump_active)
  SUB_BINARY_SENSOR(heat_element_active)
  SUB_BINARY_SENSOR(error_present)
  SUB_BINARY_SENSOR(setting_antibact)
  SUB_BINARY_SENSOR(setting_green)
  SUB_BINARY_SENSOR(setting_voyage)

  void setup() override {}

  void loop() override {
    process_tx_queue_();
    read_serial_();
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

  // ---- Public control API ----

  void set_power(bool on) {
    uint8_t data = on ? 0x01 : 0x00;
    send_confirm_(FKT_ONOFF, &data, 1);
  }

  void set_mode(const std::string &mode) {
    uint8_t s = current_settings_;
    s &= ~(SETTING_GREEN | SETTING_VOYAGE);
    if (mode == "green")        s |= SETTING_GREEN;
    else if (mode == "voyage")  s |= SETTING_VOYAGE;
    send_confirm_(FKT_SETTINGS, &s, 1);
  }

  void set_target_temp(float temp) {
    if (temp < 30.0f || temp > 65.0f) return;
    uint8_t int_part  = (uint8_t)temp;
    uint8_t frac_part = (uint8_t)((temp - int_part) * 255.0f);
    // DOC: HI=frac, LO=int
    uint8_t data[2]   = {frac_part, int_part};
    send_confirm_(FKT_TARGET_TEMP, data, 2);
  }

  void request_all_temps() {
    const int fkts[] = {FKT_T_MAX, FKT_T_MIN, FKT_TW1, FKT_TW2,
                        FKT_T_AIR, FKT_T_EVAP, FKT_TW3, FKT_T_HP};
    for (int f : fkts) request_function_(f);
  }

  void request_function(int fkt) {
    enqueue_read_(fkt);
  }

 protected:
  uint8_t rx_buf_[64];
  int     rx_idx_       = 0;
  bool    in_frame_     = false;
  int     expected_len_ = 0;
  uint8_t current_settings_ = 0x00;

  static const int TX_QUEUE_SIZE = 16;
  int     tx_queue_[TX_QUEUE_SIZE];
  int     tx_queue_head_ = 0;
  int     tx_queue_tail_ = 0;
  uint32_t tx_last_ms_   = 0;
  static const uint32_t TX_MIN_DELAY_MS = 600;

  void enqueue_read_(int fkt) {
    int next = (tx_queue_tail_ + 1) % TX_QUEUE_SIZE;
    if (next != tx_queue_head_) {
      tx_queue_[tx_queue_tail_] = fkt;
      tx_queue_tail_ = next;
    }
  }

  void process_tx_queue_() {
    if (tx_queue_head_ == tx_queue_tail_) return;
    uint32_t now = millis();
    if (now - tx_last_ms_ < TX_MIN_DELAY_MS) return;
    int fkt = tx_queue_[tx_queue_head_];
    tx_queue_head_ = (tx_queue_head_ + 1) % TX_QUEUE_SIZE;
    tx_last_ms_ = now;
    build_read_(fkt);
  }

  static uint8_t hex_char_to_nibble_(uint8_t c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
  }

  static uint8_t hex_to_byte_(uint8_t hi, uint8_t lo) {
    return (hex_char_to_nibble_(hi) << 4) | hex_char_to_nibble_(lo);
  }

  // CORRECTION: DOC Ariston dit HI = Decimales, LO = Entier
  static float decode_temp_(uint8_t hi, uint8_t lo) {
    if (hi == JANUS_TEMP_NC_HI && lo == JANUS_TEMP_NC_LO) return NAN;
    return (float)lo + (float)hi / 255.0f;
  }

  static bool check_lrc_(uint8_t *frame, int len) {
    uint8_t sum = 0;
    for (int i = 1; i <= len - 2; i++) sum += (frame[i] & 0x7F);
    return (sum & 0x7F) == (frame[len - 1] & 0x7F);
  }

  void read_serial_() {
    while (available()) {
      uint8_t raw = read();
      if (raw == 0xFF) continue;
      if (raw == JANUS_STX) {
        rx_idx_ = 0; in_frame_ = true; expected_len_ = 0;
        rx_buf_[rx_idx_++] = JANUS_STX; continue;
      }
      if (!in_frame_) continue;
      if (rx_idx_ >= 63) { in_frame_ = false; rx_idx_ = 0; continue; }
      rx_buf_[rx_idx_++] = raw;

      if (rx_idx_ == 9) {
        uint8_t data_len = hex_to_byte_(rx_buf_[7] & 0x7F, rx_buf_[8] & 0x7F);
        expected_len_ = 11 + (data_len > 16 ? 0 : data_len) * 2;
      }

      if (rx_idx_ >= 11 && (rx_buf_[rx_idx_ - 2] & 0x7F) == JANUS_ETX) {
        decode_frame_(rx_buf_, rx_idx_);
        in_frame_ = false; rx_idx_ = 0;
      }
    }
  }

  void decode_frame_(uint8_t *frame, int len) {
    if (len < 11 || frame[0] != JANUS_STX) return;
    
    // On affiche la trame brute pour comprendre ce qui arrive
    char logbuf[128];
    int pos = 0;
    for (int i = 0; i < len && pos < 120; i++)
      pos += snprintf(logbuf + pos, 128 - pos, "%02X ", frame[i]);
    
    if (!check_lrc_(frame, len)) {
       ESP_LOGD("aquanext", "LRC Invalide sur : %s", logbuf);
       // ON CONTINUE QUAND MÊME LE DÉCODAGE
    } else {
       ESP_LOGD("aquanext", "LRC OK sur : %s", logbuf);
    }

    uint8_t msgt = frame[1] & 0x7F;
    char fkt_str[4] = {(char)(frame[2]&0x7F), (char)(frame[3]&0x7F), (char)(frame[4]&0x7F), 0};
    int fkt = strtol(fkt_str, nullptr, 16);
    int d_len = hex_to_byte_(frame[7]&0x7F, frame[8]&0x7F);

    // Si d_len est délirant, on arrête
    if (d_len > 16) return;

    uint8_t data[16] = {};
    for (int i = 0; i < d_len; i++) {
      data[i] = hex_to_byte_(frame[9+i*2]&0x7F, frame[10+i*2]&0x7F);
    }

    // Log des infos extraites
    ESP_LOGD("aquanext", "Decodage: MSGT=%02X FKT=%03X Len=%d", msgt, fkt, d_len);

    // ... (le reste du switch fkt ne change pas)

    if (msgt != JANUS_MSGT_READ) return;

    switch (fkt) {
      case FKT_STATUS:
        if (d_len < 9) break;
        if (temperature_target_sensor_) temperature_target_sensor_->publish_state(decode_temp_(data[0], data[1]));
        if (temperature_dome_sensor_)   temperature_dome_sensor_->publish_state(decode_temp_(data[4], data[5]));
        if (mode_text_sensor_) {
          const char* modes[] = {"boost", "green", "voyage", "auto"};
          mode_text_sensor_->publish_state(data[7] < 4 ? modes[data[7]] : "unknown");
        }
        if (power_binary_sensor_) power_binary_sensor_->publish_state(data[8] & 0x01);
        if (heat_pump_active_binary_sensor_) heat_pump_active_binary_sensor_->publish_state(data[8] & 0x02);
        if (heat_element_active_binary_sensor_) heat_element_active_binary_sensor_->publish_state(data[8] & 0x04);
        break;

      case FKT_SETTINGS:
        current_settings_ = data[0];
        if (setting_antibact_binary_sensor_) setting_antibact_binary_sensor_->publish_state(data[0] & SETTING_ANTIBACT);
        if (setting_green_binary_sensor_)    setting_green_binary_sensor_->publish_state(data[0] & SETTING_GREEN);
        if (setting_voyage_binary_sensor_)   setting_voyage_binary_sensor_->publish_state(data[0] & SETTING_VOYAGE);
        break;

      case FKT_T_AIR:  if (temperature_air_sensor_)  temperature_air_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_T_EVAP: if (temperature_evap_sensor_) temperature_evap_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_TW1:    if (temperature_tw1_sensor_)  temperature_tw1_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_TW2:    if (temperature_tw2_sensor_)  temperature_tw2_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_TW3:    if (temperature_tw3_sensor_)  temperature_tw3_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_T_HP:   if (temperature_t_hp_sensor_) temperature_t_hp_sensor_->publish_state(decode_temp_(data[0], data[1])); break;

      // CORRECTION: DOC dit format 4 octets Little Endian pour les minutes
      case FKT_HP_H:
      case FKT_HE_H: {
        uint32_t mins = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
        float hours = mins / 60.0f;
        if (fkt == FKT_HP_H && hp_hours_sensor_) hp_hours_sensor_->publish_state(hours);
        if (fkt == FKT_HE_H && he_hours_sensor_) he_hours_sensor_->publish_state(hours);
        break;
      }
    }
  }

  void build_read_(int fkt) {
    uint8_t frame[24]; int idx = 0;
    for (int i = 0; i < 7; i++) frame[idx++] = 0xFF;
    frame[idx++] = JANUS_STX;
    frame[idx++] = JANUS_MSGT_READ;
    char f_s[4]; snprintf(f_s, 4, "%03X", fkt);
    frame[idx++] = f_s[0]; frame[idx++] = f_s[1]; frame[idx++] = f_s[2];
    frame[idx++] = '0'; frame[idx++] = '0'; // PAD
    frame[idx++] = '0'; frame[idx++] = '1'; // LEN
    frame[idx++] = JANUS_ETX;
    uint8_t lrc = 0; for (int i = 8; i < idx; i++) lrc += (frame[i] & 0x7F);
    frame[idx++] = (lrc & 0x7F);
    frame[idx++] = JANUS_CR;
    write_array(frame, idx);
  }

  void send_confirm_(int fkt, uint8_t *data, int data_len) {
    uint8_t frame[40]; int idx = 0;
    for (int i = 0; i < 7; i++) frame[idx++] = 0xFF;
    frame[idx++] = JANUS_STX;
    frame[idx++] = JANUS_MSGT_WRITE;
    frame[idx++] = '0' + data_len;
    char f_s[4]; snprintf(f_s, 4, "%03X", fkt);
    frame[idx++] = f_s[0]; frame[idx++] = f_s[1]; frame[idx++] = f_s[2];
    frame[idx++] = '0'; frame[idx++] = '0';
    for (int i = 0; i < data_len; i++) {
        char h[3]; snprintf(h, 3, "%02X", data[i]);
        frame[idx++] = h[0]; frame[idx++] = h[1];
    }
    frame[idx++] = JANUS_ETX;
    uint8_t lrc = 0; for (int i = 8; i < idx; i++) lrc += (frame[i] & 0x7F);
    frame[idx++] = (lrc & 0x7F);
    frame[idx++] = JANUS_CR;
    write_array(frame, idx);
  }
};

} // namespace aquanext
} // namespace esphome