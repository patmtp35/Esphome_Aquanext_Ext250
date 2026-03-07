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
#define JANUS_STX       0x02
#define JANUS_ETX       0x03
#define JANUS_CR        0x0D
#define JANUS_MSGT_READ 0xC1
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
#define SETTING_ANTIBACT  0x01
#define SETTING_GREEN     0x02
#define SETTING_VOYAGE    0x04
#define SETTING_DEFROST   0x08
#define SETTING_HP_NC     0x10

class AquaNextComponent : public Component, public uart::UARTDevice {
 public:
  // ---- Sensors (temperatures) ----
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
    read_serial_();
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

  // ---- Public control API (called from lambda in YAML) ----

  void set_power(bool on) {
    uint8_t data = on ? 0x01 : 0x00;
    send_confirm_(FKT_ONOFF, &data, 1);
    ESP_LOGI("aquanext", "set_power -> %s", on ? "ON" : "OFF");
  }

  void set_mode(const std::string &mode) {
    request_function_(FKT_SETTINGS);
    delay(300);
    uint8_t s = current_settings_;
    s &= ~(SETTING_GREEN | SETTING_VOYAGE);
    if (mode == "green")       s |= SETTING_GREEN;
    else if (mode == "voyage") s |= SETTING_VOYAGE;
    else if (mode == "boost")  { /* bits already cleared */ }
    else {
      ESP_LOGW("aquanext", "Mode inconnu : %s", mode.c_str());
      return;
    }
    send_confirm_(FKT_SETTINGS, &s, 1);
    ESP_LOGI("aquanext", "set_mode -> %s (0x%02X)", mode.c_str(), s);
  }

  void set_target_temp(float temp) {
    if (temp < 30.0f || temp > 65.0f) {
      ESP_LOGW("aquanext", "Température hors limites : %.1f", temp);
      return;
    }
    uint8_t int_part  = (uint8_t)temp;
    uint8_t frac_part = (uint8_t)((temp - int_part) * 255.0f);
    uint8_t data[2]   = {int_part, frac_part};
    send_confirm_(FKT_TARGET_TEMP, data, 2);
    ESP_LOGI("aquanext", "set_target_temp -> %.2f°C", temp);
  }

  void set_antibact(bool on) {
    request_function_(FKT_SETTINGS);
    delay(300);
    uint8_t s = current_settings_;
    if (on) s |= SETTING_ANTIBACT; else s &= ~SETTING_ANTIBACT;
    send_confirm_(FKT_SETTINGS, &s, 1);
    ESP_LOGI("aquanext", "set_antibact -> %s", on ? "ON" : "OFF");
  }

  void request_all_temps() {
    const int fkts[] = {FKT_T_MAX, FKT_T_MIN, FKT_TW1, FKT_TW2,
                        FKT_T_AIR, FKT_T_EVAP, FKT_TW3, FKT_T_HP};
    for (int f : fkts) {
      request_function_(f);
      delay(150);
    }
  }

  void request_status() {
    request_function_(FKT_SETTINGS);
    delay(150);
    request_function_(FKT_SW_MB);
    delay(150);
    request_function_(FKT_HP_H);
    delay(150);
    request_function_(FKT_HE_H);
  }

 protected:
  // ---- Internal state ----
  uint8_t rx_buf_[64];
  int rx_idx_ = 0;
  bool in_frame_ = false;
  uint8_t current_settings_ = 0x00;

  // ---- Utilities ----

  static uint8_t hex_char_to_nibble_(uint8_t c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
  }

  static uint8_t hex_to_byte_(uint8_t hi, uint8_t lo) {
    return (hex_char_to_nibble_(hi) << 4) | hex_char_to_nibble_(lo);
  }

  static float decode_temp_(uint8_t hi, uint8_t lo) {
    if (hi == JANUS_TEMP_NC_HI && lo == JANUS_TEMP_NC_LO) return NAN;
    return (float)hi + (float)lo / 255.0f;
  }

  static bool check_lrc_(uint8_t *frame, int len) {
    // Structure: ... DATA | ETX(frame[-3]) | LRC(frame[-2], 1 byte binaire) | CR(frame[-1])
    // LRC = somme de MSGT(frame[1]) jusqu'a ETX(frame[-3]) inclus, modulo 256
    uint8_t sum = 0;
    for (int i = 1; i <= len - 3; i++) sum += frame[i];  // inclut ETX
    uint8_t lrc = frame[len - 2];  // LRC = 1 byte binaire
    return sum == lrc;
  }

  // ---- Serial reception ----
  void read_serial_() {
    while (available()) {

      uint8_t b = read();

      ESP_LOGD("aquanext_rx", "RX byte: 0x%02X", b);

      if (b == JANUS_STX) {
        rx_idx_ = 0;
        in_frame_ = true;
        rx_buf_[rx_idx_++] = b;
      } else if (in_frame_) {
        if (rx_idx_ < 64) rx_buf_[rx_idx_++] = b;

        if (b == JANUS_CR) {
          in_frame_ = false;
          decode_frame_(rx_buf_, rx_idx_);
          rx_idx_ = 0;
        }
      }
    }
  }

  // ---- Frame decoding ----

  void decode_frame_(uint8_t *frame, int len) {
    if (len < 12) return;
    if (frame[0] != JANUS_STX) return;
    if (frame[len - 1] != JANUS_CR) return;
    if (!check_lrc_(frame, len)) {
      ESP_LOGW("aquanext", "LRC invalide");
      return;
    }

    uint8_t msgt = frame[1];
    char fkt_str[4] = {(char)frame[2], (char)frame[3], (char)frame[4], 0};
    int fkt = strtol(fkt_str, nullptr, 16);
    // Structure: STX(0) MSGT(1) FKT(2,3,4) PAD(5,6) LEN(7,8) DATA(9..) ETX LRC CR
    int data_len = hex_to_byte_(frame[7], frame[8]);

    uint8_t data[16] = {};
    for (int i = 0; i < data_len && i < 16; i++) {
      data[i] = hex_to_byte_(frame[9 + i * 2], frame[10 + i * 2]);
    }

    if (msgt != JANUS_MSGT_READ) return;

    switch (fkt) {

      case FKT_STATUS: {
        if (data_len < 7) break;
        // DATA: TargetTemp(2) | Unknown(2) | DomeTemp(2) | Status1 | Program | Symbols | Status4
        float t_target = decode_temp_(data[0], data[1]);
        float t_dome   = decode_temp_(data[4], data[5]);
        uint8_t program = data[7];
        uint8_t symbols = data[8];

        if (!std::isnan(t_target) && temperature_target_sensor_ != nullptr)
          temperature_target_sensor_->publish_state(t_target);
        if (!std::isnan(t_dome) && temperature_dome_sensor_ != nullptr)
          temperature_dome_sensor_->publish_state(t_dome);

        if (mode_text_sensor_ != nullptr) {
          std::string mode_str;
          switch (program) {
            case 0x00: mode_str = "boost";   break;
            case 0x01: mode_str = "green";   break;
            case 0x02: mode_str = "voyage";  break;
            case 0x03: mode_str = "auto";    break;
            default:   mode_str = "unknown"; break;
          }
          mode_text_sensor_->publish_state(mode_str);
        }

        if (power_binary_sensor_)        power_binary_sensor_->publish_state(symbols & 0x01);
        if (heat_pump_active_binary_sensor_) heat_pump_active_binary_sensor_->publish_state(symbols & 0x02);
        if (heat_element_active_binary_sensor_) heat_element_active_binary_sensor_->publish_state(symbols & 0x04);
        break;
      }

      case FKT_ERRORS: {
        bool any_error = (data[1] || data[2] || data[3]);
        if (error_present_binary_sensor_) error_present_binary_sensor_->publish_state(any_error);
        ESP_LOGD("aquanext", "Errors: %02X %02X %02X", data[1], data[2], data[3]);
        break;
      }

      case FKT_SETTINGS: {
        uint8_t s = data[0];
        current_settings_ = s;
        if (setting_antibact_binary_sensor_) setting_antibact_binary_sensor_->publish_state(s & SETTING_ANTIBACT);
        if (setting_green_binary_sensor_)    setting_green_binary_sensor_->publish_state(s & SETTING_GREEN);
        if (setting_voyage_binary_sensor_)   setting_voyage_binary_sensor_->publish_state(s & SETTING_VOYAGE);
        break;
      }

      case FKT_T_MAX:  if (temperature_t_max_sensor_)  temperature_t_max_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_T_MIN:  if (temperature_t_min_sensor_)  temperature_t_min_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_TW1:    if (temperature_tw1_sensor_)    temperature_tw1_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_TW2:    if (temperature_tw2_sensor_)    temperature_tw2_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_T_AIR:  if (temperature_air_sensor_)    temperature_air_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_T_EVAP: if (temperature_evap_sensor_)   temperature_evap_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_TW3:    if (temperature_tw3_sensor_)    temperature_tw3_sensor_->publish_state(decode_temp_(data[0], data[1])); break;
      case FKT_T_HP:   if (temperature_t_hp_sensor_)   temperature_t_hp_sensor_->publish_state(decode_temp_(data[0], data[1])); break;

      case FKT_SW_MB: {
        char ver[8];
        snprintf(ver, sizeof(ver), "%02X%02X%02X", data[0], data[1], data[2]);
        if (fw_version_text_sensor_) fw_version_text_sensor_->publish_state(std::string(ver));
        break;
      }

      case FKT_HP_H: {
        uint32_t mins = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                        ((uint32_t)data[2] << 8)  | data[3];
        if (hp_hours_sensor_) hp_hours_sensor_->publish_state(mins / 60.0f);
        break;
      }

      case FKT_HE_H: {
        uint32_t mins = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                        ((uint32_t)data[2] << 8)  | data[3];
        if (he_hours_sensor_) he_hours_sensor_->publish_state(mins / 60.0f);
        break;
      }
    }
  }

  // ---- Frame building & sending ----
  void build_and_send_(uint8_t msgt, int fkt, uint8_t *data, int data_len, uint8_t len_field) {
    uint8_t frame[64];
    int idx = 0;

    // 7 bytes préambule
    for (int i = 0; i < 7; i++) frame[idx++] = 0xFF;

    frame[idx++] = JANUS_STX;
    frame[idx++] = msgt;

    // FKT ASCII HEX sur 3 caractères
    char fkt_str[4];
    snprintf(fkt_str, sizeof(fkt_str), "%03X", fkt);
    frame[idx++] = fkt_str[0];
    frame[idx++] = fkt_str[1];
    frame[idx++] = fkt_str[2];

    // padding
    frame[idx++] = '0';
    frame[idx++] = '0';

    // LEN ASCII HEX sur 2 caractères
    char len_str[3];
    snprintf(len_str, sizeof(len_str), "%02X", len_field);
    frame[idx++] = len_str[0];
    frame[idx++] = len_str[1];

    // DATA (encodée en ASCII HEX)
    for (int i = 0; i < data_len; i++) {
      char hex[3];
      snprintf(hex, sizeof(hex), "%02X", data[i]);
      frame[idx++] = hex[0];
      frame[idx++] = hex[1];
    }

    frame[idx++] = JANUS_ETX;

    // LRC binaire : somme de MSGT jusqu'à ETX inclus
    uint8_t lrc = 0;
    for (int i = 8; i < idx; i++) {
      lrc += frame[i];
    }
    frame[idx++] = lrc;

    frame[idx++] = JANUS_CR;

    // debug TX en une ligne
    char logbuf[200];
    int pos = 0;
    pos += snprintf(logbuf + pos, sizeof(logbuf) - pos, "TX raw:");
    for (int i = 0; i < idx && pos < (int) sizeof(logbuf) - 4; i++) {
      pos += snprintf(logbuf + pos, sizeof(logbuf) - pos, " %02X", frame[i]);
    }
    ESP_LOGD("aquanext", "%s", logbuf);

    write_array(frame, idx);
    flush();

    ESP_LOGD("aquanext", "TX [%d bytes] FKT=%03X LEN=%02X", idx, fkt, len_field);
  }

  void request_function_(int fkt) {
    build_and_send_(JANUS_MSGT_READ, fkt, nullptr, 0, 0);
  }

  void send_confirm_(int fkt, uint8_t *data, int data_len) {
    build_and_send_(JANUS_MSGT_WRITE, fkt, data, data_len, data_len);
  }
};

}  // namespace aquanext
}  // namespace esphome
