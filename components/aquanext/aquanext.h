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
    ESP_LOGI("aquanext", "set_power -> %s", on ? "ON" : "OFF");
  }

  void set_mode(const std::string &mode) {
    // Note: pas de delay() ici - on utilise current_settings_ deja connu
    uint8_t s = current_settings_;
    s &= ~(SETTING_GREEN | SETTING_VOYAGE);
    if (mode == "green")        s |= SETTING_GREEN;
    else if (mode == "voyage")  s |= SETTING_VOYAGE;
    else if (mode == "boost")   { /* bits already cleared */ }
    else {
      ESP_LOGW("aquanext", "Mode inconnu : %s", mode.c_str());
      return;
    }
    send_confirm_(FKT_SETTINGS, &s, 1);
    ESP_LOGI("aquanext", "set_mode -> %s (0x%02X)", mode.c_str(), s);
  }

  void set_target_temp(float temp) {
    if (temp < 30.0f || temp > 65.0f) {
      ESP_LOGW("aquanext", "Temperature hors limites : %.1f", temp);
      return;
    }
    uint8_t int_part  = (uint8_t)temp;
    uint8_t frac_part = (uint8_t)((temp - int_part) * 255.0f);
    uint8_t data[2]   = {int_part, frac_part};
    send_confirm_(FKT_TARGET_TEMP, data, 2);
    ESP_LOGI("aquanext", "set_target_temp -> %.2f C", temp);
  }

  void set_antibact(bool on) {
    uint8_t s = current_settings_;
    if (on) s |= SETTING_ANTIBACT; else s &= ~SETTING_ANTIBACT;
    send_confirm_(FKT_SETTINGS, &s, 1);
    ESP_LOGI("aquanext", "set_antibact -> %s", on ? "ON" : "OFF");
  }

  void request_all_temps() {
    // Pas de delay() - on envoie une requete a la fois depuis le YAML interval
    const int fkts[] = {FKT_T_MAX, FKT_T_MIN, FKT_TW1, FKT_TW2,
                        FKT_T_AIR, FKT_T_EVAP, FKT_TW3, FKT_T_HP};
    for (int f : fkts) {
      request_function_(f);
    }
  }

  void request_status() {
    request_function_(FKT_SETTINGS);
    request_function_(FKT_SW_MB);
    request_function_(FKT_HP_H);
    request_function_(FKT_HE_H);
  }

  // Requete individuelle - a appeler depuis YAML interval pour espacer les requetes
  void request_function(int fkt) {
    request_function_(fkt);
  }

 protected:
  // ---- Internal state ----
  uint8_t rx_buf_[64];
  int     rx_idx_       = 0;
  bool    in_frame_     = false;
  int     expected_len_ = 0;
  uint8_t current_settings_ = 0x00;

  // ---- Echo TX suppression ----
  // Sur bus Janus2 half-duplex, on recoit notre propre TX en echo
  // On ignore les N premiers octets apres chaque TX
  int     echo_skip_    = 0;  // nb d'octets TX restant a ignorer

  // ---- TX queue (evite les envois simultanees qui corrompent les reponses) ----
  static const int TX_QUEUE_SIZE = 16;
  int     tx_queue_[TX_QUEUE_SIZE];
  int     tx_queue_head_ = 0;
  int     tx_queue_tail_ = 0;
  uint32_t tx_last_ms_   = 0;
  static const uint32_t TX_MIN_DELAY_MS = 600;  // delai minimum entre deux TX

  void enqueue_read_(int fkt) {
    int next = (tx_queue_tail_ + 1) % TX_QUEUE_SIZE;
    if (next == tx_queue_head_) {
      ESP_LOGW("aquanext", "TX queue pleine, FKT=%03X ignore", fkt);
      return;
    }
    tx_queue_[tx_queue_tail_] = fkt;
    tx_queue_tail_ = next;
  }

  void process_tx_queue_() {
    if (tx_queue_head_ == tx_queue_tail_) return;  // queue vide
    uint32_t now = millis();
    if (now - tx_last_ms_ < TX_MIN_DELAY_MS) return;  // trop tot
    int fkt = tx_queue_[tx_queue_head_];
    tx_queue_head_ = (tx_queue_head_ + 1) % TX_QUEUE_SIZE;
    tx_last_ms_ = now;
    build_read_(fkt);
  }

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
    // LRC = somme des octets BRUTS de frame[1] (MSGT) jusqu'a frame[len-2] (ETX)
    // Le bit 7 peut etre a 1 sur n'importe quel octet (bit de parité MARK du Janus2)
    // On compare modulo 128 (& 0x7F) pour absorber ce bit parasite sur le LRC lui-meme
    uint8_t sum = 0;
    for (int i = 1; i <= len - 2; i++) sum += frame[i];
    return (sum & 0x7F) == (frame[len - 1] & 0x7F);
  }

  // ---- Serial reception ----
  // Tout est stocke masque (& 0x7F) sauf detection STX sur raw==0x02 strict
  // Longueur calculee depuis LEN (pos 7,8) -> cloture exacte sans dependre du CR
  void read_serial_() {
    while (available()) {
      uint8_t raw = read();
      // Ignore les echos de notre propre TX (bus half-duplex)
      if (echo_skip_ > 0) {
        echo_skip_--;
        continue;
      }

      // Ignore les 0xFF : ce sont les echos de notre propre preambule (bus half-duplex)
      if (raw == 0xFF) continue;

      ESP_LOGD("aquanext_rx", "RX byte: 0x%02X", raw);

      // STX strict : uniquement 0x02 brut (0x82 = MSGT corrompu, pas un STX)
      if (raw == JANUS_STX) {
        rx_idx_       = 0;
        in_frame_     = true;
        expected_len_ = 0;
        rx_buf_[rx_idx_++] = JANUS_STX;
        continue;
      }

      if (!in_frame_) continue;

      // Securite anti-debordement
      if (rx_idx_ >= 63) {
        ESP_LOGW("aquanext", "Buffer overflow, reset");
        in_frame_ = false;
        rx_idx_   = 0;
        continue;
      }

      // Stocke l'octet BRUT - masque 0x7F applique uniquement a l'interpretation
      rx_buf_[rx_idx_++] = raw;

      // Calcule longueur attendue des que LEN est disponible (pos 7 et 8)
      if (rx_idx_ == 9) {
        uint8_t len_hi = rx_buf_[7] & 0x7F;
        uint8_t len_lo = rx_buf_[8] & 0x7F;
        // Securite: LEN ASCII hex valide uniquement ('0'-'9','A'-'F')
        bool len_valid = ((len_hi >= '0' && len_hi <= '9') || (len_hi >= 'A' && len_hi <= 'F'))
                      && ((len_lo >= '0' && len_lo <= '9') || (len_lo >= 'A' && len_lo <= 'F'));
        uint8_t data_len = len_valid ? hex_to_byte_(len_hi, len_lo) : 0;
        // Sanity check: data_len > 16 est suspect (trame corrompue)
        // On plafonne a 16 octets de data max (32 chars ASCII)
        if (data_len > 16) {
          ESP_LOGW("aquanext", "data_len suspect (%d), force a 0", data_len);
          data_len = 0;
        }
        expected_len_ = 11 + data_len * 2;
        ESP_LOGD("aquanext", "Longueur trame attendue: %d (data=%d)", expected_len_, data_len);
      }

      // Cloture sur ETX+LRC : si on voit ETX a la position attendue ou apres pos 9
      // Cela gere le cas ou le ballon repond avec moins de data que LEN l'indique
      if (rx_idx_ >= 11 && (rx_buf_[rx_idx_ - 2] & 0x7F) == JANUS_ETX) {
        // ETX detecte en avant-derniere position -> LRC est le dernier octet recu
        in_frame_     = false;
        int frame_len = rx_idx_;
        rx_idx_       = 0;
        expected_len_ = 0;
        decode_frame_(rx_buf_, frame_len);
        continue;
      }

      // Cloture aussi par longueur calculee (securite)
      if (expected_len_ > 0 && rx_idx_ >= expected_len_) {
        in_frame_     = false;
        int frame_len = rx_idx_;
        rx_idx_       = 0;
        expected_len_ = 0;
        decode_frame_(rx_buf_, frame_len);
      }
    }
  }

  // ---- Frame decoding ----
  // frame[] : STX MSGT FKT(3) PAD(2) LEN(2) DATA ETX LRC  — octets BRUTS
  // Le masque & 0x7F est applique octet par octet au moment de l'interpretation
  void decode_frame_(uint8_t *frame, int len) {
    if (len < 11) {
      ESP_LOGW("aquanext", "Trame trop courte: %d", len);
      return;
    }
    if (frame[0] != JANUS_STX) return;

    // LRC verifie mais non bloquant : JoHu (auteur doc Janus2) confirme
    // que le calcul LRC n'a jamais ete valide dans ses tests non plus.
    // On logue un warning mais on decode quand meme.
    if (!check_lrc_(frame, len)) {
      char logbuf[200];
      int pos = 0;
      for (int i = 0; i < len && pos < 190; i++)
        pos += snprintf(logbuf + pos, sizeof(logbuf) - pos, "%02X ", frame[i]);
      ESP_LOGD("aquanext", "LRC invalide (decode quand meme): %s", logbuf);
    }

    uint8_t msgt    = frame[1] & 0x7F;
    char fkt_str[4] = {
      (char)(frame[2] & 0x7F),
      (char)(frame[3] & 0x7F),
      (char)(frame[4] & 0x7F),
      0
    };
    int  fkt      = strtol(fkt_str, nullptr, 16);
    int  data_len = hex_to_byte_(frame[7] & 0x7F, frame[8] & 0x7F);

    uint8_t data[16] = {};
    for (int i = 0; i < data_len && i < 16; i++) {
      data[i] = hex_to_byte_(frame[9 + i * 2] & 0x7F, frame[10 + i * 2] & 0x7F);
    }

    ESP_LOGD("aquanext", "Frame OK: MSGT=0x%02X FKT=%03X data_len=%d", msgt, fkt, data_len);

    if (msgt != JANUS_MSGT_READ) return;

    switch (fkt) {

      case FKT_STATUS: {
        if (data_len < 9) break;
        float t_target  = decode_temp_(data[0], data[1]);
        float t_dome    = decode_temp_(data[4], data[5]);
        uint8_t program = data[7];
        uint8_t symbols = data[8];

        if (!std::isnan(t_target) && temperature_target_sensor_)
          temperature_target_sensor_->publish_state(t_target);
        if (!std::isnan(t_dome) && temperature_dome_sensor_)
          temperature_dome_sensor_->publish_state(t_dome);

        if (mode_text_sensor_) {
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

        if (power_binary_sensor_)
          power_binary_sensor_->publish_state(symbols & 0x01);
        if (heat_pump_active_binary_sensor_)
          heat_pump_active_binary_sensor_->publish_state(symbols & 0x02);
        if (heat_element_active_binary_sensor_)
          heat_element_active_binary_sensor_->publish_state(symbols & 0x04);
        break;
      }

      case FKT_ERRORS: {
        bool any_error = (data[1] || data[2] || data[3]);
        if (error_present_binary_sensor_)
          error_present_binary_sensor_->publish_state(any_error);
        ESP_LOGD("aquanext", "Errors: %02X %02X %02X", data[1], data[2], data[3]);
        break;
      }

      case FKT_SETTINGS: {
        uint8_t s = data[0];
        current_settings_ = s;
        if (setting_antibact_binary_sensor_) setting_antibact_binary_sensor_->publish_state(s & SETTING_ANTIBACT);
        if (setting_green_binary_sensor_)    setting_green_binary_sensor_->publish_state(s & SETTING_GREEN);
        if (setting_voyage_binary_sensor_)   setting_voyage_binary_sensor_->publish_state(s & SETTING_VOYAGE);
        ESP_LOGD("aquanext", "Settings: 0x%02X", s);
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
        ESP_LOGD("aquanext", "FW: %s", ver);
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

      default:
        ESP_LOGD("aquanext", "FKT inconnu: %03X", fkt);
        break;
    }
  }

  // ---- Frame building & sending ----
  // READ  : FF*7 + STX MSGT FKT[3] PAD[2] LEN[2 ASCII] ETX LRC CR
  //         LEN='01' pour demander une valeur
  // WRITE : FF*7 + STX MSGT LEN[1 ASCII] FKT[3] PAD[2] DATA[LEN*2 ASCII] ETX LRC CR
  //         LEN='1','2'... (1 seul char ASCII = nb d'octets data)

  void build_read_(int fkt) {
    uint8_t frame[80];
    int idx = 0;

    for (int i = 0; i < 7; i++) frame[idx++] = 0xFF;
    frame[idx++] = JANUS_STX;
    frame[idx++] = JANUS_MSGT_READ;

    char fkt_str[4];
    snprintf(fkt_str, sizeof(fkt_str), "%03X", fkt);
    frame[idx++] = fkt_str[0];
    frame[idx++] = fkt_str[1];
    frame[idx++] = fkt_str[2];

    frame[idx++] = '0';  // PAD
    frame[idx++] = '0';
    frame[idx++] = '0';  // LEN = '01'
    frame[idx++] = '1';

    frame[idx++] = JANUS_ETX;

    uint8_t lrc = 0;
    for (int i = 8; i < idx; i++) lrc += frame[i];
    frame[idx++] = lrc;
    frame[idx++] = JANUS_CR;

    char logbuf[200];
    int pos = snprintf(logbuf, sizeof(logbuf), "TX READ:");
    for (int i = 7; i < idx && pos < (int)sizeof(logbuf) - 4; i++)
      pos += snprintf(logbuf + pos, sizeof(logbuf) - pos, " %02X", frame[i]);
    ESP_LOGD("aquanext", "%s FKT=%03X", logbuf, fkt);

    write_array(frame, idx);
    flush();
    // echo_skip_ desactive : l'ADUM1201 isole le bus, pas d'echo TX en RX
  }

  void build_write_(int fkt, uint8_t *data, int data_len) {
    uint8_t frame[80];
    int idx = 0;

    for (int i = 0; i < 7; i++) frame[idx++] = 0xFF;
    frame[idx++] = JANUS_STX;
    frame[idx++] = JANUS_MSGT_WRITE;

    // WRITE: LEN vient AVANT FKT (1 seul char ASCII)
    frame[idx++] = '0' + data_len;  // '1', '2'...

    char fkt_str[4];
    snprintf(fkt_str, sizeof(fkt_str), "%03X", fkt);
    frame[idx++] = fkt_str[0];
    frame[idx++] = fkt_str[1];
    frame[idx++] = fkt_str[2];

    frame[idx++] = '0';  // PAD
    frame[idx++] = '0';

    for (int i = 0; i < data_len; i++) {
      char hex[3];
      snprintf(hex, sizeof(hex), "%02X", data[i]);
      frame[idx++] = hex[0];
      frame[idx++] = hex[1];
    }

    frame[idx++] = JANUS_ETX;

    uint8_t lrc = 0;
    for (int i = 8; i < idx; i++) lrc += frame[i];
    frame[idx++] = lrc;
    frame[idx++] = JANUS_CR;

    char logbuf[200];
    int pos = snprintf(logbuf, sizeof(logbuf), "TX WRITE:");
    for (int i = 7; i < idx && pos < (int)sizeof(logbuf) - 4; i++)
      pos += snprintf(logbuf + pos, sizeof(logbuf) - pos, " %02X", frame[i]);
    ESP_LOGD("aquanext", "%s FKT=%03X", logbuf, fkt);

    write_array(frame, idx);
    flush();
    // echo_skip_ desactive : l'ADUM1201 isole le bus, pas d'echo TX en RX
  }

  void request_function_(int fkt) {
    enqueue_read_(fkt);
  }

  void send_confirm_(int fkt, uint8_t *data, int data_len) {
    build_write_(fkt, data, data_len);
  }
};

}  // namespace aquanext
}  // namespace esphome
