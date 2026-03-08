#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace aquanext {

#define JANUS_STX        0x02
#define JANUS_ETX        0x03
#define JANUS_CR         0x0D
#define JANUS_MSGT_READ  0xC1
#define JANUS_MSGT_WRITE 0xC2
#define JANUS_TEMP_NC_HI 0xFE
#define JANUS_TEMP_NC_LO 0x7F

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

#define SETTING_ANTIBACT 0x01
#define SETTING_GREEN    0x02
#define SETTING_VOYAGE   0x04

class AquaNextComponent : public Component, public uart::UARTDevice {
 public:
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
  SUB_TEXT_SENSOR(mode)
  SUB_TEXT_SENSOR(fw_version)
  SUB_BINARY_SENSOR(power)
  SUB_BINARY_SENSOR(heat_pump_active)
  SUB_BINARY_SENSOR(heat_element_active)
  SUB_BINARY_SENSOR(error_present)
  SUB_BINARY_SENSOR(setting_antibact)
  SUB_BINARY_SENSOR(setting_green)
  SUB_BINARY_SENSOR(setting_voyage)

  void setup() override {}
  void loop() override { process_tx_queue_(); read_serial_(); }
  float get_setup_priority() const override { return setup_priority::DATA; }

  // API Publique
  void set_power(bool on) { uint8_t d = on ? 0x01 : 0x00; send_confirm_(FKT_ONOFF, &d, 1); }
  
  void set_mode(const std::string &m) {
    uint8_t s = current_settings_;
    s &= ~(SETTING_GREEN | SETTING_VOYAGE);
    if (m == "green") s |= SETTING_GREEN;
    else if (m == "voyage") s |= SETTING_VOYAGE;
    send_confirm_(FKT_SETTINGS, &s, 1);
  }

  void set_target_temp(float t) {
    if (t < 30 || t > 65) return;
    uint8_t i = (uint8_t)t, f = (uint8_t)((t - i) * 255.0f);
    uint8_t d[2] = {f, i}; // Little Endian Janus
    send_confirm_(FKT_TARGET_TEMP, d, 2);
  }

  void set_antibact(bool on) {
    uint8_t s = current_settings_;
    if (on) s |= SETTING_ANTIBACT; else s &= ~SETTING_ANTIBACT;
    send_confirm_(FKT_SETTINGS, &s, 1);
  }

  void request_all_temps() {
    const int fkts[] = {FKT_T_MAX, FKT_T_MIN, FKT_TW1, FKT_TW2, FKT_T_AIR, FKT_T_EVAP, FKT_TW3, FKT_T_HP};
    for (int f : fkts) request_function(f);
  }

  void request_function(int fkt) { enqueue_read_(fkt); }

 protected:
  uint8_t rx_buf_[64];
  int rx_idx_ = 0;
  bool in_frame_ = false;
  uint8_t current_settings_ = 0;
  uint32_t tx_last_ms_ = 0;
  int tx_q_[16];
  int tx_h_=0, tx_t_=0;

  void enqueue_read_(int fkt) {
    int n = (tx_t_ + 1) % 16;
    if (n != tx_h_) { tx_q_[tx_t_] = fkt; tx_t_ = n; }
  }

  void process_tx_queue_() {
    if (tx_h_ == tx_t_ || millis() - tx_last_ms_ < 600) return;
    build_read_(tx_q_[tx_h_]);
    tx_h_ = (tx_h_ + 1) % 16;
    tx_last_ms_ = millis();
  }

  static uint8_t hex_to_byte_(uint8_t hi, uint8_t lo) {
    auto n = [](uint8_t c) -> uint8_t { 
        if (c>='0'&&c<='9') return c-'0'; 
        if (c>='A'&&c<='F') return c-'A'+10; 
        return 0; 
    };
    return (n(hi) << 4) | n(lo);
  }

  static float decode_temp_(uint8_t hi, uint8_t lo) {
    if (hi == JANUS_TEMP_NC_HI) return NAN;
    return (float)lo + (float)hi / 255.0f;
  }

  void read_serial_() {
    while (available()) {
      uint8_t r = read();
      
      // On ignore les octets de synchro FF
      if (r == 0xFF) continue;

      // RECTIFICATION : Le bus Janus2 utilise souvent le 8ème bit pour la parité.
      // On force la remise à zéro du bit de poids fort (Parité Space/Mark)
      r &= 0x7F; 

      if (r == JANUS_STX) { 
        rx_idx_ = 0; 
        in_frame_ = true; 
        rx_buf_[rx_idx_++] = r; 
        continue; 
      }
      
      if (!in_frame_) continue;
      
      rx_buf_[rx_idx_++] = r;

      // On cherche la fin de trame (ETX + LRC + CR)
      // Dans ta trame logguée, le 03 (ETX) est arrivé très tard.
      if (r == JANUS_CR && rx_idx_ > 5) {
        decode_frame_(rx_buf_, rx_idx_);
        in_frame_ = false;
      }

      if (rx_idx_ >= 63) in_frame_ = false; // Sécurité overflow
    }
  }

  void decode_frame_(uint8_t *f, int len) {
    // Debug pour voir la trame
    char b[128]; int p=0;
    for(int i=0; i<len && p<120; i++) p+=snprintf(b+p, 128-p, "%02X ", f[i]);
    
    // On ignore les trames trop courtes (bruit)
    if (len < 10) return;

    // Extraction du type de message (on masque le bit 7)
    uint8_t msgt = f[1] & 0x7F;
    
    // Extraction de la fonction (FKT) en ASCII
    char fs[4] = {(char)(f[2]&0x7F), (char)(f[3]&0x7F), (char)(f[4]&0x7F), 0};
    int fkt = strtol(fs, nullptr, 16);

    // Extraction de la longueur de données (Position 7 et 8 pour READ)
    // Attention: sur une REPONSE, la structure peut différer
    int d_len = hex_to_byte_(f[7]&0x7F, f[8]&0x7F);

    ESP_LOGD("aquanext", "Trame recue: MSGT=%02X FKT=%03X Len=%d Raw: %s", msgt, fkt, d_len, b);

    // Si c'est juste un echo de notre propre requête (MSGT 0x41), on ne traite pas
    if (msgt == 0x41 && len <= 12) return; 

    // Décodage des données si présentes
    uint8_t d[16] = {0};
    if (len > 12) {
       for(int i=0; i<d_len && i<16; i++) {
         d[i] = hex_to_byte_(f[9+i*2]&0x7F, f[10+i*2]&0x7F);
       }
    }

    // Traitement des données
    switch(fkt) {
      case FKT_T_EVAP:
        if (temperature_evap_sensor_) temperature_evap_sensor_->publish_state(decode_temp_(d[0], d[1]));
        break;
      case FKT_T_AIR:
        if (temperature_air_sensor_) temperature_air_sensor_->publish_state(decode_temp_(d[0], d[1]));
        break;
      case FKT_STATUS:
        if (temperature_target_sensor_) temperature_target_sensor_->publish_state(decode_temp_(d[0], d[1]));
        if (temperature_dome_sensor_) temperature_dome_sensor_->publish_state(decode_temp_(d[4], d[5]));
        break;
    }
  }}

  void build_read_(int fkt) {
    uint8_t f[24]; int i=0;
    for(int j=0; j<7; j++) f[i++] = 0xFF;
    f[i++] = JANUS_STX; f[i++] = JANUS_MSGT_READ;
    char fs[4]; snprintf(fs, 4, "%03X", fkt);
    f[i++] = fs[0]; f[i++] = fs[1]; f[i++] = fs[2];
    f[i++] = '0'; f[i++] = '0'; f[i++] = '0'; f[i++] = '1';
    f[i++] = JANUS_ETX;
    uint8_t l=0; for(int j=8; j<i; j++) l+=(f[j]&0x7F);
    f[i++] = (l&0x7F); f[i++] = JANUS_CR;
    write_array(f, i);
  }

  void send_confirm_(int fkt, uint8_t *d, int dl) {
    uint8_t f[40]; int i=0;
    for(int j=0; j<7; j++) f[i++] = 0xFF;
    f[i++] = JANUS_STX; f[i++] = JANUS_MSGT_WRITE;
    f[i++] = '0' + dl;
    char fs[4]; snprintf(fs, 4, "%03X", fkt);
    f[i++] = fs[0]; f[i++] = fs[1]; f[i++] = fs[2];
    f[i++] = '0'; f[i++] = '0';
    for(int j=0; j<dl; j++) {
      char h[3]; snprintf(h, 3, "%02X", d[j]);
      f[i++] = h[0]; f[i++] = h[1];
    }
    f[i++] = JANUS_ETX;
    uint8_t l=0; for(int j=8; j<i; j++) l+=(f[j]&0x7F);
    f[i++] = (l&0x7F); f[i++] = JANUS_CR;
    write_array(f, i);
  }
};

} // namespace aquanext
} // namespace esphome