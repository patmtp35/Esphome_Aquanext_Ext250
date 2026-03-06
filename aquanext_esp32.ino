/**
 * Chaffoteaux AquaNext / Ariston Nuos - Janus2 Protocol
 * ESP32 - Lecture série + publication MQTT
 *
 * Matériel requis :
 *   - Level Shifter 5V <-> 3.3V bidirectionnel (ex: BSS138)
 *   - Connexion sur le connecteur HMI<->Mainboard (TTL 5V, 9600 8N1)
 *
 * Câblage :
 *   Carte mère PIN3 (TX HMI) -> Level Shifter -> ESP32 GPIO16 (RX2)
 *   Carte mère PIN4 (RX HMI) -> Level Shifter -> ESP32 GPIO17 (TX2)  [optionnel, lecture seule]
 *   Carte mère PIN1 (GND)    -> GND ESP32
 *   ATTENTION : Ne pas connecter le 5V de la carte mère à l'ESP32 !
 *
 * Protocole Janus2 :
 *   STX(0x02) | MSGT(0xC1) | FKT(3 ascii hex) | 00 | LEN(2 ascii) | DATA | ETX(0x03) | LRC | CR(0x0D)
 *   LRC = somme de tous les octets après STX (ETX inclus) & 0xFF
 *
 * Configuration MQTT : modifier les #define ci-dessous
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ============================================================
// CONFIGURATION - À MODIFIER
// ============================================================
#define WIFI_SSID       "VotreSSID"
#define WIFI_PASSWORD   "VotreMotDePasse"
#define MQTT_BROKER     "192.168.1.x"   // IP de votre broker MQTT (ex: Home Assistant)
#define MQTT_PORT       1883
#define MQTT_USER       ""              // laisser vide si pas d'auth
#define MQTT_PASSWORD   ""
#define MQTT_PREFIX     "aquanext"      // préfixe des topics MQTT

// GPIO Serial vers la carte mère (via level shifter 5V->3.3V)
#define SERIAL2_RX_PIN  16
#define SERIAL2_TX_PIN  17

// Intervalle de reconnexion WiFi/MQTT (ms)
#define RECONNECT_INTERVAL 5000
// ============================================================

HardwareSerial JanusSerial(2);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// Buffer de réception
#define BUF_SIZE 64
uint8_t rxBuf[BUF_SIZE];
int rxIdx = 0;
bool inFrame = false;

// Settings courants (mis à jour à chaque réception FKT=007)
// Utilisé pour modifier un seul bit sans écraser les autres
uint8_t currentSettings = 0x00;

// ============================================================
// Utilitaires
// ============================================================

// Convertit 2 caractères ASCII hex en byte (ex: '4','1' -> 0x41)
uint8_t hexToByte(uint8_t hi, uint8_t lo) {
  auto h = [](uint8_t c) -> uint8_t {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
  };
  return (h(hi) << 4) | h(lo);
}

// Décode une température Janus2 (2 octets : entier + décimale/255)
// Retourne -999.0 si capteur non connecté (FE7F)
float decodeTemp(uint8_t hi, uint8_t lo) {
  if (hi == 0xFE && lo == 0x7F) return -999.0;
  return (float)hi + (float)lo / 255.0;
}

// Vérifie le LRC d'une trame (octets après STX jusqu'à CR exclu)
bool checkLRC(uint8_t* frame, int len) {
  // frame[0] = STX, frame[len-3] = LRC_hi, frame[len-2] = LRC_lo, frame[len-1] = CR
  uint8_t sum = 0;
  // Sommer depuis MSGT(frame[1]) jusqu'à ETX inclus (frame[len-4] = ETX)
  for (int i = 1; i <= len - 4; i++) {
    sum += frame[i];
  }
  // LRC dans la trame (2 chars ASCII hex)
  uint8_t lrc = hexToByte(frame[len - 3], frame[len - 2]);
  return (sum == lrc);
}

// ============================================================
// Publication MQTT
// ============================================================
void publish(const char* subtopic, const char* value) {
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%s", MQTT_PREFIX, subtopic);
  mqtt.publish(topic, value);
  Serial.printf("[MQTT] %s = %s\n", topic, value);
}

void publishFloat(const char* subtopic, float value) {
  char buf[16];
  if (value <= -999.0) {
    publish(subtopic, "NC");  // capteur non connecté
  } else {
    snprintf(buf, sizeof(buf), "%.2f", value);
    publish(subtopic, buf);
  }
}

void publishInt(const char* subtopic, int value) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%d", value);
  publish(subtopic, buf);
}

// ============================================================
// Décodage des trames Janus2
// ============================================================
void decodeFrame(uint8_t* frame, int len) {
  // Structure minimale : STX(1) + MSGT(1) + FKT(3) + 00(2) + LEN(2) + DATA + ETX(1) + LRC(2) + CR(1)
  if (len < 12) return;
  if (frame[0] != 0x02) return;  // STX
  if (frame[len - 1] != 0x0D) return;  // CR

  if (!checkLRC(frame, len)) {
    Serial.println("[WARN] LRC invalide, trame ignorée");
    return;
  }

  // MSGT
  uint8_t msgt = frame[1];

  // FKT : 3 caractères ASCII hex -> entier
  char fktStr[4] = {(char)frame[2], (char)frame[3], (char)frame[4], 0};
  int fkt = strtol(fktStr, nullptr, 16);

  // LEN : 2 caractères ASCII hex
  int dataLen = hexToByte(frame[6], frame[7]);

  // DATA commence à frame[8], longueur dataLen*2 octets ASCII
  // On extrait les octets de données
  uint8_t data[16];
  int dataBytes = 0;
  for (int i = 0; i < dataLen && i < 8; i++) {
    data[i] = hexToByte(frame[8 + i * 2], frame[9 + i * 2]);
    dataBytes++;
  }

  Serial.printf("[JANUS2] MSGT=0x%02X FKT=0x%03X LEN=%d\n", msgt, fkt, dataLen);

  // ---- Décodage selon Function ID ----

  if (msgt == 0xC1) {
    switch (fkt) {

      case 0x003: { // Status cyclique (1s)
        if (dataBytes < 7) break;
        // DATA: TargetTemp(2) | Unknown(2) | DomeTemp(2) | Status1(1) | Program(1) | DisplaySymbols(1) | Status4(1)
        float targetTemp = decodeTemp(data[0], data[1]);
        float domeTemp   = decodeTemp(data[4], data[5]);
        uint8_t program  = data[7];
        uint8_t symbols  = data[8];

        publishFloat("temperature/target",  targetTemp);
        publishFloat("temperature/dome",    domeTemp);

        // Programme
        const char* progStr = "unknown";
        switch (program) {
          case 0x00: progStr = "boost";   break;
          case 0x01: progStr = "green";   break;
          case 0x02: progStr = "voyage";  break;
          case 0x03: progStr = "auto";    break;
        }
        publish("mode/program", progStr);

        // Symboles affichés (bitmask)
        publish("state/on",         (symbols & 0x01) ? "1" : "0");
        publish("state/heatpump",   (symbols & 0x02) ? "1" : "0");
        publish("state/heatelement",(symbols & 0x04) ? "1" : "0");
        break;
      }

      case 0x004: { // Erreurs
        if (dataBytes < 3) break;
        // Errorbits1, Errorbits2, Errorbits3
        char buf[8];
        snprintf(buf, sizeof(buf), "0x%02X", data[1]);
        publish("errors/bits1", buf);
        snprintf(buf, sizeof(buf), "0x%02X", data[2]);
        publish("errors/bits2", buf);
        snprintf(buf, sizeof(buf), "0x%02X", data[3]);
        publish("errors/bits3", buf);

        // Erreurs connues bit1
        publish("errors/t_air_sensor",  (data[1] & 0x03) ? "1" : "0"); // H7
        publish("errors/t_evap_sensor", (data[1] & 0x0C) ? "1" : "0"); // H6
        publish("errors/tw3_sensor",    (data[1] & 0xC0) ? "1" : "0"); // H8
        publish("errors/gas_pressure",  (data[2] & 0x02) ? "1" : "0"); // H1
        publish("errors/tw1_sensor",    (data[3] & 0x03) ? "1" : "0"); // H8
        publish("errors/tw2_sensor",    (data[3] & 0x0C) ? "1" : "0"); // H8
        publish("errors/anode",         (data[3] & 0x20) ? "1" : "0"); // F5
        publish("errors/empty_tank",    (data[3] & 0x40) ? "1" : "0"); // F4
        break;
      }

      case 0x005: publishFloat("temperature/t_max",  decodeTemp(data[0], data[1])); break;
      case 0x006: publishFloat("temperature/t_min",  decodeTemp(data[0], data[1])); break;
      case 0x00A: publishFloat("temperature/tw1",    decodeTemp(data[0], data[1])); break;
      case 0x00B: publishFloat("temperature/tw2",    decodeTemp(data[0], data[1])); break;
      case 0x00C: publishFloat("temperature/t_air",  decodeTemp(data[0], data[1])); break;
      case 0x00D: publishFloat("temperature/t_evap", decodeTemp(data[0], data[1])); break;
      case 0x00E: publishFloat("temperature/tw3",    decodeTemp(data[0], data[1])); break;
      case 0x012: publishFloat("temperature/t_hp",   decodeTemp(data[0], data[1])); break;

      case 0x007: { // Settings (bitmask)
        uint8_t s = data[0];
        currentSettings = s;  // sauvegarde pour les commandes d'écriture
        publish("settings/anti_bacteria", (s & 0x01) ? "1" : "0");
        publish("settings/green_mode",    (s & 0x02) ? "1" : "0");
        publish("settings/voyage",        (s & 0x04) ? "1" : "0");
        publish("settings/defrost",       (s & 0x08) ? "1" : "0");
        publish("settings/hp_nc",         (s & 0x10) ? "1" : "0");
        // Publier aussi le mode courant
        const char* modeStr = "boost";
        if (s & 0x02) modeStr = "green";
        else if (s & 0x04) modeStr = "voyage";
        publish("mode/program_settings", modeStr);
        break;
      }

      case 0x008: { // Version firmware Mainboard
        char ver[8];
        snprintf(ver, sizeof(ver), "%02X%02X%02X", data[0], data[1], data[2]);
        publish("info/fw_version", ver);
        break;
      }

      case 0x010: { // HP_h : heures pompe à chaleur
        uint32_t mins = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
        publishInt("stats/hp_hours", mins / 60);
        break;
      }

      case 0x011: { // HE_h : heures résistance électrique
        uint32_t mins = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
        publishInt("stats/he_hours", mins / 60);
        break;
      }

      case 0x014: { // Time_W : durée de chauffe
        publishInt("settings/time_w_hours", data[0]);
        break;
      }
    }
  }
}

// ============================================================
// Réception série - machine à états
// ============================================================
void readJanusSerial() {
  while (JanusSerial.available()) {
    uint8_t b = JanusSerial.read();

    if (b == 0x02) { // STX : début de trame
      rxIdx = 0;
      inFrame = true;
      rxBuf[rxIdx++] = b;
    } else if (inFrame) {
      if (rxIdx < BUF_SIZE) {
        rxBuf[rxIdx++] = b;
      }
      if (b == 0x0D) { // CR : fin de trame
        inFrame = false;
        // Dump hex pour debug
        Serial.print("[RAW] ");
        for (int i = 0; i < rxIdx; i++) Serial.printf("%02X ", rxBuf[i]);
        Serial.println();
        decodeFrame(rxBuf, rxIdx);
        rxIdx = 0;
      }
    }
  }
}

// ============================================================
// Commandes vers la carte mère (HMI -> Main)
// Permet de demander des données ou de changer des paramètres
// ============================================================

// Calcule le LRC et envoie une commande
void sendCommand(uint8_t fkt_hi, uint8_t fkt_mid, uint8_t fkt_lo,
                 uint8_t* data, int dataLen) {
  // Construction de la trame ASCII
  // STX | 0xFF*7 | 0xC1 | FKT(3) | 00 | LEN(2) | DATA | ETX | LRC | CR
  // Note: le HMI envoie 7 bytes 0xFF avant STX d'après la doc

  uint8_t frame[64];
  int idx = 0;

  // Préambule 7x 0xFF (observé dans les trames HMI->Main)
  for (int i = 0; i < 7; i++) frame[idx++] = 0xFF;

  frame[idx++] = 0x02;  // STX
  frame[idx++] = 0xC1;  // MSGT

  // FKT en ASCII (3 chars hex)
  char fktStr[4];
  snprintf(fktStr, 4, "%c%c%c", fkt_hi, fkt_mid, fkt_lo);
  frame[idx++] = fkt_hi;
  frame[idx++] = fkt_mid;
  frame[idx++] = fkt_lo;

  frame[idx++] = '0';  // padding 00
  frame[idx++] = '0';

  // LEN
  char lenStr[3];
  snprintf(lenStr, 3, "%02X", dataLen);
  frame[idx++] = lenStr[0];
  frame[idx++] = lenStr[1];

  // DATA (octets -> 2 chars hex chacun)
  for (int i = 0; i < dataLen; i++) {
    char hex[3];
    snprintf(hex, 3, "%02X", data[i]);
    frame[idx++] = hex[0];
    frame[idx++] = hex[1];
  }

  frame[idx++] = 0x03;  // ETX

  // LRC : somme depuis MSGT jusqu'à ETX inclus (après les 7x FF et STX)
  uint8_t lrc = 0;
  for (int i = 8; i < idx; i++) lrc += frame[i];  // depuis MSGT (index 8)

  char lrcStr[3];
  snprintf(lrcStr, 3, "%02X", lrc);
  frame[idx++] = lrcStr[0];
  frame[idx++] = lrcStr[1];

  frame[idx++] = 0x0D;  // CR

  JanusSerial.write(frame, idx);
}

// Demande une lecture de données (MSGT=0xC1)
void requestFunction(int fkt) {
  char fktStr[4];
  snprintf(fktStr, 4, "%03X", fkt);
  uint8_t data = 0x01;
  sendCommand(fktStr[0], fktStr[1], fktStr[2], &data, 1);
}

// ============================================================
// Commandes d'écriture (MSGT=0xC2 = "Confirm/Set")
// Basées sur les trames HMI->Main de la doc Janus2
// ============================================================

/**
 * Envoi d'une commande de type "Confirm" (MSGT=0xC2)
 * Différent de sendCommand qui utilise MSGT=0xC1
 * Structure : 7x0xFF | STX | 0xC2 | FKT(1 byte=len?) | FKT(3 ascii) | 00 | DATA | ETX | LRC | CR
 * Ex doc: 02 C2 1 001 00 01 03 48 0D  -> OnOff ON
 */
void sendConfirm(int fkt, uint8_t* data, int dataLen) {
  uint8_t frame[64];
  int idx = 0;

  // Préambule 7x 0xFF
  for (int i = 0; i < 7; i++) frame[idx++] = 0xFF;

  frame[idx++] = 0x02;   // STX

  // MSGT = 0xC2 pour les commandes d'écriture
  frame[idx++] = 0xC2;

  // Dans les trames C2 de la doc, le 3ème byte semble être la longueur en chiffre ASCII
  // Ex: "Â1001..." -> C2 + '1' + "001"
  // Le '1' = dataLen en ASCII
  frame[idx++] = '0' + dataLen;  // longueur en ASCII ('1' pour 1 byte, '2' pour 2 bytes)

  // FKT en ASCII 3 chars hex
  char fktStr[4];
  snprintf(fktStr, 4, "%03X", fkt);
  frame[idx++] = fktStr[0];
  frame[idx++] = fktStr[1];
  frame[idx++] = fktStr[2];

  // Padding 00
  frame[idx++] = '0';
  frame[idx++] = '0';

  // DATA (octets -> 2 chars ASCII hex)
  for (int i = 0; i < dataLen; i++) {
    char hex[3];
    snprintf(hex, 3, "%02X", data[i]);
    frame[idx++] = hex[0];
    frame[idx++] = hex[1];
  }

  frame[idx++] = 0x03;  // ETX

  // LRC : somme depuis MSGT (index 8) jusqu'à ETX inclus
  uint8_t lrc = 0;
  for (int i = 8; i < idx; i++) lrc += frame[i];

  char lrcStr[3];
  snprintf(lrcStr, 3, "%02X", lrc);
  frame[idx++] = lrcStr[0];
  frame[idx++] = lrcStr[1];

  frame[idx++] = 0x0D;  // CR

  // Debug
  Serial.print("[CMD TX] ");
  for (int i = 0; i < idx; i++) Serial.printf("%02X ", frame[i]);
  Serial.println();

  JanusSerial.write(frame, idx);
}

// ---- Fonctions de contrôle de haut niveau ----

/**
 * ON / OFF
 * MSGT=C2, FKT=001, data=0x01(ON) ou 0x00(OFF)
 * Ref doc: 02 C2 1 001 00 01 03 48 0D
 */
void setOnOff(bool on) {
  uint8_t data = on ? 0x01 : 0x00;
  sendConfirm(0x001, &data, 1);
  Serial.printf("[CMD] OnOff -> %s\n", on ? "ON" : "OFF");
}

/**
 * Changer le mode de programme
 * Le mode est encodé dans le bitmask Settings (FKT=007) :
 *   Bit1 (0x02) = Green
 *   Bit2 (0x04) = Voyage
 *   Les autres bits (AntiBact=0x01, Defrost=0x08, HP_NC=0x10) sont conservés
 *
 * Modes : "boost" (ni green ni voyage), "green" (bit1), "voyage" (bit2), "auto" (???)
 *
 * IMPORTANT : On lit d'abord le settings actuel avant d'écrire pour ne pas
 * écraser les autres bits. En attendant la réponse asynchrone, on stocke le
 * settings courant dans currentSettings.
 *
 * Ref doc Confirm Setting: 02 C2 1 007 00 0A 03 5E 0D
 */
void setMode(const char* mode) {
  uint8_t s = currentSettings;

  // Effacer les bits de mode (green=0x02, voyage=0x04)
  s &= ~(0x02 | 0x04);

  if (strcmp(mode, "green") == 0) {
    s |= 0x02;
  } else if (strcmp(mode, "voyage") == 0) {
    s |= 0x04;
  } else if (strcmp(mode, "boost") == 0) {
    // Boost = ni green ni voyage, juste effacer les bits
    // (aucun bit supplémentaire)
  } else if (strcmp(mode, "auto") == 0) {
    // "Auto" semble activer les deux selon certaines observations
    // À confirmer sur ton appareil — désactivé par sécurité pour l'instant
    Serial.println("[WARN] Mode 'auto' : comportement non confirmé, non envoyé");
    return;
  } else {
    Serial.printf("[WARN] Mode inconnu : %s\n", mode);
    return;
  }

  sendConfirm(0x007, &s, 1);
  Serial.printf("[CMD] Mode -> %s (settings byte=0x%02X)\n", mode, s);
}

/**
 * Changer la température cible
 * FKT=000 (TargetTemp), encodage : octet entier + octet décimal*255
 * Ref: même encodage que les températures lues
 * Temp min/max selon T_Min et T_Max lus depuis la carte mère
 */
void setTargetTemp(float temp) {
  uint8_t intPart  = (uint8_t)temp;
  uint8_t fracPart = (uint8_t)((temp - intPart) * 255.0);
  uint8_t data[2]  = {intPart, fracPart};
  sendConfirm(0x000, data, 2);
  Serial.printf("[CMD] TargetTemp -> %.2f°C (0x%02X 0x%02X)\n", temp, intPart, fracPart);
}

// ============================================================
// MQTT callback (commandes entrantes)
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[32] = {0};
  memcpy(msg, payload, min(length, (unsigned int)31));
  Serial.printf("[MQTT RX] %s = %s\n", topic, msg);

  // ---- Lectures ----

  if (strstr(topic, "cmd/request_temps")) {
    requestFunction(0x005);  delay(200);  // T_Max
    requestFunction(0x006);  delay(200);  // T_Min
    requestFunction(0x00A);  delay(200);  // TW1
    requestFunction(0x00B);  delay(200);  // TW2
    requestFunction(0x00C);  delay(200);  // T_AIR
    requestFunction(0x00D);  delay(200);  // T_EVAP
    requestFunction(0x00E);  delay(200);  // TW3
    requestFunction(0x012);              // T_HP
  }

  if (strstr(topic, "cmd/request_status")) {
    requestFunction(0x007);  delay(200);  // Settings
    requestFunction(0x008);  delay(200);  // Version FW
    requestFunction(0x010);  delay(200);  // HP hours
    requestFunction(0x011);              // HE hours
  }

  // ---- Contrôle ON/OFF ----
  // Topic: aquanext/cmd/set_power  |  payload: "ON" ou "OFF"
  if (strstr(topic, "cmd/set_power")) {
    if (strcasecmp(msg, "ON") == 0 || strcmp(msg, "1") == 0) {
      setOnOff(true);
    } else if (strcasecmp(msg, "OFF") == 0 || strcmp(msg, "0") == 0) {
      setOnOff(false);
    }
  }

  // ---- Changement de mode ----
  // Topic: aquanext/cmd/set_mode  |  payload: "boost" / "green" / "voyage" / "auto"
  if (strstr(topic, "cmd/set_mode")) {
    // D'abord demander les settings actuels pour lire currentSettings,
    // puis attendre la réponse (qui met à jour currentSettings) avant d'écrire
    requestFunction(0x007);
    delay(300);  // laisser le temps de recevoir et décoder la réponse
    setMode(msg);
  }

  // ---- Température cible ----
  // Topic: aquanext/cmd/set_temp  |  payload: "55" ou "55.5"
  if (strstr(topic, "cmd/set_temp")) {
    float temp = atof(msg);
    if (temp >= 30.0 && temp <= 65.0) {
      setTargetTemp(temp);
    } else {
      Serial.printf("[WARN] Température hors limites : %.1f (doit être entre 30 et 65°C)\n", temp);
    }
  }

  // ---- Anti-bactérien (cycle haute température) ----
  // Topic: aquanext/cmd/set_antibact  |  payload: "1" ou "0"
  if (strstr(topic, "cmd/set_antibact")) {
    uint8_t s = currentSettings;
    if (strcmp(msg, "1") == 0) s |= 0x01; else s &= ~0x01;
    sendConfirm(0x007, &s, 1);
    Serial.printf("[CMD] AntiBact -> %s\n", msg);
  }
}

// ============================================================
// Connexion WiFi + MQTT
// ============================================================
void setupWifi() {
  Serial.printf("[WiFi] Connexion à %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connecté, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] Échec, mode hors ligne");
  }
}

void reconnectMQTT() {
  if (!mqtt.connected() && WiFi.status() == WL_CONNECTED) {
    Serial.print("[MQTT] Connexion...");
    if (mqtt.connect("AquaNextESP32", MQTT_USER, MQTT_PASSWORD)) {
      Serial.println(" OK");
      mqtt.subscribe(MQTT_PREFIX "/cmd/#");
    } else {
      Serial.printf(" Échec (rc=%d)\n", mqtt.state());
    }
  }
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[AquaNext] Démarrage...");

  // UART2 : 9600 8N1, TTL 5V via level shifter
  JanusSerial.begin(9600, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);

  setupWifi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  reconnectMQTT();

  Serial.println("[AquaNext] Prêt - écoute du bus Janus2...");
}

unsigned long lastReconnect = 0;

void loop() {
  // Gestion MQTT
  if (!mqtt.connected()) {
    unsigned long now = millis();
    if (now - lastReconnect > RECONNECT_INTERVAL) {
      lastReconnect = now;
      reconnectMQTT();
    }
  } else {
    mqtt.loop();
  }

  // Lecture et décodage du bus Janus2
  readJanusSerial();
}
