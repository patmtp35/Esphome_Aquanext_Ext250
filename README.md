# 🔥 Chaffoteaux AquaNext / Ariston Nuos — Janus2 ESPHome Integration

Intégration ESPHome pour piloter et monitorer un ballon thermodynamique **Chaffoteaux AquaNext** (et modèles apparentés **Ariston Nuos 250**) via l'ESP32, en exploitant le bus série interne entre la carte mère (**Janus2**) et l'interface utilisateur (IHM).

> **Basé sur le travail de reverse engineering de [jojomojo97/NuosJanus2](https://github.com/jojomojo97/NuosJanus2)**  
> Discussion Home Assistant : [Ariston Nuos 250 / Chaffoteaux Aquanext performance 200](https://community.home-assistant.io/t/ariston-nuos-250-chaffoteaux-aquanext-performance-200/810314)

---

## 📋 Compatibilité

| Modèle | Carte mère | Testé |
|---|---|---|
| Chaffoteaux AquaNext EXT 250 (2011–2014) | Janus2 | ✅ |
| Ariston Nuos 250 EXT SOL | Janus2 SW 010204 | ✅ (ref. NuosJanus2) |
| Ariston Nuos Janus2 (variantes) | Janus2 | ⚠️ Partiellement |

---

## ⚡ Fonctionnalités

### Lecture (automatique, cyclique)
- 🌡️ Températures : consigne, dôme, air, évaporateur, TW1/TW2/TW3, sortie PAC
- 📊 Mode de fonctionnement actuel : Boost / Green / Voyage / Auto
- 🔄 État : allumé/éteint, pompe à chaleur active, résistance active
- ⏱️ Compteurs d'heures PAC et résistance électrique
- ⚠️ Présence d'erreur
- ℹ️ Version firmware carte mère

### Contrôle (via Home Assistant)
- ⚡ Marche / Arrêt
- 🔘 Changement de mode : **Boost**, **Green**, **Voyage**
- 🌡️ Température cible (30–65°C, pas de 1°C)
- 🦠 Activation anti-bactérien

---

## 🔧 Matériel nécessaire

| Composant | Exemple | Prix |
|---|---|---|
| ESP32 | **ESP32 WROOM DevKit V1** (recommandé) | ~5€ |
| Isolateur galvanique | **ADUM1201** (recommandé) ou BSS138 | ~2€ |
| Adaptateur RJ12 à vis | XMSJSIJ 6P6C vers 6 broches | ~3€ |
| Alimentation 5V isolée | Hi-Link HLK-PM01 (230V→5V) | ~3€ |
| Câble dupont | — | — |

> ⚠️ **L'isolateur galvanique est fortement recommandé** pour une installation permanente sur un appareil raccordé au secteur 230V.  
> ⚠️ **L'ESP32 WROOM est recommandé** — le S2 Mini présente une tension 3.3V instable sous charge WiFi (~2.78V) qui perturbe l'ADUM1201.

---

## 🔌 Câblage

### Option 1 — ADUM1201 (isolation galvanique, recommandé installation permanente)

```
        ADUM1201
    ┌─────────────┐
VDD1│1           8│VDD2 ◄──── 3.3V ESP32 WROOM
VIA1│2           7│VOB  ────► RJ12 Pin2 (RX carte mère)
VOA │3           6│VIB  ◄──── GPIO16 TX ESP32
GND1│4           5│GND2 ◄──── GND ESP32
    └─────────────┘
     ▲
     ├── RJ12 Pin3 → VDD1 (5V)
     ├── RJ12 Pin4 → GND1
     └── RJ12 Pin1 TX → VIA1
```

| De | Vers | Description |
|---|---|---|
| RJ12 Pin1 TXD0 | ADUM Pin2 (VIA1) | TX carte mère → isolateur |
| RJ12 Pin2 RXD0 | ADUM Pin7 (VOB) | RX carte mère ← isolateur |
| RJ12 Pin3 5V | ADUM Pin1 (VDD1) | Alim 5V côté ballon |
| RJ12 Pin4 GND | ADUM Pin4 (GND1) | GND côté ballon |
| ADUM Pin3 (VOA) | ESP32 GPIO17 (RX) | Signal RX vers ESP32 |
| ADUM Pin6 (VIB) | ESP32 GPIO16 (TX) | Signal TX depuis ESP32 |
| ESP32 3.3V | ADUM Pin8 (VDD2) | Alim 3.3V côté ESP32 |
| ESP32 GND | ADUM Pin5 (GND2) | GND côté ESP32 |

> ⚠️ **GND1 et GND2 ne doivent pas être reliés ensemble** — c'est l'isolation galvanique.

### Option 2 — Résistance série (test uniquement)

```
RJ12 Pin1 TXD0 ──[1kΩ]──► GPIO17 (RX) ESP32
RJ12 Pin4 GND  ──────────► GND ESP32
```

### Connecteur RJ12 externe (port flash/service Janus2)

```
RJ12 Pin1 = TXD0  (sortie carte mère)
RJ12 Pin2 = RXD0  (entrée carte mère)
RJ12 Pin3 = VCC 5V
RJ12 Pin4 = GND
RJ12 Pin5 = MODE  (ne pas connecter)
RJ12 Pin6 = RESET (ne pas connecter)
```

> Le connecteur RJ12 externe répond uniquement aux requêtes (pas de status automatique).  
> Le connecteur IHM interne (câble bleu 6 fils) donne accès au **bus HMI** avec le status cyclique toutes les secondes.

---

## 📁 Structure du projet

```
Esphome_Aquanext_Ext250/
├── aquanext-ext250.yaml            # Configuration ESPHome principale
├── components/
│   └── aquanext/
│       ├── aquanext.h              # Composant C++ (protocole Janus2)
│       └── __init__.py             # Déclaration composant ESPHome
└── README.md
```

---

## 🚀 Installation

### 1. Cloner le projet

```bash
git clone https://github.com/patmtp35/Esphome_Aquanext_Ext250.git
cd Esphome_Aquanext_Ext250
```

### 2. Créer le fichier `secrets.yaml`

```yaml
wifi_ssid: "VotreSSID"
wifi_password: "VotreMotDePasse"
ota_pswd: "votre_mot_de_passe_ota"
ip_aquanext: "192.168.x.x"
ip_gateway: "192.168.x.x"
ip_dns: "192.168.x.x"
```

### 3. Compiler et flasher

```bash
esphome run aquanext-ext250.yaml
```

Ou via l'interface graphique ESPHome dans Home Assistant.

> 💡 Si vous mettez à jour `aquanext.h` sur GitHub, faites **"Clean Build Files"** avant de recompiler pour forcer le re-téléchargement du composant.

### 4. Intégration Home Assistant

L'ESP32 sera détecté automatiquement. Les entités disponibles :

| Entité | Type | Description |
|---|---|---|
| `sensor.aquanext_temperature_consigne` | Sensor | Température cible |
| `sensor.aquanext_temperature_dome` | Sensor | Température dôme ballon |
| `sensor.aquanext_temperature_air_entree` | Sensor | Température air entrée |
| `sensor.aquanext_temperature_evaporateur` | Sensor | Température évaporateur |
| `sensor.aquanext_sonde_eau_tw1` | Sensor | Sonde eau TW1 |
| `sensor.aquanext_sonde_eau_tw2` | Sensor | Sonde eau TW2 |
| `sensor.aquanext_temperature_sortie_pac` | Sensor | Température sortie PAC |
| `sensor.aquanext_heures_pac` | Sensor | Compteur heures PAC |
| `sensor.aquanext_heures_resistance` | Sensor | Compteur heures résistance |
| `text_sensor.aquanext_mode_fonctionnement` | Text Sensor | Mode actuel (boost/green/voyage) |
| `text_sensor.aquanext_version_firmware` | Text Sensor | Version firmware |
| `switch.aquanext_marche_arret` | Switch | ON/OFF |
| `switch.aquanext_anti_bacterien` | Switch | Anti-bactérien |
| `select.aquanext_mode_de_chauffe` | Select | boost/green/voyage |
| `number.aquanext_temperature_cible` | Number | Consigne 30–65°C |
| `binary_sensor.aquanext_pac_active` | Binary Sensor | PAC en cours |
| `binary_sensor.aquanext_resistance_active` | Binary Sensor | Résistance en cours |
| `binary_sensor.aquanext_erreur_presente` | Binary Sensor | Erreur active |
| `button.aquanext_rafraichir_temperatures` | Button | Forcer mise à jour |

---

## 📡 Protocole Janus2 (résumé)

> Documentation complète : [NuosJanus2 par jojomojo97](https://github.com/jojomojo97/NuosJanus2)

### Structure de trame Main → HMI

```
0x02 | 0xC1 | FKT(3 ascii) | 00(2) | LEN(2 ascii) | DATA(ascii) | 0x03 | LRC(1 byte) | 0x0D
 STX   MSGT   Function ID   Padding  Longueur        Payload       ETX    Checksum      CR
```

### LRC
```
LRC = somme de MSGT jusqu'à ETX inclus, modulo 256
LRC est encodé en 1 byte BINAIRE (pas 2 chars ASCII)

### Encodage température

```
Temp(°C) = octet_entier + octet_décimal / 255
Exemple  : 0x33 + 0x40/0xFF = 51.25°C
Absent   : 0xFE 0x7F = capteur non connecté
```

### Function IDs principaux

| FKT | Nom | Description |
|---|---|---|
| 0x000 | TargetTemp | Température consigne |
| 0x001 | OnOff | Marche/arrêt |
| 0x003 | Status | Statut cyclique (~1s) |
| 0x004 | Errors | Erreurs actives |
| 0x005 | T_Max | Température max |
| 0x006 | T_Min | Température min |
| 0x007 | Settings | Bitmask paramètres |
| 0x008 | SW_MB | Version firmware |
| 0x00A–0x00E | TW1/TW2/T_AIR/T_EVAP/TW3 | Sondes température |
| 0x010/0x011 | HP_h / HE_h | Compteurs horaires |
| 0x012 | T_HP | Température sortie PAC |

### Settings bitmask (FKT 0x007)

| Bit | Valeur | Paramètre |
|---|---|---|
| 0 | 0x01 | Anti-bactérien |
| 1 | 0x02 | Mode Green |
| 2 | 0x04 | Mode Voyage |
| 3 | 0x08 | Dégivrage |
| 4 | 0x10 | HP_NC |

---

## 🐛 Bugs corrigés (v1.1.0)

- **LRC** : le checksum est 1 byte **binaire**, pas 2 chars ASCII comme supposé initialement
- **Offsets décodage** : LEN lu à `frame[7,8]` et DATA depuis `frame[9]` (décalage d'un octet corrigé)
- **GPIO inversés** : sur le câblage validé, `GPIO16=TX` et `GPIO17=RX`
- **Composant ESPHome** : ajout du fichier `__init__.py` manquant (obligatoire pour les external_components)

---

## ⚠️ Avertissements

- Ce projet est fourni **à titre expérimental**, sans garantie.
- Les commandes d'écriture (changement de mode, température) ont été déduites du protocole — **tester d'abord en lecture seule** pour valider le câblage.
- **Utiliser impérativement l'isolation galvanique** (ADUM1201) pour une installation permanente — le ballon est raccordé au secteur 230V.
- Certains Function IDs peuvent différer selon la version de firmware de votre appareil.

---

## 🤝 Contributions

Les retours d'expérience, corrections de protocole et pull requests sont les bienvenus, notamment pour :
- Confirmer le comportement du mode **Auto**
- Valider les commandes d'écriture (set_mode, set_temp) sur le bus HMI interne
- Tester sur d'autres variantes de la carte Janus2

---

## 📜 Licence

MIT — voir [LICENSE](LICENSE)

---

## 🙏 Remerciements

- [jojomojo97](https://github.com/jojomojo97) — reverse engineering du protocole Janus2
- Communauté [Home Assistant](https://community.home-assistant.io/t/ariston-nuos-250-chaffoteaux-aquanext-performance-200/810314) pour les discussions et analyses
