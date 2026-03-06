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
| ESP32 | ESP32 DevKit V1 | ~5€ |
| Level shifter 5V ↔ 3.3V | Module BSS138 bidirectionnel | ~1€ |
| Câble dupont | — | — |

> ⚠️ **Le level shifter est OBLIGATOIRE.** Le bus interne fonctionne en TTL 5V. Brancher directement sur l'ESP32 (3.3V) risque d'endommager le microcontrôleur.

---

## 🔌 Câblage

```
Carte mère Janus2          Level Shifter        ESP32
──────────────────         ─────────────        ──────────
PIN1  GND          ──────► LV-GND ◄──────────►  GND
PIN2  VCC 5V       ──────► HV                   (ne pas connecter à l'ESP32)
                           LV ◄─────────────────  3.3V
PIN3  TX (HMI)     ──────► HV-A1  LV-B1 ───────►  GPIO16 (RX2)
PIN4  RX (HMI)     ◄──────  HV-A2  LV-B2 ◄──────  GPIO17 (TX2)
```

> Le connecteur se situe sur le câble bleu reliant la carte mère à l'IHM (6 fils).  
> Voir le schéma électrique sur l'étiquette intérieure du ballon (réf. `42.0.06.02090.01`).

---

## 📁 Structure du projet

```
aquanext/
├── aquanext.yaml                   # Configuration ESPHome principale
├── components/
│   └── aquanext/
│       └── aquanext.h              # Composant C++ (protocole Janus2)
└── secrets.yaml                    # Vos secrets WiFi/API (non versionné)
```

---

## 🚀 Installation

### 1. Cloner le projet

```bash
git clone https://github.com/VOTRE_USER/aquanext-esphome.git
cd aquanext-esphome
```

### 2. Créer le fichier `secrets.yaml`

```yaml
wifi_ssid: "VotreSSID"
wifi_password: "VotreMotDePasse"
api_key: "votre_cle_api_esphome_32bytes_base64"
ota_password: "votre_mot_de_passe_ota"
```

### 3. Compiler et flasher

```bash
esphome run aquanext.yaml
```

Ou via l'interface graphique ESPHome dans Home Assistant.

### 4. Intégration Home Assistant

L'ESP32 sera détecté automatiquement. Les entités disponibles :

| Entité | Type | Description |
|---|---|---|
| `sensor.temperature_consigne` | Sensor | Température cible actuelle |
| `sensor.temperature_dome` | Sensor | Température dôme ballon |
| `sensor.temperature_air` | Sensor | Température air entrée |
| `sensor.temperature_evaporateur` | Sensor | Température évaporateur |
| `sensor.heures_pac` | Sensor | Compteur heures PAC |
| `sensor.heures_resistance` | Sensor | Compteur heures résistance |
| `text_sensor.mode` | Text Sensor | Mode actuel (boost/green/voyage) |
| `switch.marche_arret` | Switch | ON/OFF |
| `switch.anti_bacterien` | Switch | Anti-bactérien |
| `select.mode_de_chauffe` | Select | Changer le mode |
| `number.temperature_cible` | Number | Régler la consigne (30–65°C) |
| `binary_sensor.erreur` | Binary Sensor | Erreur présente |
| `button.rafraichir_temperatures` | Button | Forcer la mise à jour |

---

## 📡 Protocole Janus2 (résumé)

> Documentation complète : [NuosJanus2 par jojomojo97](https://github.com/jojomojo97/NuosJanus2)

### Structure de trame Main → HMI

```
0x02 | 0xC1 | FKT(3 chars hex) | 00 | LEN(2 chars) | DATA | 0x03 | LRC | 0x0D
 STX   MSGT   Function ID       NUL   Longueur data   Payload  ETX   Check  CR
```

### LRC = somme des octets depuis MSGT jusqu'à ETX inclus, modulo 256

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

## ⚠️ Avertissements

- Ce projet est fourni **à titre expérimental**, sans garantie.
- Les commandes d'écriture (changement de mode, température) ont été déduites du protocole — **tester d'abord en lecture seule** pour valider le câblage.
- L'isolation galvanique (optocoupleur) est recommandée pour une installation permanente.
- Certains Function IDs peuvent différer selon la version de firmware de votre appareil.

---

## 🤝 Contributions

Les retours d'expérience, corrections de protocole et pull requests sont les bienvenus, notamment pour :
- Confirmer le comportement du mode **Auto**
- Tester sur d'autres variantes de la carte Janus2
- Ajouter le support du connecteur RJ12 externe

---

## 📜 Licence

MIT — voir [LICENSE](LICENSE)

---

## 🙏 Remerciements

- [jojomojo97](https://github.com/jojomojo97) — reverse engineering du protocole Janus2
- Communauté [Home Assistant](https://community.home-assistant.io/t/ariston-nuos-250-chaffoteaux-aquanext-performance-200/810314) pour les discussions et analyses
