# LoRa AT Modem — TTGO LoRa32 T3 v1.6.1 (433 MHz)

Firmware PlatformIO/Arduino qui transforme une carte **LilyGO TTGO LoRa32 T3
v1.6.1** (ESP32 + SX1278, 433 MHz) en modem LoRa brut piloté par commandes
**AT** sur le port série USB.

Le jeu de commandes implémenté suit `AT_COMMANDS.md` (interface at-os3) :
fréquence, profil de modulation (SF/BW/CR/préambule), profil paquet
(CRC/LDRO/longueur), puissance PA_BOOST, sync word, inversion IQ, envoi/réception
de trames en hexadécimal, lecture de registre SX1278, etc.

Le pilote radio est **[RadioLib](https://github.com/jgromes/RadioLib)**.

## Brochage (T3 v1.6.1)

LoRa SX1278 (SPI) :

| Signal | GPIO |
|--------|-----:|
| SCK    | 5    |
| MISO   | 19   |
| MOSI   | 27   |
| NSS/CS | 18   |
| RST    | 23   |
| DIO0   | 26   |
| DIO1   | 33   |

OLED SSD1306 128×64 (I²C, adresse 0x3C) :

| Signal | GPIO |
|--------|-----:|
| SDA    | 21   |
| SCL    | 22   |
| RST    | -1 (non câblé) |

## Écran OLED — indicateurs

L'écran affiche en temps réel (~3 Hz) :

```
┌──────────────────────────┐
│ 433.500MHz           RX   │  fréquence + mode (RX / STBY)
│ SF9 BW125 4/5            │  profil de modulation
│ P10 SW12 IQn C1         │  puissance, sync word, IQ, CRC
│ TX:1 RX:0 E:0           │  compteurs (TX / RX / erreurs CRC)
│ RSSI-57 SNR7            │  métriques du dernier paquet reçu
│ TX 5B 68656C6C6F        │  dernier événement + payload (hex tronqué)
└──────────────────────────┘
```

L'OLED est **optionnel** : si l'écran n'est pas détecté, le modem AT continue de
fonctionner normalement. Si l'écran reste noir, certaines révisions de carte
câblent le reset OLED sur GPIO16 — recompilez alors avec `-D PIN_OLED_RST=16`
dans `platformio.ini`.

Ces valeurs sont passées en `build_flags` dans `platformio.ini` ; modifiez-les
si votre révision de carte diffère.

## Compiler / flasher

```bash
pio run                # compilation
pio run -t upload      # flash via USB
pio device monitor -b 115200   # console série
```

## Liaison série

- **115200 baud, 8N1**
- `\n` exécute une commande ; `\r` est ignoré (donc `\r\n` accepté)
- Commandes en MAJUSCULES, analyse stricte
- Réponses terminées par `\r\n`
- Commande inconnue / mal formée → `+ERR=4`
- Au démarrage : `+READY`

## Exemple — réception

```text
AT
AT+BAND=436995000
AT+PARAMETER=8,6,3,8
AT+PKT=1,2,0
AT+SYNCWORD=18
AT+IQI=0
AT+CRFOP=5
AT+MODE=0
```

À la réception d'un paquet :

```text
+RCV=<addr>,<len>,<hexdata>,<rssi_dbm>,<snr_db>,<freq_err_hz>
```

## Exemple — émission

```text
AT+SEND=0,4,70696E67     # envoie "ping"
+OK
+SENDED
```

## Notes d'implémentation

- `AT+ADDRESS` et `AT+NETWORKID` sont des registres « ombre » de compatibilité :
  ils n'altèrent pas le contenu des paquets LoRa bruts (l'adresse n'apparaît
  que comme premier champ de `+RCV`).
- `AT+IPR` est un no-op de compatibilité : le débit série n'est pas modifié.
- `AT+REG=<hh>` lit un registre SX1278 via une sous-classe RadioLib
  (`SX1278AT`) car `getMod()` est protégé dans RadioLib 6.x.
- RSSI/SNR/FEI proviennent de `getRSSI()`, `getSNR()`, `getFrequencyError()`.
- ⚠️ La puissance et la fréquence acceptées par le firmware ne garantissent pas
  la conformité réglementaire d'émission dans votre région/bande.
