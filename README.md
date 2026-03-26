# LoRa Node (ESP32 + E22-900T33S)

This project is an ESP32 firmware (`Node.ino`) for a LoRa node built with an E22-900T33S transceiver.  
It combines:

- LoRa messaging (direct or via repeater)
- Local Wi-Fi Access Point control panel
- Dynamic runtime configuration saved to flash
- OLED status screens
- GPS reading
- Battery and charging monitoring

## What the code does

The firmware starts the ESP32 as a Wi-Fi AP and hosts a web UI to control node behavior without reflashing:

- Configure LoRa address, network ID, channel, repeater address, and CRYPT key
- Send messages to default target or to discovered nearby nodes
- View runtime status, message stats, and radio diagnostics
- Change AP SSID and persist it in `Preferences`
- Restart automatically after config updates

It also keeps a nearby-node registry by processing periodic `HELLO` beacons and marks nodes online/offline using a TTL window.

## Main components

- `LoRa_E22` for E22 module communication and configuration
- `WebServer` for local configuration and monitoring pages
- `Preferences` for persistent config storage (`lora-config`)
- `Adafruit_SSD1306` OLED UI with multi-screen display and popup states
- `TinyGPSPlus` + `Serial1` for GPS parsing
- Battery ADC + charging pin logic for status display

## Default behavior

- Default node address low byte: `0x02`
- Default network ID: `0x01`
- Default channel: `0x41` (915 MHz region mapping in current setup)
- Default CRYPT key: `0x8002`
- AP SSID format: `LM-<MY_ADDL>` (for example `LM-2`)
- AP password: `12345678`

## Web routes

### UI pages

- `/` - Main control panel (send message, configure node, logs)
- `/setup` - Wi-Fi AP SSID setup page
- `/status` - Device + E22 diagnostic status page

### Actions

- `/send?msg=...` - Send direct message
- `/send?msg=...&relay=1` - Send via repeater mode
- `/send?msg=...&to=AABB` - Send to specific 16-bit hex address
- `/config?...` - Save new node config and restart
- `POST /setup/save` - Save AP SSID and restart

### JSON APIs

- `/api/status` - Node, traffic, battery, and E22 config info
- `/api/log` - Message log entries
- `/api/nodes` - Nearby node list with RSSI and online status

## Hardware mapping (from code)

### E22 module

- TX: GPIO `4`
- RX: GPIO `22`
- AUX: GPIO `18`
- M0: GPIO `21`
- M1: GPIO `19`

### OLED (I2C)

- SDA: GPIO `27`
- SCL: GPIO `33`
- Address: `0x3C`
- Size: `128x64`

### GPS (Serial1)

- TX: GPIO `25`
- RX: GPIO `26`
- Baud: `9600`

### Battery + buttons

- Battery voltage ADC: GPIO `36`
- Charging detect: GPIO `34`
- Next button: GPIO `32`
- Previous button: GPIO `35` (input-only, external pull-up recommended)

## Quick start

1. Open `Node.ino` in Arduino IDE.
2. Install required libraries:
   - `LoRa_E22`
   - `Adafruit GFX Library`
   - `Adafruit SSD1306`
   - `TinyGPSPlus`
3. Select your ESP32 board and upload.
4. Connect to AP (`LM-...`, password `12345678`).
5. Open the AP IP (shown on Serial/OLED, usually `192.168.4.1`) in browser.
6. Configure node parameters and save.

## Notes

- All nodes should share compatible `NETID`, channel, and `CRYPT` settings.
- `CRYPT = 0000` disables encryption in current logic.
- Repeater behavior depends on both app setting (`USE_REPEATER`) and E22 transmission mode configuration applied during setup.
