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

## Local database design

For local storage (mobile app, desktop app, or gateway service), use SQLite with the following tables.

### 1) Contacts

Store known peers for direct messaging and address book UI.

```sql
CREATE TABLE contacts (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  lora_address TEXT NOT NULL UNIQUE,     -- Example: "0002"
  display_name TEXT NOT NULL,
  avatar_url TEXT,
  is_blocked INTEGER NOT NULL DEFAULT 0,
  created_at TEXT NOT NULL DEFAULT (datetime('now')),
  updated_at TEXT NOT NULL DEFAULT (datetime('now'))
);
```

### 2) Groups

Store group metadata and ownership.

```sql
CREATE TABLE groups (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  group_uuid TEXT NOT NULL UNIQUE,       -- Stable app-level group ID
  group_name TEXT NOT NULL,
  owner_contact_id INTEGER NOT NULL,     -- Group owner
  created_at TEXT NOT NULL DEFAULT (datetime('now')),
  updated_at TEXT NOT NULL DEFAULT (datetime('now')),
  FOREIGN KEY (owner_contact_id) REFERENCES contacts(id)
);
```

### 3) Group members

Map contacts to groups, including owner/admin/member roles.

```sql
CREATE TABLE group_members (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  group_id INTEGER NOT NULL,
  contact_id INTEGER NOT NULL,
  role TEXT NOT NULL DEFAULT 'member',   -- owner | admin | member
  joined_at TEXT NOT NULL DEFAULT (datetime('now')),
  is_active INTEGER NOT NULL DEFAULT 1,
  UNIQUE (group_id, contact_id),
  FOREIGN KEY (group_id) REFERENCES groups(id) ON DELETE CASCADE,
  FOREIGN KEY (contact_id) REFERENCES contacts(id) ON DELETE CASCADE
);
```

### 4) Messages

Store both direct and group messages in one table.

```sql
CREATE TABLE messages (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  message_uuid TEXT NOT NULL UNIQUE,
  chat_type TEXT NOT NULL,               -- direct | group
  from_contact_id INTEGER NOT NULL,
  to_contact_id INTEGER,                 -- used when chat_type = 'direct'
  group_id INTEGER,                      -- used when chat_type = 'group'
  payload TEXT NOT NULL,                 -- encrypted/plain text payload
  payload_type TEXT NOT NULL DEFAULT 'text',
  delivery_status TEXT NOT NULL DEFAULT 'pending', -- pending/sent/delivered/failed
  sent_at TEXT,
  received_at TEXT,
  created_at TEXT NOT NULL DEFAULT (datetime('now')),
  FOREIGN KEY (from_contact_id) REFERENCES contacts(id),
  FOREIGN KEY (to_contact_id) REFERENCES contacts(id),
  FOREIGN KEY (group_id) REFERENCES groups(id) ON DELETE CASCADE
);
```

### Recommended indexes

```sql
CREATE INDEX idx_contacts_lora_address ON contacts(lora_address);
CREATE INDEX idx_group_members_group_id ON group_members(group_id);
CREATE INDEX idx_group_members_contact_id ON group_members(contact_id);
CREATE INDEX idx_messages_direct_chat ON messages(chat_type, from_contact_id, to_contact_id, created_at);
CREATE INDEX idx_messages_group_chat ON messages(chat_type, group_id, created_at);
CREATE INDEX idx_messages_delivery_status ON messages(delivery_status);
```

### Notes for usage

- A **direct chat** row has `chat_type='direct'` and `to_contact_id` set.
- A **group chat** row has `chat_type='group'` and `group_id` set.
- Keep the group owner in `groups.owner_contact_id`; also add the same owner in `group_members` with `role='owner'`.
- Use `message_uuid` for deduplication when receiving repeated LoRa packets.
- Enable SQLite foreign keys at startup: `PRAGMA foreign_keys = ON;`.

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
