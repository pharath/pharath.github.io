---
title: "Mac and iOS Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - mac
  - iOS
  - cheatsheet
---

# Wifi settings

```bash
cd /System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/
./airport en1 prefs # set wifi connection behaviour (eg. DisconnectOnLogout=NO)
```

# ssh 

**Achtung:** `ssh` muss auf Mac erst aktiviert werden: unter "System Preferences"/"Sharing" bei "Remote login" Häkchen setzen!

| command | description |
| :---: | :---: |
`caffeinate -u` | prevent the system from sleeping and (-u for) prevent the system from sleeping [source](https://apple.stackexchange.com/questions/53802/waking-display-from-terminal-general-waking/161527)

# AirDrop 

- works over Bluetooth (though WLAN must be enabled)
- "AirDrop uses Bluetooth to create a peer-to-peer Wi-Fi network between the devices. That means you don't need to be connected to your router or even the internet in order to have an AirDrop connection. You do have to have Wi-Fi and Bluetooth turned on, however.", [source](https://www.lifewire.com/what-is-airdrop-how-does-it-work-1994512)

## File Transfer From iPad to iPhone without Wifi

- turn on wifi on both devices
- turn on **Airdrop** (under "Settings" &rarr; "General" &rarr; "Airdrop" &rarr; Select "Everyone")
- select the photo, file, etc. which you want to transfer
- tap on "Share" button on **both** devices
  - when the other device is discovered, it will be listed in the AirDrop field in the "Share" menu of each device 
- tap on the device to which you want to send the file
