---
title: "Ubuntu Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
toc: true
toc_sticky: true
categories:
- Cheatsheet
tags:
- ubuntu
- cheatsheet
---

# OS

## tty

- To switch ttys use ctrl + alt + F\#, where \# is the tty number you want to switch to.
- see [Switching between ttys](https://unix.stackexchange.com/a/167388)
    - Ubuntu 17.10+ assigns the X server to tty1 and a "dumb terminal" / "console" to tty2-7
        - older Ubuntus assign X server to tty7 instead of tty1!

## Unattended Upgrade

To see if there was an unattended upgrade check `/var/log/apt/history.log`. 

If `nvidia-smi` does not show the usual output check `/var/log/apt/history.log` because Nvidia also does some unattended upgrades, e.g.

```bash
Start-Date: 2022-12-03  06:13:58
Commandline: /usr/bin/unattended-upgrade
Upgrade: libnvidia-compute-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-compute-510:i386 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-encode-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-encode-510:i386 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), nvidia-kernel-common-510:amd64 (510.85.02-0ubuntu1, 510.108.03-0ubuntu0.20.04.1), xserver-xorg-video-nvidia-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-gl-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-gl-510:i386 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-fbc1-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-decode-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-decode-510:i386 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-cfg1-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), nvidia-utils-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), nvidia-dkms-510:amd64 (510.85.02-0ubuntu1, 510.108.03-0ubuntu0.20.04.1), nvidia-compute-utils-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), nvidia-driver-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), libnvidia-extra-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1), nvidia-kernel-source-510:amd64 (510.85.02-0ubuntu0.20.04.1, 510.108.03-0ubuntu0.20.04.1)
End-Date: 2022-12-03  06:15:52

Start-Date: 2022-12-03  06:16:02
Commandline: /usr/bin/unattended-upgrade
Upgrade: libnvidia-common-510:amd64 (510.85.02-0ubuntu1, 510.108.03-0ubuntu0.20.04.1)
End-Date: 2022-12-03  06:16:03
```

# Creating Shortcuts

## For Bluetooth Settings Icon in Ubuntu Activities

- Create .desktop file (".desktop" ending required!) with this content

```bash
[Desktop Entry]
Name=open_bluetooth_settings
Exec=gnome-control-center bluetooth
Terminal=false
Type=Application
```
- Make sure that your script is executable, like this:

```bash
sudo chmod +x /path/to/script.sh  
```

(if it does not execute, copy it to ~/Desktop/. There right-click on it and select "Allow Launching".)

![allow launching pic](https://i.ibb.co/2ZQfnGY/allow-launching.png)

- Move .desktop file to `/usr/share/applications/`. Press `Super` to open Activities and in the application grid view select "All" (instead of "Frequent") and search the created .desktop file there. Right-click on it and select "Add to Favorites". Now the .desktop file should be in the dock and can be accessed via shortcut `Super + <position_in_dock>`.

- It also won't work with if your script uses the sudo command, or anything else that requires user input.

If you want it to open a terminal window when you run it (if you needed to add input or watch the output) set Terminal to true.

```bash
Terminal=true
```

## Nautilus

Press `F4` to open the current directory in the terminal:
- see [askubuntu](https://askubuntu.com/a/696901)
    - **note**: do not forget `chmod +x Terminal`

# Network

## On the server computer:

1. Connect server and client via Ethernet cable.
2. On the computer, which is connected to the Internet, click the network icon in the panel and go to "Settings" at the bottom of the menu. [der Schieberegler von "Wired" beim Server sollte bereits aktiv sein, der Schieberegler beim Client sollte inaktiv sein (auch nachdem Kabel eingesteckt wurde)]
3. Click on the "Settings symbol" of your Wired Connection (Leave your wireless connection untouched).
4. On the "IPv4 Settings tab", select Method: "Shared to other computers". Click on "Apply" and close this window.
5. On the server reconnect by clicking on the Wired Network's toggle switch [dh. einmal aus und an], so it gets a new IP address. (The two computers must be connected by an ethernet cable for this step, so connect them now if you haven't already.) [Toggle Switch auf Client sollte noch inactive sein]
6. **Do not change IP settings. Leave everything as it is.**

## On the client computer:

1. Connect simply via toggle switch of Wired Connection. **IP settings are configured automatically!**

Mostly from: [Instructions](https://askubuntu.com/questions/359856/share-wireless-internet-connection-through-ethernet)

# Power

- [source](https://unix.stackexchange.com/a/317933) The Settings which determine what happens, when the Battery Level is critically low are in `/etc/UPower/UPower.conf`. There, the relevant entries are:
```bash
PercentageAction=2
CriticalPowerAction=HybridSleep
```

- for laptops: [source](https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-class-power)
   - `cat /sys/class/power_supply/BAT0/capacity` outputs laptop battery level (in percent)
   - `cat /sys/class/power_supply/BAT0/status` outputs laptop battery status (values: "Unknown", "Charging", "Discharging", "Not charging", "Full")
   - `cat /sys/class/power_supply/BAT0/capacity_level` outputs laptop battery level description (values: "Unknown", "Critical", "Low", "Normal", "High", "Full"). From doc: "Coarse representation of battery capacity."

# Gnome Shell

## Extensions

[extensions.gnome.org](https://extensions.gnome.org/)

Here you can manage all extensions installed on your machine.

### Installed Extensions

List installed extensions using 
```bash
# all
gnome-extensions list

# locally installed
ls .local/share/gnome-shell/extensions/

# system wide installed
ls /usr/share/gnome-shell/extensions/
```

```bash
# local
compiz-alike-magic-lamp-effect@hermes83.github.com
harddiskled@bijidroid.gmail.com
jiggle@jeffchannell.com
noannoyance@daase.net
panel-osd@berend.de.schouwer.gmail.com
switchWorkSpace@sun.wxg@gmail.com
user-theme@gnome-shell-extensions.gcampax.github.com
weeks-start-on-monday@extensions.gnome-shell.fifi.org
```

```bash
# system wide
desktop-icons@csoriano
ubuntu-appindicators@ubuntu.com
ubuntu-dock@ubuntu.com
```

## Launch new instance when there is no instance open ON CURRENT WORKSPACE

E.g. if a `gedit` instance is already open on some workspace, if you open a new instance, Ubuntu will always switch to that workspace first before opening the new instance which is presumably not what you want.

To change this behaviour: From [askubuntu](https://askubuntu.com/a/1162672)
```bash
gsettings set org.gnome.shell.extensions.dash-to-dock isolate-workspaces true
```

## Remove "Window is ready" popup and focus window

see [NoAnnoyance v2](https://extensions.gnome.org/extension/2182/noannoyance/)

## Switch Workspace with Ctrl+AboveTab

see [Switch Workspace](https://extensions.gnome.org/extension/1231/switch-workspace/)

# Tracker

see [wiki.ubuntuusers.de](https://wiki.ubuntuusers.de/Tracker/)

## Troubleshooting

Try everything on [this page](https://www.tobiasheide.de/hoher-cpu-verbrauch-durch-tracker-miner-fs-unter-ubuntu-20-04/).

Show the Tracker status:
```bash
$ tracker status
Currently indexed: 1458 files, 227 folders
Remaining space on database partition: 7,4 GB (5,92%)
Data is still being indexed: Estimated less than one second left
```

To see **where** Tracker hangs (here: see the last line in the output below):
```bash
$ tracker daemon
Store:
27 Mär 2023, 01:02:26:  ✗     Store                - Unavailable

Miners:
27 Mär 2023, 01:02:26:  ✗     Extractor            - Not running or is a disabled plugin
27 Mär 2023, 01:02:26:    1%  File System          - Crawling recursively directory 'file:///home/bra-ket/Desktop/react/tutorial/node_modules/@jest/expect-utils' 
```

Last time the solution was:

```bash
Über GUI Konfigurieren

Aufruf über Einstellungen -> Suchen. Dort gibt es oben in der Titelleiste den Button "Orte durchsuchen" über den Konfiguriert werden kann welche Verzeichnisse durchsucht werden sollen. Diesen Button habe ich die ersten Male auf der Einstellungsseite völlig übersehen.
```

In this GUI excluding the "Home" folder stopped `tracker-miner-fs` which was stuck (see above in the output of "tracker daemon") and had about `95%` CPU in `htop`.

# Appearance

## Restart GUI

Press Alt + F2, then type `r` and press Enter.

other methods: [https://linuxconfig.org/how-to-restart-gui-on-ubuntu-20-04-focal-fossa](https://linuxconfig.org/how-to-restart-gui-on-ubuntu-20-04-focal-fossa)

## Dock like MacOS


```bash
gsettings set org.gnome.shell.extensions.dash-to-dock extend-height false

gsettings set org.gnome.shell.extensions.dash-to-dock show-show-apps-button false 

gsettings set org.gnome.shell.extensions.dash-to-dock show-trash true
```

- add Launcher icon to dock: via .desktop icon to favorites using "[launch show-apps from command line](https://www.reddit.com/r/gnome/comments/emkaxp/invoking_gnome_show_applications_from_command_line/)"

## Hide Dock permanently

see [source](https://www.linuxuprising.com/2018/08/how-to-remove-or-disable-ubuntu-dock.html) Option 3

Hide:

```bash
gsettings set org.gnome.shell.extensions.dash-to-dock autohide false

gsettings set org.gnome.shell.extensions.dash-to-dock dock-fixed false

gsettings set org.gnome.shell.extensions.dash-to-dock intellihide false
```

Show:

```bash
gsettings set org.gnome.shell.extensions.dash-to-dock autohide true

gsettings set org.gnome.shell.extensions.dash-to-dock dock-fixed true

gsettings set org.gnome.shell.extensions.dash-to-dock intellihide true
```

# Set Default Applications

## Config files

- `/usr/share/applications/defaults.list` (root)
- `/etc/gnome/defaults.list` (root)
- `~/.config/mimeapps.list` (local user)

## Change default app

### Using xdg-settings

```bash
xdg-settings set default-web-browser firefox.desktop
```

### Using xdg-mime

```bash
xdg-mime default some_app some_filetype
```

E.g. 

```bash
xdg-mime default org.kde.okular.desktop application/pdf
```

creates an entry in the local MIME database `~/.config/mimeapps.list`.

## Change default mail client

If not already present, add
 
```bash
MimeType=x-scheme-handler/mailto;x-scheme-handler/mailspring;
```

to `~/.local/share/applications/mailspring_mailspring.desktop` and run 

```bash
update-desktop-database ~/.local/share/applications
```

There should be a new line `x-scheme-handler/mailto=mailspring_mailspring.desktop` in `~/.config/mimeapps.list` now.

## "Open with ..." and set default app for a given file

### For xdg-open

`xdg-mime default some_app some_filetype` (see above)

### For mimeopen

```bash
mimeopen -d myfile.pdf
```

will give you a list of applications that can open the file, and (the `-d` flag) will also update the default application for you. *Note*: After running this command the default works for `mimeopen myfile.pdf` only. `xdg-open myfile.pdf` and `nautilus` defaults need to be specified via `xdg-mime default some_app some_filetype` (see above)!

# Pager

## Nano

- `M-U` means "Meta Key" + <kbd>Alt</kbd>. The "Meta key" is not present on most keyboards. (Its use in software is for primarily historical reasons.) Usually, the meta key is emulated by another key on your keyboard. On Windows and Linux, it is usually the <kbd>Alt</kbd> key. On Mac OS X, that key (aka <kbd>Option</kbd>) already has other uses, and so <kbd>Escape</kbd> is used instead.

## Most

- set `most` as default pager is `.bashrc` to get coloured `man` pages

# Converter

## pptx (or ppt) to pdf

- `libreoffice --headless --invisible --convert-to pdf *.ppt`

# Sound

List all microphones:
```bash
sudo arecord -l
```

Record an audio file `/tmp/test-mic.wav` with 10s duration, where `hw:2,0` means using **card** `2` and **device** `0` (as shown by `arecord -l`):
```bash
arecord -f cd -c 1 -d 10 --device="hw:2,0" /tmp/test-mic.wav
```

Play an audio file `/tmp/test-mic.wav`:
```bash
aplay `/tmp/test-mic.wav`
```

# Webcam

## Troubleshooting

### Flickering

- caused by 50Hz vs 60Hz powerline frequency differences (see [source](https://blog.christophersmart.com/2017/02/07/fixing-webcam-flicker-in-linux-with-udev/))
    - **solution**: try `v4l2-ctl --set-ctrl power_line_frequency=0` or `v4l2-ctl --set-ctrl power_line_frequency=1`

# rofi

## Install

Follow the steps in [autotools](https://github.com/davatorium/rofi/blob/master/INSTALL.md#autotools) and [checkout](https://github.com/davatorium/rofi/blob/master/INSTALL.md#install-a-checkout-from-git).
 
```bash
git clone --recursive https://github.com/DaveDavenport/rofi
cd rofi
git checkout tags/1.7.5
autoreconf -i
mkdir build
cd build/
# run this multiple times to check which dependencies are missing until 
# you have installed all missing dependencies
../configure --disable-check
make
sudo make install
```

Select `sidebar by Qball` theme with "Rofi Theme Selector" (type "Rofi Theme Selector" in Ubuntu "Activities overview").

`sudo nvim /usr/local/share/rofi/themes/sidebar.rasi` and change
```css
window {
    height:   100%;
    //width: 30em;
    width: 100%;
    location: west;
    anchor:   west;
    border:  0px 2px 0px 0px;
    text-color: @lightwhite;
}
```

In Ubuntu Settings create a new Shortcut (press `'+'` button at the bottom to create a custom shortcut): 
- bind `super + b` to command `rofi -show bookmarksTree`.

# PDF Reader

## Okular

```bash
/usr/share/okular/tools.xml
```

Examples:
```xml
<annotation type="GeomSquare" width="1" color="#FF6868" />
```

```xml
<tool type="note-linked" id="1">
    <engine hoverIcon="tool-note" type="PickPoint" color="#ffff88">
        <annotation icon="Note" type="Text" color="#ffff88" />
    </engine>
    <shortcut>1</shortcut>
</tool>,
<tool type="note-inline" id="2" name="Inline Left 14 Mono">
    <engine hoverIcon="tool-note-inline" block="true" type="PickPoint" color="#ffffdd">
        <annotation opacity="0.85" type="FreeText" color="#ffffdd"
            font="Monospace\\,14\\,-1\\,5\\,50\\,0\\,0\\,0\\,0\\,0" />
    </engine>
    <shortcut>2</shortcut>
</tool>,
<tool type="highlight" id="3">
    <engine type="TextSelector" color="#ffff00">
        <annotation type="Highlight" color="#ffff00" />
    </engine>
    <shortcut>3</shortcut>
</tool>,
<tool type="highlight" id="4" name="Highlighter (pale)">
    <engine type="TextSelector" color="#ffff00">
        <annotation opacity="0.3" type="Highlight" color="#ffff00" />
    </engine>
    <shortcut>4</shortcut>
</tool>,
<tool type="underline" id="5">
    <engine type="TextSelector" color="#000000">
        <annotation type="Underline" color="#000000" />
    </engine>
    <shortcut>5</shortcut>
</tool>,
<tool type="underline" id="6">
    <engine type="TextSelector" color="#da0000">
        <annotation opacity="0.7" type="Underline" color="#da0000" />
    </engine>
    <shortcut>6</shortcut>
</tool>,
<tool type="straight-line" id="7">
    <engine points="2" type="PolyLine" color="#ff4a26">
        <annotation width="4" opacity="0.8" type="Line" color="#ff4a26" />
    </engine>
    <shortcut>7</shortcut>
</tool>,
<tool type="rectangle" id="8">
    <engine block="true" type="PickPoint" color="#ffff00">
        <annotation width="5" opacity="0.1" innerColor="#ffff00" type="GeomSquare" color="#ffff00" />
    </engine>
    <shortcut>8</shortcut>
</tool>,
<tool type="ellipse" id="9">
    <engine block="true" type="PickPoint" color="#00ffff">
        <annotation width="5" type="GeomCircle" color="#00ffff" />
    </engine>
    <shortcut>9</shortcut>
</tool>,
<tool type="ink" id="10" name="Freehand Line (2.0\\, black)">
    <engine type="SmoothLine" color="#000000">
        <annotation width="2" type="Ink" color="#000000" />
    </engine>
</tool>,
<tool type="ink" id="11" name="Freehand Line (2.0\\, Red)">
    <engine type="SmoothLine" color="#ff0000">
        <annotation width="2" type="Ink" color="#ff0000" />
    </engine>
</tool>,
<tool type="polygon" id="12">
    <engine points="-1" type="PolyLine" color="#007eee">
        <annotation width="1" type="Line" color="#007eee" />
    </engine>
</tool>,
<tool type="ink" id="13" name="Freehand Line (1.0\\, blue)">
    <engine type="SmoothLine" color="#0000ff">
        <annotation width="2" type="Ink" color="#0000ff" />
    </engine>
</tool>,
<tool type="stamp" id="14">
    <engine hoverIcon="bookmarks" size="64" block="true" type="PickPoint">
        <annotation opacity="0.5" icon="bookmarks" type="Stamp" />
    </engine>
</tool>,
<tool type="strikeout" id="15">
    <engine type="TextSelector" color="#c00000">
        <annotation opacity="0.8" type="StrikeOut" color="#c00000" />
    </engine>
</tool>,
<tool type="rectangle" id="16">
    <engine block="true" type="PickPoint" color="#ff0000">
        <annotation width="5" opacity="0.1" innerColor="#ffff00" type="GeomSquare" color="#ff0000" />
    </engine>
</tool>,
<tool type="note-inline" id="17" name="Heading Centre 48 Opaque">
    <engine hoverIcon="tool-note-inline" block="true" type="PickPoint" color="#ffffff">
        <annotation type="FreeText" color="#ffffff"
            font="Sans Serif\\,48\\,-1\\,5\\,50\\,0\\,0\\,0\\,0\\,0" align="1" />
    </engine>
</tool>,
<tool type="note-inline" id="18" name="Inline Center 12">
    <engine hoverIcon="tool-note-inline" block="true" type="PickPoint" color="#ffffee">
        <annotation opacity="0.85" type="FreeText" color="#ffffee"
            font="Sans Serif\\,12\\,-1\\,5\\,50\\,0\\,0\\,0\\,0\\,0" align="1" />
    </engine>
</tool>,
<tool type="polygon" id="19" name="Red Filled Polygon">
    <engine points="-1" type="PolyLine" color="#ff4a26">
        <annotation width="1" opacity="0.8" innerColor="#ff4a26" type="Line" color="#ff4a26" />
    </engine>
</tool>,
<tool type="polygon" id="20" name="Whiteout Poly">
    <engine points="-1" type="PolyLine" color="#ffffff">
        <annotation width="1" innerColor="#ffffff" type="Line" color="#ffffff" />
    </engine>
</tool>,
<tool type="note-inline" id="21" name="Inline Left 10 Mono">
    <engine hoverIcon="tool-note-inline" block="true" type="PickPoint" color="#ffffdd">
        <annotation opacity="0.85" type="FreeText" color="#ffffdd"
            font="Monospace\\,10\\,-1\\,5\\,50\\,0\\,0\\,0\\,0\\,0" />
    </engine>
</tool>,
<tool type="note-inline" id="22" name="Inline Left 6 Mono">
    <engine hoverIcon="tool-note-inline" block="true" type="PickPoint" color="#ffffdd">
        <annotation type="FreeText" color="#ffffdd"
            font="Monospace\\,6\\,-1\\,5\\,50\\,0\\,0\\,0\\,0\\,0" />
    </engine>
</tool>
```
