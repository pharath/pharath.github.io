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

# Shortcuts

## Disable Annoying Gnome Shortcuts

- **emoji autocomplete popup**: <kbd>ctrl</kbd><kbd>shift</kbd><kbd>e</kbd> (annoying when using IntelliJ)
  - run `ibus-setup`, there you can remove this shortcut under "Emoji" &rarr; "Keyboard Shortcuts", [unix.stackexchange](https://unix.stackexchange.com/a/623413)

## For Bluetooth Settings Icon in Ubuntu Activities

- Create `.desktop` file (`.desktop` ending required!) with this content

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

(if it does not execute, copy it to `~/Desktop/`. There right-click on it and select "Allow Launching".)

![allow launching pic](https://i.ibb.co/2ZQfnGY/allow-launching.png)

- Move `.desktop` file to `/usr/share/applications/`. Press `Super` to open Activities and in the application grid view select "All" (instead of "Frequent") and search the created `.desktop` file there. Right-click on it and select "Add to Favorites". Now the `.desktop` file should be in the dock and can be accessed via shortcut `Super + <position_in_dock>`.
- Warning: It won't work if your script uses the `sudo` command, or anything else that requires user input.

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
disableworkspaceanim@owilliams.mixxx.org
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

## Gnome-Terminal

| command | description |
| :--- | :--- |
`gnome-terminal --title="TITLE"` |

# Gnome Display Manager (GDM)

- a display manager
- a graphical login manager
- for x11 and Wayland (windowing systems)

| command | description |
| :--- | :--- |
`sudo systemctl restart gdm.service` | restart GDM (kills the current login session, so that you have to log in again, but some processes keep running, eg. `tmux`, `nohup` processes (eg. `nohup process &`), [disowned background processes](https://www.networkworld.com/article/969269/how-to-keep-processes-running-after-logging-off-in-linux.html), etc.)

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

# Storage

## Disk

`baobab` aka **Disk Usage Analyzer**

## duck

supports many "protocols" like Google Drive, Dropbox, Amazon S3, OneDrive, ownCloud, WebDAV, etc

TODO: duck CLI commands

[doc: duck CLI](https://docs.duck.sh/cli/)

## mega.io CLI

[github repo](https://github.com/meganz/MEGAcmd)

[UserGuide](https://github.com/meganz/MEGAcmd/blob/master/UserGuide.md)

install:

```bash
wget https://mega.nz/linux/repo/xUbuntu_20.04/amd64/megacmd-xUbuntu_20.04_amd64.deb && sudo apt install "$PWD/megacmd-xUbuntu_20.04_amd64.deb"
```

There is a "interactive mode" (like python repl) and a "scriptable mode".

[list of commands](https://github.com/meganz/MEGAcmd/blob/master/UserGuide.md#command-summary)

### Login

| command | description |
| :--- | :--- |
`mega-login email password` | where `!` in `password` must be escaped with backslash

### Storage Space Management

- <span style="color:red">**Unlike Git**</span>, in mega a file version that has a size of X MB will use X MB space! (You can check this via `mega-du --versions`.)

| command | description |
| :--- | :--- |
`mega-du -h` | Prints size used by files/folders
`mega-du --versions -h remotefile` | total size of all file versions of `remotefile` together

### Upload

| command | description |
| :--- | :--- |
`mega-put localfile remotedest` | upload `localfile`, where `localfile` is relative to the current directory of the shell (not relative to `mega-lpwd`!), automatically creates a new version of `localfile` if it is already existing in the remote

### Download

| command | description |
| :--- | :--- |
`mega-ls -lh dir/` | list of files with size, Modification date for files and creation date for folders (see `mega-ls --help`)
`mega-ls --versions remotefile` | list versions of `remotefile` (<span style="color:red">**warning:**</span> mega supports only <span style="color:red">**100 file versions**</span>! mega uses a smart algorithm to decide which version to remove first, see [help.mega.io](https://help.mega.io/files-folders/restore-delete/file-version-history))
`mega-ls -lh --versions remotefile` | list versions and sizes of `remotefile`
`mega-get remotefile` | download the latest version of `remotefile` into the current directory
`mega-get remotefile#1723078907` | download the version `#1723078907` of `remotefile` into the current directory

### Remote File Operations

| command | description |
| :--- | :--- |
`mega-cp remote/src/path/ remote/dest/path/` | only for copying remote files (you cannot `mega-cp` local files to the remote! use `mega-put` for that)
`mega-rm remotefile` | remove all versions of `remotefile`
`mega-rm remotefile#1723078907` | remove a specific version of `remotefile`, workaround to remove the latest version (A): STEP 1: download and upload the previous version (B), STEP 2: then the latest version (A) is assigned an ID so that you can `mega-rm remotefile#1723078907` on the ID of the latest version (A), STEP 3: then delete the previous version (B) (which is unnecessary now because you have uploaded the same file in STEP 1)
`mega-mkdir remotefolder` | create a new folder `remotefolder` on the remote (warning: do not use a trailing slash behind the foldername like in `mega-mkdir remotefolder/` or you will get an error `Use -p to create folders recursively`)

### Interactive Mode

| command | description |
| :--- | :--- |
`mega-lpwd` | only used for "interactive mode", whereas in "scriptable mode" the current directory of the shell is used as `lpwd`

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

**Get**:

```bash
# To view the current default for opening directories:
xdg-mime query default inode/directory
```

**Set**:

```bash
xdg-mime default some_app some_filetype
```

E.g. 

```bash
xdg-mime default org.kde.okular.desktop application/pdf

# To set Dolphin as default file manager:
xdg-mime default org.kde.dolphin.desktop inode/directory
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

# File Manager

## Dolphin

```bash
sudo apt install dolphin
```

```bash
sudo nvim /usr/share/applications/org.kde.dolphin.desktop

# in org.kde.dolphin.desktop set:
Exec=dolphin %u --new-window
```

# Pager

## Nano

- `M-U` means "Meta Key" + <kbd>Alt</kbd>. The "Meta key" is not present on most keyboards. (Its use in software is for primarily historical reasons.) Usually, the meta key is emulated by another key on your keyboard. On Windows and Linux, it is usually the <kbd>Alt</kbd> key. On Mac OS X, that key (aka <kbd>Option</kbd>) already has other uses, and so <kbd>Escape</kbd> is used instead.

## Most

- set `most` as default pager is `.bashrc` to get coloured `man` pages

# Mail

use Thunderbird

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

## Troubleshooting

**Problem 1:** No sound in **Totem** Videos:

- let the video that has no sound play in Totem
- open Ubuntu Sound Settings
- under section **"Volume Levels"** below eg. "System Sounds", "Firefox", etc. a new item for the currently playing video will be shown
- check if the Volume for this video is **enabled** and is **sufficiently high**
- in addition, check the Volume in the Totem Video App

# Webcam

## Troubleshooting

### Flickering

- caused by 50Hz vs 60Hz powerline frequency differences (see [source](https://blog.christophersmart.com/2017/02/07/fixing-webcam-flicker-in-linux-with-udev/))
  - **solution**: try `v4l2-ctl --set-ctrl power_line_frequency=0` or `v4l2-ctl --set-ctrl power_line_frequency=1`

# buku

Bookmark Manager

```bash
pip3 install buku
pip3 install flask # fixes error
# gui
pip3 install "buku[server]"
# start bukuserver, open in firefox
bukuserver run --host 127.0.0.1 --port 5001
# import firefox bookmarks
buku --import ~/Downloads/bookmarks27jan24.html
```

```bash
# buku -S:

# Search bookmarks with ALL the keywords "kernel" and "debugging" in URL, title or tags:
buku -S kernel debugging

# buku -t:

# whitespaces are considered, eg. in "some tag with whitespaces"
# minus signs are not considered, eg. in "some - tag - with - minus - signs"

# OR
buku -t tag1, tag2

# AND
buku -t tag1 + tag2

# NOT
buku -t tag1 - tagToExclude1, tagToExclude2

# add tags
# (omitting the "+" removes all tags and replaces them with "newTag")
buku -t tag1 -s pattern -u --tag + newTag

# remove tags
buku -t tag1 -s pattern -u --tag - newTag

# print just urls
buku -t tag1, tag2 -f 1
```

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

### Shortcuts

| command | description |
| :--- | :--- |
F6 | toggle annotation tools bar
F7 | toggle page preview pane
ctrl + m | toggle menu bar
ctrl + shift + , | configure okular
ctrl + e | configure shortcuts

### Annotation Tools

**Configure annotation tools via GUI**:

- right click on the annotation widget/bar (**F6** to toggle the annotation widget/bar)

**Configure annotation tools via `tools.xml`**:

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
