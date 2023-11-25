---
title: "Linux Cheatsheet"
read_time: false
excerpt: "Some essential Linux commands"
header:
  teaser: /assets/images/linux_teaser.jpg
  overlay_image: /assets/images/linux_teaser.jpg
  overlay_filter: 0.5 
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - linux
  - cheatsheet
---

# Apps

## linux

| command | description |
| :--- | :--- |
`htop` | activity monitor (sieht besser aus als `top`)
`hardinfo` | hardware info
`ncdu` | like `du -sh`, but more convenient (because avoids typing)

## ffmpeg

| command | description |
| :--- | :--- |
`ffmpeg` | mp4 to mp3 converter
`ffmpeg -i foo.mp4 bar.mp3` | convert foo.mp4 to bar.mp3
`ffmpeg -i source.mp4 -ss 00:00:00 -t 00:00:00 -vcodec copy -acodec copy outsplice.mp4` | crop source.mp4 from start time `-ss` to time `-t`
`ffmpeg -i input.mov -qscale 0 output.mp4` | convert input.mov to output.mp4
`ffmpeg -i input.mp4 -ss 00:05:20 -t 00:10:00 -c:v copy -c:a copy output1.mp4` | take the input video input.mp4, and cut out 10 minutes from it starting from 00:05:20 (5 minutes and 20 second mark), i.e. the output video will be from 00:05:20 to 00:15:20. If you specify a duration that will result in a stop time that is beyond the length of the input video, the output video will end where the input video ends. [source](https://shotstack.io/learn/use-ffmpeg-to-trim-video/)
`ffmpeg -i input.mp4 -ss 00:05:10 -to 00:15:30 -c:v copy -c:a copy output2.mp4` | uses `-to` to specify an exact time to cut to from the starting position. The cut video will be from 00:05:10 to 00:15:30, resulting in a 10 minutes and 20 seconds video. If you specify a time `-to` that is longer than the input video, e.g. `-to 00:35:00` when the input video is 20 minutes long, the cut video will end where the input video ends. If you specify a `-to` that is smaller than `-ss`, then the command won't run. You'll get the following error: `Error: -to value smaller than -ss; aborting.` [source](https://shotstack.io/learn/use-ffmpeg-to-trim-video/)

## convert (imagemagick)

| command | description |
| :--- | :--- |
`sudo apt install imagemagick` | image editing, converting, compressing, etc.
`convert path/to/image.png -resize 640x path/to/output_image.png` | compress an `image.png` by resizing / scaling down ([source](https://askubuntu.com/a/781588))
`convert path/to/image.png -quality 50% path/to/output_image.png` | compress an `image.png` by reducing its quality ([source](https://askubuntu.com/a/781588))
`convert *.PNG mydoc.pdf` | create a pdf from all `.PNG` files in the current folder; **important:** must run `sudo mv /etc/ImageMagick-6/policy.xml /etc/ImageMagick-6/policy.xmlout` first to fix `convert-im6.q16: attempt to perform an operation not allowed by the security policy PDF' @ error/constitute.c/IsCoderAuthorized/413.` error, [askubuntu](https://askubuntu.com/a/1081907)

## pdftoppm (poppler)

| command | description |
| :--- | :--- |
`sudo apt install poppler-utils` | install `pdftoppm`
`pdftoppm -png myfile.pdf > myfile.png` | convert **single page** PDF with poppler
`pdftoppm -png myfile.pdf myfile` | converts **multipage** PDF with poppler
`pdftoppm -jpeg myfile.pdf > myfile.jpg` | convert pdf to jpeg

## other

| command | description |
| :--- | :--- |
pyTranscriber | generates subtitles for `.mp3` files via Google Speech Recognition API using Autosub (GUI)

| command | description |
| :--- | :--- |
goldendict | dict for fast lookup (ctrl + c + c)

| command | description |
| :--- | :--- |
pycharm-community |

| command | description |
| :--- | :--- |
docker |

| command | description |
| :--- | :--- |
eog | picture viewer ([shortcuts](https://help.gnome.org/users/eog/stable/index.html.en))

| command | description |
| :--- | :--- |
pinta | picture editor ([shortcuts](https://www.pinta-project.com/user-guide/shortcuts/))

| command | description |
| :--- | :--- |
gedit | texteditor
zum Lesen: | unter F10/Preferences/Font & Colors/ Font ändern zu "TeX Gyre Termes Math Regular"
ctrl + h | find and replace (halte im "Find & Replace"-Fenster `alt` gedrückt für schnelle Auswahl der Optionen)
F10 | menu (u.a. Shortcuts)
F1 | help, Shortcut overview

| command | description |
| :--- | :--- |
kazam | screen recorder

| command | description |
| :--- | :--- |
joplin | Notes
alt + entsprechende Taste im menu | im Menu stehen alle Shortcuts !
ctrl + l | change view (editor/markdown viewer/both)
F10 | show all notebooks sidebar
F11 | show all Notes sidebar
ctrl + shift + l | focus note selection
ctrl + shift + b | focus body
ctrl + shift + n | focus title

| command | description |
| :--- | :--- |
dconf-editor | zB `gsettings set org.gnome.desktop.interface clock-show-weekday true` geht irgendwie nicht, stattdessen in dconf-editor zu org.gnome.desktop.interface navigieren und clock-show-weekday aktivieren. 

| command | description |
| :--- | :--- |
lm-sensors | get CPU temperature (using command `sensors`)

| command | description |
| :--- | :--- |
telegram-desktop | Telegram
zoom-client |
discord |

| command | description |
| :--- | :--- |
ticker | stock monitor

| command | description |
| :--- | :--- |
Tor-Browser-Bundle Webdownload | installation: see [here](https://wiki.ubuntuusers.de/Tor/Installation/#Tor-Browser-Bundle-Webdownload)

| command | description |
| :--- | :--- |
inxi -Fxz |        inxi  - Command line system information script for console and IRC
inxi -G | get Graphics info, eg. display resolution, GPU, etc.

| command | description |
| :--- | :--- |
cuda | [installation](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#removing-cuda-tk-and-driver) via .deb file 
nvidia-cuda-toolkit | manuell installiert mit `sudo apt install nvidia-cuda-toolkit`, nachdem cuda per .deb file installiert wurde 
nvidia-docker2 | [installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

| command | description |
| :--- | :--- |
droidcam | use Android smartphone cam as Ubuntu webcam

| command | description |
| :--- | :--- |
psensor | Unter "sensor preferences" im Tab "Application Indicator" das Kästchen "Display sensor in the label" aktivieren, damit ein bestimmter Wert im System Tray angezeigt wird.
conky | see [configuration](https://linuxconfig.org/ubuntu-20-04-system-monitoring-with-conky-widgets)

| command | description |
| :--- | :--- |
mailspring | mail client similar to Apple Mail

| command | description |
| :--- | :--- |
peek | screen2gif recorder

| command | description |
| :--- | :--- |
sqlite3 | A command line interface for SQLite version 3
sqlitebrowser | light GUI editor for SQLite databases

| command | description |
| :--- | :--- |
sudo apt-get install gnome-tweaks | GNOME tweak tool

| command | description |
| :--- | :--- |
sudo apt-get install dconf-editor | dconf editor

| command | description |
| :--- | :--- |
plank | dock similar to macOS

| command | description |
| :--- | :--- |
whatsapp-for-linux | whatsapp

| command | description |
| :--- | :--- |
1. sudo apt-get install compiz compiz-gnome compiz-plugins-extra | compiz Fenstermanager dependencies
2. sudo apt install compizconfig-settings-manager | compiz Fenstermanager

| command | description |
| :--- | :--- |
doxygen | create class diagram for C++ projects (Doxygen  is  a  documentation system for C++, C, Java, Objective-C, IDL (Corba and Microsoft flavors), Fortran, Python, VHDL and to some extent PHP, C#, and D.)
doxywizard | a tool to configure and run doxygen on your source files

| command | description |
| :--- | :--- |
qt5 | installiere via offline installer `qt-opensource-linux-x64-5.9.5.run`, weil qtcreator nur so automatisch qt konfiguriert (s. bookmarks in CS\/coding\/docker\/)
qtcreator | for GUI development

| command | description |
| :--- | :--- |
python2.7 | einfach per `apt install python2.7` installieren (Achtung: some apps also need `python2.7-dev`!); starten per `python2.7` (nur `python2` geht nicht!)

| command | description |
| :--- | :--- |
ocrmypdf | `ocrmypdf in.pdf out.pdf`

| command | description |
| :--- | :--- |
`ipad_charge` | [github link](https://github.com/mkorenkov/ipad_charge/wiki) automatically start charging ipad when connected to ubuntu

| command | description |
| :--- | :--- |
(nicht mehr) sudo apt install powerline | maybe install https://github.com/powerline/fonts, if symbols (e.g. branch symbol in git repo) are not displayed correctly	
sudo apt install fish | install fish shell (add `exec fish` in `.bashrc` to start automatically; in `fish_config` theme `fish default` wählen)
curl https://raw.githubusercontent.com/oh-my-fish/oh-my-fish/master/bin/install \| fish | install oh my fish (necessary for bobthefish)
omf install bobthefish | install powerline in fish

| command | description |
| :--- | :--- |
sudo snap install postman | http client 

| command | description |
| :--- | :--- |
dive | for docker image dependencies tree (see [github repo](https://github.com/wagoodman/dive))

| command | description |
| :--- | :--- |
barrier | share keyboard and mouse between multiple computers (client-server model) [**Note:** uncheck "enable SSL" box in barrier "Menu" -> "Change Settings" (on all devices)]

| command | description |
| :--- | :--- |
backgroundremover | `pip install backgroundremover` and then download `u2net.pth` from `https://drive.google.com/uc?id=1ao1ovG1Qtx4b7EoskHXmi2E9rp5CHLcZ` (or `https://drive.google.com/file/d/1ao1ovG1Qtx4b7EoskHXmi2E9rp5CHLcZ/view`) and save it as `/home/bra-ket/.u2net/u2net.pth`

| command | description |
| :--- | :--- |
v4l2-ctl | for setting webcam powerline frequency (see below)

| command | description |
| :--- | :--- |
sudo snap install pdftk | use `pdftk full-pdf.pdf cat 12-15 output outfile_p12-15.pdf` to save pages 12-15 from `full-pdf.pdf` in `outfile_p12-15.pdf`

| command | description |
| :--- | :--- |
`sudo apt install fzf` | general-purpose command-line fuzzy finder
`apt-cache show fzf` | will show `Refer /usr/share/doc/fzf/README.Debian for quick instructions on how to add keybindings for Bash, Zsh, Fish to call fzf.`
`vim /usr/share/doc/fzf/README.Debian` | how to `source` fzf

| command | description |
| :--- | :--- |
`sudo apt install fd-find` | fast alternative to `find`
`ln -s $(which fdfind) ~/.local/bin/fd` | because the binary name `fd` is already used by another package

| command | description |
| :--- | :--- |
`sudo apt install keepass2` | password manager

| command | description |
| :--- | :--- |
`sudo apt install gpick` | color picker (get hex code of a color by pointing with the mouse at some point on the screen, [stackoverflow](https://stackoverflow.com/a/10243630/12282296))

# Dotfiles

**Versioning**: see [atlassian.com](https://www.atlassian.com/git/tutorials/dotfiles)

# My aliases

`alias listssids='/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport /usr/local/bin/airport'`

In `~/.bashrc` (Ubuntu default):
`alias l='ls -CF'` <br />
`alias la='ls -A'` <br />
`alias ll='ls -alF'` <br />
(die restlichen Ubuntu alias gehen nicht bei Macbook Pro Mid 2010 wegen Doppelbindestrich Argument `--color=auto`)

In `~/.bash_aliases`:

`alias phth_ticker='ticker --config ~/snap/ticker/common/.ticker.yaml'`

| command | description |
| :--- | :--- |
`alias` | List all aliases
`type some_alias` | check the meaning of a specific alias

# System Folder

| command | description |
| :--- | :--- |
`~/.local/share/Trash/files` | `rm FILE` command moves `FILE` to this location

# General commands

| command | description |
| :--- | :--- |
ctrl + r + Suchbegriff | reverse search (mehrmals ctrl + r drücken, um zwischen den Suchbegriff enthaltenden commands auszuwählen, danach `->` um zu übernehmen bzw `Enter` um auszuführen)

## Root

| command | description |
| :--- | :--- |
`sudo` | an acronym for **SuperUser & Do** or **Switch User & DO**
`sudo -i` | run single command with root privileges (does not require root password !)
`sudo -s` | run single command with root privileges (does not require root password !) + do not change user and working directory
`su` | switches to super user (root user) (requires root password !) (in Ubuntu: root account disabled by default for improved security)
`su -` | `-` (`-l`, `--login`) flag: the shell switches from its original directory to a login shell that simulates an actual login.
`sudo su postgres` | Switches to the specified user's account (here: `postgres`) **and** it will inherit the original user's environment variables to target user
`sudo su - postgres` | Switches to the specified user's account (here: `postgres`), **but does not** inherit the original user's environment variables, instead it resets all environment variables and creates them again

## Nohup

| command | description |
| :--- | :--- |
`some_command &` | run process `some_command` in the background
`nohup gedit &` | start gedit in the background AND do not stop gedit, when shell is stopped. (Dies war ein einfaches Beispiel, aber es macht den eigentlichen Nutzen klar, wenn man z.B. per SSH auf einem fremden Rechner arbeitet und dort einen langwierigen Prozess starten möchte, die ssh-Verbindung aber während des Prozesses nicht permanent aktiv sein soll, weil man etwa den eigenen Rechner ausschalten möchte.)

## symlinks

Main advantages: 
- when you need to have a folder in multiple locations on your machine
  - **Saves storage** by avoiding duplicate folders on the machine
  - **Better versioning** by avoiding duplicate folders on the machine

| command | description |
| :--- | :--- |
`ln -s path/to/existing/FILE path/to/LINK` | create a symlink to a **file** (no `/` at the end of the paths!)
`ln -s path/to/existing/DIR path/to/LINK` | create a symlink to a **directory** (no `/` at the end of the paths!)
`readlink -f LINK` | show symlink target \[**ACHTUNG**: das heißt **nicht**, dass das target auch existiert, s.i [LINK](https://serverfault.com/a/76049) !\] 

## Open Files from the Terminal

| command | description |
| :--- | :--- |
xdg-open file | open file using default application
gio open file | same as xdg-open, but depends on what desktop the user has installed, whereas xdg-open is desktop agnostic

## Process Management

| command | description |
| :--- | :--- |
`top` | activity monitor
`ps` | wie `top`, aber keine real-time updates (dh. nur ein snapshot)
`ps -eo pid,lstart,cmd | grep 2127686` | start time of PID 2127686 (e.g. to get terminal start time, run `echo $$`, and then this command)
`echo $$` | show PID of current shell
`kill PID` | stop process with id `PID`, sends SIGTERM (i.e. kills gracefully) (see [notes bash]({% post_url 2021-09-25-cheatsheet-bash %}#job-control) and [kill doc](https://man7.org/linux/man-pages/man1/kill.1.html))
`pkill process_name` | stop all processes containing `process_name` (which is a regular expression), sends SIGTERM (i.e. kills gracefully), *Warning:* use `pgrep` first to check which processes will be killed
`pgrep process_name` | list all PIDs containing `process_name` (which is a regular expression)

Check PID of a window: 
```bash
xprop _NET_WM_PID | sed 's/_NET_WM_PID(CARDINAL) = //' | ps `cat`
```

## Get Paths

| command | description |
| :--- | :--- |
realpath foo.bar | get path to file "foo.bar" (like `pwd` + foo.bar)
readlink -f foo.bar | get path to file "foo.bar" (like `pwd` + foo.bar)

## Redirection, Pipe Tricks

| command | description |
| :--- | :--- |
`ls | wc -l` | count files in a directory
`history | tail -n 30` | show last 30 commands

# diff

Differences between two directory trees

| command | description |
| :--- | :--- |
`diff -r dir1/ dir2/` | outputs exactly what the differences are between corresponding files
`diff -qr dir1/ dir2/` | just getting a list of corresponding files whose content differs
`diff -qrN dir1/ dir2/` | to see differences for files that may not exist in either directory

# apt, apt-get, snap, dpkg, pkg-config

## Difference between apt and apt-get + apt-cache:

- `apt` = most commonly used command options from `apt-get` and `apt-cache` see [here](https://itsfoss.com/apt-vs-apt-get-difference/)
- So with `apt`, you get all the necessary tools in one place. You won’t be lost under tons of command options. The main aim of `apt` is to provide an efficient way of handling packages in a way “pleasant for end users”.
- `apt`:
  - shows progress bar while installing or removing a program
  - prompts number of packages that can be upgraded when you update the repository database (i.e. `apt update`)
  - same can be achieved with apt-get (but you need additional options)
- When you use `apt` to install a package, under the hood it uses `dpkg`. When you install a package using `apt`, it first creates a list of all the dependencies and downloads it from the repository.
  - Once the download is finished it calls `dpkg` to install all those files, satisfying all the dependencies.

| command | description |
| :--- | :--- |
`sudo apt update` |
`sudo apt [-y] upgrade` | `-y` oder `—yes` für automatic yes to prompts	
`apt --help` |
`sudo apt autoremove` | remove not needed packages (NOTE: This command will remove all unused packages (orphaned dependencies). Explicitly installed packages will remain.)
`sudo apt-mark auto $PACKAGES` | mark packages in variable `PACKAGES` as `automatically installed`, if accidentally marked as `manually installed`

## apt

### Include directories for C++

- `/usr/local/include/`
  - e.g. `/usr/local/include/opencv4/opencv2/`
- `/opt/`
  - e.g. `/opt/ros/`

### PPAs

**Personal Package Archives (PPAs)** are software repositories designed for Ubuntu users and are easier to install than other third-party repositories. PPAs are often used to distribute pre-release software so that it can be tested. ([source](https://help.ubuntu.com/stable/ubuntu-help/addremove-ppa.html.en))

Adding PPAs:

```bash
# add repository
sudo add-apt-repository ppa:whatever/ppa
```

Removing PPAs:

```bash
# remove repository
sudo add-apt-repository --remove ppa:whatever/ppa
```

You can also remove PPAs by deleting the `.list` files from `/etc/apt/sources.list.d` directory.

### Install

| command | description |
| :--- | :--- |
sudo apt install ./name.deb | install a .deb file
`sudo apt-get install <package name>=<version>` | install a specific version

### Uninstall

| command | description |
| :--- | :--- |
`sudo apt purge ...` | Removing packages with `sudo apt purge ...` or `sudo apt --purge remove ...` will remove them and all their global (i.e., systemwide) configuration files. This is usually what people mean when they talk about completely removing a package. **This does not remove packages that were installed as dependencies**, when you installed the package you're now removing. Assuming those packages aren't dependencies of any other packages, and that you haven't marked them as manually installed, you can remove the dependencies with `sudo apt autoremove` or (if you want to delete their systemwide configuration files too) `sudo apt --purge autoremove`.
`sudo apt --purge remove ...` | see `sudo apt purge ...`
`sudo apt autoremove` | remove the **dependencies** that are no longer needed
`sudo apt --purge autoremove` | remove **systemwide configuration files and the dependencies** that are no longer needed

### apt-cache

| command | description |
| :--- | :--- |
`apt-cache policy <package name>` | shows installed package version and also all the available versions in the repository according to the version of Ubuntu in which you are running
`apt-cache search <package_name>` | find specific package names

### apt-mark

| command | description |
| :--- | :--- |
`sudo apt-mark hold package_name` | hold packages (i.e. do not upgrade, remove or modify `package_name`)
`sudo apt-mark unhold package_name` | unhold "held" packages 

## dpkg

| command | description |
| :--- | :--- |
`sudo dpkg -l | less`	| list all installed dpkg packages [meaning of tags ii, rc, ...](https://askubuntu.com/questions/18804/what-do-the-various-dpkg-flags-like-ii-rc-mean)

| command | description |
| :--- | :--- |
|**Tipp:**|AM BESTEN DIE FOLGENDEN 3 ALLE AUSFÜHREN, DA JEDER EINEN ANDEREN OUTPUT HAT !
`sudo dpkg -l package`	|		confirm whether package is already installed (wenn nicht installed, dann wird “no packages found matching package” angezeigt) (ACHTUNG: exakten Namen schreiben, zB “lua” findet “lua5.1” nicht !) 
`sudo dpkg -l | grep package`	|	confirm whether package is already installed (wenn nicht installed, dann wird nichts angezeigt) (ACHTUNG regexp: zB “lua” findet “lua5.1” ! )
`sudo dpkg-query -s package`	| prüfe ob package installiert ist (ACHTUNG regexp: exakten Namen schreiben, zB “lua” findet “lua5.1” nicht !) und print weitere Informationen zum package

see also [how-to-show-history-of-installed-packages](https://www.linuxuprising.com/2019/01/how-to-show-history-of-installed.html)

| command | description |
| :--- | :--- |
`grep " install \| remove " /var/log/dpkg.log` | list recently installed OR removed packages (in the current month)
`grep " install " /var/log/dpkg.log.1` | list recently installed packages (in the previous month)
`zgrep " install " /var/log/dpkg.log.2.gz` | list recently installed packages (go back 2 months, same for >2 months)
`vim /var/log/apt/history.log` | view apt history

| command | description |
| :--- | :--- |
`sudo dpkg -i package_file.deb` | install `package_file.deb` (alternative: `sudo apt install ./name.deb`)
`sudo dpkg -P *some_package*` | purge `*some_package*`
`sudo dpkg -r *some_package*` | remove `*some_package*`
`sudo apt remove package` | uninstall `package_file.deb`

| command | description |
| :--- | :--- |
`dpkg -l | grep ^..r`   |   list all broken packages (**r** state (on the third field) means: reinst-required (package broken, reinstallation required))

| command | description |
| :--- | :--- |
`sudo vim /var/lib/dpkg/info/nvidia-cuda-toolkit.list` | in `/var/lib/dpkg/info/` sind die installation files (`.conffiles`, `.list`, `.md5sums`) für alle packages (hier: `nvidia-cuda-toolkit`)

## snap

- see `man snap` for details

| command | description |
| :--- | :--- |
snap list |
snap find *package* |
sudo snap install *package* |
sudo snap remove *package* |
sudo snap remove --purge *package* |

## pkg-config

| command | description |
| :--- | :--- |
`man pkg-config` | description of all pkg-config flags
`pkg-config --libs-only-l json-c` | was man im CMakeLists.txt in `target_link_libraries` eintragen muss (hier: `-ljson-c` Achtung: das `-l` muss auch im CMakeLists.txt rein!)
`pkg-config --libs-only-L json-c` | location of .so library file (hier: in ubuntu 18.04: findet er nicht, ist aber in `/lib/x86_64-linux-gnu`; in ubuntu 20.04: `-L/usr/local/lib`) (see also: [difference .so vs .a libraries](https://stackoverflow.com/a/9810368/12282296)) (muss nicht in CMakeLists.txt rein, s. [Minimalbsp](https://github.com/pharath/home/tree/master/assets/_code_examples/jsonc))
`pkg-config --cflags json-c` | include paths of the corresponding library with .h header files (hier: in ubuntu 18.04: `-I/usr/include/json-c`; in ubuntu 20.04: `-I/usr/local/include -I/usr/local/include/json-c`) (muss nicht in CMakeLists.txt rein)

# tty, terminal session management

From [stackexchange](https://unix.stackexchange.com/a/21294):
- A `tty` (teletype) is a native terminal device, the backend is either **hardware or kernel emulated**.
- A `pty` (pseudo-tty) is a terminal device which is **emulated by another program** (example: `xterm`, `screen`, or `ssh` are such programs). 
- A `pts` is the slave part of a `pty`.
- **More info**: see [stackexchange](https://unix.stackexchange.com/a/21294) and `man pty`

| command | description |
| :--- | :--- |
`tty` | zeigt Namen des aktiven terminals
`ls -ltr /dev/ttys*` | zeigt Namen aller aktiven terminals 
`last` | zeige letzte terminal logins
`whoami` | print the user name associated with the current effective user ID 

# chmod, Groups

**Default Permissions:**

From [baeldung](https://www.baeldung.com/linux/new-files-dirs-default-permission):

- On Linux, by default, when we create new **files**, they are given `rw-rw-r–` permissions.
  - The first `rw-` signifies read-write permissions for the user or the owner of the file
  - The second `rw-` indicates read-write permissions for the group the file belongs to
  - The final `r–` read permission is for all other users
- Similarly, for newly created **directories**, the default permission is `rwxrwxr-x`.

| command | description |
| :--- | :--- |
`chmod permissions-file` | is an abbreviation of change mode. A file's mode is the set of permissions attached to it that control access. Zu *permissions*: s. [here](https://askubuntu.com/tags/chmod/info).

| command | description |
| :--- | :--- |
`groups` | list all groups the currently logged in user belongs to (first group is the primary group)
`groups user` | same as `groups`, but for specific user `user`
`id` | "
`id user` | "

| command | description |
| :--- | :--- |
less /etc/group	| view all groups present on the system
cat /etc/group | "
getent group | "

| command | description |
| :--- | :--- |
getent group docker | list all members of group docker

| command | description |
| :--- | :--- |
`sudo groupadd docker` | add new group docker
`sudo usermod -aG docker $USER` | add my user to the docker group
`newgrp docker` | log out and log back in so that group membership is re-evaluated (nach group Aenderungen); wenn das nicht geht, reboot

# useradd, usermod, deluser

source: [baeldung](https://www.baeldung.com/linux/change-default-home-directory)

Run these commands as `root` user. Run them in TTY mode (press e.g. ctrl + alt + F3).

| command | description |
| :--- | :--- |
`sudo passwd root` | set the root's password
`ctrl + alt + f1` or `ctrl + alt + f2` ... | switch the TTY in order to log in as `root`
`sudo useradd -m baeldung` | creating a user called baeldung, in the default way
`sudo useradd -m -d /home/baeldung baeldung` | (**important:** do not use a slash character at the end of `/home/baeldung`, otherwise the terminal will show `/home/baeldung` instead of the `~` symbol at the start of each line !) create a user and set the location for the `home` directory at the same time
`sudo usermod -d /usr/baeldung baeldung` | change the user's home directory to `/usr/baeldung`
`sudo usermod -m -d /usr/baeldung baeldung` | also move the existing content to the new location
`startx` | start the Ubuntu GUI when you are in the tty
`sudo deluser --remove-home userName` | delete user `userName`

# bash

## Terminal Title

Change the title of the current terminal: `echo -ne "\033]0;SOME TITLE HERE\007"`, [askubuntu](https://askubuntu.com/a/22417)

## terminal shortcuts

![jumping with command line cursor](/assets/images/moving_cli.png)

| command | description |
| :--- | :--- |
ctrl + w | delete the word in front of the cursor
esc + backspace | delete a part of a path in front of the cursor (e.g. to get from `this/is/some/path` to `this/is/some/`)
alt + d | delete the word after the cursor
ctrl + a | jump to the beginning of the line
ctrl + e | jump to the end of the line
ctrl + s | freeze/block terminal
ctrl + q | unfreeze/unblock terminal
fn + links | scrolle nach ganz oben
cmd + oben | focus letzte input Zeile (zB gut, wenn man zB schnell hochscrollen will)

## terminal commands and bash scripting

### Shell

| command | description |
| :--- | :--- |
`echo $$` | display PID of current shell
`echo $0` | check current shell type
`bash` | start new bash shell instance in current bash shell (the new shell will have a different PID than the old one, check shell PID via `echo $$`)
`cat /etc/shells` | list all shells
`chsh` | change shell (you will be prompted to enter one of the shells in `cat /etc/shells`)

### env, PATH
	
| command | description |
| :--- | :--- |
`printenv` | Print the values of the specified environment VARIABLE(s).
`env` | show all environment variables
`env NAME=VALUE` | Set each NAME to VALUE in the environment
`echo $PATH` | Variable, die alle Pfade enthält, in denen Shell-Programme/Shell-Befehle (ls, echo, df, nautilus, etc.) gesucht werden
`echo $Variable` | display content of the variable "`Variable`"

### Finding Program Paths

| command | description |
| :--- | :--- |
`which <program>` | show the path of a program
`which python3` |		
`whereis python3` |

### Running Multiple Commands

| command | description |
| :--- | :--- |
`do_something1 && do_something2_that_depended_on_something1` | only run "something2", if "something1" completes successfully
`do_something1; do_something2` | run "something2" irrespective of "something1"

### find, locate

| command | description |
| :--- | :--- |
`find /opt/ -iname pattern` | find all files (hier: in dir `/opt/` ), for which base of file name (path with leading dirs removed) matches shell pattern `pattern` (Achtung: `pattern` muss genau übereinstimmen! Falls Endung unbekannt, mit Sternchen `*` am Ende suchen, dh. `pattern*` statt `pattern` suchen (wie bei `ls` Befehl).
`find /opt/ -name pattern` | wie -iname, aber case-sensitive
`find /opt/ -iname pattern -type f` | nur files suchen
`find /opt/ -iname pattern -type d` | nur dirs suchen
`find /opt/ ( -iname pattern1 -o -iname pattern2 )` | `-o` für oder
`find /opt/ -size +1G` | nur files, die über 1GB groß sind
`locate <file>` | faster than find, but uses a database which must be updated via `sudo updatedb` to find recent changes
`locate -i <file>` | case insensitive
`locate -b '\file.xyz'` | exact match (Note: the slash and the quotation marks are necessary)
`sudo updatedb` | update the `locate` command's database

### grep

| command | description |
| :--- | :--- |
`grep -rn -e 'nvidia' /var/log/apt/history.log*` | `r`: recursively look at all files in the folder, `n`: show line numbers, `e`: regex pattern, here: `nvidia` (w/o this some regex patterns will not work)
`grep -rnw -e 'nvidia' /var/log/apt/history.log*` | `w`: match whole words only (i.e. if the pattern `nvidia` is a substring of a word, it is not matched)
`grep -r -E 'orange|mango' .` | logical OR operator
`l | grep 150 | xargs rm -v` | pipe output of `grep` to `rm`
`l | grep xyzpattern | xargs cp -iv -t 150/` | pipe output of `grep` to `cp`
`l | grep xyzpattern | xargs mv -iv -t 1024p/` | pipe output of `grep` to `mv`

### sed

- for **line-based input**
  - thus, hard to replace `\n` with `sed`, `tr` is better here ([stackoverflow](https://stackoverflow.com/a/1252010))
- "Sed uses basic regular expressions (BRE). In a BRE, in order to have them treated literally, the characters `$.*[\^` need to be quoted by preceding them by a backslash, except inside character sets (`[…]`). Letters, digits and `(){}+?|` must not be quoted (you can get away with quoting some of these in some implementations)." ([more](https://unix.stackexchange.com/a/33005))

| command | description |
| :--- | :--- |
`sed 's/unix/linux/' geekfile.txt` | replaces the word 'unix' with 'linux' in the file 'geekfile.txt'. `sed` is mostly used to replace text in a file. Examples: see [here](https://www.geeksforgeeks.org/sed-command-in-linux-unix-with-examples/).
`sed -E` | use extended (ERE) regular expression syntax
`sed -e 's/ /\\ /g'` | useful when paths contain spaces and you need to escape these spaces with backslash, eg. when using `realpath`, `pwd`, `scp`, etc

### tr

- to replace **single characters** by **single characters**, [stackoverflow](https://stackoverflow.com/a/18366326/12282296)
- [tr SET notation](https://phoenixnap.com/kb/linux-tr)
- hard to replace `\n` with `sed`, `tr` is better for this task ([stackoverflow](https://stackoverflow.com/a/1252010))
  - pipe with `tr a b | sed -e 's/find/replace/g'` to replace strings

| command | description |
| :--- | :--- |
`tr '\n' ' '` | replace all `\n` with spaces, [tr SET notation](https://phoenixnap.com/kb/linux-tr)
`tr -d '\n'` | delete all `\n`, [tr SET notation](https://phoenixnap.com/kb/linux-tr)

### redirection, sort, head, tail

| command | description |
| :--- | :--- |
`exec > some_file` | redirect all shell output to `some_file`
`ls -ltr | vim -` | zeige Output eines Befehls in vim (ACHTUNG: Leerzeichen hinter "vim" nicht vergessen!)
`Befehl | head -3` |	zeige oberste 3 Zeilen des Outputs
`Befehl | tail -3` |
`du -sch ./folder | sort -rh | head -5` | zeige disk usage (=size) of folder (`-h` für human readable; `-s` für zeige auch Subdirectories; `-c` für zeige grand total am Ende) (`sort -rh` für sortiere nach size, wobei `-r` für reverse und `-h` für compare human readable sizes)
`echo "blabla" >> filename` | write output to file *filename*
`echo "blabla" | tee filename` | write output to file *filename*
`tee` | read from standard input and write to both standard output **and** files [doc](http://manpages.ubuntu.com/manpages/bionic/man1/tee.1.html) (name derived from "T-junction", since `tee` is usually used in pipes)

### Open in File Browser

| command | description |
| :--- | :--- |
nautilus .	|	öffne current directory in File Browser

### comments

| command | description |
| :--- | :--- |
`# ein comment` | Kommentar in command line

### Folder Operations

| command | description |
| :--- | :--- |
pwd | zeige current working directory
cd path/to/somedir |
mkdir -p /folder/subfolder/subsubfolder	| erstellt folder und subfolder automatisch, falls sie noch nicht existieren
cp -a | attempts to make a copy that's as close to the original as possible: same directory tree, same file types, same contents, same metadata (times, permissions, extended attributes, etc.). Always use `cp -a` instead of `cp -r`. (see [cp -a vs cp -r](https://unix.stackexchange.com/a/44981))

| command | description |
| :--- | :--- |
`dirs` | print the directory stack (left to right)
`dirs -v` | print the directory stack (one entry per line), prefixing each entry with its index in the stack, [doc](https://www.gnu.org/software/bash/manual/html_node/Directory-Stack-Builtins.html#Directory-Stack-Builtins)
`pushd path/to/somedir` | Adds a directory to the top of the directory stack, [doc](https://www.gnu.org/software/bash/manual/html_node/Directory-Stack-Builtins.html#Directory-Stack-Builtins)
`pushd +N` | Brings the Nth directory (counting from the left of the list printed by dirs, starting with zero) to the top of the list by rotating the stack, [doc](https://www.gnu.org/software/bash/manual/html_node/Directory-Stack-Builtins.html#Directory-Stack-Builtins)
`popd +N` | Removes the Nth directory (counting from the left of the list printed by dirs), starting with zero, from the stack, [doc](https://www.gnu.org/software/bash/manual/html_node/Directory-Stack-Builtins.html#Directory-Stack-Builtins)
`popd -N` | like `popd +N`, but counting from the right

### File Operations

| command | description |
| :--- | :--- |
mv -iv | **Tipp:** IMMER -iv BENUTZEN! (-i für bestätigen, -v für ausgeführte Aktion zeigen)
rm -iv | **Tipp:** IMMER -iv BENUTZEN! (-i für bestätigen, -v für ausgeführte Aktion zeigen)
cp -iv | **Tipp:** IMMER -iv BENUTZEN! (-i für bestätigen, -v für ausgeführte Aktion zeigen)

### history, script

| command | description |
| :--- | :--- |
`history` | get a list of the last 1000 commands 
`history | grep command_to_search` | search some pattern within the history generated list
`script` | start saving all input and output in the current terminal session in the file `typescript` (end recording via ctrl + d - this does not close the terminal here; use `script /path/to/mylogfile.txt` to save it in `/path/to/mylogfile.txt`; `typescript` will be overwritten if you start `script` twice without providing a name!). [source](https://askubuntu.com/a/557309)

# wmctrl

```bash
sudo apt install wmctrl
```

From [superuser](https://superuser.com/questions/142945/bash-command-to-focus-a-specific-window).

## List Window

- `wmctrl -l` (list windows)

## Focus Window

- `wmctrl -a window-name` (go to workspace and focus **by window name**)
  - map this to alt + shift + 1,2,3,etc. (after renaming the windows properly, renaming command: see below)
- `wmctrl -i -a 0x066f5d24` (go to workspace and focus **by window ID**)

## Rename Window

- `wmctrl -i -a 0x066f5d24 -T "new-name"` (rename window)

## With xdotool

- [more complex commands](https://superuser.com/a/950287)

# Unzipping

| command | description |
| :--- | :--- |
unzip file -d destination	|	unzip to destination
tar -C ./data/ -zxvf ~/Downloads/mnist.tgz | für .tgz (wobei -C target_location -zxvf source.tgz), .tar.gz
oder andersrum: |
tar -zxvf ~/Downloads/mnist.tgz -C ./data/ |
tar -C ./data/ -jxvf ~/Downloads/datei.tar.bz2 | für .tar.bz2 (dh. -j flag statt -z flag)
tar -C ~/ -xvf tor-browser-linux64-10.5.2_en-US.tar.xz | für .tar.xz

# System information

## Software

**Note**: `lsb_release` and `uname` may report different Kernel versions!

| command | description |
| :--- | :--- |
cat /etc/os-release | Ubuntu Version (lang)
cat /etc/lsb-release | Ubuntu Version (lang)
`lsb_release -a` | Ubuntu Version (kurz)
`lsb_release -cs` | Ubuntu Version (e.g. "focal")
hostnamectl | Ubuntu Version (mittel) mit Linux Kernel Version
uname --help | Returns the help manual for the uname command, including all available options.
uname -a | Prints all information for the server/system you're on.
uname -s | Prints the kernel name
uname -n | Prints the node name
uname -r | Prints the kernel release data
uname -v | Prints the kernel version data
uname -m | Prints the machine data
uname -p | Prints the processor information
uname -i | Prints the platform hardware information
uname -o | Prints the operating system information

## Hardware

| command | description |
| :--- | :--- |
lscpu |
lshw |
hwinfo --short |
lspci |
lsscsi |
lsusb |
inxi -Fx |
lsblk | list [block devices](#block-device-vs-character-device), e.g. to see all drives attached to your system, including their sizes and partitions
df -H |
pydf |
sudo fdisk -l |
`mount | column -t` |
free -m |

| command | description |
| :--- | :--- |
sudo dmidecode -t processor |
sudo dmidecode -t memory |
sudo dmidecode -t bios |

| command | description |
| :--- | :--- |
cat /proc/cpuinfo |
cat /proc/meminfo |
cat /proc/version |
cat /proc/scsi/scsi |
cat /proc/partitions |

## Udev

**Udev** is the **Linux subsystem** that supplies your computer with **device events**. In plain English, that means it's the code that **detects** when you have things **plugged into** your computer, like a network card, external hard drives (including USB thumb drives), mouses, keyboards, joysticks and gamepads, DVD-ROM drives, and so on.
- **udev scripting**: see [tutorial](https://opensource.com/article/18/11/udev)
  - how to create a udev script triggered by some **udev event**, such as plugging in a specific thumb drive

| command | description |
| :--- | :--- |
`udevadm monitor` | tap into udev in real time and see what it sees when you plug in different devices. The monitor function prints received events for: `UDEV`: the event udev sends out after rule processing, `KERNEL`: the kernel uevent
`udevadm control --reload` | should load all rules (but reboot if you want to be sure)

## Display

| command | description |
| :--- | :--- |
`xrandr` | list all display modes; set the size, orientation and/or reflection of the outputs for a screen; can also set the screen size
`xrandr --fb 2560x1440` | set the screen resolution, when no physical display is connected (e.g. when connecting to Jetson AGX via Teamviewer or VNCviewer, put this in `/etc/xdg/autostart/resolution_screen_teamviewer.sh`, `chmod +x /etc/xdg/autostart/resolution_screen_teamviewer.sh`, create `/etc/xdg/autostart/resolution_screen_teamviewer.desktop` and reboot and connect via Teamviewer again)

# Manage Drives (hard drive, usb flash drive)

| command | description |
| :--- | :--- |
diskutil list |
diskutil info /dev/disk2s2 |

| command | description |
| :--- | :--- |
sudo diskutil mountDisk /dev/disk2s2 | (Partitionsname disk2s2 steht in rechter Spalte bei diskutil list; /dev/disk2 mounted alle Unterpartitionen)
sudo diskutil umountDisk /dev/disk2s2 |
`mount_ntfs -o r "/Volumes/Volume node"` | (r für read-only; rw für read-write (NICHT MACHEN! Es gibt einen Grund warum das bei Mac per default nicht geht!)

| command | description |
| :--- | :--- |
df | zeige alle Laufwerke, ganz rechts steht die Location mit dem Inhalt des Datenträgers (zB `/media/bra-ket/UBUNTU 20_0`)
sudo fdisk -l | wie df, aber mehr Details
lsusb | list usb devices
lsblk | list [block devices](#block-device-vs-character-device)

## Block Device vs. Character Device

[unix.stackexchange](https://unix.stackexchange.com/a/259197):

"Probably you will never be able to find a simple definition of this. But in the most general and simplistic way, if you compare a character device to a block device, you can say **the character device** gives you direct access to the hardware, as in you put in one byte, that byte gets to the hardware (of course it is not as simple as that in this day and age). Whereas, **the block device** reads from and writes to the device in blocks of different sizes. You can specify the block size but since the communication is a block at a time, there is a buffering time involved."

"Think of a **block device** as a **hard disk** where you read and write one block of data at a time and, the **character device** is a **serial port**. You send one byte of data and other side receives that byte and then the next, and so forth and so on."

## Storage, Hard Disk, HDD, SSD

Must knows:
- SSD
  - [fragmentation](https://superuser.com/questions/97071/do-ssds-get-fragmented-and-if-they-do-is-that-an-issue)
  - [wear leveling](https://en.wikipedia.org/wiki/Wear_leveling)
  - [wear leveling](https://www.dell.com/support/kbdoc/de-de/000137999/hard-drive-why-do-solid-state-devices-ssd-wear-out?lang=en): As the term suggests, **wear leveling** provides a method for distributing program and erase cycles uniformly throughout all of the memory blocks within the SSD. This prevents continuous program and erase cycles to the same memory block, resulting in greater extended life to the overall NAND flash memory.

| command | description |
| :--- | :--- |
`hdparm` | get statistics about the hard disk, alter writing intervals, acoustic management, and DMA settings. It can also set parameters related to drive caches, sleep mode, power management, acoustic management, and DMA settings
`sudo hdparm -I /dev/sda` | Request **identification info** directly from the drive, which is displayed in a new expanded format with considerably more detail than with the older `-i` option. (source: `man hdparm -I`); may differ from information provided by `-i` option! (source: `man hdparm -i`)

## Eject

1. press on **eject** button for all partitions in nautilus
2. open **gnome-disks**
3. press on **stop** button below all partitions, if any partition is still mounted (else no stop button should be available)
4. press **power off** button in the top bar (only after all partitions have been unmounted in step 3!)
5. disk LED will be turned off now
6. close gnome-disks via alt + F4

| command | description |
| :--- | :--- |
`sudo eject /media/SDD` |

Troubleshooting:

1. `One or more applications are keeping the volume busy`

| command | description |
| :--- | :--- |
`sudo fuser -mv /media/SDD` | displays all processes accessing `/media/SDD`, where the `m` tells it to look on the given location, the `v` switches the output to a human readable list instead of just a bunch of PIDs. [askubuntu](https://askubuntu.com/a/578631)

2. `Disconnecting from filesystem` notification does not disappear automatically when ejecting an external hard drive (by pressing the eject button in Nautilus)

- **Possible Solutions that worked once:**
  - close all Nautilus/Dolphin windows that access the external hard drive
  - press **ctrl + c** on some file that is stored on the **internal** hard drive (if ctrl + c was pressed on some file that is stored on the **external** hard drive, this will block the external hard drive when you try to eject it)

3. in gnome-disks: External hard drive shows a **"loading"** symbol and the power off button for this external hard drive is **grayed out**

- **Possible Solutions that worked once:**
  - just wait
  - close gnome-disks and open it again
  - after a while the power off button for this external hard drive is not **grayed out** any more and the physical LED on the hard drive stops blinking

## Disk Usage

- [FAQ](https://unix.stackexchange.com/a/120312)
  - How much disk space does a file use?
  - Why is the total from `du` different from the sum of the file sizes?
  - What are these numbers from `df` exactly?
  - What's using the space on my disk?
- [Why is there a discrepancy in disk usage reported by df and du?](https://unix.stackexchange.com/questions/9612/why-is-there-a-discrepancy-in-disk-usage-reported-by-df-and-du)
- [Why du and df display different values](http://linuxshellaccount.blogspot.com/2008/12/why-du-and-df-display-different-values.html)

| command | description |
| :--- | :--- |
ncdu | like `du -sh`, but more convenient (because avoids typing)
`du -sh *` | 
`du -sch *` | `-c` to show grand total
`du -sh * | sort -h` | "ascending": largest file in the last output line
`du -sh * | sort -rh` | "descending": largest file in the first output line (`-r` for "reverse order")
`du -sh * .[^.]*` | 
`du -sh * .[^.]* | sort -h` | 
`du -h -d 1 *` | `-d 1` or `--max-depth=1` display the sizes of only the directories immediately within the specified path. If we were to specify 2 it would go a level further.
`du -h -d 1 -t 1G /` | show the sizes of all first level directories larger than 1GB within the root `/` path

| command | description |
| :--- | :--- |
`df -h` | to analyze the whole filesystem

# Camera

| command | description |
| :--- | :--- |
`mpv /dev/video0` | check, if device `/dev/video0` is the webcam
`vlc v4l2:///dev/video0` | check, if device `/dev/video0` is the webcam
`mplayer tv://device=/dev/video0` | check, if device `/dev/video0` is the webcam
`gst-launch-1.0 v4l2src device=/dev/video2 name=cam_src ! videoconvert ! videoscale ! video/x-raw,format=RGB ! queue ! videoconvert ! ximagesink name=img_origin` | check, if device `/dev/video2` is the webcam
`v4l2-ctl --list-devices` | list cameras
`v4l2-ctl -d /dev/video0 --list-ctrls` | show all settings
`v4l2-ctl -L -d /dev/videoN` | list available settings of camera `/dev/videoN`
`v4l2-ctl -d /dev/video0 --list-formats-ext` | show all supported camera resolutions
`v4l2-ctl --set-ctrl power_line_frequency=0 -d /dev/videoN` | set powerline frequency (50Hz vs 60Hz) of camera `/dev/videoN`
`v4l2-ctl -d /dev/video2 --list-formats-ext` | check supported pixel formats, fps and resolutions of camera `/dev/video2`
`v4l2-ctl --device /dev/video3 --set-ctrl=zoom_absolute=120` | zoom in

# GPU information

| command | description |
| :--- | :--- |
nvidia-smi -q -d temperature | temperature info including critical temperature values, shutdown temperature etc.
nvidia-smi --query-gpu=name --format=csv | get GPU name

# curl, wget

`curl` and `wget` are retrieval commands.

| command | description |
| :--- | :--- |
`wget -nc` | `-nc` for "do not overwrite existing files"
`wget -O output_file -q https://checkip.amazonaws.com -P DESTINATION` | `-O output_file`: benutze Minuszeichen "-" statt `output_file` wenn output direkt in Terminal erscheinen soll; `-q` für quiet; `-P` für Zielordner
`wget -A pdf,jpg -m -p -E -k -K -np http://site/path/` | get all pdfs and jpgs from site
`wget --accept pdf,jpg --mirror --page-requisites --adjust-extension --convert-links --backup-converted --no-parent http://site/path/` | same as above using long option names
`curl -s https://checkip.amazonaws.com` | `-s` für silent

# gpg, apt-key

**Note**: Do not forget to remove the respective sources list in `/etc/apt/sources.list.d/` as well.

| command | description |
| :--- | :--- |
gpg --list-keys | list your keys (will list only the ones stored in `~/.gnugpg`, but not the ones stored in `/etc/apt/trusted.gpg.d/`, see [stackoverflow](https://askubuntu.com/a/1262753))
gpg --delete-keys A3C4F0F979CAA22CDBA8F512EE8CBC9E886DDD89 | delete key A3C4F0F979CAA22CDBA8F512EE8CBC9E886DDD89 from keyring
apt-key list |
sudo apt-key del "27B2 5BF6 36CF 72B4 334D  AC98 F84C B847 29F1 B545" | it is safer to use the whole fingerprint, the keyid could have duplicates (at least when you use PGP for emails, I read you should share your whole fingerprint and not just the keyid)

# cron

The software utility cron also known as cron job is a time-based job scheduler in Unix-like computer operating systems. Users who set up and maintain software environments use cron to schedule jobs to run periodically at fixed times, dates, or intervals.

| command | description |
| :--- | :--- |
`crontab -e` | opens a file in which jobs can be specified (read this file for more info)

# network

## socket statistics

"socket" (aka the 2-Tuple (IP, Port), see DatKom.md)

| command | description |
| :--- | :--- |
`sudo netstat -lpn | grep :8889` | zeigt pid des Prozesses auf port 8889 (port kann dann mit `kill \<pid\>` frei gemacht werden)
`ss` | `ss` is the new `netstat` (`ss` is faster, more human-readable and easier to use); displays stats for PACKET, TCP, UDP, DCCP, RAW, and Unix domain sockets, [linux.com](https://www.linux.com/topic/networking/introduction-ss-command/)

## Download, Upload

| command | description |
| :--- | :--- |
`md5sum file` | to check if the file `file` is not corrupted after transferring it (check if the `md5sum` is the same on both sides of the file transfer)

## ssh

| command | description |
| :--- | :--- |
`w` | list all ssh sessions
`enter ~ .` | disconnect (when frozen), see `man ssh`, [stackoverflow](https://stackoverflow.com/questions/28981112/how-do-i-close-a-frozen-ssh-session)
`ssh bra-ket@10.14.14.60` | installiere vorher openssh-server auf beiden Computern
`firefox -no-remote -no-xshm` | display firefox on local client (no -X or -Y flag needed in previous ssh command)

**Achtung**: 
- erst in den Server einloggen und **dann** erst in den Computer einloggen, der die Internetverbindung des Servers benutzt !
- **Error**: "X11 connection rejected because of wrong authentication." 
  - `~/.Xauthority` löschen und nochmal per ssh einloggen kann helfen bei xauth Problemen (siehe [issue](https://unix.stackexchange.com/a/494742)) ! 
  - (prüfe evtl noch) nach [source](https://www.cyberciti.biz/faq/x11-connection-rejected-because-of-wrong-authentication/):   
    - Make sure X11 SSHD Forwarding Enabled
    - Make sure X11 client forwarding enabled

### Graphics/Display

| command | description |
| :--- | :--- |
`ssh -Y bra-ket@10.14.14.60` | display graphical output on trusted local client (**Caution**: may lead to security issues), [difference -X vs -Y flag](https://askubuntu.com/a/35518)
`ssh -X bra-ket@10.14.14.60` | display graphical output on **un**trusted local client, [difference -X vs -Y flag](https://askubuntu.com/a/35518)
`export DISPLAY=localhost:10.0` | set display (use `w` or `xauth list` to list diplays) ("**:0**" ist der server monitor; zB. "**localhost:10.0**" ist der client monitor, wobei `localhost:=127.0.0.1` (`127.0.0.1` is the loopback Internet protocol (IP) address also referred to as the localhost. The address is used to establish an IP connection to the same machine or computer being used by the end-user. The same convention is defined for computers that support IPv6 addressing using the connotation of `::1`.)

### On Macs

| command | description |
| :--- | :--- |
`caffeinate -u` | **for Mac**: prevent the system from sleeping and (-u for) prevent the system from sleeping [source](https://apple.stackexchange.com/questions/53802/waking-display-from-terminal-general-waking/161527)

### ssh keys

| command | description |
| :--- | :--- |
`ssh-keygen -R 10.14.14.92` | remove `10.14.14.92` from `.ssh/known_hosts` (falls aus Versehen geaddet)

## scp

**Achtung:** Spaces müssen im path **DOPPELT** escapet werden ! (s. [hier](https://stackoverflow.com/a/20364170/12282296))

| command | description |
| :--- | :--- |
scp *source* *target* | immer Anführungszeichen um den *source* Pfad setzen!
`scp -rv Macbook:"~/Desktop/Uni/FS1/Essential\ Astrophysics\ WS1819" ~/Desktop/` | spaces DOPPELT escapen (hier: 1. mit `"` **UND** 2. mit `\`) 
`scp -r [!.]* source target` | exclude hidden files

## rsync

Rsync patterns: [stackexchange](https://unix.stackexchange.com/a/2503)

| command | description |
| :--- | :--- |
`rsync -a path/to/source/ path/to/destination/` | copy directory; **note**: always use `/` at the end of the `path/to/source/` (**Warning**: `-a` better than `-r` because `-r` tag does not copy some stuff, e.g. symlinks)
`rsync -avz *source* *destination*` | `-z` flag: compressing and transfer the files (comes in handy while transferring a huge amount of data over a slow internet connection)
`rsync -av --progress` | show progress report

### rsync exclude

| command | description |
| :--- | :--- |
`rsync -a --exclude="SomeDirForPythonInstall"` | exclude directory "SomeDirForPythonInstall"
`rsync -a --exclude=".*"` | excludes hidden files and directories
`rsync -a --exclude=".*/"` | exclude hidden directories only
`rsync -av --progress sourcefolder /destinationfolder --exclude thefoldertoexclude` | exclude `thefoldertoexclude`
`rsync -av --progress sourcefolder /destinationfolder --exclude thefoldertoexclude --exclude anotherfoldertoexclude` | you can use `-exclude` multiple times ([stackoverflow](https://stackoverflow.com/a/14789400/12282296))
`rsync -av --progress ../../kitcar-gazebo-simulation/ ./kitcar-gazebo-simulation/ --exclude '*.bag'` | exclude all files ending with `.bag` in the current directory, no recursive traversal ([patterns](https://unix.stackexchange.com/a/2503))

### Resume partially scp-transferred files using Rsync

[source](https://ostechnix.com/how-to-resume-partially-downloaded-or-transferred-files-using-rsync/)

| command | description |
| :--- | :--- |
`rsync -P -rsh=ssh ubuntu.iso sk@192.168.225.22:/home/sk/` | resume partially transferred file `ubuntu.iso`
`rsync --partial -rsh=ssh ubuntu.iso sk@192.168.225.22:/home/sk/` | see above
`rsync -avP ubuntu.iso sk@192.168.225.22:/home/sk/` | see above
`rsync -av --partial ubuntu.iso sk@192.168.225.22:/home/sk/` | see above

# tree

| command | description |
| :--- | :--- |
`tree -H ./ > result.html` | save directory tree to file 
`firefox ./result.html` | view html tree created by `tree` command

# xmodmap, xev

**Deprecated**, use `setxkbmap` instead.

Use 
```bash
$ xev
``` 

to find the keycode of a key. Then, in order to remap this key, run e.g.
```bash
Syntax: xmodmap -e "keycode [keyNumber] = [normal] [shift] [NoIdea] [NoIdea] [altGr] [shift+altGr]"
$ xmodmap -e "keycode 48 = bracketright braceright NoSymbol NoSymbol adiaeresis Adiaeresis"
$ xmodmap -e "keycode 47 = bracketleft braceleft NoSymbol NoSymbol odiaeresis Odiaeresis"
$ xmodmap -e "keycode 45 = 0x06b 0x04b"
```

**Note**: You can look up `[keyNumber]` under [keysymdef.h](https://cs.gmu.edu/~sean/stuff/n800/keyboard/keysymdef.h).

Alternatively, create:
```bash
# .Xmodmap in $HOME/ directory

keycode 47 = bracketleft braceleft NoSymbol NoSymbol odiaeresis Odiaeresis
keycode 48 = bracketright braceright NoSymbol NoSymbol adiaeresis Adiaeresis
```

then in your `.bashrc` add
```bash
xmodmap ~/.Xmodmap
```

This will load these maps automatically after reboot.

TODO: The `xmodmap ~/.Xmodmap` in your `.bashrc` is executed every time a new terminal instance is launched which is **bad for terminal startup time**. Find another solution.

# Markdown

## retext

**Note**: Install the old `markdown` package version `python3.8 -m pip install markdown==3.2` first.

| command | description |
| :--- | :--- |
`retext markdown_file.md` | edit markdown_file.md
`retext --preview markdown_file.md` | preview `markdown_file.md`
**Tipp:**| Shortcuts: s. Menu &rarr; File und Edit
ctrl + e | preview on/off
ctrl + l | live preview on/off (die live updates brauchen manchmal bisschen)

## grip

- see [manpages.ubuntu.com](https://manpages.ubuntu.com/manpages/focal/man1/grip.1.html)
- "Preview GitHub Markdown files like Readme locally"
- view in firefox
- `grip file.md 6420` (to open a second file on a different port, here `6420`)
