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

## system info

| command    | description                                                |
| :--------- | :--------------------------------------------------------- |
| `htop`     | activity monitor (sieht besser aus als `top`)              |
| `hardinfo` | hardware info                                              |
| `ncdu`     | like `du -sh`, but more convenient (because avoids typing) |

## ffmpeg

| command                                                                                                         | description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| :-------------------------------------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ffmpeg -i foo.mp4 bar.mp3`                                                                                     | mp42mp3: convert `foo.mp4` to `bar.mp3`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `ffmpeg -i input.webm -c:v libx264 -preset slow -crf 22 -c:a aac -b:a 128k output.mp4`                          | webm2mp4: convert `input.webm` to `output.mp4`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| `ffmpeg -i source.mp4 -ss 00:00:00 -t 00:00:00 -vcodec copy -acodec copy outsplice.mp4`                         | crop `source.mp4` from start time `-ss` to time `-t`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `ffmpeg -i input.mov -qscale 0 output.mp4`                                                                      | mov2mp4: convert `input.mov` to `output.mp4`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `ffmpeg -i input.mp4 -ss 00:05:20 -t 00:10:00 -c:v copy -c:a copy output1.mp4`                                  | take the input video `input.mp4`, and cut out 10 minutes from it starting from 00:05:20 (5 minutes and 20 second mark), i.e. the output video will be from 00:05:20 to 00:15:20. If you specify a duration that will result in a stop time that is beyond the length of the input video, the output video will end where the input video ends. [source](https://shotstack.io/learn/use-ffmpeg-to-trim-video/)                                                                                                                                                                            |
| `ffmpeg -i input.mp4 -ss 00:05:10 -to 00:15:30 -c:v copy -c:a copy output2.mp4`                                 | uses `-to` to specify an exact time to cut to from the starting position. The cut video will be from 00:05:10 to 00:15:30, resulting in a 10 minutes and 20 seconds video. If you specify a time `-to` that is longer than the input video, e.g. `-to 00:35:00` when the input video is 20 minutes long, the cut video will end where the input video ends. If you specify a `-to` that is smaller than `-ss`, then the command won't run. You'll get the following error: `Error: -to value smaller than -ss; aborting.` [source](https://shotstack.io/learn/use-ffmpeg-to-trim-video/) |
| `ffmpeg -i animated.gif -movflags faststart -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" video.mp4` | gif2mp4: convert `animated.gif` to `video.mp4`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| `ffmpeg -i input.mp4 -ss 00:00:30 -t 00:00:50 -q:a 0 -map a output.mp3`                                         | mp42mp3: extract audio                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `ffmpeg -i input.mp4 -vcodec libx265 -crf 28 output.mp4`                                                        | push the compression lever further by increasing the CRF value — add, say, 4 or 6, since a reasonable range for H.265 may be 24 to 30. Note that lower CRF values correspond to higher bitrates, and hence produce higher quality videos., [unix.stackexchange](https://unix.stackexchange.com/a/38380)                                                                                                                                                                                                                                                                                  |

## vlc

| command                                                  | description  |
| :------------------------------------------------------- | :----------- |
| `vlc -L -- "-some-filename-that-starts-with-a-dash.mp4"` | loop a video |

## convert (imagemagick)

| command                                                           | description                                                                                                                                                                                                                                                                                                                                                        |
| :---------------------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sudo apt install imagemagick`                                    | image editing, converting, compressing, etc.                                                                                                                                                                                                                                                                                                                       |
| `convert path/to/image.png -resize 640x path/to/output_image.png` | compress an `image.png` by resizing / scaling down ([source](https://askubuntu.com/a/781588))                                                                                                                                                                                                                                                                      |
| `convert path/to/image.png -quality 50% path/to/output_image.png` | compress an `image.png` by reducing its quality ([source](https://askubuntu.com/a/781588))                                                                                                                                                                                                                                                                         |
| `convert *.PNG mydoc.pdf`                                         | create a pdf from all `.PNG` files in the current folder; **important:** must run `sudo mv /etc/ImageMagick-6/policy.xml /etc/ImageMagick-6/policy.xmlout` first to fix `convert-im6.q16: attempt to perform an operation not allowed by the security policy PDF' @ error/constitute.c/IsCoderAuthorized/413.` error, [askubuntu](https://askubuntu.com/a/1081907) |
| `convert EXAMPLE.png EXAMPLE.svg`                                 | convert png to svg                                                                                                                                                                                                                                                                                                                                                 |

## pdftoppm (poppler)

| command                                  | description                              |
| :--------------------------------------- | :--------------------------------------- |
| `sudo apt install poppler-utils`         | install `pdftoppm`                       |
| `pdftoppm -png myfile.pdf > myfile.png`  | convert **single page** PDF with poppler |
| `pdftoppm -png myfile.pdf myfile`        | converts **multipage** PDF with poppler  |
| `pdftoppm -jpeg myfile.pdf > myfile.jpg` | convert pdf to jpeg                      |

## pdftk

- crop pdf, extract pdf pages, cut pdf
- merge pdfs
- add pdf toc

| command                                                                                                                                                                                                                                 | description                                                                                                                                        |
| :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sudo apt install pdftk`                                                                                                                                                                                                                |
| `sudo snap install pdftk`                                                                                                                                                                                                               |
| `pdftk full-pdf.pdf cat 12-15 output outfile_p12-15.pdf`                                                                                                                                                                                | to save pages 12-15 from `full-pdf.pdf` in `outfile_p12-15.pdf`                                                                                    |
| `pdftk mt01.pdf mt02.pdf mt03.pdf mt04.pdf mt05.pdf mt06.pdf mt07.pdf mt08.pdf mt09.pdf mt10.pdf mt11.pdf mt12.pdf mt13.pdf mt14.pdf mt15.pdf mt16.pdf mt17.pdf mt18.pdf mt19.pdf mt20.pdf mt21.pdf mt22.pdf cat output mergedfile.pdf` | merge all files into one file `mergedfile.pdf`                                                                                                     |
| `pdftk input.pdf dump_data > metadata.txt`                                                                                                                                                                                              | dumps the current metadata to a file (edit this file in order to modify/add a new toc, see [youtube](https://www.youtube.com/watch?v=5dv_02v0zzc)) |
| `pdftk input.pdf update_info_utf8 metadata.txt output input_with_toc.pdf`                                                                                                                                                               | add a toc to `input.pdf`, where `metadata.txt` contains the toc, see [youtube](https://www.youtube.com/watch?v=5dv_02v0zzc)                        |

## other

| command       | description                                                                                |
| :------------ | :----------------------------------------------------------------------------------------- |
| pyTranscriber | generates subtitles for `.mp3` files via Google Speech Recognition API using Autosub (GUI) |

| command    | description                         |
| :--------- | :---------------------------------- |
| goldendict | dict for fast lookup (ctrl + c + c) |

| command           | description |
| :---------------- | :---------- |
| pycharm-community |

| command | description |
| :------ | :---------- |
| docker  |

| command | description                                                                         |
| :------ | :---------------------------------------------------------------------------------- |
| eog     | picture viewer ([shortcuts](https://help.gnome.org/users/eog/stable/index.html.en)) |

| command | description                                                                       |
| :------ | :-------------------------------------------------------------------------------- |
| pinta   | picture editor ([shortcuts](https://www.pinta-project.com/user-guide/shortcuts/)) |

| command    | description                                                                                           |
| :--------- | :---------------------------------------------------------------------------------------------------- |
| gedit      | texteditor                                                                                            |
| zum Lesen: | unter F10/Preferences/Font & Colors/ Font ändern zu "TeX Gyre Termes Math Regular"                    |
| ctrl + h   | find and replace (halte im "Find & Replace"-Fenster `alt` gedrückt für schnelle Auswahl der Optionen) |
| F10        | menu (u.a. Shortcuts)                                                                                 |
| F1         | help, Shortcut overview                                                                               |

| command | description     |
| :------ | :-------------- |
| kazam   | screen recorder |

| command                           | description                               |
| :-------------------------------- | :---------------------------------------- |
| joplin                            | Notes                                     |
| alt + entsprechende Taste im menu | im Menu stehen alle Shortcuts !           |
| ctrl + l                          | change view (editor/markdown viewer/both) |
| F10                               | show all notebooks sidebar                |
| F11                               | show all Notes sidebar                    |
| ctrl + shift + l                  | focus note selection                      |
| ctrl + shift + b                  | focus body                                |
| ctrl + shift + n                  | focus title                               |

| command      | description                                                                                                                                                                                           |
| :----------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| dconf-editor | zB `gsettings set org.gnome.desktop.interface clock-show-weekday true` geht irgendwie nicht, stattdessen in dconf-editor zu org.gnome.desktop.interface navigieren und clock-show-weekday aktivieren. |

| command    | description                                   |
| :--------- | :-------------------------------------------- |
| lm-sensors | get CPU temperature (using command `sensors`) |

| command          | description |
| :--------------- | :---------- |
| telegram-desktop | Telegram    |
| zoom-client      |
| discord          |

| command | description   |
| :------ | :------------ |
| ticker  | stock monitor |

| command                        | description                                                                                            |
| :----------------------------- | :----------------------------------------------------------------------------------------------------- |
| Tor-Browser-Bundle Webdownload | installation: see [here](https://wiki.ubuntuusers.de/Tor/Installation/#Tor-Browser-Bundle-Webdownload) |

| command   | description                                                       |
| :-------- | :---------------------------------------------------------------- |
| inxi -Fxz | inxi - Command line system information script for console and IRC |
| inxi -G   | get Graphics info, eg. display resolution, GPU, etc.              |

| command             | description                                                                                                                     |
| :------------------ | :------------------------------------------------------------------------------------------------------------------------------ |
| cuda                | [installation](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#removing-cuda-tk-and-driver) via .deb file |
| nvidia-cuda-toolkit | manuell installiert mit `sudo apt install nvidia-cuda-toolkit`, nachdem cuda per .deb file installiert wurde                    |
| nvidia-docker2      | [installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)                     |

| command  | description                                 |
| :------- | :------------------------------------------ |
| droidcam | use Android smartphone cam as Ubuntu webcam |

| command | description                                                                                                                                                               |
| :------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| psensor | CPU and GPU temperature, Unter "sensor preferences" im Tab "Application Indicator" das Kästchen "Display sensor in the label" aktivieren, damit ein bestimmter Wert im System Tray angezeigt wird. |
| conky   | see [configuration](https://linuxconfig.org/ubuntu-20-04-system-monitoring-with-conky-widgets)                                                                            |

| command    | description                       |
| :--------- | :-------------------------------- |
| mailspring | mail client similar to Apple Mail |

| command | description         |
| :------ | :------------------ |
| peek    | screen2gif recorder |

| command       | description                                   |
| :------------ | :-------------------------------------------- |
| sqlite3       | A command line interface for SQLite version 3 |
| sqlitebrowser | light GUI editor for SQLite databases         |

| command                           | description      |
| :-------------------------------- | :--------------- |
| sudo apt-get install gnome-tweaks | GNOME tweak tool |

| command                           | description  |
| :-------------------------------- | :----------- |
| sudo apt-get install dconf-editor | dconf editor |

| command | description           |
| :------ | :-------------------- |
| plank   | dock similar to macOS |

| command            | description |
| :----------------- | :---------- |
| whatsapp-for-linux | whatsapp    |

| command                                                          | description                        |
| :--------------------------------------------------------------- | :--------------------------------- |
| 1. sudo apt-get install compiz compiz-gnome compiz-plugins-extra | compiz Fenstermanager dependencies |
| 2. sudo apt install compizconfig-settings-manager                | compiz Fenstermanager              |

| command    | description                                                                                                                                                                                          |
| :--------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| doxygen    | create class diagram for C++ projects (Doxygen is a documentation system for C++, C, Java, Objective-C, IDL (Corba and Microsoft flavors), Fortran, Python, VHDL and to some extent PHP, C#, and D.) |
| doxywizard | a tool to configure and run doxygen on your source files                                                                                                                                             |

| command   | description                                                                                                                                                     |
| :-------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| qt5       | installiere via offline installer `qt-opensource-linux-x64-5.9.5.run`, weil qtcreator nur so automatisch qt konfiguriert (s. bookmarks in CS\/coding\/docker\/) |
| qtcreator | for GUI development                                                                                                                                             |

| command   | description                                                                                                                                           |
| :-------- | :---------------------------------------------------------------------------------------------------------------------------------------------------- |
| python2.7 | einfach per `apt install python2.7` installieren (Achtung: some apps also need `python2.7-dev`!); starten per `python2.7` (nur `python2` geht nicht!) |

| command  | description               |
| :------- | :------------------------ |
| ocrmypdf | `ocrmypdf in.pdf out.pdf` |

| command       | description                                                                                                             |
| :------------ | :---------------------------------------------------------------------------------------------------------------------- |
| `ipad_charge` | [github link](https://github.com/mkorenkov/ipad_charge/wiki) automatically start charging ipad when connected to ubuntu |

| command                                                                                 | description                                                                                                               |
| :-------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------ |
| (nicht mehr) sudo apt install powerline                                                 | maybe install https://github.com/powerline/fonts, if symbols (e.g. branch symbol in git repo) are not displayed correctly |
| sudo apt install fish                                                                   | install fish shell (add `exec fish` in `.bashrc` to start automatically; in `fish_config` theme `fish default` wählen)    |
| curl https://raw.githubusercontent.com/oh-my-fish/oh-my-fish/master/bin/install \| fish | install oh my fish (necessary for bobthefish)                                                                             |
| omf install bobthefish                                                                  | install powerline in fish                                                                                                 |

| command                   | description |
| :------------------------ | :---------- |
| sudo snap install postman | http client |

| command | description                                                                               |
| :------ | :---------------------------------------------------------------------------------------- |
| dive    | for docker image dependencies tree (see [github repo](https://github.com/wagoodman/dive)) |

| command | description                                                                                                                                                            |
| :------ | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| barrier | share keyboard and mouse between multiple computers (client-server model) [**Note:** uncheck "enable SSL" box in barrier "Menu" -> "Change Settings" (on all devices)] |

| command           | description                                                                                                                                                                                                                                                         |
| :---------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| backgroundremover | `pip install backgroundremover` and then download `u2net.pth` from `https://drive.google.com/uc?id=1ao1ovG1Qtx4b7EoskHXmi2E9rp5CHLcZ` (or `https://drive.google.com/file/d/1ao1ovG1Qtx4b7EoskHXmi2E9rp5CHLcZ/view`) and save it as `/home/bra-ket/.u2net/u2net.pth` |

| command  | description                                        |
| :------- | :------------------------------------------------- |
| v4l2-ctl | for setting webcam powerline frequency (see below) |

| command                                | description                                                                                                                          |
| :------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------- |
| `sudo apt install fzf`                 | general-purpose command-line fuzzy finder                                                                                            |
| `apt-cache show fzf`                   | will show `Refer /usr/share/doc/fzf/README.Debian for quick instructions on how to add keybindings for Bash, Zsh, Fish to call fzf.` |
| `vim /usr/share/doc/fzf/README.Debian` | how to `source` fzf                                                                                                                  |

| command                                 | description                                                     |
| :-------------------------------------- | :-------------------------------------------------------------- |
| `sudo apt install fd-find`              | fast alternative to `find`                                      |
| `ln -s $(which fdfind) ~/.local/bin/fd` | because the binary name `fd` is already used by another package |

| command                     | description      |
| :-------------------------- | :--------------- |
| `sudo apt install keepass2` | password manager |

| command                  | description                                                                                                                                                   |
| :----------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `sudo apt install gpick` | color picker (get hex code of a color by pointing with the mouse at some point on the screen, [stackoverflow](https://stackoverflow.com/a/10243630/12282296)) |

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

| command           | description                           |
| :---------------- | :------------------------------------ |
| `alias`           | List all aliases                      |
| `type some_alias` | check the meaning of a specific alias |

# System Folder

| command                      | description                                     |
| :--------------------------- | :---------------------------------------------- |
| `~/.local/share/Trash/files` | `rm FILE` command moves `FILE` to this location |

# General commands

| command                | description                                                                                                                                                        |
| :--------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ctrl + r + Suchbegriff | reverse search (mehrmals ctrl + r drücken, um zwischen den Suchbegriff enthaltenden commands auszuwählen, danach `->` um zu übernehmen bzw `Enter` um auszuführen) |

## Root

| command              | description                                                                                                                                                                                         |
| :------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sudo`               | an acronym for **SuperUser & Do** or **Switch User & DO**                                                                                                                                           |
| `sudo -i`            | run single command with root privileges (does not require root password !)                                                                                                                          |
| `sudo -s`            | run single command with root privileges (does not require root password !) + do not change user and working directory                                                                               |
| `su`                 | switches to super user (root user) (requires root password !) (in Ubuntu: root account disabled by default for improved security)                                                                   |
| `su -`               | `-` (`-l`, `--login`) flag: the shell switches from its original directory to a login shell that simulates an actual login.                                                                         |
| `sudo su postgres`   | Switches to the specified user's account (here: `postgres`) **and** it will inherit the original user's environment variables to target user                                                        |
| `sudo su - postgres` | Switches to the specified user's account (here: `postgres`), **but does not** inherit the original user's environment variables, instead it resets all environment variables and creates them again |

## Nohup

| command          | description                                                                                                                                                                                                                                                                                                                                                                                                  |
| :--------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `some_command &` | run process `some_command` in the background                                                                                                                                                                                                                                                                                                                                                                 |
| `nohup gedit &`  | start gedit in the background AND do not stop gedit, when shell is stopped. (Dies war ein einfaches Beispiel, aber es macht den eigentlichen Nutzen klar, wenn man z.B. per SSH auf einem fremden Rechner arbeitet und dort einen langwierigen Prozess starten möchte, die ssh-Verbindung aber während des Prozesses nicht permanent aktiv sein soll, weil man etwa den eigenen Rechner ausschalten möchte.) |

## symlinks

Main disadvantages:

- **dangerous**: `rm -r symlinkToDirectory/` deletes the contents of the symlinked directory, [stackoverflow](https://stackoverflow.com/a/62647612/12282296)
  - whereas `rm -r symlinkToDirectory` deletes the symlink only

Main advantages:

- when you need to have a folder in multiple locations on your machine
  - **Saves storage** by avoiding duplicate folders on the machine
  - **Better versioning** by avoiding duplicate folders on the machine

| command                                    | description                                                                                                                             |
| :----------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------- |
| `ln -s path/to/existing/FILE path/to/LINK` | create a symlink to a **file** (no `/` at the end of the paths!)                                                                        |
| `ln -s path/to/existing/DIR path/to/LINK`  | create a symlink to a **directory** (no `/` at the end of the paths!)                                                                   |
| `readlink -f LINK`                         | show symlink target \[**ACHTUNG**: das heißt **nicht**, dass das target auch existiert, s.i [LINK](https://serverfault.com/a/76049) !\] |

## Open Files from the Terminal

| command       | description                                                                                                |
| :------------ | :--------------------------------------------------------------------------------------------------------- |
| xdg-open file | open file using default application                                                                        |
| gio open file | same as xdg-open, but depends on what desktop the user has installed, whereas xdg-open is desktop agnostic |

## Process Management

| command                                 | description                                                                                                                                                                                                   |
| :-------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `top`                                   | activity monitor                                                                                                                                                                                              |
| `ps`                                    | wie `top`, aber keine real-time updates (dh. nur ein snapshot)                                                                                                                                                |
| `ps -eo pid,lstart,cmd \| grep 2127686` | start time of PID 2127686 (e.g. to get terminal start time, run `echo $$`, and then this command)                                                                                                             |
| `echo $$`                               | show PID of current shell                                                                                                                                                                                     |
| `kill PID`                              | stop process with id `PID`, sends SIGTERM (i.e. kills gracefully) (see [notes bash]({% post_url 2021-09-25-cheatsheet-bash %}#job-control) and [kill doc](https://man7.org/linux/man-pages/man1/kill.1.html)) |
| `pkill process_name`                    | stop all processes containing `process_name` (which is a regular expression), sends SIGTERM (i.e. kills gracefully), _Warning:_ use `pgrep` first to check which processes will be killed                     |
| `pgrep process_name`                    | list all PIDs containing `process_name` (which is a regular expression)                                                                                                                                       |

Check PID of a window (pick a window with the cursor):

```bash
xprop _NET_WM_PID | sed 's/_NET_WM_PID(CARDINAL) = //' | ps `cat`
```

This will make your cursor a cross with which you can click on an open window. It will report the PID and command in the terminal you ran it in.

In general, `xprop` and `xwininfo` will provide you with a lot of information about an open window.

## Get Paths

| command               | description                                       |
| :-------------------- | :------------------------------------------------ |
| `realpath foo.bar`    | get path to file "foo.bar" (like `pwd` + foo.bar) |
| `readlink -f foo.bar` | get path to file "foo.bar" (like `pwd` + foo.bar) |

## Redirection, Pipe Tricks

| command                 | description                |
| :---------------------- | :------------------------- |
| `ls \| wc -l`           | count files in a directory |
| `history \| tail -n 30` | show last 30 commands      |

# comm

To compare **just the filenames** in `dir1` and `dir2`:

[linuxquestions.org](https://www.linuxquestions.org/questions/linux-general-1/how-to-compare-only-the-file-names-between-two-directories-diff-4175477358/#post5029670)

```bash
comm <(ls dir1) <(ls dir2)
```

The output will be differentiated by 0, 1, or 2 leading tabs as:

files only in dir1
files only in dir2
files in both dirs

Options to `comm` allow selection of the columns you want.

# diff

[groups.google.com](https://groups.google.com/g/linux.redhat.list/c/jp-inxNQQJk/m/2nSsxlY3scMJ)

Compare listings of two directories using process substitution in bash:

```bash
diff <(ls dir1) <(ls dir2)
```

Differences between two directory trees

| command                 | description                                                          |
| :---------------------- | :------------------------------------------------------------------- |
| `diff -r dir1/ dir2/`   | outputs exactly what the differences are between corresponding files |
| `diff -qr dir1/ dir2/`  | just getting a list of corresponding files whose content differs     |
| `diff -qrN dir1/ dir2/` | to see differences for files that may not exist in either directory  |

# apt, apt-get, snap, dpkg, pkg-config

## Difference between apt and apt-get + apt-cache:

- `apt` = most commonly used command options from `apt-get` and `apt-cache` see [here](https://itsfoss.com/apt-vs-apt-get-difference/)
- So with `apt`, you get all the necessary tools in one place. You won’t be lost under tons of command options. The main aim of `apt` is to provide an efficient way of handling packages in a way "pleasant for end users".
- `apt`:
  - shows progress bar while installing or removing a program
  - prompts number of packages that can be upgraded when you update the repository database (i.e. `apt update`)
  - same can be achieved with apt-get (but you need additional options)
- When you use `apt` to install a package, under the hood it uses `dpkg`. When you install a package using `apt`, it first creates a list of all the dependencies and downloads it from the repository.
  - Once the download is finished it calls `dpkg` to install all those files, satisfying all the dependencies.

| command                        | description                                                                                                                                         |
| :----------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sudo apt update`              |
| `sudo apt [-y] upgrade`        | `-y` oder `—yes` für automatic yes to prompts                                                                                                       |
| `apt --help`                   |
| `sudo apt remove package`      | uninstall `package_file.deb`                                                                                                                        |
| `sudo apt autoremove`          | remove not needed packages (NOTE: This command will remove all unused packages (orphaned dependencies). Explicitly installed packages will remain.) |
| `sudo apt-mark auto $PACKAGES` | mark packages in variable `PACKAGES` as `automatically installed`, if accidentally marked as `manually installed`                                   |

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

| command                                         | description                |
| :---------------------------------------------- | :------------------------- |
| sudo apt install ./name.deb                     | install a .deb file        |
| `sudo apt-get install <package name>=<version>` | install a specific version |

### Uninstall

| command                       | description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| :---------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `sudo apt purge ...`          | Removing packages with `sudo apt purge ...` or `sudo apt --purge remove ...` will remove them and all their global (i.e., systemwide) configuration files. This is usually what people mean when they talk about completely removing a package. **This does not remove packages that were installed as dependencies**, when you installed the package you're now removing. Assuming those packages aren't dependencies of any other packages, and that you haven't marked them as manually installed, you can remove the dependencies with `sudo apt autoremove` or (if you want to delete their systemwide configuration files too) `sudo apt --purge autoremove`. |
| `sudo apt --purge remove ...` | see `sudo apt purge ...`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| `sudo apt autoremove`         | remove the **dependencies** that are no longer needed                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `sudo apt --purge autoremove` | remove **systemwide configuration files and the dependencies** that are no longer needed                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

### apt-cache

| command                           | description                                                                                                                                       |
| :-------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------ |
| `apt-cache policy <package name>` | shows installed package version and also all the available versions in the repository according to the version of Ubuntu in which you are running |
| `apt-cache search <package_name>` | find specific package names                                                                                                                       |

### apt-mark

| command                             | description                                                          |
| :---------------------------------- | :------------------------------------------------------------------- |
| `sudo apt-mark hold package_name`   | hold packages (i.e. do not upgrade, remove or modify `package_name`) |
| `sudo apt-mark unhold package_name` | unhold "held" packages                                               |
| `apt-mark showhold`                 | show held packages                                                   |

## dpkg

| command                                                | description                                                                                                                                          |
| :----------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sudo dpkg -l \| less`                                 | list all installed dpkg packages [meaning of tags ii, rc, ...](https://askubuntu.com/questions/18804/what-do-the-various-dpkg-flags-like-ii-rc-mean) |
| `sudo dpkg -L package`                                 | show all files which were installed by a package                                                                                                     |
| `sudo vim /var/lib/dpkg/info/nvidia-cuda-toolkit.list` | in `/var/lib/dpkg/info/` sind die installation files (`.conffiles`, `.list`, `.md5sums`) für alle packages (hier: `nvidia-cuda-toolkit`)             |
| `dpkg -l \| grep ^..r`                                 | list all broken packages (`r` state (on the third field) means: reinst-required (package broken, reinstallation required))                           |

### Basics

| command                         | description                                                             |
| :------------------------------ | :---------------------------------------------------------------------- |
| `sudo dpkg -i package_file.deb` | install `package_file.deb` (alternative: `sudo apt install ./name.deb`) |
| `sudo dpkg -P some_package`     | purge `some_package`                                                    |
| `sudo dpkg -r some_package`     | remove `some_package`                                                   |

### Confirm Whether Package Is Already Installed

<span style="color:red">**Tipp:** AM BESTEN DIE FOLGENDEN 3 ALLE AUSFÜHREN, DA JEDER EINEN ANDEREN OUTPUT HAT !</span>

<span style="color:red">**ACHTUNG**</span>: bei allen folgenden commands den exakten Namen schreiben, zB `lua` findet `lua5.1` nicht !

| command                        | description                                                                                                                   |
| :----------------------------- | :---------------------------------------------------------------------------------------------------------------------------- |
| `sudo dpkg -l package`         | confirm whether package is already installed (wenn nicht installed, dann wird `no packages found matching package` angezeigt) |
| `sudo dpkg -l \| grep package` | confirm whether package is already installed (wenn nicht installed, dann wird nichts angezeigt)                               |
| `sudo dpkg-query -s package`   | prüfe ob package installiert ist und print weitere Informationen zum package                                                  |

### Show History Of Installed Packages

see also [how-to-show-history-of-installed-packages](https://www.linuxuprising.com/2019/01/how-to-show-history-of-installed.html)

| command                                        | description                                                               |
| :--------------------------------------------- | :------------------------------------------------------------------------ |
| `grep " install \| remove " /var/log/dpkg.log` | list recently installed OR removed packages (in the current month)        |
| `grep " install " /var/log/dpkg.log.1`         | list recently installed packages (in the previous month)                  |
| `zgrep " install " /var/log/dpkg.log.2.gz`     | list recently installed packages (go back 2 months, same for `>2` months) |
| `vim /var/log/apt/history.log`                 | view apt history                                                          |

## snap

- see `man snap` for details
- updates: according to [Snap tutorial](https://snapcraft.io/docs/quickstart-tour#heading--refreshing) Snaps are automatically updated in the background once per day., [askubuntu](https://askubuntu.com/a/1034315)
  - However, if you don't close the application it will not be updated and you will receive daily notifications to do so. [askubuntu](https://askubuntu.com/a/1034315)

| command                            | description |
| :--------------------------------- | :---------- |
| snap list                          |
| snap find _package_                |
| sudo snap install _package_        |
| sudo snap remove _package_         |
| sudo snap remove --purge _package_ |

## pkg-config

| command                           | description                                                                                                                                                                                                                                                                                                                                                              |
| :-------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `man pkg-config`                  | description of all pkg-config flags                                                                                                                                                                                                                                                                                                                                      |
| `pkg-config --libs-only-l json-c` | was man im CMakeLists.txt in `target_link_libraries` eintragen muss (hier: `-ljson-c` Achtung: das `-l` muss auch im CMakeLists.txt rein!)                                                                                                                                                                                                                               |
| `pkg-config --libs-only-L json-c` | location of .so library file (hier: in ubuntu 18.04: findet er nicht, ist aber in `/lib/x86_64-linux-gnu`; in ubuntu 20.04: `-L/usr/local/lib`) (see also: [difference .so vs .a libraries](https://stackoverflow.com/a/9810368/12282296)) (muss nicht in CMakeLists.txt rein, s. [Minimalbsp](https://github.com/pharath/home/tree/master/assets/_code_examples/jsonc)) |
| `pkg-config --cflags json-c`      | include paths of the corresponding library with .h header files (hier: in ubuntu 18.04: `-I/usr/include/json-c`; in ubuntu 20.04: `-I/usr/local/include -I/usr/local/include/json-c`) (muss nicht in CMakeLists.txt rein)                                                                                                                                                |

# tty, terminal session management

From [stackexchange](https://unix.stackexchange.com/a/21294):

- A `tty` (teletype) is a native terminal device, the backend is either **hardware or kernel emulated**.
- A `pty` (pseudo-tty) is a terminal device which is **emulated by another program** (example: `xterm`, `screen`, or `ssh` are such programs).
- A `pts` is the slave part of a `pty`.
- **More info**: see [stackexchange](https://unix.stackexchange.com/a/21294) and `man pty`

| command              | description                                                       |
| :------------------- | :---------------------------------------------------------------- |
| `tty`                | zeigt Namen des aktiven terminals                                 |
| `ls -ltr /dev/ttys*` | zeigt Namen aller aktiven terminals                               |
| `last`               | zeige letzte terminal logins                                      |
| `whoami`             | print the user name associated with the current effective user ID |

# chmod, Groups

**Default Permissions:**

From [baeldung](https://www.baeldung.com/linux/new-files-dirs-default-permission):

- On Linux, by default, when we create new **files**, they are given `rw-rw-r–` permissions.
  - The first `rw-` signifies read-write permissions for the user or the owner of the file
  - The second `rw-` indicates read-write permissions for the group the file belongs to
  - The final `r–` read permission is for all other users
- Similarly, for newly created **directories**, the default permission is `rwxrwxr-x`.

| command                  | description                                                                                                                                                                        |
| :----------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `chmod permissions-file` | is an abbreviation of change mode. A file's mode is the set of permissions attached to it that control access. Zu _permissions_: s. [here](https://askubuntu.com/tags/chmod/info). |

| command       | description                                                                                |
| :------------ | :----------------------------------------------------------------------------------------- |
| `groups`      | list all groups the currently logged in user belongs to (first group is the primary group) |
| `groups user` | same as `groups`, but for specific user `user`                                             |
| `id`          | "                                                                                          |
| `id user`     | "                                                                                          |

| command         | description                           |
| :-------------- | :------------------------------------ |
| less /etc/group | view all groups present on the system |
| cat /etc/group  | "                                     |
| getent group    | "                                     |

| command             | description                      |
| :------------------ | :------------------------------- |
| getent group docker | list all members of group docker |

| command                         | description                                                                                                            |
| :------------------------------ | :--------------------------------------------------------------------------------------------------------------------- |
| `sudo groupadd docker`          | add new group docker                                                                                                   |
| `sudo usermod -aG docker $USER` | add my user to the docker group                                                                                        |
| `newgrp docker`                 | log out and log back in so that group membership is re-evaluated (nach group Aenderungen); wenn das nicht geht, reboot |

# useradd, usermod, deluser

source: [baeldung](https://www.baeldung.com/linux/change-default-home-directory)

Run these commands as `root` user. Run them in TTY mode (press e.g. ctrl + alt + F3).

| command                                      | description                                                                                                                                                                                                                                                        |
| :------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sudo passwd root`                           | set the root's password                                                                                                                                                                                                                                            |
| `ctrl + alt + f1` or `ctrl + alt + f2` ...   | switch the TTY in order to log in as `root`                                                                                                                                                                                                                        |
| `sudo useradd -m baeldung`                   | creating a user called baeldung, in the default way                                                                                                                                                                                                                |
| `sudo useradd -m -d /home/baeldung baeldung` | (**important:** do not use a slash character at the end of `/home/baeldung`, otherwise the terminal will show `/home/baeldung` instead of the `~` symbol at the start of each line !) create a user and set the location for the `home` directory at the same time |
| `sudo usermod -d /usr/baeldung baeldung`     | change the user's home directory to `/usr/baeldung`                                                                                                                                                                                                                |
| `sudo usermod -m -d /usr/baeldung baeldung`  | also move the existing content to the new location                                                                                                                                                                                                                 |
| `startx`                                     | start the Ubuntu GUI when you are in the tty                                                                                                                                                                                                                       |
| `sudo deluser --remove-home userName`        | delete user `userName`                                                                                                                                                                                                                                             |

# bash

## Change Terminal Title

Change the title of the current terminal: `echo -ne "\033]0;SOME TITLE HERE\007"`, [askubuntu](https://askubuntu.com/a/22417)

## Gnome Terminal Shortcuts

![jumping with command line cursor](/assets/images/moving_cli.png)

| command         | description                                                                                              |
| :-------------- | :------------------------------------------------------------------------------------------------------- |
| ctrl + w        | delete the word in front of the cursor                                                                   |
| esc + backspace | delete a part of a path in front of the cursor (e.g. to get from `this/is/some/path` to `this/is/some/`) |
| alt + d         | delete the word after the cursor                                                                         |
| ctrl + a        | jump to the beginning of the line                                                                        |
| ctrl + e        | jump to the end of the line                                                                              |
| ctrl + s        | freeze/block terminal                                                                                    |
| ctrl + q        | unfreeze/unblock terminal                                                                                |
| fn + links      | scrolle nach ganz oben                                                                                   |
| cmd + oben      | focus letzte input Zeile (zB gut, wenn man zB schnell hochscrollen will)                                 |

## Bash Scripting

### Change Shell

| command           | description                                                                                                                                   |
| :---------------- | :-------------------------------------------------------------------------------------------------------------------------------------------- |
| `echo $$`         | display PID of current shell                                                                                                                  |
| `echo $0`         | check current shell type                                                                                                                      |
| `bash`            | start new bash shell instance in current bash shell (the new shell will have a different PID than the old one, check shell PID via `echo $$`) |
| `cat /etc/shells` | list all shells                                                                                                                               |
| `chsh`            | change shell (you will be prompted to enter one of the shells in `cat /etc/shells`)                                                           |

### env, PATH

| command          | description                                                                                                                      |
| :--------------- | :------------------------------------------------------------------------------------------------------------------------------- |
| `printenv`       | Print the values of the specified environment VARIABLE(s).                                                                       |
| `env`            | show all environment variables                                                                                                   |
| `env NAME=VALUE` | Set each NAME to VALUE in the environment                                                                                        |
| `echo $PATH`     | spezielle Variable, die alle Pfade enthält, in denen Shell-Programme/Shell-Befehle (ls, echo, df, nautilus, etc.) gesucht werden |
| `echo $Variable` | display the content of the variable "`Variable`"                                                                                 |

### Running Multiple Commands

| command                                                      | description                                                   |
| :----------------------------------------------------------- | :------------------------------------------------------------ |
| `do_something1 && do_something2_that_depended_on_something1` | only run "something2", if "something1" completes successfully |
| `do_something1; do_something2`                               | run "something2" irrespective of "something1"                 |

### ls

| command         | description                                   |
| :-------------- | :-------------------------------------------- |
| `ls -d */`      | list directories only                         |
| `ls -d /etc/*/` | list directories only in a specific directory |

### find

- **Best Practices:**
  - put the search pattern in quotes, otherwise you might get the error `paths must precede expression`

| command                                                                                  | description                                                                                                                                                                                                                                                                                                |
| :--------------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `find /opt/ -iname "pattern"`                                                            | find all files (hier: in dir `/opt/` ), for which base of file name (path with leading dirs removed) matches shell pattern `pattern` (Achtung: `pattern` muss genau übereinstimmen! Falls Endung unbekannt, mit Sternchen `*` am Ende suchen, dh. `pattern*` statt `pattern` suchen (wie bei `ls` Befehl). |
| `find /opt/ -name "pattern"`                                                             | wie -iname, aber case-sensitive                                                                                                                                                                                                                                                                            |
| `find /opt/ -iname "pattern" -type f`                                                    | nur files suchen                                                                                                                                                                                                                                                                                           |
| `find /opt/ -iname "pattern" -type d`                                                    | nur dirs suchen                                                                                                                                                                                                                                                                                            |
| `find /opt/ -iname "pattern1" -iname "pattern2"`                                         | logical "AND"                                                                                                                                                                                                                                                                                              |
| `find /opt/ ! -iname "pattern1"`                                                         | logical "NOT"                                                                                                                                                                                                                                                                                              |
| `find /opt/ ( -iname "pattern1" -o -iname "pattern2" )`                                  | logical "OR"                                                                                                                                                                                                                                                                                               |
| `find /opt/ ( -iname "pattern1" -or -iname "pattern2" )`                                 | logical "OR", not POSIX compliant                                                                                                                                                                                                                                                                          |
| `find /opt/ -size +1G`                                                                   | nur files, die über 1GB groß sind                                                                                                                                                                                                                                                                          |
| `find . -iname "pattern" -printf '%Tc %p\n'`                                             | show timestamps (modified time)                                                                                                                                                                                                                                                                            |
| `find . -newermt "2024-07-26" -not -newermt "2024-07-27"`                                | find by modified time                                                                                                                                                                                                                                                                                      |
| `find /path/to/dir -newermt "yyyy-mm-dd HH:mm:ss" -not -newermt "yyyy-mm-dd HH:mm:ss+1"` | list file in the folder `/path/to/dir` modified between `yyyy-mm-dd HH:mm:ss` and `yyyy-mm-dd HH:mm:ss + 1` second, [unix.stackexchange](https://unix.stackexchange.com/questions/424319/how-to-find-files-based-on-timestamp)                                                                             |
| `find /path/to/dir -newerat`                                                             | find by access time                                                                                                                                                                                                                                                                                        |
| `find /path/to/dir -newerct`                                                             | find by creation time                                                                                                                                                                                                                                                                                      |
| `find . -regextype sed -regex ".*/[a-f0-9\-]\{36\}\.jpg"`                                | regex, there's an implicit `^ ... $` surrounding your regex (it must match the WHOLE result line), `valid types are 'findutils-default', 'awk', ' egrep', 'ed', 'emacs', 'gnu-awk', 'grep', 'posix-awk', 'posix-basic', 'posix-egrep', 'posix -extended', 'posix-minimal-basic', 'sed'`                    |
| `find . -iname "searchPattern" -print0 \| xargs -0 someCommand`                          | apply `someCommand` on each of the found files                                                                                                                                                                                                                                                             |
| `find . -iname "searchPattern" -print0 \| xargs -0 du -sh`                               | show the size of each found file                                                                                                                                                                                                                                                                           |
| `find . -iname "searchPattern" -exec rm -v "{}" \+`                                      | remove the found files; from `man find`: `-exec`: "The specified command is run once for each matched file."; `+`: best explanation: [stackoverflow](https://stackoverflow.com/a/6085237)                                                                                                                  |
| `find path_A -name '*AAA*' -exec mv -t path_B "{}" \+`                                   | move the found files                                                                                                                                                                                                                                                                                       |

### locate

| command                 | description                                                                                              |
| :---------------------- | :------------------------------------------------------------------------------------------------------- |
| `locate <file>`         | faster than `find`, but uses a database which must be updated via `sudo updatedb` to find recent changes |
| `locate -i <file>`      | case insensitive                                                                                         |
| `locate -b '\file.xyz'` | exact match (Note: the slash and the quotation marks are necessary)                                      |
| `sudo updatedb`         | update the `locate` command's database                                                                   |

### Finding Program Paths

| command           | description                |
| :---------------- | :------------------------- |
| `which <program>` | show the path of a program |
| `which python3`   |
| `whereis python3` |

### regex

- <span style="color:red">**in vim's "find and replace"**</span>: you must escape `{, }, /, (, ), |, +` (but not: `[, ]`) and some other characters with a backslash "\\" for the regex find pattern and the replace pattern to work
- match URLs: `https?:\/\/(www\.)?[-a-zA-Z0-9@:%._\+~#=]{1,256}\.[a-zA-Z0-9()]{1,6}\b([-a-zA-Z0-9()@:%_\+.~#?&//=]*)`, [stackoverflow](https://stackoverflow.com/a/3809435)
  - without http protocol: `[-a-zA-Z0-9@:%._\+~#=]{1,256}\.[a-zA-Z0-9()]{1,6}\b([-a-zA-Z0-9()@:%_\+.~#?&//=]*)`

**Special Characters**:

| Char  | Description                        | Meaning                                                   |
| :---- | :--------------------------------- | :-------------------------------------------------------- |
| `\`   | Backslash                          | Used to escape a special character                        |
| `^`   | Caret                              | Beginning of a string                                     |
| `$`   | Dollar sign                        | End of a string                                           |
| `.`   | Period or dot                      | Matches any single character                              |
| `\|`  | Vertical bar or pipe symbol        | Matches previous OR next character/group                  |
| `?`   | Question mark                      | Match zero or one of the previous                         |
| `*`   | Asterisk or star                   | Match zero, one or more of the previous                   |
| `+`   | Plus sign                          | Match one or more of the previous                         |
| `( )` | Opening and closing parenthesis    | Group characters                                          |
| `[ ]` | Opening and closing square bracket | Matches a range of characters                             |
| `{ }` | Opening and closing curly brace    | Matches a specified number of occurrences of the previous |

**Quantifiers:**

- `X`, exactly n times: `X{n}`
- `X`, at least n times: `X{n,}`
- `X`, at least n but not more than m times: `X{n,m}`
- related:
  - [regexp-quantifiers](https://javascript.info/regexp-quantifiers)

**Character Classes:**

- `\w` (word)
- `\d` (digit)
- `\s` (whitespace)
- `\S` (not whitespace)
- `(\w|\d)` (word or digit)
- `(\w{1,}|\d)` (at least one word or more words or exactly one digit)
- etc

**Numeric References and Capture Groups:**

- `([A-Z])\w+\1` (numeric reference `\1` refers to the results of capture group number 1, which is `([A-Z])` in this example)
  - eg. in the string `RegRxr was created by gskinner.com.` this would match `RegRxr`
  - eg. in the string `RegExr was created by gskinner.com.` this would <span style="color:red">**not**</span> match `RegExr`
- this is <span style="color:red">**very useful for "find and replace" in vim**</span> because you can replace text with a numeric reference, eg. `:%s/\([A-Z]\)\w\+/\1/g` replaces all matches of the pattern `([A-Z])\w+` with the first capture group in the find pattern (here: `([A-Z])`)

**Negative Lookahead**:

- `word\(atom\)\@!` (useful to search `word` <span style="color:red">**not followed by**</span> `atom`)
  - eg. in the string `RegRxr was created by gskinner.com.` the vim regex `R\(x\)\@!` would match the `R` in front of `Re`, but not the `R` in front of `Rx`
  - eg. in the string `RegRxr was created by gskinner.com.` the vim regex `c\(r\)\@!` would match the `c` in front of `co`, but not the `c` in front of `cr`

**Only match the first occurrence of a character in the line**:

- <span style="color:red">**problem**</span>: `".*"` will match a double quoted string <span style="color:red">**only if**</span> there is <span style="color:red">**only one**</span> double quoted string in the line. When there are <span style="color:red">**multiple**</span> double quoted strings in the line this pattern will match from the first double quote sign `"` in the line until the last which is likely not intended
  - <span style="color:red">**solution**</span>: use negation: `"[^"]+"` to match anything but `"` until the next `"`
  - <span style="color:red">**example**</span>:
    - vim regex: `%s/\*\*\([^\*]\+\)\*\*/<span style={{ color: "red" }}>**\1**<\/span>/g`

### Types of regex

mostly from [stackoverflow](https://stackoverflow.com/a/76099444/12282296)

- <span style="color:red">**POSIX Basic Regular Expressions (BRE)**</span>: This is a standard syntax used in UNIX-based systems for basic pattern matching. It uses a limited set of metacharacters, including `^`, `$`, `.`, `*`, `+`, `?`, `[`, `]`, `(`, `)`, and .
- <span style="color:red">**POSIX Extended Regular Expressions (ERE)**</span>: This is a more powerful syntax used in UNIX-based systems for more advanced pattern matching. It adds more metacharacters, including `{`, `}`, `|`, and `^`.
  - The main difference is that some backslashes are removed: `\{…\}` becomes `{…}` and `\(…\)` becomes `(…)`, [wikibooks](https://en.wikibooks.org/wiki/Regular_Expressions/POSIX-Extended_Regular_Expressions)
- <span style="color:red">**Perl-Compatible Regular Expressions (PCRE)**</span>: This is a syntax used in many programming languages, including Perl, <span style="color:red">**PHP**</span>, and <span style="color:red">**Python**</span>. It adds many advanced features, such as lookaheads, lookbehinds, named capture groups, and more.
- <span style="color:red">**JavaScript Regular Expressions**</span>: This is the syntax used by the JavaScript programming language. It is similar to PCRE but has some differences, such as the use of `\b` for word boundaries instead of `\y`.
- <span style="color:red">**.NET Regular Expressions**</span>: This is the syntax used by the .NET Framework. It is similar to PCRE but has some differences, such as the use of `(?)` for named capture groups instead of `(?P)`.
- <span style="color:red">**Vim Regex**</span>

### grep

- **Best Practices**:
  - always exclude the `... | grep pattern` command itself
    - if you cannot exclude it, then
      - in `... | grep pattern | tail -n number` the `tail` command should come last
- `man grep`:
  - "Typically PATTERNS should be quoted when grep is used in a shell"
- [single quotes vs. double quotes](https://stackoverflow.com/questions/25151067/grep-double-quotes-vs-single-quotes)
- by default, `grep` used BRE (use `-E` to use ERE)

#### Basics

| command                                           | description                                                                                                                                                                                                                                                                             |
| :------------------------------------------------ | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `grep -c`                                         | count matches, **best practice**: instead of `grep \| wc -l`                                                                                                                                                                                                                            |
| `grep -i pattern`                                 | case insensitive                                                                                                                                                                                                                                                                        |
| `grep -o pattern "$file"`                         | option `-o` to only print the matching part                                                                                                                                                                                                                                             |
| `l \| grep -e pattern1 -e pattern2`               | logical "OR": greps `pattern1` or `pattern2`                                                                                                                                                                                                                                            |
| `grep -r -E 'orange \| mango' .`                  | logical "OR" operator                                                                                                                                                                                                                                                                   |
| `l \| grep -e pattern1 \| grep -e pattern2`       | logical "AND": greps `pattern1` and `pattern2`                                                                                                                                                                                                                                          |
| `l \| grep -v pattern`                            | logical "NOT": greps everything except `pattern`                                                                                                                                                                                                                                        |
| `grep -rn -e 'nvidia' /var/log/apt/history.log*`  | `r`: recursively look at all files in the folder, `n`: show line numbers, `e`: regex pattern, here: `nvidia` (w/o this some regex patterns will not work)                                                                                                                               |
| `grep -rnw -e 'nvidia' /var/log/apt/history.log*` | `w`: match whole words only (i.e. if the pattern `nvidia` is a substring of a word, it is not matched)                                                                                                                                                                                  |
| `l \| grep 150 \| xargs rm -v`                    | pipe output of `grep` to `rm`                                                                                                                                                                                                                                                           |
| `l \| grep xyzpattern \| xargs cp -iv -t 150/`    | pipe output of `grep` to `cp`                                                                                                                                                                                                                                                           |
| `l \| grep xyzpattern \| xargs mv -iv -t 1024p/`  | pipe output of `grep` to `mv`                                                                                                                                                                                                                                                           |
| `grep -n someSearchPattern`                       | `n`: show line numbers (useful to find things in `man` and long `--help` outputs, eg. use `man command` and jump to the line that `command --help \| grep -n someSearchPattern` shows)                                                                                                  |
| `grep -oE 'pattern.{0,4}' "$file"`                | option `-o` to only print the matching part in combination with `-E` (extended regular expression) and pattern `.{0,4}` to match up to four characters after your search pattern, [stackexchange](https://unix.stackexchange.com/questions/518823/grep-only-show-x-chars-in-the-result) |
| `grep -x '.\{3,10\}'`                             | `-x` (also `--line-regexp` with GNU `grep`) match pattern to whole line                                                                                                                                                                                                                 |

#### Spaces

[How to include a space character with grep?](https://askubuntu.com/a/949334)

Make sure you quote your expression.

```bash
grep ' \.pdf' example
```

Or if there might be multiple spaces (we can't use `*` as this will match the cases where there are no preceding spaces)

```bash
grep ' \+\.pdf' example
```

`+` means "one or more of the preceding character". In BRE you need to escape it with `\` to get this special function, but you can use ERE instead to avoid this

```bash
grep -E ' +\.pdf' example
```

You can also use `\s` in `grep` to mean a space

```bash
grep '\s\+\.pdf' example
```

We should escape literal `.` because in regex `.` means any character, unless it's in a character class.

### nl

| command                    | description                                                                                                                 |
| :------------------------- | :-------------------------------------------------------------------------------------------------------------------------- |
| `command \| nl -w2 -s'> '` | add line numbers in front of each line of the output of `command`, [stackexchange](https://unix.stackexchange.com/a/222220) |

### tee

| command                                                        | description                                                                                                                                                                                                                                              |
| :------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `command \| tee file.txt`                                      | read from standard input and write to both standard output **and** the file `file.txt` (overwrites `file.txt`!) [doc](http://manpages.ubuntu.com/manpages/bionic/man1/tee.1.html) (name derived from "T-junction", since `tee` is usually used in pipes) |
| `command \| tee -a file.txt`                                   | append to `file.txt`                                                                                                                                                                                                                                     |
| `du -h \| tee disk_usage1.txt disk_usage2.txt disk_usage3.txt` | write to multiple files at once                                                                                                                                                                                                                          |
| `ls file\* \| tee third_file.txt \| wc -l`                     | forward the output as input                                                                                                                                                                                                                              |
| `echo "newline" \| sudo tee -a /etc/file.conf`                 | using `tee` in conjunction with `sudo`                                                                                                                                                                                                                   |

### awk

#### Get Rows

| command                               | description             |
| :------------------------------------ | :---------------------- |
| `<some_command> \| awk 'NR % 5 == 0'` | prints every fifth line |

#### Get Columns

| command                                     | description                                           |
| :------------------------------------------ | :---------------------------------------------------- |
| `<some_command> \| awk '{print $2}'`        | get the 2nd column of the command output              |
| `<some_command> \| awk '{print $2, $4}'`    | get the 2nd and 4th column of the command output      |
| `<some_command> \| awk '{print $2, $4}'`    | get the 2nd and 4th column of the command output      |
| `<some_command> \| awk -F '"' '{print $2}'` | Use `-F [field separator]` to split the lines on `"`s |

### cut

| command                            | description                                                                                                |
| :--------------------------------- | :--------------------------------------------------------------------------------------------------------- |
| `<some_command> \| cut -f2- -d' '` | get all columns (delimited by a space) from the 2nd to the last column of the command output               |
| `<some_command> \| cut -f2- -d' '` | tab delimiter: Press <kbd>Ctrl</kbd>+<kbd>V</kbd> and then <kbd>Tab</kbd> to use "verbatim" quoted insert. |
| `history \| cut -c 8-`             | show the history without line numbers, `-c 8-` deletes the first 7 characters                              |

### sed

- for **line-based input**
  - thus, hard to replace `\n` with `sed`, `tr` is better here ([stackoverflow](https://stackoverflow.com/a/1252010))
- "Sed uses basic regular expressions (BRE). In a BRE, in order to have them treated literally, the characters `$.*[\^` need to be quoted by preceding them by a backslash, except inside character sets (`[…]`). Letters, digits and `(){}+?|` must not be quoted (you can get away with quoting some of these in some implementations)." ([more](https://unix.stackexchange.com/a/33005))
- [Command Summary for sed](https://docstore.mik.ua/orelly/unix/sedawk/appa_03.htm)
- [Bash Variables in sed](https://askubuntu.com/a/76842)
- [sed one liners](http://sed.sourceforge.net/sed1line.txt)

| command                                           | description                                                                                                                                                                                                                                                                                                                                                                                                 |
| :------------------------------------------------ | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sed 's/unix/linux/' geekfile.txt`                | replaces the word 'unix' with 'linux' in the file 'geekfile.txt'. `sed` is mostly used to replace text in a file. Examples: see [here](https://www.geeksforgeeks.org/sed-command-in-linux-unix-with-examples/).                                                                                                                                                                                             |
| `sed -E`                                          | use extended (ERE) regular expression syntax                                                                                                                                                                                                                                                                                                                                                                |
| `sed -e 's/ /\\ /g'`                              | Useful when paths contain spaces and you need to escape these spaces with backslash, eg. when using `realpath`, `pwd`, `scp`, etc. `s/`: Substitute _replacement_ for _pattern_ on each addressed line [doc: see "s" command](https://docstore.mik.ua/orelly/unix/sedawk/appa_03.htm).                                                                                                                      |
| `sed '0,/Apple/{s/Apple/Banana/}' input_filename` | replace only the first occurrence in a file, "The first two parameters `0` and `/Apple/` are the range specifier. The `s/Apple/Banana/` is what is executed within that range. So in this case "within the range of the beginning (`0`) up to the first instance of `Apple`, replace `Apple` with `Banana`. Only the first `Apple` will be replaced.", [stackoverflow](https://stackoverflow.com/a/9453461) |
| `sed 's/$/ pattern/' filename`                    | appending `pattern` to end of a line                                                                                                                                                                                                                                                                                                                                                                        |
| `sed '/Fred Flintstone/ s/$/ pattern/' filename`  | append only to lines containing a specific string                                                                                                                                                                                                                                                                                                                                                           |
| `sed '/pattern/{G;}' filename`                    | add a newline after a pattern                                                                                                                                                                                                                                                                                                                                                                               |

### tr

- to replace **single characters** by **single characters**, [stackoverflow](https://stackoverflow.com/a/18366326/12282296)
- [tr SET notation](https://phoenixnap.com/kb/linux-tr)
- hard to replace `\n` with `sed`, `tr` is better for this task ([stackoverflow](https://stackoverflow.com/a/1252010))
  - pipe with `tr a b | sed -e 's/find/replace/g'` to replace strings

| command       | description                                                                         |
| :------------ | :---------------------------------------------------------------------------------- |
| `tr '\n' ' '` | replace all `\n` with spaces, [tr SET notation](https://phoenixnap.com/kb/linux-tr) |
| `tr -d '\n'`  | delete all `\n`, [tr SET notation](https://phoenixnap.com/kb/linux-tr)              |

### head, tail

| command                               | description                                                                                                                                                                                    |
| :------------------------------------ | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `command \| head -3`                  | show the first 3 rows of the output of `command`                                                                                                                                               |
| `command \| tail -3`                  | show the last 3 rows of the output of `command`                                                                                                                                                |
| `command \| tail +3`                  | show all rows <span style="color:red">**starting at row 3**</span> of the output of `command`                                                                                                  |
| `command \| tail +24 \| head -100`    | show the first 100 rows <span style="color:red">**starting at row 24**</span> of the output of `command`                                                                                       |
| `tail -n +10 input.txt \| head -n 91` | prints rows 10-100, ie. `tail -n +10` prints out the entire file starting from line 10, and `head -n 91` prints the first 91 lines of that (up to and including line 100 of the original file) |

### redirection

| command                         | description                                                                            |
| :------------------------------ | :------------------------------------------------------------------------------------- |
| `exec > some_file`              | redirect all shell output to `some_file`                                               |
| `ls -ltr \| vim -`              | zeige Output eines Befehls in vim (ACHTUNG: Leerzeichen hinter "vim" nicht vergessen!) |
| `echo "blabla" >> filename`     | write output to file _filename_                                                        |
| `echo "blabla" \| tee filename` | write output to file _filename_                                                        |

### Open in File Browser

| command    | description                             |
| :--------- | :-------------------------------------- |
| nautilus . | öffne current directory in File Browser |

### comments

| command         | description               |
| :-------------- | :------------------------ |
| `# ein comment` | Kommentar in command line |

### time, date

| command            | description                              |
| :----------------- | :--------------------------------------- |
| `date`             | use `date --help` for formatting options |
| `date '+%m-%d-%y'` | for `06-30-24` format                    |

### dirs, pushd, popd

| command                 | description                                                                                                                                                                                                                                                      |
| :---------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `dirs`                  | print the directory stack (left to right)                                                                                                                                                                                                                        |
| `dirs -v`               | print the directory stack (one entry per line), prefixing each entry with its index in the stack, [doc](https://www.gnu.org/software/bash/manual/html_node/Directory-Stack-Builtins.html#Directory-Stack-Builtins)                                               |
| `pushd path/to/somedir` | Adds a directory to the top of the directory stack, [doc](https://www.gnu.org/software/bash/manual/html_node/Directory-Stack-Builtins.html#Directory-Stack-Builtins)                                                                                             |
| `pushd +N`              | Brings the Nth directory (counting from the left of the list printed by dirs, starting with zero) to the top of the list by rotating the stack, [doc](https://www.gnu.org/software/bash/manual/html_node/Directory-Stack-Builtins.html#Directory-Stack-Builtins) |
| `popd +N`               | Removes the Nth directory (counting from the left of the list printed by dirs), starting with zero, from the stack, [doc](https://www.gnu.org/software/bash/manual/html_node/Directory-Stack-Builtins.html#Directory-Stack-Builtins)                             |
| `popd -N`               | like `popd +N`, but counting from the right                                                                                                                                                                                                                      |

### cd, mkdir, pwd

| command                                   | description                                                                |
| :---------------------------------------- | :------------------------------------------------------------------------- |
| `pwd`                                     | zeige current working directory                                            |
| `cd path/to/somedir`                      |
| `cd`                                      | go to home directory                                                       |
| `mkdir -p /folder/subfolder/subsubfolder` | erstellt folder und subfolder automatisch, falls sie noch nicht existieren |

### mv, rm, cp, rename

| command                                                                                     | description                                                                                                                                                                                                                                                                                    |
| :------------------------------------------------------------------------------------------ | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mv -iv`                                                                                    | **Tipp:** IMMER -iv BENUTZEN! (-i für bestätigen, -v für ausgeführte Aktion zeigen)                                                                                                                                                                                                            |
| `mv "$file" "${file%???????}"  # 7 question marks to match 7 characters`                    | rename file and remove the last 7 characters of the filename                                                                                                                                                                                                                                   |
| `rename 's/pattern/replacepattern/' *.jpg`                                                  | rename all `.jpg` files by replacing `pattern` with `replacepattern` in their filenames (`sudo apt install rename`)                                                                                                                                                                            |
| `rm -iv`                                                                                    | **Tipp:** IMMER -iv BENUTZEN! (-i für bestätigen, -v für ausgeführte Aktion zeigen)                                                                                                                                                                                                            |
| `cp -iv`                                                                                    | **Tipp:** IMMER -iv BENUTZEN! (-i für bestätigen, -v für ausgeführte Aktion zeigen)                                                                                                                                                                                                            |
| `find ../path/to/search/ -iname *searchpattern* -exec cp -iv "{}" ./destination/folder/ \;` | find and copy the found files                                                                                                                                                                                                                                                                  |
| `cp -a`                                                                                     | attempts to make a copy that's as close to the original as possible: same directory tree, same file types, same contents, same metadata (times, permissions, extended attributes, etc.). Always use `cp -a` instead of `cp -r`. (see [cp -a vs cp -r](https://unix.stackexchange.com/a/44981)) |

### history, script

| command                             | description                                                                                                                                                                                                                                                                                                                                                                   |
| :---------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `history`                           | get a list of the last 1000 commands                                                                                                                                                                                                                                                                                                                                          |
| `history \| grep command_to_search` | search some pattern within the history generated list                                                                                                                                                                                                                                                                                                                         |
| `script`                            | start saving all input and output in the current terminal session in the file `typescript` (end recording via ctrl + d - this does not close the terminal here; use `script /path/to/mylogfile.txt` to save it in `/path/to/mylogfile.txt`; `typescript` will be overwritten if you start `script` twice without providing a name!). [source](https://askubuntu.com/a/557309) |

# xclip, xsel

- `xclip` and `xsel` are [very similar!](https://askubuntu.com/a/705658/1802556)
- `xclip` and `xsel` can store text into 3 different <span style="color:red">**selections**</span> (by default it is primary selection)
- the two main selections are:
  - <span style="color:red">**Primary selection**</span> is basically what you high-light and released with the middle mouse click (which corresponds to pressing both right and left touchpad key on a laptop).
  - <span style="color:red">**The clipboard**</span> is the traditional <kbd>Ctrl</kbd> <kbd>V</kbd>.

| command                                                     | description                                                                         |
| :---------------------------------------------------------- | :---------------------------------------------------------------------------------- |
| `xsel -bc`                                                  | clear clipboard                                                                     |
| `command \| xclip`                                          | copy the output of `command` and place it in `XA_PRIMARY` (the "primary selection") |
| `command \| xclip -sel prim`                                | same as `command \| xclip`                                                          |
| `command \| xclip -sel clip`                                | copy the output of `command` and place it in `XA_CLIPBOARD` ("the clipboard")       |
| `xclip -o`                                                  | print a selection to standard out                                                   |
| `xclip -o \| xclip -sel clip`                               | Copy `XA_PRIMARY` to `XA_CLIPBOARD`                                                 |
| `pwd \| tr -d '\n' \| sed 's/$/\//g' \| xclip -selection c` | copy the output of `pwd` into the clipboard (without the linebreak at the end)      |

# wmctrl

```bash
sudo apt install wmctrl
```

From [superuser](https://superuser.com/questions/142945/bash-command-to-focus-a-specific-window).

## List Windows

- `wmctrl -l` (list windows)

## Focus Windows

- `wmctrl -a window-name` (go to workspace and focus **by window name**)
  - map this to alt + shift + 1,2,3,etc. (after renaming the windows properly, renaming command: see below)
- `wmctrl -i -a 0x066f5d24` (go to workspace and focus **by window ID**)

## Rename Windows

- `wmctrl -i -a 0x066f5d24 -T "new-name"` (rename window)

## With xdotool

- [more complex commands](https://superuser.com/a/950287)

# Unzipping

| command                                                  | description                                                                                                      |
| :------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------- |
| `unzip file -d destination`                              | unzip to destination                                                                                             |
| `tar -C ./data/ -zxvf ~/Downloads/mnist.tgz`             | für **.tgz** (wobei `-C target_location -zxvf source.tgz`), **.tar.gz**                                          |
| _oder andersrum:_                                        |
| `tar -zxvf ~/Downloads/mnist.tgz -C ./data/`             |
| `tar -C ./data/ -jxvf ~/Downloads/datei.tar.bz2`         | für **.tar.bz2** (dh. `-j` flag statt `-z` flag)                                                                 |
| `tar -C ~/ -xvf tor-browser-linux64-10.5.2_en-US.tar.xz` | für **.tar.xz**                                                                                                  |
| `zip -FF 210211.zip --out 210211-2.zip -fz`              | "fix" a broken zip file, then run `unzip 210211-2.zip`, [stackexchange](https://unix.stackexchange.com/a/634316) |

# Convert

| command                                                               | description |
| :-------------------------------------------------------------------- | :---------- |
| `find . -iname "*.txt" -exec bash -c 'mv "$0" "${0%\.txt}.md"' {} \;` | txt2md:     |

# System information

## Software

**Note**: `lsb_release` and `uname` may report different Kernel versions!

| command              | description                                                                     |
| :------------------- | :------------------------------------------------------------------------------ |
| cat /etc/os-release  | Ubuntu Version (lang)                                                           |
| cat /etc/lsb-release | Ubuntu Version (lang)                                                           |
| `lsb_release -a`     | Ubuntu Version (kurz)                                                           |
| `lsb_release -cs`    | Ubuntu Version (e.g. "focal")                                                   |
| hostnamectl          | Ubuntu Version (mittel) mit Linux Kernel Version                                |
| uname --help         | Returns the help manual for the uname command, including all available options. |
| uname -a             | Prints all information for the server/system you're on.                         |
| uname -s             | Prints the kernel name                                                          |
| uname -n             | Prints the node name                                                            |
| uname -r             | Prints the kernel release data                                                  |
| uname -v             | Prints the kernel version data                                                  |
| uname -m             | Prints the machine data                                                         |
| uname -p             | Prints the processor information                                                |
| uname -i             | Prints the platform hardware information                                        |
| uname -o             | Prints the operating system information                                         |

## Hardware

| command              | description                                                                                                                                   |
| :------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------- |
| lscpu                |
| lshw                 |
| hwinfo --short       |
| lspci                |
| lsscsi               |
| lsusb                |
| inxi -Fx             |
| lsblk                | list [block devices](#block-device-vs-character-device), e.g. to see all drives attached to your system, including their sizes and partitions |
| df -H                |
| pydf                 |
| sudo fdisk -l        |
| `mount \| column -t` |

| command                     | description |
| :-------------------------- | :---------- |
| sudo dmidecode -t processor |
| sudo dmidecode -t memory    |
| sudo dmidecode -t bios      |

| command              | description |
| :------------------- | :---------- |
| cat /proc/cpuinfo    |
| cat /proc/meminfo    |
| cat /proc/version    |
| cat /proc/scsi/scsi  |
| cat /proc/partitions |

### Memory

| command                       | description |
| :---------------------------- | :---------- |
| `free -m`                     |
| `watch free -m`               | update every 2 seconds
| `dmesg -T \| grep oom-killer` | `-T`: show timestamps; shows the OutOfMemory-killer at work. This should not show any output! If it does, it is a bad sign!

### GPU

| command                                  | description                                                                       |
| :--------------------------------------- | :-------------------------------------------------------------------------------- |
| nvidia-smi -q -d temperature             | temperature info including critical temperature values, shutdown temperature etc. |
| nvidia-smi --query-gpu=name --format=csv | get GPU name                                                                      |

## Udev

**Udev** is the **Linux subsystem** that supplies your computer with **device events**. In plain English, that means it's the code that **detects** when you have things **plugged into** your computer, like a network card, external hard drives (including USB thumb drives), mouses, keyboards, joysticks and gamepads, DVD-ROM drives, and so on.

- **udev scripting**: see [tutorial](https://opensource.com/article/18/11/udev)
  - how to create a udev script triggered by some **udev event**, such as plugging in a specific thumb drive

| command                    | description                                                                                                                                                                                                              |
| :------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `udevadm monitor`          | tap into udev in real time and see what it sees when you plug in different devices. The monitor function prints received events for: `UDEV`: the event udev sends out after rule processing, `KERNEL`: the kernel uevent |
| `udevadm control --reload` | should load all rules (but reboot if you want to be sure)                                                                                                                                                                |

## Display

| command                 | description                                                                                                                                                                                                                                                                                                                                                                      |
| :---------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `xrandr`                | list all display modes; set the size, orientation and/or reflection of the outputs for a screen; can also set the screen size                                                                                                                                                                                                                                                    |
| `xrandr --fb 2560x1440` | set the screen resolution, when no physical display is connected (e.g. when connecting to Jetson AGX via Teamviewer or VNCviewer, put this in `/etc/xdg/autostart/resolution_screen_teamviewer.sh`, `chmod +x /etc/xdg/autostart/resolution_screen_teamviewer.sh`, create `/etc/xdg/autostart/resolution_screen_teamviewer.desktop` and reboot and connect via Teamviewer again) |

# Storage

| command                    | description |
| :------------------------- | :---------- |
| diskutil list              |
| diskutil info /dev/disk2s2 |

| command                                  | description                                                                                                       |
| :--------------------------------------- | :---------------------------------------------------------------------------------------------------------------- |
| sudo diskutil mountDisk /dev/disk2s2     | (Partitionsname disk2s2 steht in rechter Spalte bei diskutil list; /dev/disk2 mounted alle Unterpartitionen)      |
| sudo diskutil umountDisk /dev/disk2s2    |
| `mount_ntfs -o r "/Volumes/Volume node"` | (r für read-only; rw für read-write (NICHT MACHEN! Es gibt einen Grund warum das bei Mac per default nicht geht!) |

| command       | description                                                                                                            |
| :------------ | :--------------------------------------------------------------------------------------------------------------------- |
| df            | zeige alle Laufwerke, ganz rechts steht die Location mit dem Inhalt des Datenträgers (zB `/media/bra-ket/UBUNTU 20_0`) |
| sudo fdisk -l | wie df, aber mehr Details                                                                                              |
| lsusb         | list usb devices                                                                                                       |
| lsblk         | list [block devices](#block-device-vs-character-device)                                                                |

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

| command                   | description                                                                                                                                                                                                                                                                   |
| :------------------------ | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `hdparm`                  | get statistics about the hard disk, alter writing intervals, acoustic management, and DMA settings. It can also set parameters related to drive caches, sleep mode, power management, acoustic management, and DMA settings                                                   |
| `sudo hdparm -I /dev/sda` | Request **identification info** directly from the drive, which is displayed in a new expanded format with considerably more detail than with the older `-i` option. (source: `man hdparm -I`); may differ from information provided by `-i` option! (source: `man hdparm -i`) |

## Eject

1. press on **eject** button for all partitions in nautilus
2. open **gnome-disks**
3. press on **stop** button below all partitions, if any partition is still mounted (else no stop button should be available)
4. press **power off** button in the top bar (only after all partitions have been unmounted in step 3!)
5. disk LED will be turned off now
6. close gnome-disks via alt + F4

| command                 | description |
| :---------------------- | :---------- |
| `sudo eject /media/SDD` |

Troubleshooting:

1. `One or more applications are keeping the volume busy`

| command                     | description                                                                                                                                                                                                                            |
| :-------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sudo fuser -mv /media/SDD` | displays all processes accessing `/media/SDD`, where the `m` tells it to look on the given location, the `v` switches the output to a human readable list instead of just a bunch of PIDs. [askubuntu](https://askubuntu.com/a/578631) |

2. `Disconnecting from filesystem` notification does not disappear automatically when ejecting an external hard drive (by pressing the eject button in Nautilus)

- **Possible Solutions that worked once:**
  - close all Nautilus/Dolphin windows that access the external hard drive
  - press **ctrl + c** on some file that is stored on the **internal** hard drive (if ctrl + c was pressed on some file that is stored on the **external** hard drive, this will block the external hard drive when you try to eject it)

3. in gnome-disks: External hard drive shows a **"loading"** symbol and the power off button for this external hard drive is **grayed out**

- **Possible Solutions that worked once:**
  - just wait
  - close gnome-disks and open it again
  - after a while the power off button for this external hard drive is not **grayed out** any more and the physical LED on the hard drive stops blinking

## du, Disk Usage

- [FAQ](https://unix.stackexchange.com/a/120312)
  - How much disk space does a file use?
  - Why is the total from `du` different from the sum of the file sizes?
  - What are these numbers from `df` exactly?
  - What's using the space on my disk?
- [Why is there a discrepancy in disk usage reported by df and du?](https://unix.stackexchange.com/questions/9612/why-is-there-a-discrepancy-in-disk-usage-reported-by-df-and-du)
- [Why du and df display different values](http://linuxshellaccount.blogspot.com/2008/12/why-du-and-df-display-different-values.html)

```bash
$ du --help
 [ ... ]
 Display values are in units of the first available SIZE from --block-size,
 and the DU_BLOCK_SIZE, BLOCK_SIZE and BLOCKSIZE environment variables.
 Otherwise, units default to 1024 bytes (or 512 if POSIXLY_CORRECT is set).
```

| command                                   | description                                                                                                                                                                                            |
| :---------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ncdu`                                    | like `du -sh`, but more convenient (because avoids typing)                                                                                                                                             |
| `du -sh *`                                |
| `du -sh ./*/`                             | show directories only                                                                                                                                                                                  |
| `du -sch *`                               | `-c` to show grand total                                                                                                                                                                               |
| `du -sh \* \| sort -h`                    | "ascending": largest file in the last output line                                                                                                                                                      |
| `du -sh \* \| sort -rh`                   | "descending": largest file in the first output line (`-r` for "reverse order")                                                                                                                         |
| `du -sch ./folder \| sort -rh \| head -5` | zeige disk usage (=size) of folder (`-h` für human readable; `-c` für zeige grand total am Ende) (`sort -rh` für sortiere nach size, wobei `-r` für reverse und `-h` für compare human readable sizes) |
| `du -sh * .[^.]*`                         | show hidden files, too (`.[^.]*` aka `.[!.]*` is a ["globbing pattern"](https://stackoverflow.com/questions/41034115/in-shell-scripting-what-does-mean))                                               |
| `du -h -d 1 *`                            | `-d 1` or `--max-depth=1` display the sizes of only the directories immediately within the specified path. If we were to specify 2 it would go a level further.                                        |
| `du -h -d 1 -t 1G /`                      | `-t`: threshold, show the sizes of all first level directories larger than 1GB within the root `/` path                                                                                                |

| command | description                     |
| :------ | :------------------------------ |
| `df -h` | to analyze the whole filesystem |

# Camera

| command                                                                                                                                                           | description                                                                |
| :---------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------- |
| `mpv /dev/video0`                                                                                                                                                 | check, if device `/dev/video0` is the webcam                               |
| `vlc v4l2:///dev/video0`                                                                                                                                          | check, if device `/dev/video0` is the webcam                               |
| `mplayer tv://device=/dev/video0`                                                                                                                                 | check, if device `/dev/video0` is the webcam                               |
| `gst-launch-1.0 v4l2src device=/dev/video2 name=cam_src ! videoconvert ! videoscale ! video/x-raw,format=RGB ! queue ! videoconvert ! ximagesink name=img_origin` | check, if device `/dev/video2` is the webcam                               |
| `v4l2-ctl --list-devices`                                                                                                                                         | list cameras                                                               |
| `v4l2-ctl -d /dev/video0 --list-ctrls`                                                                                                                            | show all settings                                                          |
| `v4l2-ctl -L -d /dev/videoN`                                                                                                                                      | list available settings of camera `/dev/videoN`                            |
| `v4l2-ctl -d /dev/video0 --list-formats-ext`                                                                                                                      | show all supported camera resolutions                                      |
| `v4l2-ctl --set-ctrl power_line_frequency=0 -d /dev/videoN`                                                                                                       | set powerline frequency (50Hz vs 60Hz) of camera `/dev/videoN`             |
| `v4l2-ctl -d /dev/video2 --list-formats-ext`                                                                                                                      | check supported pixel formats, fps and resolutions of camera `/dev/video2` |
| `v4l2-ctl --device /dev/video3 --set-ctrl=zoom_absolute=120`                                                                                                      | zoom in                                                                    |

# curl, wget

`curl` and `wget` are retrieval commands.

| command                                                                                                                                | description                                                                                                                                                                                   |
| :------------------------------------------------------------------------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `wget -nc`                                                                                                                             | `-nc` for "do not overwrite existing files"                                                                                                                                                   |
| `wget -O output_file -q https://checkip.amazonaws.com -P DESTINATION`                                                                  | `-O output_file`: benutze Minuszeichen "-" statt `output_file` wenn output direkt in Terminal erscheinen soll; `-q` für quiet; `-P` für Zielordner                                            |
| `torsocks wget "https://cdimage.debian.org/debian-cd/current/amd64/iso-cd/debian-10.1.0-amd64-xfce-CD-1.iso"`                          | download anonymously (need to install `sudo apt-get install tor torsocks`), [reddit](https://www.reddit.com/r/privacytoolsIO/comments/ddnx6x/how_to_download_files_properly_and_anonymously/) |
| `wget -A pdf,jpg -m -p -E -k -K -np http://site/path/`                                                                                 | get all pdfs and jpgs from site                                                                                                                                                               |
| `wget --accept pdf,jpg --mirror --page-requisites --adjust-extension --convert-links --backup-converted --no-parent http://site/path/` | same as above using long option names                                                                                                                                                         |
| `curl -s https://checkip.amazonaws.com`                                                                                                | `-s` für silent                                                                                                                                                                               |

# gpg, apt-key

**Note**: Do not forget to remove the respective sources list in `/etc/apt/sources.list.d/` as well.

| command                                                              | description                                                                                                                                                                             |
| :------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| gpg --list-keys                                                      | list your keys (will list only the ones stored in `~/.gnugpg`, but not the ones stored in `/etc/apt/trusted.gpg.d/`, see [stackoverflow](https://askubuntu.com/a/1262753))              |
| gpg --delete-keys A3C4F0F979CAA22CDBA8F512EE8CBC9E886DDD89           | delete key A3C4F0F979CAA22CDBA8F512EE8CBC9E886DDD89 from keyring                                                                                                                        |
| apt-key list                                                         |
| sudo apt-key del "27B2 5BF6 36CF 72B4 334D AC98 F84C B847 29F1 B545" | it is safer to use the whole fingerprint, the keyid could have duplicates (at least when you use PGP for emails, I read you should share your whole fingerprint and not just the keyid) |

# cron

The software utility cron also known as cron job is a time-based job scheduler in Unix-like computer operating systems. Users who set up and maintain software environments use cron to schedule jobs to run periodically at fixed times, dates, or intervals.

| command      | description                                                                |
| :----------- | :------------------------------------------------------------------------- |
| `crontab -e` | opens a file in which jobs can be specified (read this file for more info) |

# network

## socket statistics

"socket" (aka the 2-Tuple (IP, Port), see DatKom.md)

| command                           | description                                                                                                                                                                                                                              |
| :-------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `netstat -tulpn \| grep ':80'`    | check if port `:80` is in use                                                                                                                                                                                                            |
| `sudo netstat -lpn \| grep :8889` | zeigt pid des Prozesses auf port 8889 (port kann dann mit `kill \<pid\>` frei gemacht werden)                                                                                                                                            |
| `ss`                              | `ss` is the new `netstat` (`ss` is faster, more human-readable and easier to use); displays stats for PACKET, TCP, UDP, DCCP, RAW, and Unix domain sockets, [linux.com](https://www.linux.com/topic/networking/introduction-ss-command/) |

## Download, Upload

| command       | description                                                                                                                               |
| :------------ | :---------------------------------------------------------------------------------------------------------------------------------------- |
| `md5sum file` | to check if the file `file` is not corrupted after transferring it (check if the `md5sum` is the same on both sides of the file transfer) |

## ssh

| command                       | description                                                                                                                                |
| :---------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------- |
| `w`                           | list all ssh sessions                                                                                                                      |
| `enter ~ .`                   | disconnect (when frozen), see `man ssh`, [stackoverflow](https://stackoverflow.com/questions/28981112/how-do-i-close-a-frozen-ssh-session) |
| `ssh bra-ket@10.14.14.60`     | installiere vorher openssh-server auf beiden Computern                                                                                     |
| `firefox -no-remote -no-xshm` | display firefox on local client (no -X or -Y flag needed in previous ssh command)                                                          |

**Achtung**:

- erst in den Server einloggen und **dann** erst in den Computer einloggen, der die Internetverbindung des Servers benutzt !
- **Error**: "X11 connection rejected because of wrong authentication."
  - `~/.Xauthority` löschen und nochmal per ssh einloggen kann helfen bei xauth Problemen (siehe [issue](https://unix.stackexchange.com/a/494742)) !
  - (prüfe evtl noch) nach [source](https://www.cyberciti.biz/faq/x11-connection-rejected-because-of-wrong-authentication/):
    - Make sure X11 SSHD Forwarding Enabled
    - Make sure X11 client forwarding enabled

### Graphics/Display

| command                         | description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| :------------------------------ | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ssh -Y bra-ket@10.14.14.60`    | display graphical output on trusted local client (**Caution**: may lead to security issues), [difference -X vs -Y flag](https://askubuntu.com/a/35518)                                                                                                                                                                                                                                                                                                                                        |
| `ssh -X bra-ket@10.14.14.60`    | display graphical output on **un**trusted local client, [difference -X vs -Y flag](https://askubuntu.com/a/35518)                                                                                                                                                                                                                                                                                                                                                                             |
| `export DISPLAY=localhost:10.0` | set display (use `w` or `xauth list` to list diplays) ("**:0**" ist der server monitor; zB. "**localhost:10.0**" ist der client monitor, wobei `localhost:=127.0.0.1` (`127.0.0.1` is the loopback Internet protocol (IP) address also referred to as the localhost. The address is used to establish an IP connection to the same machine or computer being used by the end-user. The same convention is defined for computers that support IPv6 addressing using the connotation of `::1`.) |

### On Macs

| command         | description                                                                                                                                                                                              |
| :-------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `caffeinate -u` | **for Mac**: prevent the system from sleeping and (-u for) prevent the system from sleeping [source](https://apple.stackexchange.com/questions/53802/waking-display-from-terminal-general-waking/161527) |

### ssh keys

| command                     | description                                                               |
| :-------------------------- | :------------------------------------------------------------------------ |
| `ssh-keygen -R 10.14.14.92` | remove `10.14.14.92` from `.ssh/known_hosts` (falls aus Versehen geaddet) |

## scp

**Achtung:** Spaces müssen im path **DOPPELT** escapet werden ! (s. [hier](https://stackoverflow.com/a/20364170/12282296))

| command                                                                          | description                                                             |
| :------------------------------------------------------------------------------- | :---------------------------------------------------------------------- |
| `scp "source" "target"`                                                          | immer Anführungszeichen `"` um den `source` Pfad setzen!                |
| `scp -rv Macbook:"~/Desktop/Uni/FS1/Essential\ Astrophysics\ WS1819" ~/Desktop/` | spaces DOPPELT escapen (hier: mit `"` **UND** mit `\` **gleichzeitig**) |
| `scp -r [!.]* source target`                                                     | exclude hidden files                                                    |

## rsync

### rsync basics

Rsync patterns: [stackexchange](https://unix.stackexchange.com/a/2503)

| command                                         | description                                                                                                                                                                |
| :---------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rsync -a path/to/source/ path/to/destination/` | copy directory; **note**: always use `/` at the end of the `path/to/source/` (**Warning**: `-a` better than `-r` because `-r` tag does not copy some stuff, e.g. symlinks) |
| `rsync -avz *source* *destination*`             | `-z` flag: compressing and transfer the files (comes in handy while transferring a huge amount of data over a slow internet connection)                                    |
| `rsync -av --progress`                          | show progress report                                                                                                                                                       |

### rsync exclude

| command                                                                                                              | description                                                                                                                               |
| :------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------- |
| `rsync -a --exclude="SomeDirForPythonInstall"`                                                                       | exclude directory `SomeDirForPythonInstall`                                                                                               |
| `rsync -a --exclude=".*"`                                                                                            | excludes hidden files and directories                                                                                                     |
| `rsync -a --exclude=".*/"`                                                                                           | exclude hidden directories only                                                                                                           |
| `rsync -av --progress sourcefolder /destinationfolder --exclude thefoldertoexclude`                                  | exclude `thefoldertoexclude`                                                                                                              |
| `rsync -av --progress sourcefolder /destinationfolder --exclude thefoldertoexclude --exclude anotherfoldertoexclude` | you can use `-exclude` multiple times ([stackoverflow](https://stackoverflow.com/a/14789400/12282296))                                    |
| `rsync -av --progress ../../kitcar-gazebo-simulation/ ./kitcar-gazebo-simulation/ --exclude '*.bag'`                 | exclude all files ending with `.bag` in the current directory, no recursive traversal ([patterns](https://unix.stackexchange.com/a/2503)) |

### Resume partially scp-transferred files using Rsync

[source](https://ostechnix.com/how-to-resume-partially-downloaded-or-transferred-files-using-rsync/)

| command                                                           | description                                    |
| :---------------------------------------------------------------- | :--------------------------------------------- |
| `rsync -P -rsh=ssh ubuntu.iso sk@192.168.225.22:/home/sk/`        | resume partially transferred file `ubuntu.iso` |
| `rsync --partial -rsh=ssh ubuntu.iso sk@192.168.225.22:/home/sk/` | see above                                      |
| `rsync -avP ubuntu.iso sk@192.168.225.22:/home/sk/`               | see above                                      |
| `rsync -av --partial ubuntu.iso sk@192.168.225.22:/home/sk/`      | see above                                      |

# tree

| command                             | description                                      |
| :---------------------------------- | :----------------------------------------------- |
| `tree -H ./ > result.html`          | save directory tree to file                      |
| `tree -aH --du -h ./ > result.html` | report human readable sizes of files and folders |
| `tree -aJ --du -h ./ > result.json` | output json format                               |
| `firefox ./result.html`             | view html tree created by `tree` command         |

# xmodmap, xev

**Deprecated**, use `setxkbmap` instead.

Use

```bash
xev
```

to find the keycode of a key. Then, in order to remap this key, run e.g.

```bash
# Syntax: xmodmap -e "keycode [keyNumber] = [normal] [shift] [NoIdea] [NoIdea] [altGr] [shift+altGr]"
xmodmap -e "keycode 48 = bracketright braceright NoSymbol NoSymbol adiaeresis Adiaeresis"
xmodmap -e "keycode 47 = bracketleft braceleft NoSymbol NoSymbol odiaeresis Odiaeresis"
xmodmap -e "keycode 45 = 0x06b 0x04b"
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
