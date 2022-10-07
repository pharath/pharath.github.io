---
title: "WSL2 Cheatsheet"
read_time: false
excerpt: "For quick setup of WSL2"
header:
    teaser: /assets/images/Vim.jpg
    overlay_image: /assets/images/Vim.jpg
    overlay_filter: 0.5 
toc: true
toc_label: "Contents"
toc_sticky: true
categories:
    - Cheatsheet
tags:
    - wsl2
    - cheatsheet

---

# Setup
 
## Windows 10

- Select **Start** \> **Settings** \> **Time & language** \> **Language & region**.
    - Choose a language from the Windows display language menu, or, next to **Preferred languages**, select **Add a language** to install the one you want if it isn't listed.
    - press <kbd>Alt</kbd> + <kbd>Shift</kbd> to change keyboard language to German
- Right-click on the taskbar, and in the context menu that appears, select **News and interests** \> **Turn off**

### Wi-Fi: Turn off metered connection ("getaktete Verbindung")

- Eine **getaktete Verbindung** ist eine Internetverbindung, der ein Datenlimit zugeordnet wurde. 
- Mobilfunkdatenverbindungen sind standardmäßig als getaktet festgelegt. 
- WLAN- und Ethernet-Netzwerkverbindungen können als getaktet festgelegt werden, sind es standardmäßig aber nicht.

- To set a Wi-Fi network connection as metered:
    - Select **Start** \> **Settings** \> **Network & Internet** \> **Wi-Fi** \> **Manage known networks**.
    - Select the Wi-Fi network \> **Properties** \> turn off **Set as metered connection**.
 
## WSL2, Windows Terminal 

- install [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install)
- install [Windows Terminal](https://apps.microsoft.com/store/detail/windows-terminal/9N0DX20HK701?hl=de-de&gl=de)
    - open Windows Terminal Settings via <kbd>Ctrl</kbd> + <kbd>,</kbd>
    - set default profile "Ubuntu"
    - under "Ubuntu" profile settings, under "Appearance": 
        - set default color scheme "Tango Dark"
        - set default font "Ubuntu Mono"
        - set Cursor shape "filled box"

## Vim

- install [Vundle](https://github.com/VundleVim/Vundle.vim)
- install [Papercolor Theme](https://github.com/NLKNguyen/papercolor-theme) in vim
- install [Ubuntu font](https://assets.ubuntu.com/v1/0cef8205-ubuntu-font-family-0.83.zip) as default terminal font
    - unzip
    - mark all the fonts in the package 
    - right-click on the marked packages and select "install"
- install [vim-markdown](https://github.com/preservim/vim-markdown)
- install [markdown preview](https://github.com/iamcco/markdown-preview.nvim)
    - don't forget `:call mkdp#util#install()` after `:PluginInstall`

## VSCode
 
- install VSCode on Windows (not in WSL!)
- install [Remote - WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl) Extension in VSCode
    - [guide](https://code.visualstudio.com/docs/remote/wsl)
    - restart Windows Terminal after installation
    - in WSL2 run `code .`. The download will take a while and the following messages will appear:
      ```bash
      Installing VS Code Server for x64 (74b1f979648cc44d385a2286793c226e611f59e7)
      Downloading: 100%
      Unpacking: 100%
      Unpacked 2424 files and folders to /home/bra-ket/.vscode-server/bin/74b1f979648cc44d385a2286793c226e611f59e7.
      ```
- install [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) Extension in VSCode

## bash

- `~/.bashrc` should already exist 
    - set
      ```bash
      HISTSIZE= 
      HISTFILESIZE=
      ```
- create `.bash_aliases`
  ```bash
  alias gcc_phth='gcc -g -Wall -pedantic '
  ```

## C/C++ 

```bash
sudo apt update && sudo apt upgrade
sudo apt install gcc build-essential gdb
```
- `build-essential gdb` includes `g++`

## firefox

- disable addons:
    - iCloud Bookmarks
    - StickyNotes
    - TeXZilla
    - Reload in address bar

# Usage

## Starting a Windows Program in WSL

- [source](https://www.bleepingcomputer.com/news/microsoft/how-to-run-windows-10-programs-in-a-wsl-linux-shell/)
- make sure the program can be found by checking `echo $PATH`
- When you are in a WSL shell, you can execute a Windows 10 program simply by typing its full name, including the `.exe` extension.
- If you do not include the `.exe` extension when executing a command, WSL will think its a Linux command.
- If you wanted to launch a program that is not in your `PATH`, you would need to specify the full `path/to/program.exe`.
    - e.g. `"/mnt/c/program files/7-zip/7z.exe" a -tzip code.zip code/`

## Apps

| command | description |
| :---: | :---: |
explorer.exe . | open Windows Explorer in current directory
