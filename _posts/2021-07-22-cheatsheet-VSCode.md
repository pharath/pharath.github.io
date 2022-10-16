---
title: "VS Code Cheatsheet"
read_time: false
excerpt: "Some tips for VS Code."
excerpt_separator: "<!--more-->"
header:
    teaser: /assets/images/vscode.png
    overlay_image: /assets/images/vscode.png
    overlay_filter: 0.5 
categories:
  - Cheatsheet
tags:
  - vscode
  - cheatsheet
---

# Setup

- install extensions: 
    - Remote-Containers
        - on Ubuntu: only Dev-Containers
    - C/C++
    - Doxygen Documentation Generator
    - CMake
    - CMake Tools
    - Python
    - Pylance
    - Jupyter
    - Jupyter Keymap
    - Jupyter Notebook Renderers
    - TabNine Autocompletion
    - vim
    - Markdown PDF
        - for md2pdf conversion
- set "Toggle Vim Mode" command keybinding "ctrl + alt + v" in menu "Manage" (unten links) -> keyboard shortcuts -> search "togglevim" 

## Reset

- if something does not work, e.g.
    - context menu is popping up automatically on vscode startup
- [stackoverflow instructions](https://stackoverflow.com/a/36109176)
    - settings:
        - Press F1
        - Type user settings
        - Press enter
        - Click the "sheet" icon to open the settings.json file
        - delete the file's contents and save
    - extensions:
        - delete `~/.vscode/extensions`

# Navigation

| command | description |
| :---: | :---: |
ctrl + alt + v | toggle vim mode (this custom shortcut must have been configured previously) 
ctrl + w | close current tab
ctrl + o | open file
ctrl + n | new file

## Focus

ctrl + 0 | focus file explorer
ctrl + 1 | focus editor group 1
ctrl + 2 | focus editor group 2, usw.
ctrl + w | close current editor group (after all tabs in this group have been closed)
alt + 1 | focus first tab in current editor group
alt + 2 | focus second tab in current editor group, usw.
ctrl + j | show panel (problems, output, terminal, debug console)
ctrl + b | show side bar (file explorer)

## Fold

| command | description |
| :---: | :---: |
ctrl-k ctrl-t | change theme
ctrl-k ctrl-0 | collapse all
ctrl-k ctrl-1 | collapse level 1
ctrl-k ctrl-j | unfold all

## Jump

| command | description |
| :---: | :---: |
ctrl + shift + . | focus breadcrumbs (jump between methods/functions)
ctrl + alt + - | go back (i.e. jump to previous location) (Windows: alt + left)
ctrl + shift + - | go forward (i.e. jump to next location) (Windows: alt + right)
ctrl + k ctrl + d | "move last selection to next find match" (like `*` in Vim)
ctrl + d | "add selection to next find match" (for quickly modifying multiple occurrences of the selected code simultaneously)

## Write

| command | description |
| :---: | :---: |
ctrl + shift + up/down | place cursor on multiple lines
ctrl + h | replace (in the replace window press ctrl + alt + enter to replace all)
ctrl + c | copy line
ctrl + x | cut line
ctrl + shift + up/down, ctrl + x | select and cut multiple lines
ctrl + shift + k | delete line
ctrl + k ctrl + c | add line comment
ctrl + k ctrl + u | remove line comment
shift + alt + DownArrow | copy line down

## Format

- **automatic code format** (in command palette: "format document"): ctrl + shift + I

# Git

Symbols next to the filename in each tab:

| symbol | meaning |
| :---: | :---: |
**A** | Added (This is a new file that has been added to the repository)
**M** | Modified (An existing file has been changed)
**D** | Deleted (a file has been deleted)
**U** | Untracked (The file is new or has been changed but has not been added to the repository yet)
**C** | Conflict (There is a conflict in the file)
**R** | Renamed (The file has been renamed)
**S** | Submodule (In repository exists another subrepository)
