---
title: "Jupyter Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - jupyter
  - cheatsheet
toc: true
toc_label: "Contents"

---

# jupyter

## jupyterlab

**Dark Mode**: Menubar "Settings" &rarr; "JupyterLab Theme" &rarr; "JupyterLab Dark"

### Shortcuts

| command | description |
| :--- | :--- |
ctrl + shift + c | [command palette](https://jupyterlab.readthedocs.io/en/stable/user/commands.html#command-palette)
ctrl + b | Show/Hide Left Sidebar 

**Mac**:

| command | description |
| :--- | :--- |
cmd + Shift + c | Search; “Shortcut finder”

In notebook mode, Cell selection mode (dh nicht in einer Cell):

| command | description |
| :--- | :--- |
cmd + , | open Shortcut Settings
cmd + i | open contextual help (workflow: öffne in separatem tab horizontal unter code editor; contextual help zeigt Inhalt von Variablen, source code für Funktionen, … etc)
ctrl + Shift + q | close and shutdown notebook
ctrl + Shift + 5 | phth defined: previous tab
ctrl + Shift + 6 | phth defined: next tab
o | hide cell output
o, o | show cell output
Shift + o | hide all cell outputs
Shift + o, Shift + o | show all cell outputs

### Command List

- [Command List](https://jupyterlab.readthedocs.io/en/stable/user/commands.html#commands-list)

## magics

[list of all magics](https://ipython.readthedocs.io/en/stable/interactive/magics.html#cell-magics)

### line magics

```python
%reset   # unset all variables
%cd
%mkdir
%cat
%autosave 0   # disable autosave in notebook
```

### cell magics

```python
%%bash
```

## Convert notebooks to other formats

| command | description |
| :---: | :---: |
jupyter nbconvert --to html notebook.ipynb | pass --execute flag, if cells should be run before converting
jupyter nbconvert --to pdf notebook.ipynb |

