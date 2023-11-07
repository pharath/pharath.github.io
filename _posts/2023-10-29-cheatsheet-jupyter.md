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

### Shortcuts

| command | description |
| :--- | :--- |
ctrl + shift + c | [command palette](https://jupyterlab.readthedocs.io/en/stable/user/commands.html#command-palette)
ctrl + b | Show/Hide Left Sidebar 

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

