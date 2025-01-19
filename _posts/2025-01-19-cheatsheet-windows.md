---
title: "Windows Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - windows
  - cheatsheet
---

# Windows

## Powershell

| command                                      | description                                                          |
| :------------------------------------------- | :------------------------------------------------------------------- |
| `(Get-PSReadLineOption).HistorySavePath`     | to get the path of the Powershell `history.txt`                      |
| `cat (Get-PSReadLineOption).HistorySavePath` | to print the content of the Powershell `history.txt`                 |
| `gci env:`                                   | list all Powershell environment variable names and their values      |
| `$env:VARIABLE`                              | eg. `%AppData%`: to get the path of `%AppData%` enter `$env:APPDATA` |
| `cd $env:APPDATA`                            |

