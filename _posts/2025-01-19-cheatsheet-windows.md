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

### select-string

- `Select-String [-Culture <String>] [-Pattern] <String[]> [-Path] <String[]> [-SimpleMatch] [-CaseSensitive]`
- parameters:
  - defaults: case insensitive
  - `-Pattern`: by default the string is an argument to this parameter (ie. when no parameter is specified at all). By default None. Specifies the text to find on each line. The pattern value is treated as a regular expression.
  - `-CaseSensitive`: by default false.
  - `-SimpleMatch`: by default false. Indicates that the cmdlet uses a simple match rather than a regular expression match. In a simple match, `Select-String` searches the input for the text in the `Pattern` parameter. It doesn't interpret the value of the `Pattern` parameter as a regular expression statement.

| command                                      | description                                                          |
| :------------------------------------------- | :------------------------------------------------------------------- |
| `ipconfig.exe \| Select-String "ipv4"` | like `grep` on Linux

### Special Locations

- Current Spotlight Background Image: `C:\Users\H1524\AppData\Roaming\Microsoft\Windows\Themes\CachedFiles`
