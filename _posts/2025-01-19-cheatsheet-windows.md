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

## Powershell

| command                                      | description                                                          |
| :------------------------------------------- | :------------------------------------------------------------------- |
| `cmd`                                        | start a new sub-command-prompt in the current command prompt         |
| `Get-Help command`                           | show help for `command`                                              |
| `command -Verbose`                           |                                                                      |
| `rm -Force .\dir\`                           |                                                                      |
| `(Get-PSReadLineOption).HistorySavePath`     | to get the path of the Powershell `history.txt`                      |
| `cat (Get-PSReadLineOption).HistorySavePath` | to print the content of the Powershell `history.txt`                 |
| `gci env:`                                   | list all Powershell environment variable names and their values      |
| `$env:VARIABLE`                              | eg. `%AppData%`: to get the path of `%AppData%` enter `$env:APPDATA` |
| `cd $env:APPDATA`                            |                                                                      |
| `pushd`                                      | use `pushd` without a path to list all directories on the stack      |
| `popd`                                       |                                                                      |
| `start file`                                 | like `xdg-open file`                                                 |
| `powershell .\myscript.ps1 1> outfile.txt`   | Pipe/Redirect the success stream (`1>`)                              |
| `powershell .\myscript.ps1 *> outfile.txt`   | Pipe/Redirect all streams (`*>`)                                     |

## Path

- reload `Path` while in Powershell (eg. when `Path` has changed while working in Powershell):
  - `$env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path","User")`

### find

| command                                                                                                                                                                      | description                                                                                                                   |
| :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------- |
| `Get-ChildItem *.foo`                                                                                                                                                        |                                                                                                                               |
| `Get-ChildItem -Recurse 'C:\Program Files' -Include log4j-core-*.*.[0-9].jar, log4j-core-*.*.[1-9][0-9].jar -ErrorAction SilentlyContinue -Force \| ForEach-Object FullName` | `[1-9][0-9]` matches exactly 2 characters (digits), and also matching just one digit (`[0-9]`) requires an additional pattern |

### ls (aka Get-ChildItem)

`ls` is an alias for `Get-ChildItem`.

| command                                                  | description                           |
| :------------------------------------------------------- | :------------------------------------ |
| `Get-ChildItem \| Sort-Object LastWriteTime -Descending` |                                       |
| `ls \| Sort-Object LastWriteTime -Descending`            |                                       |
| `ls \| sort LastWriteTime -Descending`                   |                                       |
| `ls \| sort LastAccessTime -Descending`                  |                                       |
| `ls -File \| sort LastAccessTime -Descending`            | only files (not directories)          |
| `ls *pattern* \| sort LastAccessTime -Descending`        | only filenames that contain `pattern` |

### select-string

- `Select-String [-Culture <String>] [-Pattern] <String[]> [-Path] <String[]> [-SimpleMatch] [-CaseSensitive]`
- parameters:
  - defaults: case insensitive
  - `-Pattern`: by default the string is an argument to this parameter (ie. when no parameter is specified at all). By default None. Specifies the text to find on each line. The pattern value is treated as a regular expression.
  - `-CaseSensitive`: by default false.
  - `-SimpleMatch`: by default false. Indicates that the cmdlet uses a simple match rather than a regular expression match. In a simple match, `Select-String` searches the input for the text in the `Pattern` parameter. It doesn't interpret the value of the `Pattern` parameter as a regular expression statement.

| command                                | description          |
| :------------------------------------- | :------------------- |
| `ipconfig.exe \| Select-String "ipv4"` | like `grep` on Linux |

### Special Locations

- Current Spotlight Background Image: `C:\Users\H1524\AppData\Roaming\Microsoft\Windows\Themes\CachedFiles`

### winget

- `winget --info` (show where packages are installed/stored)
