---
title: "Bash Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - bash 
  - cheatsheet
---

# bash scripting

## Shebang

beste Erklärung: [askubuntu discussion](https://stackoverflow.com/questions/7670303/purpose-of-usr-bin-python3-shebang/7670338#7670338)

oder aus [Wikipedia](https://en.wikipedia.org/wiki/Shebang_(Unix)):

In computing, a shebang is the character sequence consisting of the characters number sign and exclamation mark (#!) at the beginning of a script.

When a text file with a shebang is used as if it is an executable in a Unix-like operating system, the program loader mechanism parses the rest of the file's initial line as an interpreter directive. The loader executes the specified interpreter program, passing to it as an argument the path that was initially used when attempting to run the script, so that the program may use the file as input data.[8] For example, if a script is named with the path path/to/script, and it starts with the following line, #!/bin/sh, then the program loader is instructed to run the program /bin/sh, passing path/to/script as the first argument. In Linux, this behavior is the result of both kernel and user-space code.[9] 
The shebang line is usually ignored by the interpreter, because the "#" character is a comment marker in many scripting languages; some language interpreters that do not use the hash mark to begin comments still may ignore the shebang line in recognition of its purpose.[10] 

## Was bedeutet $# in einem bash script? (s. [askubuntu post](https://askubuntu.com/questions/939620/what-does-mean-in-bash))

Zum Beispiel:

```bash
if [[ $# -gt 0 ]]; then
```

$# steht für: number of arguments (wie argc in C)


## Vergleichsoperatoren

```bash
if [[ $# -gt 0 ]]; then
else
fi
```

`-gt` für "greater than" (ie der `>` operator) in der condition
`-eq` für "equal to" (ie der `=` operator)

## Standard Stream Redirection: stdin, stdout and stderr

- see "[What is /dev/null](https://linuxhint.com/what_is_dev_null/)"

### File Descriptor (part of POSIX API)

- see [file descriptor](https://en.wikipedia.org/wiki/File_descriptor)
- Each Unix process should have three standard POSIX file descriptors, corresponding to the three standard streams: `stdin`, `stdout` and `stderr`.
- Each file descriptor has a **non**-negative integer value: `stdin`: 0, `stdout`: 1, `stderr`: 2.

### Redirect stderr to a text file

- `asdfadsa 2> error.txt`

### Redirect stdout to a text file

- **Note:** If no file descriptor value is specified, bash will use `stdout` (i.e. file descriptor value `1`) by default.
- `echo "Hello World" > log.txt`

### Redirect stderr AND stdout to /dev/null

- `grep -r hello /sys/ &> /dev/null`

### Redirect ALL output to /dev/null

- In certain situations, the output may not be useful at all. Using redirection, we can dump all the output into the void:
  - `> /dev/null 2>&1`
    - `> /dev/null` dumps all the `stdout` to `/dev/null`
    - `2>&1` redirects `stderr` (file descriptor value: `2`) to `stdout` (file descriptor value: `1`)
- phth note: `> /dev/null 2>&1` idea came from `~/git/geohot/openpilot/update_requirements.sh`
