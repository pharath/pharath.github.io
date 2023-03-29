---
title: "Bash Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
toc: true
toc_label: "Contents"
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - bash 
  - cheatsheet
---

# Shebang

beste Erklärung: [askubuntu discussion](https://stackoverflow.com/questions/7670303/purpose-of-usr-bin-python3-shebang/7670338#7670338)

oder aus [Wikipedia](https://en.wikipedia.org/wiki/Shebang_(Unix)):

In computing, a **shebang** is the character sequence consisting of the characters number sign and exclamation mark (`#!`) at the beginning of a script.

When a text file with a shebang is used as if it is an executable in a Unix-like operating system, the program loader mechanism parses the rest of the file's initial line as an **interpreter directive**. The loader executes the specified interpreter program, passing to it as an argument the path that was initially used when attempting to run the script, so that the program may use the file as input data. **For example**, if a script is named with the path `path/to/script`, and it starts with the following line, `#!/bin/sh`, then the program loader is instructed to run the program `/bin/sh`, passing `path/to/script` as the first argument. **In Linux**, this behavior is the result of both kernel and user-space code.

The **shebang line** is usually **ignored by the interpreter**, because the "`#`" character is a **comment marker** in many scripting languages; some language interpreters that do not use the hash mark to begin comments still may ignore the shebang line in recognition of its purpose.

# No-op Command

Colon: `:`

from [bash manual](https://www.gnu.org/software/bash/manual/html_node/Bourne-Shell-Builtins.html#Bourne-Shell-Builtins):
- "Do nothing beyond expanding arguments and performing redirections. The return status is zero."

# Double-ampersand vs semicolon

- semicolon: run the command no matter what the exit status of the previous command is
- double-ampersand: only run the command if the previous command is a success

```bash
$ false ; echo "OK"
OK
$ true ; echo "OK"
OK
$ false && echo "OK"
$ true && echo "OK"
OK
$ false || echo "OK"
OK
$ true || echo "OK"
$
```

# Variables

## `export` vs setting a variable

see [baeldung.com](https://www.baeldung.com/linux/bash-variables-export)

```bash
$ MYVAR=1729
$ export MYVAR=1729
```

The first definition creates a variable named MYVAR and assigns it the value 1729. **This is a shell variable**.

**The second definition with the export command** is another way of defining a variable. It creates a variable named MYVAR, assigns it the value 1729, and marks it for export to all child processes created from that shell. **This is an environment variable**.

The main difference between these two is that the `export` command makes the variable available to all the subsequent commands executed in that shell. This command does that by setting the export attribute for the shell variable `MYVAR`. **The export attribute marks MYVAR for automatic export to the environment of the child processes created by the subsequent commands**:
```bash
$ export MYVAR=1729
$ echo $MYVAR
1729
$ bash    # Open a new child shell
$ echo $MYVAR
1729
```

**Note:** We can access bash environment variables only one way; the parent shell exports its variables to the child shell's environment, but **the child shell can't export variables back to the parent shell**.

# If

## Command as condition

You can specify commands as a condition of `if`. If the command returns `0` in its exitcode that means that the condition is `true`; otherwise `false`. [source](https://stackoverflow.com/a/11287896/12282296)

```bash
$ if /bin/true; then echo that is true; fi
that is true
$ if /bin/false; then echo that is true; fi
$
```

## Not

```bash
if ! grep -q sysa /etc/passwd ; then
```

# Was bedeutet $# in einem bash script? (s. [askubuntu post](https://askubuntu.com/questions/939620/what-does-mean-in-bash))

Zum Beispiel:

```bash
if [[ $# -gt 0 ]]; then
```

`$#` steht für: number of arguments (wie `argc` in C)

# Vergleichsoperatoren

```bash
if [[ $# -gt 0 ]]; then
else
fi
```

`-gt` für "greater than" (ie der `>` operator) in der condition
`-eq` für "equal to" (ie der `=` operator)

# Standard Stream Redirection: stdin, stdout and stderr

- see "[What is /dev/null](https://linuxhint.com/what_is_dev_null/)"

## File Descriptor (part of POSIX API)

- see [file descriptor](https://en.wikipedia.org/wiki/File_descriptor)
- Each Unix process should have three standard POSIX file descriptors, corresponding to the three standard streams: `stdin`, `stdout` and `stderr`.
- Each file descriptor has a **non**-negative integer value: `stdin`: 0, `stdout`: 1, `stderr`: 2.

## Redirect stderr to a text file

- `asdfadsa 2> error.txt`

## Redirect stdout to a text file

- **Note:** If no file descriptor value is specified, bash will use `stdout` (i.e. file descriptor value `1`) by default.
- `echo "Hello World" > log.txt`

## Redirect stderr AND stdout to /dev/null

- `grep -r hello /sys/ &> /dev/null`

## Redirect ALL output to /dev/null

- In certain situations, the output may not be useful at all. Using redirection, we can dump all the output into the void:
  - `> /dev/null 2>&1`
    - `> /dev/null` dumps all the `stdout` to `/dev/null`
    - `2>&1` redirects `stderr` (file descriptor value: `2`) to `stdout` (file descriptor value: `1`)
- phth note: `> /dev/null 2>&1` idea came from `~/git/geohot/openpilot/update_requirements.sh`

# Test

There are two syntaxes for using the test command.

```bash
test EXPRESSION
[ EXPRESSION ]
```

Note that in the case of `[`, there is a space at both ends of the `EXPRESSION`.
- e.g. `test 1 -eq 2 && echo "true" || echo "false"` &rarr; `false`
- alternatively: `$ [ 1 -eq 2 ] && echo "true" || echo "false"`

## Check if File exists

```bash
FILE=/etc/resolv.conf
if [ -f "$FILE" ]; then
    echo "$FILE exists."
fi
```

# Job control

## List running jobs

- `jobs -l` (also shows the `jobID`s)

## bg

- let a process run in the background: `bg %[jobID]`
    - But output (e.g. `stdout`, `stderr`, etc) will still be printed in the terminal! Redirect output to `/dev/null` to avoid this.
- sends a `SIGCONT`
- like `kill -CONT [processid]`
- to **start** a process in the background: `mycommand &`
    - additionally, to avoid any output of this process in the current terminal window: `command > /dev/null 2>&1 &`

## fg

- brings a process to the foreground: `fg %[jobID]`

## nohup

- make a process continue running even if the shell session accidentally dies: `nohup firefox &`
- makes a process ignore the `SIGHUP` signal sent to the process
- example:
    - `nohup gedit &`, now if the terminal dies `gedit` will **not** be closed!
- use case:
    - wenn man z.B. per `ssh` auf einem fremden Rechner arbeitet und dort einen langwierigen Prozess starten möchte, die ssh-Verbindung aber während des Prozesses nicht permanent aktiv sein soll, weil man etwa den eigenen Rechner ausschalten möchte

# Exit codes

**Important**: Exit codes, in general, depend on the shell used! Different shells have different exit codes!

## Bash

Exit code of the **last command**:

```bash
$ echo $?
```

Exit Codes With Special Meanings: see [link](https://tldp.org/LDP/abs/html/exitcodes.html)

### Signal numbers

From Bash manual:
> When a command terminates on a fatal signal whose number is N, Bash uses the value 128+N as the exit status
These **signal numbers** are given below for each signal.

#### ctrl - c

- sends the signal `SIGINT` (signal number: 2, bash exit code: 130)
    - **note**: Exit code 130 can mean either `SIGINT` or `_exit(130)` in bash! `SIGINT` and `_exit(130)` are not the same (more details on [stackoverflow](https://unix.stackexchange.com/a/386856)).
- like `kill -SIGINT [processPID]` (**Note**: `kill` sends the `SIGTERM` (termination) signal by default unless you specify the signal to send.)
- can be intercepted by a program so it can clean its self up before exiting, or not exit at all

#### ctrl - z

- suspends a running process
- sends a `SIGTSTP`
- like `kill -TSTP [processid]`
- cannot be intercepted by the program
- **note**: the process will **halt**, so if you want it to continue running in the background use `bg`!

#### ctrl - s

- sends a `SIGSTOP`

#### ctrl - q 

- sends a `SIGCONT`

