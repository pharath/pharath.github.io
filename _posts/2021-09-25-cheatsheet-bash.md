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

# References

- [tldp.org](https://tldp.org/LDP/Bash-Beginners-Guide/html/Bash-Beginners-Guide.html)
- [gnu.org](https://www.gnu.org/savannah-checkouts/gnu/bash/manual/bash.html)
  - the same on [devdocs.io: bash](https://devdocs.io/bash/) ("DevDocs combines multiple API documentations in a fast, organized, and searchable interface.", [devdocs.io](https://devdocs.io/))
- [mywiki.wooledge.org](https://mywiki.wooledge.org/BashSheet#Syntax)

# Profiling

## Check the Execution Time 

`time COMMAND`
- e.g. `time source ./.bashrc`

## Give verbose output with timestamps and linenumbers

From [https://stackoverflow.com/a/4338046](https://stackoverflow.com/a/4338046):

**With modifying** the script:

Add
```bash
#!/bin/bash -x
# Note the -x flag above, it is required for this to work
PS4='+ $(date "+%s.%N ($LINENO) ")'
```
at the beginning of your script.

**Without modifying** the script:
```bash
PS4='+ $(date "+%s.%N ($LINENO) ")' bash -x .bashrc
```

or:
```bash
PS4='+ $EPOCHREALTIME ($LINENO) ' bash -x .bashrc
```

Note: `PS4` is a special variable used by `set -x` to prefix tracing output. ([more](https://www.thegeekstuff.com/2008/09/bash-shell-take-control-of-ps1-ps2-ps3-ps4-and-prompt_command/))

## Step through Bash scripts

From [stackexchange](https://unix.stackexchange.com/a/39672):

It's not exactly profiling, but you can trace your script as it runs. 
- Put `set -xv` before the section you want to trace and `set +xv` after the section. 
- `set -x` enables xtrace, which will show every line that executes. 
- `set -v` enables verbose mode, which will also show lines that may have an effect, but are not executed, such as variable assignment.

# How to call one shell script from another shell script?

[stackoverflow](https://stackoverflow.com/a/8352939/12282296)

Three ways:

- **Make the other script executable** with `chmod a+x /path/to/file`, add the `#!/bin/bash` line (called shebang) at the top, and the path where the file is to the `$PATH` environment variable. Then you can call it as a normal command;
- **Or call it with the source command** (which is an alias for `.`), like this: `source /path/to/script`
- **Or use the bash command** to execute it, like: `/bin/bash /path/to/script`

The first and third approaches execute the script as another process, so variables and functions in the other script will not be accessible.

**The second approach executes the script in the first script's process, and pulls in variables and functions from the other script (so they are usable from the calling script)**. It will of course run all the commands in the other script, not only set variables.

In the second method, if you are using exit in second script, it will `exit` the first script as well. Which will not happen in first and third methods.

# Source instead of Execute

[unix.stackexchange.com](https://unix.stackexchange.com/a/424495)

Assuming that you are running bash, put the following code near the start of the script that you want to be sourced but not executed:

```bash
if [ "${BASH_SOURCE[0]}" -ef "$0" ]
then
  echo "Hey, you should source this script, not execute it!"
  exit 1
fi
```

# Exit on Error

## set -e

**Bad practice**: Only use `set -e` for debugging purposes.

[stackoverflow](https://stackoverflow.com/a/19622569/12282296):

It's recommended to use:

```bash
trap 'do_something' ERR
```

to run `do_something` function when errors occur. (see below in section "bash traps")

[stackoverflow](https://stackoverflow.com/a/2871034):

```bash
#!/bin/bash
set -e
# Any subsequent(*) commands which fail will cause the shell script to exit immediately
```

You can also disable this behavior with `set +e`.

**Warning:** When you use `set -e` in a script that you want to be sourced, you **must** put a `set +e` at the end of this script as well as at each point in the script at which the script might exit. Otherwise, `set -e` will continue to be set in the shell in which the script was sourced. This will cause the shell to exit whenever one of the following commands entered in this shell fails (and also when tab autocompletion cannot find a matching completion!).

> **history**: `set -e` was an attempt to add "automatic error detection" to the shell. Its goal was to cause the shell to abort any time an error occurred, so you don't have to put || exit 1 after each important command. This does not work well in practice.
> [bash FAQ](http://mywiki.wooledge.org/BashFAQ/105)

- related:
  - [bash strict mode](http://redsymbol.net/articles/unofficial-bash-strict-mode/)

## bash traps

**Warning**: Do not forget to remove traps after setting them.

| command | description |
| :--- | :--- |
`trap 'do_something' ERR` | set a trap that runs `do_something` on signal `ERR`; "the SIG prefix is optional" (from `trap --help`), ie. you can write eg. `SIGINT` or `INT`
`trap - [signal]` | remove a trap, [stackoverflow](https://stackoverflow.com/questions/31201572/how-to-untrap-after-a-trap-command)
`trap -l` | print a list of signal names and their corresponding numbers
`ERR` | similar to `set -e`, [stackoverflow](https://stackoverflow.com/a/26261518/12282296)
`EXIT` | useful for cleanup operations, see below "examples" &rarr; "exit traps", [exit traps](http://redsymbol.net/articles/bash-exit-traps/)

- related:
  - [bash trap example](https://opensource.com/article/20/6/bash-trap)
  - [SIGINT traps](https://www.baeldung.com/linux/bash-signal-handling)

### Examples

**Example 1**: report the line number where an error occurred:

```bash
FILENAME=$0
# Colors
RED=$(tput setaf 1)
NC=$(tput sgr0) # No Color

err_report() {
   printf "%s%s: Error on line %s.%s\n" "${RED}" "${FILENAME}" "$1" "${NC}"
}
trap 'err_report $LINENO' ERR

# do some work

trap - ERR
```

**Example 2**: "exit traps" to perform cleanup operations:

```bash
#!/bin/bash
function finish {
  # Your cleanup code here
}
trap finish EXIT
```

**Example 3**: only exit at the end of a loop: [stackoverflow](https://stackoverflow.com/questions/26797953/bash-trap-exit-only-at-the-end-of-loop)

- **important (related to Example 3)**: behaviour when pressing <kbd>ctrl</kbd> + <kbd>c</kbd>
  - observed in `scripts/dl/cyberdrop-status.sh`
  - A SIGINT (<kbd>ctrl</kbd> + <kbd>c</kbd>) is sent to the whole **process group** (see section "Process Group ID, Process Group Leader"), ie. to all processes that have the same **process group ID** (PGID).
    - So, eg. when there is a `sleep` inside a while loop in your bash script, the `sleep` process will receive a SIGINT and the script process will also receive a SIGINT.
    - If this bash script has a SIGINT `trap` that does **not** exit the script on SIGINT, then the script will continue running, whereas the `sleep` process will terminate immediately (while [returning the remaining time](https://man7.org/linux/man-pages/man3/sleep.3.html)).
    - This means that the next statement in the script after the `sleep` command is executed. If `sleep` is the last command in the loop, then the next iteration of the loop will start.

### Process Group ID, Process Group Leader

[unix.stackexchange](https://unix.stackexchange.com/questions/737385/how-can-i-identify-the-leader-of-a-process-group-in-linux):

I understand that when a process is started from a shell in linux or unix, a new process group is created with that process as the process leader, giving its PID equal to a new PGID which is used for any processes spawn from that process.

As I understand it, in the case where the process leader is terminated, a new process becomes leader. This new leader would then have a PID inequal to the PGID of the process group.

[How to get the PID and corresponding PGID of each process](https://unix.stackexchange.com/a/149756):

Simplest way to determine the process group ID is to use `ps`:

```bash
ps ax -O tpgid
```

The second column will be the process group ID.

**phth**: here you can run eg.

```bash
# get the PID and its corresponding PGID
ps ax -O tpgid | grep some_PID
ps ax -O tpgid | grep some_PGID
ps ax -O tpgid | grep name_of_some_process
```

to get your desired information.

The output will look eg. like this:

```bash
$ ps ax -O tpgid | grep some_PID
3521072 3669289 S pts/18   00:00:00 bash
3669289 3669289 S pts/18   00:00:00 /bin/bash ./dl/someprocess.sh
3669347 3669289 S pts/18   00:00:00 sleep 10
3669396 3669395 S pts/23   00:00:00 grep --color=auto 3669289
```

[How to get the PGID of the current process in bash](https://stackoverflow.com/questions/71444028/print-pgid-in-a-bash-script):

```bash
ps -o pgid= $$
```

- related:
  - [PID, PPID, SID, PGID, UID, EUID](https://stackoverflow.com/questions/41498383/what-do-the-identifiers-pid-ppid-sid-pgid-uid-euid-mean)
  - [Process Group](https://en.wikipedia.org/wiki/Process_group)
  - [pgrep -g PGID](https://unix.stackexchange.com/questions/737385/how-can-i-identify-the-leader-of-a-process-group-in-linux)
  - [Process Group Leader](https://en.wikipedia.org/wiki/Process_group#Details)
  - [what happens on SIGINT in bash](https://superuser.com/questions/1829830/behavior-of-sigint-with-bash)

# Exit after some Time

[stackoverflow](https://stackoverflow.com/a/71409539)

```bash
SECONDS=0
while (( SECONDS < 60)); do
  echo "keep running"
  sleep 5
done
```

# Shebang

beste Erklärung: [askubuntu discussion](https://stackoverflow.com/questions/7670303/purpose-of-usr-bin-python3-shebang/7670338#7670338)

oder aus [Wikipedia](https://en.wikipedia.org/wiki/Shebang_(Unix)):

In computing, a **shebang** is the character sequence consisting of the characters number sign and exclamation mark (`#!`) at the beginning of a script.

When a text file with a shebang is used as if it is an executable in a Unix-like operating system, the program loader mechanism parses the rest of the file's initial line as an **interpreter directive**. The loader executes the specified interpreter program, passing to it as an argument the path that was initially used when attempting to run the script, so that the program may use the file as input data. **For example**, if a script is named with the path `path/to/script`, and it starts with the following line, `#!/bin/sh`, then the program loader is instructed to run the program `/bin/sh`, passing `path/to/script` as the first argument. **In Linux**, this behavior is the result of both kernel and user-space code.

The **shebang line** is usually **ignored by the interpreter**, because the "`#`" character is a **comment marker** in many scripting languages; some language interpreters that do not use the hash mark to begin comments still may ignore the shebang line in recognition of its purpose.

# Core

## Command Substitution

Syntax: `$(command substitution)`

Nested Variables:
```bash
DIRNAME="$(dirname "$FILE")"
```

## No-op Command

Colon: `:`

from [bash manual](https://www.gnu.org/software/bash/manual/html_node/Bourne-Shell-Builtins.html#Bourne-Shell-Builtins):
- "Do nothing beyond expanding arguments and performing redirections. The return status is zero."

## Double-Ampersand vs Semicolon

- **semicolon**: run the command no matter what the exit status of the previous command is
- **double-ampersand**: only run the command if the previous command is a success

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

## Variables

### Best Practices

- `echo "$somevar"` instead of `echo $somevar`
  - "Quoting variables prevents word splitting and glob expansion, and prevents the script from breaking when input contains spaces, line feeds, glob characters and such.", [shellcheck.net](https://www.shellcheck.net/wiki/SC2086)

### `export` vs setting a variable

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

### Variable Expansion

["Parameter expansion" (A.K.A "Variable expansion")](https://unix.stackexchange.com/questions/403867/what-is-parameter-expansion-a-k-a-variable-expansion-in-shell-scripting-in)

### Check if a Variable is Set

[stackoverflow](https://stackoverflow.com/a/3601734)

```bash
if [[ -n "${var+x}" ]]; then
  # do something
fi
```

## If

```bash
if [ conditions ]; then
   # Things
elif [ other_conditions ]; then
   # Other things
else
   # In case none of the above occurs
fi
```

Note: The spaces are needed around the brackets. Otherwise, it won't work. This is because `[` itself is a command. 

### Double Square-Brackets vs Single Square-Brackets

- "If you're writing a `#!/bin/bash` script then I recommend using `[[` instead. The double square-brackets `[[...]]` form has more features, a more natural syntax, and fewer gotchas that will trip you up.", [stackoverflow](https://stackoverflow.com/a/20449556)
- "`[[` is bash's improvement to the `[` command. It has several enhancements that make it a better choice if you write scripts that target bash.", [stackoverflow](https://stackoverflow.com/questions/3427872/whats-the-difference-between-and-in-bash)

### Double Parentheses, Arithmetic Expansion

[stackoverflow](https://stackoverflow.com/a/2188369)

- "Double parentheses are used for **arithmetic operations**"
- "and they enable you to **omit the dollar signs** on integer and array **variables** and include spaces around operators for readability."

[mywiki.wooledge.org](https://mywiki.wooledge.org/ArithmeticExpression)

- POSIX sh (and all shells based on it, including Bash and ksh) uses the `$(( ))` syntax to do arithmetic, using the same syntax as C.
- Bash calls this an "Arithmetic Expansion", and it obeys the same basic rules as all other `$...` substitutions.

### Command as condition

You can specify commands as a condition of `if`. If the command returns `0` in its exitcode that means that the condition is `true`; otherwise `false`. [source](https://stackoverflow.com/a/11287896/12282296)

```bash
$ if /bin/true; then echo that is true; fi
that is true
$ if /bin/false; then echo that is true; fi
$
```

### Pipe

#### Pipe on a Condition

[How to pipe on a condition in Bash](https://stackoverflow.com/a/56870935)

#### Examples

Using `find` in combination with `xargs`: the flags `-print0` and `-0` are necessary because filenames may contain spaces:

```bash
# duplicates.txt: filenames with file sizes
for pattern in $(du -sh searchPatternWith\ \(Spaces\)/* | cut -f2- -d'/' | cut -f1 -d'_'); do count=$(find . -iname *$pattern* -print0 | xargs -0 ls | grep .mp4$ | wc -l); if [[ $count -ge 2 ]]; then find . -iname *$pattern* -print0 | xargs -0 du | grep -v searchPatternWith | tee -a duplicates.txt; fi; done

# duplicates2.txt: only filenames
for pattern in $(du -sh searchPatternWith\ \(Spaces\)/* | cut -f2- -d'/' | cut -f1 -d'_'); do count=$(find . -iname *$pattern* -print0 | xargs -0 ls | grep .mp4$ | wc -l); if [[ $count -ge 2 ]]; then find . -iname *$pattern* -print0 | xargs -0 du | grep -v searchPatternWith | grep -v jpg$ | tee -a duplicates2.txt; fi; done
```

## `$#`

from: [askubuntu post](https://askubuntu.com/questions/939620/what-does-mean-in-bash))

Zum Beispiel:

```bash
if [[ $# -gt 0 ]]; then
# ...
```

`$#` means: number of arguments (like `argc` in C)

## `$1`, `$2`, etc.

- called the **Positional Parameters**

[bash.cyberciti.biz](https://bash.cyberciti.biz/guide/$1)

- `$1` is the first command-line argument passed to the shell script.
- It is a way to access the value of the first parameter supplied when executing the script or function.
- For example, `$0`, `$1`, `$3`, `$4` and so on.
- If you run `./script.sh filename1 dir1`, then:
  - `$0` is the name of the script itself (`script.sh`)
  - `$1` is the first argument (`filename1`)
  - `$2` is the second argument (`dir1`)
  - `$9` is the ninth argument
  - `${10}` is the tenth argument and must be enclosed in brackets after `$9`.
  - `${11}` is the eleventh argument.

## Passing Arguments to Scripts

[redhat.com](https://www.redhat.com/sysadmin/arguments-options-bash-scripts)

```bash
#!/bin/bash
############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Add description of the script functions here."
   echo
   echo "Syntax: scriptTemplate [-g|h|v|V]"
   echo "options:"
   echo "g     Print the GPL license notification."
   echo "h     Print this Help."
   echo "v     Verbose mode."
   echo "V     Print software version and exit."
   echo
}

############################################################
############################################################
# Main program                                             #
############################################################
############################################################

# Set variables
Name="world"

############################################################
# Process the input options. Add options as needed.        #
############################################################
# Get the options
while getopts ":hn:" option; do
   case $option in
      h) # display Help
         Help
         exit;;
      n) # Enter a name
         Name=$OPTARG;;
     \?) # Invalid option
         echo "Error: Invalid option"
         exit;;
   esac
done


echo "hello $Name!"
```

### `optstring`

[How to specify an optstring](https://stackoverflow.com/questions/13251732/how-to-specify-an-optstring-in-the-getopt-function)

It is just a string, and each character of this string represents an option. If this option requires an argument, you have to follow the option character by `:`.

For example, `"cdf:g"` accepts the options `c`, `d`, `f`, and `g`; `f` requires an additional argument.

[What the first colon means](https://unix.stackexchange.com/questions/426483/what-is-the-purpose-of-the-very-first-character-of-the-option-string-of-getopts)

If the very first character of optstring is a colon, getopts will not produce any diagnostic messages for missing option arguments or invalid options.

### `$OPTARG`

[quora.com](https://www.quora.com/What-is-Optarg-in-a-shell-script)

`$OPTARG` stores the value of the argument of the option

**Example**: when you run `./script.sh -p "someArg"` then the `$OPTARG` variable in the `p)` case in the `while getopts` construct contains `"someArg"`

## Comparison Operators

```bash
if [[ $# -gt 0 ]]; then
  # ...
else
  # ...
fi
```

- `=` vs `==` vs `-eq`, [stackoverflow](https://stackoverflow.com/a/20449556)
  - `=` and `==` are for string comparisons
  - `-eq` is for numeric comparisons
    - `-eq` is in the same family as `-lt`, `-le`, `-gt`, `-ge`, and `-ne`
  - `==` is specific to **bash** (not present in **sh (Bourne shell)**, ...).
  - Using **POSIX** `=` is preferred for compatibility.
  - In **bash** the two are equivalent, and in sh `=` is the only one that will work.

## Logical Operators

### And

`[command] && [command]`

An **AND conditional** causes the second command to be executed only if the first command ends and exits successfully. 

### Or

`[command] || [command]`

An **OR conditional** causes the second command to be executed only if the first command ends and exits with a failure exit code (any non-zero exit code). 

### Not

```bash
if ! grep -q sysa /etc/passwd ; then
```

## `test`

There are two syntaxes for using the `test` command.

```bash
test EXPRESSION
[ EXPRESSION ]
```

Note that in the case of `[`, there is a space at both ends of the `EXPRESSION`.
- e.g. `test 1 -eq 2 && echo "true" || echo "false"` &rarr; `false`
- alternatively: `$ [ 1 -eq 2 ] && echo "true" || echo "false"`

### Check if File exists

```bash
FILE=/etc/resolv.conf
if [ -f "$FILE" ]; then
    echo "$FILE exists."
fi
```

## `printf`

Don't use variables in the printf format string. Use `printf "..%s.." "$foo"`.

```bash
printf "%sphth: Need to rename some windows to make some shortcuts work:%s\n" "${RED}" "${NC}"
# instead of
printf "${RED}phth: Need to rename some windows to make some shortcuts work:${NC}\n"
```

## Colors

- [stackoverflow](https://stackoverflow.com/a/4332530/12282296)

# Standard Stream Redirection: `stdin`, `stdout` and `stderr`

- see "[What is `/dev/null`](https://linuxhint.com/what_is_dev_null/)"

## File Descriptor (part of POSIX API)

- see [file descriptor](https://en.wikipedia.org/wiki/File_descriptor)
- Each Unix process should have three standard POSIX file descriptors, corresponding to the three standard streams: `stdin`, `stdout` and `stderr`.
- Each file descriptor has a **non**-negative integer value: `stdin`: 0, `stdout`: 1, `stderr`: 2.

## Redirect `stderr` to a text file

- `asdfadsa 2> error.txt`

## Redirect `stdout` to a text file

- **Note:** If no file descriptor value is specified, bash will use `stdout` (i.e. file descriptor value `1`) by default.
- `echo "Hello World" > log.txt`

## Redirect to `/dev/null`

- Send the output to `/dev/null`: `command 1> /dev/null`.
- Send the error to `/dev/null`: `command 2> /dev/null`.
- Send both output and error to `/dev/null`: `command 2>&1 /dev/null`.

## Redirect `stderr` AND stdout to `/dev/null`

- `grep -r hello /sys/ &> /dev/null`

## Redirect ALL output to `/dev/null`

- `> /dev/null 2>&1`
  - [unix.stackexchange](https://unix.stackexchange.com/a/119650)
  - In certain situations, the output may not be useful at all. Using redirection, we can dump all the output into the void:
    - `> /dev/null 2>&1`
      - `> /dev/null` dumps all the `stdout` to `/dev/null`
      - `2>&1` redirects `stderr` (file descriptor value: `2`) to `stdout` (file descriptor value: `1`)
  - phth note: `> /dev/null 2>&1` idea came from `~/git/geohot/openpilot/update_requirements.sh`

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

- Exit Codes With Special Meanings: see [link](https://tldp.org/LDP/abs/html/exitcodes.html)
- Only use 64-113 as user-defined exit codes: see [link](https://tldp.org/LDP/abs/html/exitcodes.html)

### Check Exit Code and Run Code

```bash
# check, if my custom shortcuts for focussing certain windows work
RED=$(tput setaf 1)
NC=$(tput sgr0) # No Color
wmctrl -l | grep -q main-tmux
if ! "$?" > /dev/null 2>&1; then
    printf "%sphth: Need to rename some windows to make some shortcuts work:%s\n" "${RED}" "${NC}"
    cat scripts/rename-windows.sh
fi
```

### Signal numbers

`man bash`:
```bash
When a command terminates on a fatal signal whose number is N, Bash uses the value 128+N as the exit status
```

These **signal numbers** are given below for each signal.

See also:
- Zombie (&rarr; Operating Systems Notes)

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

# Formatting

## Clear the Terminal

```bash
if [[ "$error_count" -eq 0 ]]; then
    # clear the terminal
    printf "\033c"
fi
```
