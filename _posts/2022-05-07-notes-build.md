---
title: "Notes on Building Code"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Notes
  - Build
tags:
  - build
  - notes
toc: true
toc_label: "Contents"

---

# configure Script

- [purpose of configure scripts](https://en.wikipedia.org/wiki/Configure_script)
- [check "failed" warnings](https://github.com/edenhill/librdkafka/issues/370#issuecomment-142095337)

## Workflow

```bash
mkdir build/
cd build/
``` 
Then 
- run `../configure`,
- check in the output which dependencies are missing,
- if `libXYZ` is missing, install `sudo apt install libXYZ libXYZ-dev` (`libXYZ-dev` is necessary for the headers)
- run `../configure` again
- check in the output which dependencies are missing now,
- ... etc., until `../configure` finishes without errors
  - usually there will be some message like `Now type 'make' to build` at the end

## Options

Often `../configure` can be run with some flags.

Example:
```bash
../configure --disable-check
```
- if the `check` package is a dependency (Can be disabled using the `--disable-check` configure flag) 
  - `check` is used for **build-time tests** and does not affect functionality.

## Example Output

When all dependencies are met:
```bash
bra-ket@braket-pc:~/git/rofi/build$ ../configure --disable-check 
checking for gcc... gcc
checking whether the C compiler works... yes
checking for C compiler default output file name... a.out
checking for suffix of executables... 
checking whether we are cross compiling... no
checking for suffix of object files... o
checking whether we are using the GNU C compiler... yes
checking whether gcc accepts -g... yes
checking for gcc option to accept ISO C89... none needed
checking whether gcc understands -c and -o together... yes
checking for flex... flex
checking lex output file root... lex.yy
checking lex library... -lfl
checking whether yytext is a pointer... yes
checking for bison... bison -y
checking for grep that handles long lines and -e... /usr/bin/grep
checking for egrep... /usr/bin/grep -E
checking if bison is the parser generator... yes
checking for a sed that does not truncate output... /usr/bin/sed
checking for gawk... no
checking for mawk... mawk
checking for flex version... 2.6.4
checking for a BSD-compatible install... /usr/bin/install -c
checking whether build environment is sane... yes
checking for a thread-safe mkdir -p... /usr/bin/mkdir -p
checking whether make sets $(MAKE)... yes
checking whether make supports the include directive... yes (GNU style)
checking whether make supports nested variables... yes
checking dependency style of gcc... gcc3
checking whether make supports nested variables... (cached) yes
checking whether we are using the GNU C compiler... (cached) yes
checking whether gcc accepts -g... (cached) yes
checking for gcc option to accept ISO C89... (cached) none needed
checking whether gcc understands -c and -o together... (cached) yes
checking for gcc option to accept ISO C99... none needed
checking how to run the C preprocessor... gcc -E
checking for ANSI C header files... yes
checking for sys/types.h... yes
checking for sys/stat.h... yes
checking for stdlib.h... yes
checking for string.h... yes
checking for memory.h... yes
checking for strings.h... yes
checking for inttypes.h... yes
checking for stdint.h... yes
checking for unistd.h... yes
checking minix/config.h usability... no
checking minix/config.h presence... no
checking for minix/config.h... no
checking whether it is safe to define __EXTENSIONS__... yes
checking for ranlib... ranlib
checking for ar... ar
checking the archiver (ar) interface... ar
checking for getline... yes
checking for open... yes
checking for sysconf... yes
checking for strtok_r... yes
checking for flock... yes
checking for ftruncate... yes
checking for fcntl... yes
checking for setlocale... yes
checking for atexit... yes
checking for glob... yes
checking for readdir... yes
checking math.h usability... yes
checking math.h presence... yes
checking for math.h... yes
checking for library containing floor... -lm
checking for library containing ceil... none required
checking for library containing round... none required
checking sysexits.h usability... yes
checking sysexits.h presence... yes
checking for sysexits.h... yes
checking setjmp.h usability... yes
checking setjmp.h presence... yes
checking for setjmp.h... yes
checking for pkg-config... /usr/bin/pkg-config
checking pkg-config is at least version 0.9.0... yes
checking for _NKUTILS_INTERNAL_XKBCOMMON... yes
checking for _NKUTILS_INTERNAL_GIO... yes
checking for _NKUTILS_INTERNAL_GOBJECT... yes
checking for glib... yes
checking for GW_XCB_INTERNAL... yes
checking for stdlib.h... (cached) yes
checking for pango... yes
checking for cairo... yes
checking for libsn... yes
checking for gdkpixbuf... yes
checking for pkg-config... (cached) /usr/bin/pkg-config
checking pkg-config is at least version 0.16... yes
checking for GLIB... yes
checking for GLIB - version >= 2.0.0... yes (version 2.64.6)
checking that generated files are newer than configure... done
checking locale.h usability... yes
checking locale.h presence... yes
checking for locale.h... yes
checking for _NKUTILS_INTERNAL_GLIB... yes
checking for _NKUTILS_INTERNAL_TEST... yes
configure: creating ./config.status
config.status: creating Makefile
config.status: creating doc/rofi.doxy
config.status: creating pkgconfig/rofi.pc
config.status: creating config.h
config.status: executing depfiles commands

-------------------------------------
Desktop File drun            Enabled
Window switcher mode         Enabled
Asan address sanitize        Disabled
Code Coverage                Disabled
Check based tests            Disabled
-------------------------------------
Now type 'make' to build

```

# gcc, g++

## Flags

in [official doc](http://gcc.gnu.org/onlinedocs/gcc/)

also see [Basic Overview](https://caiorss.github.io/C-Cpp-Notes/compiler-flags-options.html)

### `-include`

in [official doc](https://gcc.gnu.org/onlinedocs/gcc/Preprocessor-Options.html)

### `-I`

in [official doc](https://gcc.gnu.org/onlinedocs/gcc/Directory-Options.html#Directory-Options)

Show default include directories:
```bash
# for C
gcc -xc -E -v -

# for C++
gcc -xc++ -E -v -
```

see [How to tell g++ compiler where to search for include files?](https://stackoverflow.com/questions/15478005/how-to-tell-g-compiler-where-to-search-for-include-files)

# make

Using less cores:
```bash
N_Cores=$(( $(nproc --all) / 2 ))
make -j $N_Cores
```

## Makefile

Source: [doc](https://www.gnu.org/software/make/manual/make.html#Recipes)

### Rule

- A **rule** appears in the makefile and says 
  - when and how to remake certain files, called the rule's **targets** (most often only one per rule). 
  - It lists the other files that are the **prerequisites** of the target, 
  - and the **recipe** to use to create or update the target.
- the `all` target (aka "default goal")
  - The **order of rules** is not significant, except for determining the **default goal**: the target for `make` to consider, if you do not otherwise specify one. The default goal is the **first target of the first rule** in the first makefile.
    - (...) We usually write the makefile so that the first rule is the one for **compiling the entire program** or all the programs described by the makefile (often with a target called `all`).

### Recipe

The **recipe** of a rule consists of **one or more shell command lines** to be executed, one at a time, in the order they appear. Typically, the result of executing these commands is that the target of the rule is brought up to date.

Users use many different shell programs, but recipes in makefiles are always interpreted by `/bin/sh` unless the makefile specifies otherwise.

### Variables

`VAR ?= VALUE`
- set to a value only if `VAR` not already set

### Phony Targets

Common phony targets: `all`, `install`, `clean`, `distclean`, `TAGS`, `info`, `check`

see [simple explanation](https://stackoverflow.com/a/2145605)
- in comments:
  - @eSKay: 'why is it called 'phony'?' -- because it's not a real target. That is, the target name isn't a file that is produced by the commands of that target. - Bernard
  - "phony" means fraudulent; fake; having a misleading appearance

# CMake

**Note**: You must always link shared libraries (`.so` files) **and** include header files. [best stackoverflow explanation](https://stackoverflow.com/a/1186836/12282296)
- i.e. just linking an `.so` library does **not** automatically make all the header files of this library available! `include_directories` must be specified for the header files.

## Basic Workflow

```bash
mkdir build
cd build/
cmake ..
make   # or to get more information: `make VERBOSE=1`
sudo make install
sudo make uninstall   # only possible if you specified an uninstall target in CMakeLists.txt
```

## Understand Shared Libraries (.so files)

[Best Tutorial](https://www.cprogramming.com/tutorial/shared-libraries-linux-gcc.html)

Three files needed:

File 1: foo.h:
```cpp	
#ifndef foo_h__
#define foo_h__
 
extern void foo(void);
 
#endif  // foo_h__
```

File 2: foo.c:
```cpp	
#include <stdio.h>
 
 
void foo(void)
{
    puts("Hello, I am a shared library");
}
```

File 3: main.c:
```cpp	
#include <stdio.h>
#include "foo.h"
 
int main(void)
{
    puts("This is a shared library test...");
    foo();
    return 0;
}
```

**Basic steps**:
```bash
$ gcc -c -Wall -Werror -fpic foo.c   # fpic: position independent code (see below)
$ gcc -shared -o libfoo.so foo.o
$ gcc -Wall -o test main.c -lfoo
/usr/bin/ld: cannot find -lfoo
collect2: ld returned 1 exit status
$ gcc -L/home/username/foo -Wall -o test main.c -lfoo
$ ./test
./test: error while loading shared libraries: libfoo.so: cannot open shared object file: No such file or directory
$ echo $LD_LIBRARY_PATH
$ LD_LIBRARY_PATH=/home/username/foo:$LD_LIBRARY_PATH
$ ./test
./test: error while loading shared libraries: libfoo.so: cannot open shared object file: No such file or directory
$ export LD_LIBRARY_PATH=/home/username/foo:$LD_LIBRARY_PATH
$ ./test
This is a shared library test...
Hello, I am a shared library
```

**Position independent code**: PIC is code that works no matter where in memory it is placed. Because several different programs can all use one instance of your shared library, the library cannot store things at fixed addresses, since the location of that library in memory will vary from program to program.

If we want to **install** our library so everybody on the system can use it:
```bash
$ cp /home/username/foo/libfoo.so /usr/lib
$ chmod 0755 /usr/lib/libfoo.so
$ ldconfig
$ ldconfig -p | grep foo
libfoo.so (libc6) => /usr/lib/libfoo.so
$ unset LD_LIBRARY_PATH
$ gcc -Wall -o test main.c -lfoo
$ ldd test | grep foo
libfoo.so => /usr/lib/libfoo.so (0x00a42000)
$ ./test
This is a shared library test...
Hello, I am a shared library
```

## Think in modules! (Best CMakeLists.txt Intro!)

**Start simple** and then complicate the project further!

See this [stackoverflow](https://stackoverflow.com/a/39600062/12282296) post!

Simple example project: [stackoverflow](https://stackoverflow.com/a/45843676/12282296)

## Quick Uninstall

from [stackoverflow](https://stackoverflow.com/questions/41471620/cmake-support-make-uninstall):

`install_manifest.txt` file is created when you run `make install`.

So just run:
```bash
xargs rm < install_manifest.txt
```

## Add an Uninstall Target, uninstall.cmake

Add an uninstall target to the CMAKE generated Makefile:

see [github gist](https://gist.github.com/royvandam/3033428)

## Show Variables

see [stackoverflow](https://stackoverflow.com/a/42658058/12282296)

To show the value of CMakeLists.txt variables, add a line like so:
```bash
message(STATUS "Value of VAR = ${VAR}")
```
The message should be visible in the terminal after running `cmake ..` (look carefully!).

**Or**: Add `set(CMAKE_VERBOSE_MAKEFILE ON)` at the top of your CMakeLists.txt.

In `events.log` (**colcon**):

If you are using `colcon`, you can also find the message in the `events.log`. Run
```bash
cat log/latest_build/events.log
```
and search through all the `StdoutLine`s to see the value of `VAR`.

## Check if a library is installed

From [stackoverflow](https://serverfault.com/questions/54736/how-to-check-if-a-library-is-installed):

```bash
ldconfig -p | grep libjpeg
```

```bash
pkg-config --list-all | grep jpeg
dpkg -l | grep libjpeg
```

## pkg-config

see [Overview](https://linuxhint.com/pkg-config-linux-command/)

| command | description |
| :--- | :--- |
`pkg-config --list-all` | Show all the packages that have a `.pc` extension on your system. Contains all packages listed in the `PKG_CONFIG_PATH` variable path
`pkg-config opencv4 --libs` | display the **link flags** associated with a given package
`pkg-config opencv4 --cflags` | prints the **compile flags** and the associated pre-processor required to compile a package plus the flags for its dependencies
`pkg-config opencv4 --modversion` | check the version of a library
`pkg-config mylib --cflags-only-I` | only yields the include paths in Cflags

### pkg-config in g++

```bash
g++ myprogram.cpp `pkg-config --libs opencv` -o myprogram
```

### pkg-config file in CMake

From [stackoverflow](https://stackoverflow.com/a/45843676/12282296):

You may also export a pkg-config file. This file allows a third-party application to easily import your library:
- with Makefile, see `pkg-config`
- with Autotools, see `PKG_CHECK_MODULES`
- with cmake, see `pkg_check_modules`

Example: OpenCV: `/usr/local/lib/pkgconfig/opencv.pc` (see [stackoverflow](https://stackoverflow.com/a/10420608/12282296))

## cmake config file

[doc](https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#package-configuration-file)

Example: [OpenCV](https://github.com/opencv/opencv/blob/master/cmake/templates/OpenCVConfig.cmake.in)
- defines variables such as `OpenCV_LIBS` and `OpenCV_INCLUDE_DIRS`
- see [stackoverflow](https://stackoverflow.com/a/42658058/12282296)
- makes OpenCV available for external projects by using:
```cmake
#    find_package(OpenCV REQUIRED)
#    include_directories(${OpenCV_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11
#    target_link_libraries(MY_TARGET_NAME ${OpenCV_LIBS})
```

## `include_directories` vs. `target_include_directories`

[stackoverflow](https://stackoverflow.com/a/31969632/12282296)

E.g. if you have a CMakeLists.txt that builds two ROS nodes, then it will have two targets. If **both** targets use the include directories use `include_directories`. If **only one of them** uses the include directories use `target_include_directories`.

## `set_target_properties`

Specifies which public header files should be installed when running `sudo make install`. The install location is specified by 
```bash
install(TARGETS galaxis_lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
```

Do not forget the quotation signs around `"${HEADERS}"`! Otherwise **only the first header** file in the `${HEADERS}` list will be installed.
```bash
set_target_properties(galaxis_lib PROPERTIES
    PUBLIC_HEADER "${HEADERS}")
```

## `find_library`

From [stackoverflow](https://stackoverflow.com/a/41909627/12282296):

This is a good idea, *even if* you know the path to your library. CMake will error out if the library vanished or got a new name. This helps to spot error early and to make it clear to the user (may yourself) what causes a problem.

To find a library `foo` and store the path in `FOO_LIB` use

```cmake
find_library(FOO_LIB foo)
```

also see [stackoverflow](https://stackoverflow.com/a/40776072/12282296)

## Troubleshooting

```bash
/tmp/ccsulwjG.o: In function cv::String::~String()':
  tmp.cpp:(.text._ZN2cv6StringD2Ev[_ZN2cv6StringD5Ev]+0x14): undefined reference tocv::String::deallocate()' /tmp/ccsulwjG.o: In function cv::String::operator=(cv::String const&)':
  tmp.cpp:(.text._ZN2cv6StringaSERKS0_[_ZN2cv6StringaSERKS0_]+0x28): undefined reference tocv::String::deallocate()' collect2: error: ld returned 1 exit status
```
- A linker error. 
- Set dependencies, like `-lopencv_core -lopencv_highgui -lopencv_imgproc`.

# `ament_cmake` (ROS 2)

[doc](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html)

- `ament_cmake` is the build system for CMake based packages in ROS 2
- It is a set of scripts enhancing CMake and adding convenience functionality for package authors.

A basic CMake outline can be produced using `ros2 pkg create <package_name>` on the command line. The basic build information is then gathered in two files: the `package.xml` and the `CMakeLists.txt`. 

## `package.xml` in `ament_cmake`

The `package.xml` must contain all **dependencies** and a bit of **metadata** 
- to allow colcon to find the correct **build order** for your packages, 
- to install the required dependencies in **CI** as well as 
- provide the information for a **release** with [bloom](https://pypi.org/project/bloom/). 
    - note: **Bloom** is a release automation tool.

## `CMakeLists.txt` in `ament_cmake`

The `CMakeLists.txt` contains the commands to build package **executables** and **libraries**.

# Autotools

Wikipedia: 
- **GNU Autotools** (aka **GNU Build System**) 
- a suite of programming tools designed to assist in making source code packages **portable to many Unix-like systems**.
- Autotools consists of the GNU utility programs 
  - Autoconf, 
  - Automake, and 
  - Libtool. 

## Overview

Wikipedia:

Autoconf
- **Autoconf** generates a configure script based on the contents of a `configure.ac` file, which characterizes a particular body of source code. 
- The `configure` script, when run, scans the build environment and generates a subordinate `config.status` script 
- which, in turn, converts other input files and most commonly `Makefile.in` into output files (`Makefile`), which are appropriate for that build environment. 
- Finally, the `make` program uses `Makefile` to generate executable programs from source code.

Automake
- **Automake** helps to create portable `Makefiles`, which are in turn processed with the `make` utility. 
- It takes its input as `Makefile.am`, and turns it into `Makefile.in`, which is used by the `configure` script to generate the file `Makefile` output. 
- It also performs **automatic dependency tracking**; 
  - every time a source file is compiled, the list of dependencies (e.g., C header files) is recorded. 
  - Later, any time make is run and a dependency appears to have changed, the dependent files will be rebuilt.

Libtool
- **Libtool** helps manage the creation of static and dynamic libraries **on various Unix-like operating systems**. 
- Libtool accomplishes this by abstracting the library-creation process, hiding differences between various systems (e.g. Linux systems vs. Solaris).

## Usage

Before running `../configure` run
```bash
autoreconf -i
```

# Bear

Bear is a tool that generates a [compilation database](https://clang.llvm.org/docs/JSONCompilationDatabase.html) (`compile_commands.json`) for clang tooling. 

```bash
sudo apt install bear
bear make   # generates the compile_commands.json file
```

## Other Tools to Generate a Compilation Database

Bear is just **one** tool that can generate a compilation database. [How to generate a compilation database in CMake, Clang, Ninja](https://www.jetbrains.com/help/clion/compilation-database.html#compdb_generate).

# clang

- a C++ compiler

## clangd

`clangd` is a **language server**.
- `clangd` is based on the **Clang C++ compiler**

### .clangd file

- a project configuration file
- yaml format

Use cases:
- e.g. to suppress the warning `In included file: main file cannot be included recursively when building a preamble` add 
```yml
Diagnostics:
  Suppress: 'pp_including_mainfile_in_preamble'
```
to `.clangd`
