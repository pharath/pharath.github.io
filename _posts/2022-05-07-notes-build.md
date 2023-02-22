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

`VAR ?= VALUE`
- set to a value only if `VAR` not already set

## CMake

**Note**: You must always link shared libraries (`.so` files) **and** include header files. [best stackoverflow explanation](https://stackoverflow.com/a/1186836/12282296)
- i.e. just linking an `.so` library does **not** automatically make all the header files of this library available! `include_directories` must be specified for the header files.

### Basic Workflow

```bash
mkdir build
cd build/
cmake ..
make   # or to get more information: `make VERBOSE=1`
sudo make install
sudo make uninstall   # only possible if you specified an uninstall target in CMakeLists.txt
```

### Understand Shared Libraries (.so files)

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

### Think in modules! (Best CMakeLists.txt Intro!)

**Start simple** and then complicate the project further!

See this [stackoverflow](https://stackoverflow.com/a/39600062/12282296) post!

Simple example project: [stackoverflow](https://stackoverflow.com/a/45843676/12282296)

### Quick Uninstall

from [stackoverflow](https://stackoverflow.com/questions/41471620/cmake-support-make-uninstall):

`install_manifest.txt` file is created when you run `make install`.

So just run:
```bash
xargs rm < install_manifest.txt
```

### Add an Uninstall Target, uninstall.cmake

Add an uninstall target to the CMAKE generated Makefile:

see [github gist](https://gist.github.com/royvandam/3033428)

### Show Variables

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

### Check if a library is installed

From [stackoverflow](https://serverfault.com/questions/54736/how-to-check-if-a-library-is-installed):

```bash
ldconfig -p | grep libjpeg
```

```bash
pkg-config --list-all | grep jpeg
dpkg -l | grep libjpeg
```

### pkg-config

see [Overview](https://linuxhint.com/pkg-config-linux-command/)

| command | description |
| :--- | :--- |
`pkg-config --list-all` | Show all the packages that have a `.pc` extension on your system. Contains all packages listed in the `PKG_CONFIG_PATH` variable path
`pkg-config opencv4 --libs` | display the **link flags** associated with a given package
`pkg-config opencv4 --cflags` | prints the **compile flags** and the associated pre-processor required to compile a package plus the flags for its dependencies
`pkg-config opencv4 --modversion` | check the version of a library
`pkg-config mylib --cflags-only-I` | only yields the include paths in Cflags

#### pkg-config in g++

```bash
g++ myprogram.cpp `pkg-config --libs opencv` -o myprogram
```

#### pkg-config file in CMake

From [stackoverflow](https://stackoverflow.com/a/45843676/12282296):

You may also export a pkg-config file. This file allows a third-party application to easily import your library:
- with Makefile, see `pkg-config`
- with Autotools, see `PKG_CHECK_MODULES`
- with cmake, see `pkg_check_modules`

Example: OpenCV: `/usr/local/lib/pkgconfig/opencv.pc` (see [stackoverflow](https://stackoverflow.com/a/10420608/12282296))

### cmake config file

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

### `include_directories` vs. `target_include_directories`

[stackoverflow](https://stackoverflow.com/a/31969632/12282296)

E.g. if you have a CMakeLists.txt that builds two ROS nodes, then it will have two targets. If **both** targets use the include directories use `include_directories`. If **only one of them** uses the include directories use `target_include_directories`.

### `set_target_properties`

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

### `find_library`

From [stackoverflow](https://stackoverflow.com/a/41909627/12282296):

This is a good idea, *even if* you know the path to your library. CMake will error out if the library vanished or got a new name. This helps to spot error early and to make it clear to the user (may yourself) what causes a problem.

To find a library `foo` and store the path in `FOO_LIB` use

```cmake
find_library(FOO_LIB foo)
```

also see [stackoverflow](https://stackoverflow.com/a/40776072/12282296)

### Troubleshooting

```bash
/tmp/ccsulwjG.o: In function cv::String::~String()':
  tmp.cpp:(.text._ZN2cv6StringD2Ev[_ZN2cv6StringD5Ev]+0x14): undefined reference tocv::String::deallocate()' /tmp/ccsulwjG.o: In function cv::String::operator=(cv::String const&)':
  tmp.cpp:(.text._ZN2cv6StringaSERKS0_[_ZN2cv6StringaSERKS0_]+0x28): undefined reference tocv::String::deallocate()' collect2: error: ld returned 1 exit status
```
- A linker error. 
- Set dependencies, like `-lopencv_core -lopencv_highgui -lopencv_imgproc`.

## `ament_cmake`

[doc](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html)

- `ament_cmake` is the build system for CMake based packages in ROS 2
- It is a set of scripts enhancing CMake and adding convenience functionality for package authors.

A basic CMake outline can be produced using `ros2 pkg create <package_name>` on the command line. The basic build information is then gathered in two files: the `package.xml` and the `CMakeLists.txt`. 

### `package.xml` in `ament_cmake`

The `package.xml` must contain all **dependencies** and a bit of **metadata** 
- to allow colcon to find the correct **build order** for your packages, 
- to install the required dependencies in **CI** as well as 
- provide the information for a **release** with [bloom](https://pypi.org/project/bloom/). 
    - note: **Bloom** is a release automation tool.

### `CMakeLists.txt` in `ament_cmake`

The `CMakeLists.txt` contains the commands to build package **executables** and **libraries**.
