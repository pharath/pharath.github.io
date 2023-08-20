---
title: "C++ Notes - Unit Testing"
read_time: false
excerpt: "For learning C++"
header:
    teaser: /assets/images/Cpp_logo.png
    overlay_image: /assets/images/Cpp_logo.png
    overlay_filter: 0.5 
toc: true
toc_label: "Contents"
toc_sticky: true
categories:
    - Notes
tags:
    - c++
    - unittest
    - notes

---

# Unit Testing

## Best Practices

- [Unit Testing Basics](https://www.jetbrains.com/help/clion/unit-testing-tutorial.html#basics)

## Test Runners

### ctest

`ctest` doesn't provide any testing macros/functions/classes on its own. It's just a **test runner**.

[Why useful?](https://www.reddit.com/r/cpp/comments/iwbdx7/is_ctest_worth_the_effort/):
- "**CTest** is useful for running multiple types of tests. You might have unit tests for C++ in **Catch2**, unit tests for python bindings in **pytest**, and you might have application-level testing that runs you program with specified arguments in a short **shell script**. CTest is the place to wire all these frameworks together so you can run all your testing in one shot. It works well for that."
- "And since I haven't seen it mentioned, the big C++ unit testing frameworks can integrate with CTest so you can see the pass/fail status of each test run by the framework"

## Test Frameworks

### Catch2

catch2 vs gtest:
- see [snorristurluson.github.io](https://snorristurluson.github.io/Catch2/)
- see [anteru.net](https://anteru.net/blog/2017/from-google-test-to-catch/)

### GoogleTest, gtest

- based on the **xUnit** architecture

#### Build

To see some **gtest samples**:
- build as [Standalone CMake Project](https://github.com/google/googletest/tree/main/googletest#standalone-cmake-project)

Build method used in AdvCpp course: 
- build by [Incorporating Into An Existing CMake Project](https://github.com/google/googletest/tree/main/googletest#incorporating-into-an-existing-cmake-project)
  - "Use CMake to download GoogleTest as part of the build's configure step. This approach doesn't have the limitations of the other methods." 

Just add to your `CMakeLists.txt` (CMake 3.14 or later):
```cmake
include(FetchContent)
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Now simply link against gtest or gtest_main as needed. Eg
add_executable(example example.cpp)
target_link_libraries(example gtest_main)
add_test(NAME example_test COMMAND example)
```
