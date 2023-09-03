---
title: "C++ Notes - The Standard Library"
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
    - std
    - library
    - notes

---

# Streams

A **stream** is a sequence of characters read from or written to an IO device. The term stream is intended to suggest that the characters are generated, or consumed, sequentially over time.

## iostream library

Defines the **types**: `istream` and `ostream`
Objects of type `istream`:
- `cin`
Objects of type `ostream`:
- `cout`
- `cerr`
- `clog`

## Buffer, Print Statements

**buffer**: A region of storage used to hold data. IO facilities often store input (or output) in a buffer and read or write the buffer independently from actions in the program. Output buffers can be explicitly flushed to force the buffer to be written. By default, reading cin flushes cout; cout is also flushed when the program ends normally.

```cpp
std::cout << "string" << std::endl
```

Writing `endl` has the effect of ending the current line and **flushing the buffer** associated with that device. Flushing the buffer ensures that all the output the program has generated so far is actually written to the output stream, rather than sitting in memory waiting to be written.
- **Best practice**: Programmers often add **print statements** during debugging. Such statements should **always flush the stream**. Otherwise, if the program crashes, output may be left in the buffer, leading to incorrect inferences about where the program crashed.

## std::flush

**Example**: see day 17: `flush.cpp`

From [Quora](https://www.quora.com/What-does-it-mean-that-endl-flushes-buffer-in-C):

When a program writes an output it does not (normally) writes directly on the device queue: it writes (= converts into text) into a buffer with a predefined capacity. The actual copy towards the output device will happen only when the buffer is full. When this happens the buffer is copied towards the device (eventually applying any required trans-coding) and then set back into its on "empty" state to be refilled with further output. When all that happens the buffer is said to be "flushed".

This works fine when you need to write a large amount of data, but there may be cases when this is not desirable:

- if you finished write a consistent amount of data, you may want to be sure nothing is still waiting to be written before going over, so you explicitly ask for a flush.
- If you are prompting a message to a user, you want to be sure it will be immediately visible (not waiting for something happening in the future ...)

Now, the expression `<<endl;` is equivalent to `<<'\n'<<flush` (it spits out a new-line character and calls the `flush` function, which in turn forces the buffer to be ... flushed)

## std::endl vs std::flush

From [stackoverflow](https://stackoverflow.com/a/59310447):

"`std::endl` is the equivalent of `std::cout.put('\n'); std::cout.flush();`"

## When does std::cout flush?

**Example:** see day 17 `when_flush.cpp`

From [stackoverflow](https://stackoverflow.com/a/22345401):

"There is no strict rule by the standard - only that `endl` WILL flush, but the implementation may flush at any time it "likes"."

# Time

- in `<chrono>`
  - Clocks, `time_point`, and `duration` for measuring how long some action takes, and as the basis for anything to do with time.
  - `day`, `month`, `year`, and `weekdays` for mapping `time_points` into our everyday lives.
  - `time_zone` and `zoned_time` to deal with differences in time reporting across the globe.

## Clocks

- **best practice:**
  - subtracting two `time_points` gives a `duration`
  - often a good idea to convert a `duration`, use `duration_cast`
  - use time-unit suffixes (in namespace `std::chrono_literals`)
    - greatly increases readability 
    - makes code more maintainable
- useful for:
  - quick measurements: 
    - Don't make statements about "efficiency" of code without first doing time measurements
    - Always measure repeatedly (to avoid rare events, cache effects, etc)

```cpp
#include <chrono>

using namespace std::chrono; // in sub-namespace std::chrono; see §3.3

auto t0 = system_clock::now();      // start time measurement
do_work();
auto t1 = system_clock::now();      // stop time measurement

// Subtracting two "time_points" gives a "duration"
cout << t1-t0 << "\n";                                            // default unit: 20223[1/00000000]s
// often a good idea to convert a "duration", use "duration_cast"
cout << duration_cast<milliseconds>(t1-t0).count() << "ms\n";     // specify unit: 2ms
cout << duration_cast<nanoseconds>(t1-t0).count() << "ns\n";      // specify unit: 2022300ns

// time-unit suffixes
this_thread::sleep_for(10ms+33us); // wait for 10 milliseconds and 33 microseconds
```

### `system_clock` vs `steady_clock`

From [stackoverflow](https://stackoverflow.com/a/31553641):

What is the difference between `steady_clock` vs `system_clock` in layman terms?
- If you're holding a `system_clock` in your hand, you would call it a **watch**, and it would tell you what time it is.
- If you're holding a `steady_clock` in your hand, you would call it a **stopwatch**, and it would tell you how fast someone ran a lap, but it would not tell you what time it is.
- If you had to, you could time someone running a lap with your **watch**. But if your watch (like mine) periodically talked to another machine (such as the atomic clock in Boulder CO) to correct itself to the current time, it might make minor mistakes in timing that lap. The **stopwatch** won't make that mistake, but it also can't tell you what the correct current time is.

## Best practices

From [stackoverflow](https://stackoverflow.com/a/31553641):

There is a simple rule I follow with the `<chrono>` library. The rule is actually not completely correct (thus it is a guideline). But it is close enough to correct to be a guideline that is nearly always followed:
- Don't use `count().`
- And a corollary: Don't use `time_since_epoch().`

The `<chrono>` library is designed around a **type-safe** system meant to protect you from units conversions mistakes. If you accidentally attempt an unsafe conversion, the error is caught at compile time (as opposed to it being a run time error).
- The member functions `count()` and `time_since_epoch()` are "escape hatches" out of this type-safe system ... to be used only in cases of emergency.

