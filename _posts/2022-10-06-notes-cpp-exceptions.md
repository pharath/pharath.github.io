---
title: "C++ Notes - Exception Handling"
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
    - exceptions
    - notes

---

# Exception Handling

- best read: [isocpp](https://isocpp.org/wiki/faq/exceptions)

## Definitions

- an exception is **raised** by **throwing** an expression
- **exception safe code:** code that properly "cleans up" during exception handling
  - must ensure that
    - objects are valid, 
    - resources don't leak, 
    - the program is restored to an appropriate state

## Stack Unwinding

- definition: [cppreference](https://en.cppreference.com/w/cpp/language/throw#Stack_unwinding)
- the process
  - that starts when an exception is thrown
  - that continues up the chain of nested function calls until 
    - a `catch` clause for the exception is found, or 
    - the `main` function itself is exited without having found a matching `catch`
- An exception that is not caught **terminates** the program.
- **Object Destruction**: 
  - from [cppreference](https://en.cppreference.com/w/cpp/language/throw#Stack_unwinding):
    - "Once the exception object is constructed, the control flow works **backwards** (up the call stack) until it reaches the start of a `try` block, ..."
    - "As the control flow moves up the call stack, destructors are invoked for all objects with automatic storage duration that are constructed, but not yet destroyed, since the corresponding try-block was entered, **in reverse order** of completion of their constructors."
    - "If an exception is thrown **from a constructor** or (rare) from a destructor of an object (regardless of the object's storage duration), destructors are called for all fully-constructed non-static non-variant members and base classes, in reverse order of completion of their constructors."
  - from Lippman:
    - When a **block** is exited during stack unwinding, the compiler guarantees that objects created in that block are properly **destroyed**.
    - If a local object is of 
      - **class type**, the destructor for that object is called automatically.
      - **built-in type**, the compiler does no work to destroy that object.
        - this includes **raw pointers**: `int *a`, `MyClass *b` (note: `b` is a raw pointer, although `MyClass` is a class type)
        - good [example](https://stackoverflow.com/a/3181111): "On you specific questions, when an exception is thrown in a constructor, all fully constructed subobjects will be destroyed. That means that in the case of `b` it will be destroyed, in the case of `c`, it being a raw pointer, nothing is done."

## throw Expression

- **throw expressions**, which the detecting part uses to indicate that it encountered something it can't handle.
  - We say that a `throw` **raises** an exception.

Example:

Naive way: by returning an error indicator (without `throw` expressions):

```cpp
Sales_item item1, item2;
cin >> item1 >> item2;
// first check that item1 and item2 represent the same book
if (item1.isbn() == item2.isbn()) {
  cout << item1 + item2 << endl;
  return 0;           // indicate success
} else {
  cerr << "Data must refer to same ISBN"
            << endl;
  return -1;          // indicate failure
}
```

better: separate the part that adds the objects from the part that manages the interaction with a user:
- throw an expression (that is an **object** of type `runtime_error`)
- throwing an exception 
  - terminates the current function (calls `std::terminate` which itself calls `std::abort`) and
    - see [std::terminate](https://en.cppreference.com/w/cpp/error/terminate)
  - transfers control to a **handler** that will know how to handle this error
- Because the statements following a `throw` are not executed, a throw is like a `return`
  - It is usually part of a conditional statement or is the last (or only) statement in a function.
- type `runtime_error`
  - in the `stdexcept` header
  - must initialize a `runtime_error` by giving it a `string` or a C-style character string

```cpp
// first check that the data are for the same item
if (item1.isbn() != item2.isbn())
  throw runtime_error("Data must refer to same ISBN");
// if we're still here, the ISBNs are the same
cout << item1 + item2 << endl;
```

## try Block

- **try blocks**, which the handling part uses to deal with an exception.
  - A try block starts with the keyword `try` and ends with one or more `catch` clauses.
  - Exceptions thrown from code executed inside a `try` block are usually handled by one of the `catch` clauses.
  - Because they "handle" the exception, catch clauses are also known as **exception handlers**.
- Following the `try` block is a list of one or more `catch` clauses. A `catch` consists of three parts: 
  - the keyword `catch`, 
  - the **declaration** of a (possibly unnamed) object within parentheses (referred to as an **exception declaration**),
  - and a **block**.
    - When a `catch` is selected to handle an exception, the associated block is executed.
- Once the `catch` finishes, execution continues with the statement immediately following the last `catch` clause of the `try` block
- When a **handler** is entered, objects created along the call chain will have been destroyed.

```cpp
try {
  program-statements
} catch (exception-declaration) {
  handler-statements
} catch (exception-declaration) {
  handler-statements
} // ...
```

## Standard Exceptions

Usage:
- to report problems encountered in the functions in the standard library
- also intended to be used in the programs we write

Headers:
- 4 headers
  - `exception`
    - defines `exception` (exception without additional information)
  - `stdexcept`
    - defines general-purpose exception classes:
      - `exception`
      - `runtime_error`
      - `range_error`
      - `overflow_error`
      - `underflow_error`
      - `logic_error`
      - `domain_error`
      - `invalid_argumentlength_error`
      - `out_of_range`
  - `new`
    - defines `bad_alloc`
  - `type_info`
    - defines `bad_cast`

### Exception Classes

- are used to pass information about what happened between a `throw` and an associated `catch`.
- we can **create**, **copy**, and **assign** objects of any of the exception types
- initialization
  - most exception type objects are initialized from either a `string` or a **C-style string** (that provides additional information about the error that occurred)
  - types that can only be default initialized (cannot take an initializer)
    - `exception`
    - `bad_alloc`
    - `bad_cast`
- exception types define a `what` function
  - takes no arguments 
  - returns a `const char*` that points to a C-style character string
    - contents of returned string:
      - types that take a string initializer: return that string
      - other types: return value depends on the compiler
  - purpose: provide some sort of textual description of the exception thrown

```cpp
// https://cplusplus.com/reference/exception/exception/what/
// exception::what
#include <iostream>       // std::cout
#include <exception>      // std::exception

struct ooops : std::exception {
  const char* what() const noexcept {return "Ooops!\n";}
};

int main () {
  try {
      throw ooops();
  } catch (std::exception& ex) {
      std::cout << ex.what();
  }
  return 0;
}
```

## Matching

- During the search for a matching `catch`, the `catch` that is found is **not** necessarily the one that matches the exception best. 
  - Instead, the selected `catch` is **the first one** that matches the exception at all. 
- As a consequence, in a list of `catch` clauses, the most specialized catch must appear first.

## Rethrow

- a `throw` that is not followed by an expression ("empty" `throw`)
- to pass an exception out to another `catch`
  - the (current) exception object is passed up the chain
- can appear only in
  - in a `catch`
  - in a function called (directly or indirectly) from a `catch`
- if it appears in other places: `std::terminate` is called

```cpp
catch (my_error &eObj) {                // specifier must be a reference type (if we want to modify the exception object, before rethrowing)
  eObj.status = errCodes::severeErr;    // modifies the exception object
  throw;                                // the status member of the exception object is severeErr
} catch (other_error eObj) {
  // ...
  throw;
}
```

## Catch-All Handler

- **informal:** to catch any unhandled exception
- to catch any exception that might occur, regardless of type
- use an **ellipsis** for the exception declaration
- A catch-all clause matches any type of exception
- often used in combination with a [rethrow](#rethrow) expression
- If a `catch(...)` is used in combination with other catch clauses, **it must be last**.
  - Any `catch` that follows a catch-all can never be matched.

```cpp
void manip() {
  try {
    // actions that cause an exception to be thrown
  }
  catch (...) {
    // work to partially handle the exception
    throw;   // rethrow
  }
}
```

## Handle a Constructor that fails

From [isocpp](https://isocpp.org/wiki/faq/exceptions#ctors-can-throw):

Options:
- 1) Throw an exception.
  - Constructors don't have a return type, so it's not possible to use return codes. 
  - **The best way** to signal constructor failure is therefore to throw an exception. 
- 2) "zombie" state: If you don’t have the option of using exceptions, the "least bad" work-around is to put the object into a "zombie" state by setting an internal status bit so the object acts sort of like it's dead even though it is technically still alive.
  - In practice the "zombie" thing gets pretty ugly.

related: [Constructors throwing Exceptions](#constructors-throwing-exceptions)
