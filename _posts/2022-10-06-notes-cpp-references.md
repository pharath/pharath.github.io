---
title: "C++ Notes - References"
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
    - references
    - notes

---

# References

## Lvalue References

- from [isocpp](https://isocpp.org/wiki/faq/references#reseating-refs):
  - Unlike a pointer, once a reference is bound to an object, it can not be "reseated" to another object. 
  - The reference itself isn't an object 
    - it has no identity; 
    - taking the address of a reference gives you the address of the referent; 
    - remember: the reference **is** its referent
- idea
  - Normally, when you use a reference, you do not use the address-of operator. You simply use the reference as you would use the target variable.
    - references are **aliases** for their target
  - enables the function to change the object being referred to

```cpp
int intOne;
int &rSomeRef = intOne;
intOne = 5;

// intOne: 5
// rSomeRef: 5
// &intOne: 0x3500
// &rSomeRef: 0x3500
```

- references (unlike other variables)
  - must be initialized when they are declared
  - cannot be reassigned
- space **before** the address-of operator is **required**
- usually "the type of a reference must match the type of the object to which it refers"
  - 2 exceptions:
    - 1) "we can initialize a reference to const from any expression that can be converted to the type of the reference" (see [reference to const](#reference-to-const))

### reference to const 

- aka "`const` reference", "lvalue reference to a `const` value"
- `X const& x` is equivalent to `const X& x`, and `X const* x` is equivalent to `const X* x`. (see [isocpp.org](https://isocpp.org/wiki/faq/const-correctness#const-ref-nonsense))
- Unlike an ordinary reference, a reference to `const` cannot be used to change the object to which the reference is bound
- "we can **initialize** a reference to `const` from any expression that can be converted to the type of the reference"
  - we can bind a reference to `const` to 
    - a nonconst object, 
    - a literal, or 
    - a more general expression
    - an object of a different type (in this case, the reference is bound to a [temporary](#temporaries) object, see p.62)
- "Binding a reference to `const` to an object says nothing about whether the underlying object itself is `const`."
  - ie. the underlying object cannot be changed by using the reference to `const`, but it might be changed by other means
- binding a **non-const** reference to a [temporary](#temporaries) is illegal in C++, only **const** references may be bound to a temporary (see p.62)

```cpp
const int ci = 1024;
const int &r1 = ci;     // ok: both reference and underlying object are const
r1 = 42;                // error: r1 is a reference to const
int &r2 = ci;           // error: nonconst reference to a const object

// Initialization and References to const:
int i = 42;
const int &r1 = i;        // we can bind a const int& to a plain int object
const int &r2 = 42;       // ok: r1 is a reference to const
const int &r3 = r1 * 2;   // ok: r3 is a reference to const
int &r4 = r * 2;          // error: r4 is a plain, non const reference

// Temporaries and References to const:
// see examples in section "Temporaries"

// A Reference to const May Refer to an Object That Is Not const:
int i = 42;
int &r1 = i;          // r1 bound to i
const int &r2 = i;    // r2 also bound to i; but cannot be used to change i
r1 = 0;               // r1 is not const; i is now 0
r2 = 0;               // error: r2 is a reference to const
```

### const references

- usually refers to [reference to const](#reference-to-const)

From [isocpp.org](https://isocpp.org/wiki/faq/const-correctness#const-ref-nonsense):
- references are always `const`
- `X& const x` is functionally equivalent to `X& x`. Since you're gaining nothing by adding the `const` after the `&`, you shouldn't add it: it will confuse people — the `const` will make some people think that the `X` is const, as if you had said `const X& x`.

## Rvalue References

- introduced by the new standard to support move operations
- a reference that must be bound to an rvalue
- obtained by using `&&`
- may be bound only to an object that is about to be destroyed
  - so that we are free to "move" resources from an rvalue reference to another object
- rvalue and lvalue references have the opposite **binding properties**:

```cpp
int i = 42;
int &r = i;             // ok: r refers to i
int &&rr = i;           // error: cannot bind an rvalue reference to an lvalue
int &r2 = i * 42;       // error: i * 42 is an rvalue
const int &r3 = i * 42; // ok: we can bind a reference to const to an rvalue
int &&rr2 = i * 42;     // ok: bind rr2 to the result of the multiplication
```

Examples of expressions that return
- lvalues
  - Functions that return lvalue references
  - operators
    - assignment
    - subscript
    - dereference
    - prefix increment/decrement (`++i`)
  - Variable expressions
- rvalues (prvalues)
  - Functions that return a nonreference type
  - operators
    - arithmetic
    - relational
    - bitwise
    - postfix increment/decrement (`i++`)
  - literals

- rvalue references refer to objects that are about to be destroyed
- **ephemeral** vs **persistent**: 
  - lvalues have persistent state, whereas **rvalues** are either **literals** or **temporary objects**
- code that uses an rvalue reference is free to take over value/resources/state from the object to which the reference refers
  - "steal" value/resources/state from the object (rhs) bound to an rvalue reference (lhs)
- you **cannot** directly bind an rvalue reference **to a variable**, even if that variable was defined as an rvalue reference type

```cpp
int &&rr1 = 42;         // ok: literals are rvalues
// Error:
int &&rr2 = rr1;        // error: the expression rr1 is an lvalue!
```

## Reference to Pointer

From [stackoverflow](https://stackoverflow.com/a/1898556):
- a reference to a pointer is like a reference to any other variable

```cpp
void fun(int*& ref_to_ptr)
{
    ref_to_ptr = 0; // set the "passed" pointer to 0
    // if the pointer is not passed by ref,
    // then only the copy(parameter) you received is set to 0,
    // but the original pointer(outside the function) is not affected.
}
```

## Pointer to Reference

From [stackoverflow](https://stackoverflow.com/a/1898556):
- A pointer to reference is illegal in C++
  - because - unlike a pointer - a reference is just a concept that allows the programmer to make **aliases** of something else.
- A pointer is a **place in memory** that has the address of something else, but a reference is NOT.

## Forwarding Reference

- The term **universal reference** was coined by Scott Meyers as a common term that could result in either an **lvalue reference** or an **rvalue reference**.
- The C++17 standard introduced the term **forwarding reference**, because the major reason to use such a reference is to forward objects. 
  - However, note that it does not automatically forward. 
  - The term does not describe what it is but what it is typically used for.

### Forwarding Reference vs Rvalue Reference

`X&&` for a specific type `X`
- declares a parameter to be an **rvalue reference**
- it can **only** be bound to a movable object (a **prvalue**, such as a temporary object, and an **xvalue**, such as an object passed with `std::move()`)
  - see move constructor and move-assignment operator
- it is **always** mutable
- you can **always** "steal" its value

`T&&` for a template parameter `T`
- declares a **forwarding reference** (also called universal reference)
- it can be bound to a mutable, immutable (i.e., `const`), or movable object
  - see `f(T&& val)` in [Perfect Forwarding](#perfect-forwarding)
- it is mutable or immutable
  - `const` is never dropped!
- you can **sometimes** "steal" its value

### `remove_reference_t`

**Problem**: References must be initialized. Therefore, the following function template itself won't work properly with lvalue arguments.

```cpp
template<typename T> void f(T&& p) // p is a forwarding reference
{
  T x; // for passed lvalues, x is a reference
  ...
}
```

**Solution**: To deal with this situation, the `std::remove_reference` type trait is frequently used to ensure that `x` is not a reference

```cpp
template<typename T> void f(T&& p) // p is a forwarding reference
{
  std::remove_reference_t<T> x; // x is never a reference
  ...
}
```

## Perfect Forwarding

- to write generic code that "forwards" the basic property of passed arguments
  - modifiable objects forwarded as modifiable
  - preserve constness
  - movable objects forwarded as movable
- called **perfect forwarding**, because the result of calling `g()` indirectly through `f()` (or `forwardToG()` respectively) will be the same as if the code called `g()` directly
- no additional copies are made

### Example with Naive Forwarding

Forward a call of `f()` to a corresponding function `g()`:

Without templates, we would have to program all three cases for `f()` separately:

```cpp
#include <utility>
#include <iostream>

class X {
  ...
};

void g(X&) {
  std::cout << "g() for variable\n";
}
void g(X const&) {
  std::cout << "g() for constant\n";
}
void g(X&&) {
  std::cout << "g() for movable object\n";
}

// let f() forward argument val to g():
void f(X& val) {
  g(val);                 // val is non-const lvalue => calls g(X&)
}
void f(X const& val) {
  g(val);                 // val is const lvalue => calls g(X const&)
}
void f(X&& val) {
  g(std::move(val));      // val is non-const lvalue => needs std::move() to call g(X&&)
                          // without move() g(X&) would be called
}

int main()
{
  X v;              // create variable
  X const c;        // create constant

  f(v);             // f() for nonconstant object calls f(X&) => calls g(X&)
  f(c);             // f() for constant object calls f(X const&) => calls g(X const&)
  f(X());           // f() for temporary calls f(X&&) => calls g(X&&)
  f(std::move(v));  // f() for movable variable calls f(X&&) => calls g(X&&)
}
```

### Example with Perfect Forwarding

The same using **perfect forwarding** to forward arguments:

```cpp
#include <utility>
#include <iostream>

class X {
  ...
};

void g(X&) {
  std::cout << "g() for variable\n";
}
void g(X const&) {
  std::cout << "g() for constant\n";
}
void g(X&&) {
  std::cout << "g() for movable object\n";
}

// let f() perfect forward argument val to g():
template<typename T>
void f(T&& val) {
  g(std::forward<T>(val)); // call the right g() for any passed argument val
}

int main()
{
  X v;                  // create variable
  X const c;            // create constant

  f(v);                 // f() for variable calls f(X&) => calls g(X&)
  f(c);                 // f() for constant calls f(X const&) => calls g(X const&)
  f(X());               // f() for temporary calls f(X&&) => calls g(X&&)
  f(std::move(v));      // f() for move-enabled variable calls f(X&&) => calls g(X&&)
}
```

### Forwarding Arguments using `static_cast<T&&>`

Forward the argument along to another function `g()`:
- **problem**: the expression `x` will **always** be an **lvalue** (recall: variables are lvalues)
- **solution**: the `static_cast` casts `x` to its original type and lvalue- or rvalue-ness, thereby achieving perfect forwarding

```cpp
class C {
  ...
};

void g(C&);
void g(C const&);
void g(C&&);

template<typename T>
void forwardToG(T&& x)
{
  g(static_cast<T&&>(x)); // forward x to g()
}

void foo()
{
  C v;
  C const c;
  forwardToG(v);              // eventually calls g(C&)
  forwardToG(c);              // eventually calls g(C const&)
  forwardToG(C());            // eventually calls g(C&&)
  forwardToG(std::move(v));   // eventually calls g(C&&)
}
```

### Forwarding Arguments using `std::forward<T>`

**best practice:** 
- use `std::forward<>()` (in header `<utility>`) instead of `static_cast` for perfect forwarding
  - better documents the programmer's intent
  - prevents errors (such as omitting one `&`)

```cpp
#include <utility>

template<typename T> 
void forwardToG(T&& x)
{
  g(std::forward<T>(x)); // forward x to g()
}
```

```cpp
// std::forward<T&&> is essentially a static_cast
// (but it is more convenient than a static_cast)
template <typename T>
T&& forward(std::remove_reference_t<T>& t) {
    return static_cast<T&&>(t);
}
```

Thus, preserving type information is a **two-step process**:

1. To preserve type information **in the arguments**, we must define `forwardToG`'s function parameters as rvalue references to a template type parameter `T&&`.
2. We must use `forward` to preserve the arguments' original types when `forwardToG` **passes those arguments** to `g`.
