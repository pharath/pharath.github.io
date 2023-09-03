---
title: "C++ Notes - auto and decltype"
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
    - auto
    - decltype
    - notes

---

## Placeholder type specifiers

cppreference:
- "For **variables**, specifies that the type of the variable that is being declared will be automatically deduced from its initializer."
- "For **functions**, specifies that the return type will be deduced from its return statements. (since C++14)"

Syntax:

```cpp
type-constraint(optional) auto                    // (1) 	(since C++11)
type-constraint(optional) decltype ( auto ) 	  // (2) 	(since C++14)
```

1. type is deduced using the rules for **template argument deduction**.
2. type is `decltype(expr)`, where `expr` is the initializer or ones used in return statements.

## auto

- tells the compiler to deduce the type from the initializer
  - thus, a variable that uses `auto` must have an initializer
- uses the **rules** for **template argument deduction** to determine the type of interest (VJp301)
  - **therefore**, the type that the compiler infers for `auto` is not always exactly the same as the initializer's type
    - references: compiler uses the referred object's type, and not the reference type
    - top-level `const`s: ignored
- as parameter type (C++20)
- as return type
- in trailing return syntax, eg. `auto f() -> int`

cppreference:
- "in the type specifier sequence of a variable: `auto x = expr;` as a type specifier. The type is deduced from the initializer."
- "If the placeholder type specifier is `auto` (...), the variable type is deduced from the initializer using the rules for **template argument deduction** from a function call (...)."
  - "For example, given `const auto& i = expr;`, the type of `i` is exactly the type of the argument `u` in an imaginary template `template<class U> void f(const U& u)` if the function call `f(expr)` was compiled."
- "If the placeholder type specifier is used **to declare multiple variables**, the deduced types must match."
  - "For example, the declaration `auto i = 0, d = 0.0;` is ill-formed, while the declaration `auto i = 0, *p = &i;` is well-formed and the `auto` is deduced as `int`."
- "In **direct-list-initialization** (but not in copy-list-initialization), when deducing the meaning of the `auto` from a **braced-init-list**, the braced-init-list must contain **only one element**, and the type of `auto` will be the type of that element:"

```cpp
// C++17

// from: https://en.cppreference.com/w/cpp/language/template_argument_deduction#Other_contexts
auto x1 = {3}; // x1 is std::initializer_list<int>
auto x2{1, 2}; // error: not a single element
auto x3{3};    // x3 is int
               // (before N3922 x2 and x3 were both std::initializer_list<int>)

// from: https://mariusbancila.ro/blog/2017/04/13/cpp17-new-rules-for-auto-deduction-from-braced-init-list/
auto a = {42};    // std::initializer_list<int>
auto b{42};       // int
auto c = {1, 2};  // std::initializer_list<int>   // therefore, quote: "but not in copy-list-initialization"
auto d{1, 2};     // error, too many 
```

## decltype

```cpp
decltype ( entity )           // (1)    (since C++11)
decltype ( expression )       // (2)    (since C++11)
```

Inspects
1. the declared type of an **entity** (no parentheses!)
2. the type and value category of an **expression**

Note: "entity" is eg. a variable, function, enumerator, or data member (VJ15.10.2)

(1) If the argument is an **unparenthesized** id-expression or an **unparenthesized** class member access expression, then decltype yields the type of the **entity** named by this expression.

Examples:
- variable **without** parentheses
- data member **without** parentheses

(2) If the argument is any other expression of type `T`, and
- if the value category of `expression` is **xvalue**, then decltype yields `T&&`
- if the value category of `expression` is **lvalue**, then decltype yields `T&`
- if the value category of `expression` is **prvalue**, then decltype yields `T`
  - If `expression` is a **function call** which returns a **prvalue** of class type (...), a temporary object is **not** introduced for that prvalue.
    - Because no temporary object is created, the **type need not be complete** or have an available destructor, and can be abstract. This rule doesn't apply to sub-expressions: in `decltype(f(g()))`, `g()` must have a complete type, but `f()` need not.

Examples:
- variable **with** parentheses
- data member **with** parentheses
- address-of operator `&x`
- indirection operator `*x`
- built-in arithmetic expressions
- a function call

Note that if the name of an object is parenthesized, it is treated as an ordinary **lvalue expression**
- thus `decltype(x)` and `decltype((x))` are often different types. 

Lippman:

- returns the type of its operand
- unlike `auto`, `decltype` returns the type of references and top-level `const`s
- `decltype((variable))` is always a reference type, whereas `decltype(variable)` is a reference type only if `variable` is a reference
- result is a reference type if the expression yields an lvalue, eg. if `p` is an `int*`
  - `decltype(*p)` is `int&` (because dereference yields an **lvalue**)
  - `decltype(&p)` is `int**` (because address-of operator yields a **prvalue**)
    - phth: 
      - if `x` is an `int` then `&x` returns a `int*` (this is simply how the address-of operator is defined, see [Operators](#operators))
      - if `y` is an `int*` then `&y` returns a `int**`

**For function calls:**

```cpp
decltype(f()) sum = x; // sum has whatever type f returns
```

- does not call `f`
- uses the type that such a call **would** return as the type for `sum`

**For arrays:**

- `decltype(someArray)` does not automatically convert an array to its corresponding pointer type
  - ie. we must add a `*` (asterisk) in the following code:

```cpp
// arrPtr() returns a pointer to an array of five ints

int odd[] = {1,3,5,7,9};
int even[] = {0,2,4,6,8};

// returns a pointer to an array of five int elements
decltype(odd) *arrPtr(int i)
{
  return (i % 2) ? &odd : &even; // returns a pointer to the array
}
```

### Common Uses

1) can be used to inspect types

```cpp
// VJ15.10.2
void g (std::string&& s)
{
  // check the type of s:
  std::is_lvalue_reference<decltype(s)>::value; // false
  std::is_rvalue_reference<decltype(s)>::value; // true (s as declared)
  std::is_same<decltype(s),std::string&>::value; // false
  std::is_same<decltype(s),std::string&&>::value; // true

  // check the value category of s used as expression:
  std::is_lvalue_reference<decltype((s))>::value; // true (s is an lvalue)
  std::is_rvalue_reference<decltype((s))>::value; // false
  std::is_same<decltype((s)),std::string&>::value; // true (T& signals an lvalue)
  std::is_same<decltype((s)),std::string&&>::value; // false
}
```

2) `decltype` can also be useful when the **"value-producing"** `auto` deduction is not sufficient.
- ie. sometimes we want to produce **references** instead of values

```cpp
// "Pointers Are Iterators" (Lippman, p118)

// assume we have a variable "pos" of some unknown iterator type, and 
// - we want to create a variable "element" that refers to the element stored by "pos". 
// - we want to preserve the value- or reference-ness of the iterator's "operator*"

auto element = *pos;          // will always make a COPY of the element, never a REFERENCE

auto& element = *pos;         // will always make a REFERENCE to the element, never a COPY
// (While standard container iterators (are required to) always produce REFERENCES, this is 
// not the case for all iterators!).

// To address this problem, we can use "decltype" so that the value- or reference-ness of
// the iterator's "operator*" is preserved:
decltype(*pos) element = *pos;

// better: without code duplication
decltype(auto) element = *pos;
```

Note: This is why `decltype(auto)` is frequently used for return types (see "decltype(auto)" &rarr; "Common Uses" &rarr; (1)).

### decltype(auto)

Like `auto`
- a **placeholder type**
- type of a variable, return type, or template argument is determined from the type of the associated expression (initializer, return value, or template argument) (VJ15.10.3)

Unlike `auto`
- actual type is determined by applying the `decltype` construct directly to the expression

```cpp
int i = 42;               // i has type int
int const& ref = i;       // ref has type int const& and refers to i

auto x = ref;             // x has type int and is a new independent object
decltype(auto) y = ref;   // y has type int const& and also refers to i
```

Sometimes `auto` creates copies, whereas `decltype(auto)` doesn't:

```cpp
// auto vs decltype(auto)
// for vector indexing:
std::vector<int> v = { 42 };
auto x = v[0];            // x denotes a new object of type int
decltype(auto) y = v[0];  // y is a reference (type int&)
// because v[0] is an lvalue and for lvalues "decltype()" returns "T&"
```

Does not allow **specifiers** or **declarator operators** that modify its type:

```cpp
decltype(auto)* p = (void*)nullptr;   // invalid
int const N = 100;
decltype(auto) const NN = N*N;        // invalid
```

#### Parentheses Problem

**Parentheses** in the initializer may be significant (since they are significant for the `decltype` construct):

```cpp
int x;
decltype(auto) z = x;                 // object of type int
decltype(auto) r = (x);               // reference of type int&
```

For the same reason, in the following example `return r` will return **by value**, whereas `return (r)` will return **by reference**:

```cpp
// in particular, parentheses can have a severe impact on the validity of return statements
// (because the return expression is used to initialize a temporary of type decltype(auto) at the call site)
int g();
...
decltype(auto) f() {
  int r = g();
  return (r);                         // run-time ERROR: returns a reference to a local object
  // - read functions.md -> "return" first!
  //
  // - under the hood the return statement initializes a temporary "temp" like this:
  // 
  // `decltype(auto) temp = (r);       // decltype(expression) -> "temp" is a reference of type int&`
  //
  // - thus, "temp" is a reference that is BOUND TO the local object r
  // - this "temp" is the result of the function call f()
  // - decltype(auto) is deduced to int&, ie. f() returns "by reference"
  // 
  // Problem: 
  // - r is a local object
  //   - thus, r will get destroyed after the "return (r);"
  // - thus, "temp" will refer to a destroyed object (dangling reference)
  //
  // Solution:
  // - use "return r;" (ie. WITHOUT parentheses)
  // - because then, the return statement initializes a temporary "temp2" like this:
  //
  // `decltype(auto) temp2 = r;       // decltype(entity) -> "temp2" is an int`
  // 
  // - thus, "temp2" is an int and the local object r is COPIED TO "temp2" before it gets destroyed
  // - decltype(auto) is deduced to int, ie. f() returns "by value"
  // - "temp2" (the result of f()) itself will be destroyed at the end of the full-expression in which it was 
  //   created, but, until then, it will not be a "dangling reference"
}
```

#### Common Uses

TODO: einfach merken: `decltype(auto)` will always correctly return by value or by reference (depending on which of these two is appropriate for each situation)

1) It is frequently convenient **for return types**:

```cpp
// - first read cpp-functions.md -> "return"
// - then read the "Parentheses Problem" example above
// - then read VJ15.10.2: the "decltype(*pos) element = *pos;" example in the section about "decltype(expr)" (without the "auto")
//
// we want:
// - If container[idx] is an lvalue: return by reference
// - If container[idx] is a prvalue: return by value, ie. return a copy
template<typename C> class Adapt
{
  C container;
  ...
  decltype(auto) operator[] (std::size_t idx) {
    return container[idx];
  }
};
```

### declval

cppreference:

- Converts any type `T` to a reference type, making it possible to use member functions in the operand of the `decltype` specifier without the need to go through constructors.
- `std::declval` is commonly used in templates where acceptable template parameters may have no constructor in common, but have the same member function whose return type is needed.

```cpp
#include <iostream>
#include <utility>
 
struct Default
{
    int foo() const { return 1; }
};
 
struct NonDefault
{
    NonDefault() = delete;
    int foo() const { return 1; }
};
 
int main()
{
    decltype(Default().foo()) n1 = 1;                   // type of n1 is int
//  decltype(NonDefault().foo()) n2 = n1;               // error: no default constructor
    decltype(std::declval<NonDefault>().foo()) n2 = n1; // type of n2 is int
    std::cout << "n1 = " << n1 << '\n'
              << "n2 = " << n2 << '\n';
}
```

VJ:

- **main intended use**: in `decltype` and `sizeof` constructs
- a function template 
- defined in header `<utility>`
- `std::declval<T>` declares an **rvalue** of type `T`
- `std::declval<T&>` declares an **lvalue** of type `T`
- only available in an **unevaluated** context
- does not have a definition
  - therefore, cannot be called
  - therefore, doesn't create an object
- can be used as a placeholder for an object reference of a specific type
  - to "use" objects of the corresponding type without creating them

```cpp
// deduces the default return type RT from the passed template parameters T1 and T2
#include <utility>
template<typename T1, typename T2,
         typename RT = std::decay_t<decltype(true ? std::declval<T1>()
                                                  : std::declval<T2>())>>
RT max (T1 a, T2 b)
{
  return b < a ? a : b;
}

// Note:
// we can also implement max() WITHOUT declval, but this requires that we are 
// able to call default constructors for the passed types:
#include <type_traits>
template<typename T1, typename T2,
         typename RT = std::decay_t<decltype(true ? T1() : T2())>>
RT max (T1 a, T2 b)
{
  return b < a ? a : b;
}
```
