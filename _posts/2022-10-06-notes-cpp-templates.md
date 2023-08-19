---
title: "C++ Notes - Templates"
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
    - templates
    - notes

---

# Templates

- **instantiation**: The process that the compiler uses to create classes or functions from templates
- templates are **not** functions or classes
- templates "can be thought of as instructions to the compiler for generating classes or functions"
- templates can make the code shorter and more manageable

## Type Traits

- aka type transformation
- in the `type_traits` header
- used for template metaprogramming

### `remove_reference` Class Template

- has 
  - one template type parameter
  - a (`public`) type member named `type`
- If we instantiate `remove_reference` with a reference type, then type will be **the referred-to type**

```cpp
remove_reference<int&>        // the `type` member will be int
remove_reference<string&>     // `type` will be string
```

**Example:** For containers:

More generally, given that `beg` is an **iterator**:
- `remove_reference<decltype(*beg)>::type` will be the type of the element to which `beg` refers, where
  - `decltype(*beg)` returns the reference type of the element type (eg. `int&`)
  - `remove_reference::type` strips off the reference (eg. `int`)

```cpp
// return a copy of an element's value:

// must use typename to use a type member of a template parameter; see § 16.1.3 (p. 670)
template <typename It>
auto fcn2(It beg, It end) ->
  typename remove_reference<decltype(*beg)>::type
{
  // process the range
  return *beg; // return a copy of an element from the range
}
```

## Function Templates

**Function Template**:

see [stackoverflow](https://www.cplusplus.com/doc/oldtutorial/templates/)

```cpp
// function template
#include <iostream>
using namespace std;

template <class T>
T GetMax (T a, T b) {
  T result;
  result = (a>b)? a : b;
  return (result);
}

int main () {
  int i=5, j=6, k;
  long l=10, m=5, n;
  k=GetMax<int>(i,j);
  n=GetMax<long>(l,m);
  cout << k << endl;
  cout << n << endl;
  return 0;
}
```

**Generic Type**: Function templates are special functions that can operate with **generic types**. This allows us to create a function template whose functionality can be adapted to more than one type or class without repeating the entire code for each type.

**Template Parameter**: In C++ this can be achieved using **template parameters**. A template parameter is a special kind of parameter that can be used to pass a type as argument: just like regular function parameters can be used to pass values to a function, template parameters allow to pass also types to a function. These function templates can use these parameters as if they were any other regular type.

## Class Templates

[source](https://www.geeksforgeeks.org/templates-cpp/):
- to create a single class to work with different data types
- useful when a class defines something that is independent of the data type. 
  - Can be useful for classes like LinkedList, BinaryTree, Stack, Queue, Array, etc.

```cpp
// C++ Program to implement
// template Array class
#include <iostream>
using namespace std;

template <typename T> class Array {
private:
    T* ptr;
    int size;

public:
    Array(T arr[], int s);
    void print();
};

template <typename T> Array<T>::Array(T arr[], int s)
{
    ptr = new T[s];
    size = s;
    for (int i = 0; i < size; i++)
        ptr[i] = arr[i];
}

template <typename T> void Array<T>::print()
{
    for (int i = 0; i < size; i++)
        cout << " " << *(ptr + i);
    cout << endl;
}

int main()
{
    int arr[5] = { 1, 2, 3, 4, 5 };
    Array<int> a(arr, 5);
    a.print();
    return 0;
}
```

### Containers

- table of all containers: [cppreference](https://en.cppreference.com/w/cpp/container#Function_table)
- each container is defined in a header file with the same name as the type
- containers are class templates (3.3)
- container types: see ["Types"](#types)

#### std::array

- "a container that encapsulates **fixed size** arrays"
- "an aggregate type"
- "same semantics as a `struct` holding a **C-style array** `T[N]` as its only non-static data member."
  - **But**: "Unlike a C-style array, it doesn't decay to `T*` automatically."

#### std::vector

- **flexible-size** array (as opposed to the **fixed-size** array `std::array`)
- `std::vector` is a template, not a type
- elements are stored contiguously
- how a `vector` grows
  - Vectors typically allocate capacity beyond what is immediately needed.
  - The container holds this **storage in reserve** and uses it to allocate new elements **as they are added**.
    - Thus, there is **no need to reallocate** ("`vector` zu einem Ort umlagern wo mehr Platz ist") the container each time an element is added
  - dramatically more efficient than reallocating the container each time an element is added
- container size management:
  - container size management functions, see Table 9.10
    - `c.shrink_to_fit()`: Request to reduce `capacity()` to equal `size()`
    - `c.capacity()`: Number of elements `c` can have before reallocation is necessary.
    - `c.reserve(n)`: Allocate space for at least `n` elements.
  - vector `size`: number of elements the vector already holds
  - vector `capacity`: how many elements it can hold before more space must be allocated

Element Access:

`front`
- "Returns a **reference to** the first element in the container."
- "Calling front on an empty container causes undefined behavior."

`back`
- "Returns a **reference to** the last element in the container."
- "Calling back on an empty container causes undefined behavior."

`push_back`
- "Appends the given element value to the end of the container"
- "If the new `size()` is greater than `capacity()` then all iterators and references (including the `end()` iterator) are invalidated. Otherwise only the end() iterator is invalidated."

`pop_back`
- "Removes the last element of the container."
- "Calling `pop_back` on an empty container results in undefined behavior."
- "Iterators and references to the last element, as well as the end() iterator, are invalidated."

#### std::string

- A specialized container that contains characters.
- similar to `vector`
  - `std::string` can be considered as `std::vector<char>`
- Convert to C-style string (null-terminated string)
  - `.c_str()` converts a string to a null-terminated string (C-style string), [stackoverflow](https://stackoverflow.com/questions/7416445/what-is-use-of-c-str-function-in-c) (s. Antwort von hkBattousai)
- **string literals** are not standard library `strings`
- elements are stored contiguously
- how a `string` grows
  - Strings typically allocate capacity beyond what is immediately needed. (like `vector`)

`string::size_type`
- a **companion type**
  - companion types make it possible to use the library types in a machine-independent manner
- an unsigned type
- big enough to hold the size of any `string`
- tedious to type `string::size_type` &rarr; use `auto` or `decltype`

**C-style Strings**:

```cpp
strlen(p)                           // Returns the length of p, not counting the null.
strcmp(p1,p2)                       // Compares p1 and p2 for equality. Returns 0 if p1 == p2, a positive value if p1 > p2, a negative value if p1 < p2.
strcat(p1,p2)                       // Appends p2 to p1. Returns p1.
strcpy(p1,p2)                       // Copies p2 into p1. Returns p1.
```

### Iterators

Generic programming:
- **Best practice:** C++ programmers
  - use `!=` rather than `<`
    - because only a few library types have the `<` operator
  - use `!=` as a matter of habit
  - use iterators rather than subscripts
    - because only a few library types have the subscript operator

## Variadic Templates

TODO

- a template function or class that can take a varying number of parameters. 
- The varying parameters are known as a **parameter pack**.
- There are two kinds of parameter packs:
  - A **template parameter pack** represents zero or more template parameters, 
  - a **function parameter pack** represents zero or more function parameters.
- We use an **ellipsis** to indicate that a template or function parameter represents a pack.

```cpp
// "Args" is a template parameter pack; "rest" is a function parameter pack
// "Args" represents zero or more template type parameters
// "rest" represents zero or more function parameters
template <typename T, typename... Args>
void foo(const T &t, const Args& ... rest);
```

Example:

```cpp
// Given these calls ...
int i = 0; double d = 3.14; string s = "how now brown cow";
foo(i, s, 42, d);     // three parameters in the pack
foo(s, 42, "hi");     // two parameters in the pack
foo(d, s);            // one parameter in the pack
foo("hi");            // empty pack

// ... the compiler will instantiate
void foo(const int&, const string&, const int&, const double&);
void foo(const string&, const int&, const char(&)[3]);
void foo(const double&, const string&);
void foo(const char(&)[3]);
// in each case,
// - the type of T is deduced from the type of the first argument
// - remaining arguments (if any) provide the number of, and types for, the additional arguments to the function
```

