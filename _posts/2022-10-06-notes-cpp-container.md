---
title: "C++ Notes - STL Container"
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
    - container
    - notes

---

# Container

- table of all containers: [cppreference](https://en.cppreference.com/w/cpp/container#Function_table)
- each container is defined in a header file with the same name as the type
- containers are class templates (3.3)
- container types: see ["Types"](#types)

## Constraints on Types That a Container Can Hold

We can define a container for a type that does **not** support an operation-specific **requirement**
- but we can use an operation only if the element type meets that operation's requirements
- eg. some classes do not have a default constructor, so that we cannot construct such containers using only an element count:
  - **operation**: construct a container using only an element count
  - **requirement**: element type must have default constructor

```cpp
// assume noDefault is a type without a default constructor
vector<noDefault> v1(10, init);   // ok: element initializer supplied
vector<noDefault> v2(10);         // error: must supply an element initializer
```

This is enabled by the fact that members are instantiated only if we use them (see [class templates](#class-templates)).

## std::array

- "a container that encapsulates **fixed size** arrays"
- "an aggregate type"
- "same semantics as a `struct` holding a **C-style array** `T[N]` as its only non-static data member."
  - **But**: Unlike a C-style array, 
    - it doesn't decay to `T*` automatically
    - it supports **copy initialization** from
      - same size array
      - same size `initializer_list`
      - smaller size `initializer_list` (missing elements are value initialized)
      - **not supported:** elements in a range, `C c(b,e)`

## std::vector

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
- "Calling `front` on an empty container causes undefined behavior."

`back`
- "Returns a **reference to** the last element in the container."
- "Calling `back` on an empty container causes undefined behavior."

By subscript:
- type of a subscript is the corresponding `size_type` (eg. `vector<int>::size_type`)
- **buffer overflow errors**: see the corresponding section in section "Arrays" (C-style)

Add Elements:

**Best practice:**
- do not define a vector of a specific size
  - often unnecessary - and can result in poorer performance - to define a vector of a specific size
  - it is usually **more efficient** to define an empty vector and add elements as the values we need become known at run time
- Starting with an **empty vector** is distinctly different from how built-in arrays are used in C or Java (where it is best to define a vector at its expected size)

`push_back`
- "Appends the given element value to the end of the container"
- "If the new `size()` is greater than `capacity()` then all iterators and references (including the `end()` iterator) are invalidated. Otherwise only the `end()` iterator is invalidated."

`pop_back`
- "Removes the last element of the container."
- "Calling `pop_back` on an empty container results in undefined behavior."
- "Iterators and references to the last element, as well as the `end()` iterator, are invalidated."

## std::string

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

# Iterators

Generic programming:
- **Best practice:** C++ programmers
  - use `!=` rather than `<`
    - because only a few library types have the `<` operator
  - use `!=` as a matter of habit
  - use iterators rather than subscripts
    - because only a few library types have the subscript operator

