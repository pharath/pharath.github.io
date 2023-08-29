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

## Declaration

- **template declaration**: 
  - must include the template parameters (which need not be the same across the declaration(s) and the definition)
  - **best practice:** 
    - declarations for all the templates needed by a given file usually should appear together at the beginning of a file before any code that uses those names

```cpp
// all three uses of calc refer to the same function template
template <typename T> T calc(const T&, const T&); // declaration
template <typename U> U calc(const U&, const U&); // declaration
// definition of the template
template <typename Type>
Type calc(const Type& a, const Type& b) { /* . . . */ }
```

## Definition

- **template definition**: 
  - the declaration of a class template or function template is called a **definition** if it has a body (VJ10.2)
  - declaration and the definition of a given template must have the same number and kind (i.e., type or nontype) of parameters

## typename

- when using class members that are types

**Problem:**
- in nontemplate code,
  - when we write `string::size_type`, the compiler has the definition of `string` and can see that `size_type` is a type
- but in template code, 
  - Assuming T is a template type parameter, 
  - When the compiler sees code such as `T::mem` it won't know until instantiation time whether `mem` is a type or a `static` data member
  - eg. `T::size_type * p;` can be either 
    - a definition of a variable named `p`
    - multiplying a `static` data member named `size_type` by a variable named `p`

**Solution:**
- By default, the language assumes that a name accessed through the scope operator is **not** a type
  - as a result, we must explicitly tell the compiler that the name is a type by using `typename` (not `class`!)

```cpp
template <typename T>
typename T::value_type top(const T& c)
{
  if (!c.empty())
    return c.back();
  else
    return typename T::value_type();
}
```

## Terminology



## Default Template Arguments

- possible for both function and class templates
- a template parameter may have a default argument only if all of the parameters to its right also have default argument
  - **for function templates:** default value can be anywhere (given the rest can be deduced)
- Whenever we use a **class template**, we must always follow the template's name with brackets
  - if we want to use the defaults, we must put an **empty bracket pair** following the template's nam
- in example
  - `F` represents the type of a callable object
  - function parameter `f` will be bound to a callable object
  - `f` will be a default-initialized object of type `F`
  - type of `T` is deduced as `Sales_data`
  - `F` is deduced as the type of `compareIsbn`
  - when `compare` is called with three arguments, 
    - the type of the third argument must be a callable object that
      - returns a type that is convertible to `bool` and
      - takes arguments of a type compatible with the types of the first two arguments

```cpp
// function templates with default args:

// compare has a default template argument, less<T>
// and a default function argument, F()
template <typename T, typename F = less<T>>
int compare(const T &v1, const T &v2, F f = F())
{
  if (f(v1, v2)) return -1;
  if (f(v2, v1)) return 1;
  return 0;
}

// users may supply their own comparison operation but are not required to do so:
bool i = compare(0, 42); // uses less; i is -1
// result depends on the isbns in item1 and item2
Sales_data item1(cin), item2(cin);
bool j = compare(item1, item2, compareIsbn);

// class templates with default args:

template <class T = int> class Numbers { // by default T is int
  public:
  Numbers(T v = 0): val(v) { }
  // various operations on numbers
  private:
  T val;
};
Numbers<long double> lots_of_precision;
Numbers<> average_precision; // empty <> says we want the default type
```

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

### Order of Execution

From [cppreference](https://en.cppreference.com/w/cpp/language/template_argument_deduction):

"Template argument deduction takes place **after** the function template name lookup (which may involve argument-dependent lookup) and **before** template argument substitution (which may involve SFINAE) and overload resolution."

ie.

1. function template name lookup (which may involve argument-dependent lookup)
2. template argument deduction
3. template argument substitution (which may involve SFINAE)
4. overload resolution

(this is also the order in which they are presented in [cppreference](https://en.cppreference.com/w/cpp/language/function_template))

### Non-type Template Parameters

- represents a value rather than a type
- specified by using a specific type name instead of the `class` or `typename` keyword
- may be 
  - an **integral type**
    - An argument bound to a nontype integral parameter **must** be a **constant** expression.
  - a **pointer** or (lvalue) **reference** to an object or to a function type (see Lippman 14.8.3)
    - Arguments bound to a pointer or reference nontype parameter **must** have **static** lifetime
    - A pointer parameter can also be instantiated by `nullptr` or a zero-valued constant expression
- is a constant value inside the template definition
  - thus, can be used when constant expressions are required, eg. to specify the size of an array

```cpp
// trick to pass variable size arrays:
template<unsigned N, unsigned M>
int compare(const char (&p1)[N], const char (&p2)[M])
{
  return strcmp(p1, p2);
}

// when we call
compare("hi", "mom")

// the compiler will instantiate:
// (Recall: the compiler inserts a null terminator at the end of a string literal)
int compare(const char (&p1)[3], const char (&p2)[4])
```

VJ15.10.3: 
- "using **deduced** nontype parameters in function templates"

```cpp
// deduce the type of the parameter N of function template f<>() from the type of the nontype parameter of S
template<auto N> struct S {};
template<auto N> int f(S<N> p);
S<42> x;
int r = f(x);
```

### Instantiation

From [cppreference](https://en.cppreference.com/w/cpp/language/function_template#Function_template_instantiation): 
- A function template by itself is **not a type, or a function, or any other entity**. 
- No code is generated from a source file that contains only template definitions. 
- In order for any code to appear, a template must be **instantiated**: 
  - the template arguments must be determined so that the compiler can generate an actual function (or class, from a class template).

There are two forms of instantiation:
- **explicit** instantiation
- **implicit** instantiation

#### Explicit Instantiation

- for "controlling" instantiations
- WITH `extern` it is an (explicit) instantiation **declaration**
- WITHOUT `extern` it is an (explicit) instantiation **definition**
- When the compiler sees an `extern` template declaration, it will **not** generate code for that instantiation in that file
  - `extern` is a promise that there will be a non`extern` use of that instantiation elsewhere in the program
  - "An explicit instantiation declaration (an extern template) prevents implicit instantiations", [cppreference](https://en.cppreference.com/w/cpp/language/function_template#Function_template_instantiation)

```cpp
// instantiation declaration and definition
extern template class Blob<string>;             // declaration
template int compare(const int&, const int&);   // definition
```

For a given instantiation 
- there may be several `extern` declarations 
- there must be **exactly one** definition

```cpp
// Syntax:

// Explicit Instantiation Definition
template return-type name < argument-list > ( parameter-list ) ; 	        // (1) 	
template return-type name ( parameter-list ) ; 	                            // (2) 	
// Explicit Instantiation Declaration
extern template return-type name < argument-list > ( parameter-list ) ; 	// (3) 	(since C++11)
extern template return-type name ( parameter-list ) ; 	                    // (4) 	(since C++11)
```

```cpp
// examples from cppreference:

template<typename T>
void f(T s)
{
    std::cout << s << '\n';
}
 
template void f<double>(double); // instantiates f<double>(double)
template void f<>(char);         // instantiates f<char>(char), template argument deduced
template void f(int);            // instantiates f<int>(int), template argument deduced
```

#### Implicit Instantiation

cppreference:
- **implicit instantiation** occurs when
  - code refers to a function in context that requires the **function definition** to exist **AND** this particular **function has not been explicitly instantiated**
- The list of **template arguments** does not have to be supplied if it can be **deduced from context**.

```cpp
#include <iostream>
 
template<typename T>
void f(T s)
{
    std::cout << s << '\n';
}
 
int main()
{
    f<double>(1); // instantiates and calls f<double>(double)
    f<>('a');     // instantiates and calls f<char>(char)
    f(7);         // instantiates and calls f<int>(int)
    void (*pf)(std::string) = f; // instantiates f<string>(string)
    pf("∇");                     // calls f<string>(string)
}
```

## Class Templates

### Scope of T

- **Begin** of scope of T is just after `template <typename T>`
- **End** of scope of T is at the end of the class definition (after the semicolon)

### Member Functions

#### Syntax

```cpp
// Given a nontemplate member function
ret_type StrBlob::member_name(parm_list)

// the corresponding class template member will look like
template <typename T>
ret_type Blob<T>::member_name(parm_list)
```

**Inside** the scope of the class template itself, we may use the name of the template **without arguments**.
- see Notes "Classes" &rarr; "Scope"
- example:

```cpp
// postfix: increment/decrement the object but return the unchanged value
template <typename T>
BlobPtr<T> BlobPtr<T>::operator++(int)
{
  // no check needed here; the call to prefix increment will do the check
  BlobPtr ret = *this; // save the current value
  ++*this;          // advance one element; prefix ++ checks the increment
  return ret;       // return the saved state
}
```

#### Instantiation

By default, a member function of a class template is instantiated **only if** the program uses that member function.

**Example 1**:

The following example instantiates the `Blob<int>` class and three of its member functions:
- `operator[]`, 
- `size`, and 
- the `initializer_list<int>` constructor.

```cpp
// instantiates Blob<int> and the initializer_list<int> constructor
Blob<int> squares = {0,1,2,3,4,5,6,7,8,9};
// instantiates Blob<int>::size() const
for (size_t i = 0; i != squares.size(); ++i)
    squares[i] = i*i; // instantiates Blob<int>::operator[](size_t)
```

**Example 2**:

Lets us instantiate a class with a type that may not meet the requirements for some of the template's operations.
- eg. for instantiation of [vectors](#constraints-on-types-that-a-container-can-hold)

### Friends

For a **nontemplate** class:

```cpp
// forward declaration necessary to befriend a specific instantiation of a template
// (because, recall, a friend declaration is not a declaration)
template <typename T> class Pal;
class C {
  friend class Pal<C>;                      // specific friendship
  template <typename T> friend class Pal2;  // general friendship
};
```

For a class template:

```cpp
template <typename T> class Pal;
template <typename T> class C2 { 
  friend class Pal<T>;                      // specific friendship
  template <typename X> friend class Pal2;  // general friendship
};
```

### Typedefs

- we can define a `typedef` that refers to an instantiation of a class template
- we **cannot** define a `typedef` that refers to a template

```cpp
typedef Blob<string> StrBlob;
```

### Type Aliases

```cpp
template<typename T>
using twin = pair<T, T>;
```

which can be used as

```cpp
// better than using "pair<string, string>" because "string" has to be specified only once
twin<string> authors; // authors is a pair<string, string>
```

We can also **fix** one or more template parameters

```cpp
template <typename T>
using partNo = pair<T, unsigned>;
```

### static members

- mostly like for any other (nontemplate) class
  - a `static` member function is instantiated only if it is used in a program
  - access a `static` member of a class template 
    - through an object of the class type or 
    - by using the scope operator to access the member directly
- there is a distinct object **for each instantiation** of a class template
  - eg. all objects of type `Foo<X>` share the same `ctr` object and `count` function, but there is a distinct `ctr` and `count` for objects of type `Foo<Y>`

```cpp
template <typename T> 
class Foo {
  public:
    static std::size_t count() { return ctr; }
    // other interface members
  private:
    static std::size_t ctr;
    // other implementation members
};
```

**Defining** a static data member:

```cpp
template <typename T>
size_t Foo<T>::ctr = 0; // define and initialize ctr
```

### Containers

- table of all containers: [cppreference](https://en.cppreference.com/w/cpp/container#Function_table)
- each container is defined in a header file with the same name as the type
- containers are class templates (3.3)
- container types: see ["Types"](#types)

#### Constraints on Types That a Container Can Hold

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

