---
title: "C++ Notes - Functions"
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
    - functions
    - notes

---

# Functions

## Scope

cppreference:
- **scope**: Each **name** that appears in a C++ program is only visible in some possibly discontiguous portion of the source code called its **scope**.
- **block scope**: 
  - The **potential scope** of a name declared in a **block (compound statement)** begins at the point of declaration and ends at the end of the block.
  - **Actual scope** is the same as potential scope unless an identical name is declared in a **nested block**, in which case the potential scope of the name in the nested block is excluded from the actual scope of the name in the enclosing block.
- **Function parameter scope**
  - **begin of scope**: The potential scope of a name declared in a function parameter (...) or of a function-local predefined variable **begins at** the point of declaration.
    - **end of scope 1**: If the enclosing function declarator is not the declarator of a function definition, its potential scope **ends at** the end of that function declarator.
    - **end of scope 2**: Otherwise, its potential scope **ends at** the end of the last exception handler of the function-try-block, or at the end of the function body if a function try block was not used. 

```cpp
// example: function parameter scope:
const int n = 3;
 
int f1(
    int n // scope of function parameter n begins,
          // scope of global n pauses
//  , int y = n // error: default argument references a function parameter
);
```

Inside class definitions:
- The definitions of the member functions of a class are **nested inside** the scope of **the class itself**.
  - Hence, if a member function uses a name of a data member, this name is resolved as the data member defined inside the class.
    - ie. we do not have to specify the data member's scope via a `className::` prefix

## Inline Functions

- A function specified as `inline` (usually) is expanded "in line" at each call. 
  - the word "usually" means that this expansion does not happen sometimes because the compiler can choose to ignore the `inline`! (see point below)
- inline functions may be defined multiple times in the program (unlike ordinary functions), but all definitions must match
  - therefore, inline functions normally are **defined in headers**
- `inline` removes the **run-time overhead** of using a normal function.
  - Explanation: **Calling** a function is likely to be slower than **evaluating** the equivalent expression.
    - On most machines, a **function call** does a lot of work:
      - Registers are saved before the call and restored after the return;
      - arguments may be copied; and
      - the program branches to a new location.
- The `inline` specification is only a **request** to the compiler.
  - The compiler may choose to ignore this request.
- In general, the inline mechanism is meant to optimize **small, straight-line functions that are called frequently**.
  - Many compilers will not inline a recursive function.
  - A 75-line function will almost surely not be expanded inline.
- 7.3.2: In practice, well-designed C++ programs tend to have lots of small (inline) functions such as `do_display`

## Parameter List

From [cppreference](https://en.cppreference.com/w/cpp/language/function#Parameter_list):

Parameter **names** declared in function declarations are usually for only self-documenting purposes. They are used (but remain optional) in function definitions.

The **type** of each function parameter in the parameter list is determined according to the following rules:
1. First, `decl-specifier-seq` and the `declarator` are combined as in any **declaration** (&rarr; see "Definitions") to determine the type.
2. If the type is "array of `T`" or "array of unknown bound of `T`", it is replaced by the type "pointer to `T`".
3. If the type is a "function type `F`", it is replaced by the type "pointer to `F`".
  - see [Return a Pointer to Function](#return-a-pointer-to-function)
4. **Top-level cv-qualifiers** are dropped from the parameter type
  - This adjustment **only** affects the **function type**, but doesn't modify the property of the parameter
  - eg. `int f(const int p, decltype(p)*);` and `int f(int, const int*);` declare the same function.

Because of these rules, the following function declarations declare exactly the same function:

```cpp
// cppreference
// rule 2. "array of `T`"
int f(char s[3]);
int f(char[]);
int f(char* s);
int f(char* const);
int f(char* volatile s);
```

The following declarations also declare exactly the same function:

```cpp
// cppreference
// rule 3. "function type `F`"
int f(int());
int f(int (*)());   // phth
int f(int (*g)());
```

## Default Arguments

- 6.5.1
- "A **default argument** is specified as an initializer for a parameter in the parameter list"
- Arguments in the call are **resolved** by position. The default arguments are used for the trailing (right-most) arguments of a call.
  - "if a parameter has a default argument, all the parameters that follow it must also have default arguments"
  - **general rule**: order the parameters so that those least likely to use a default value appear first!
- when re-declaring functions (recall, re-declaring functions is legal):
  - each parameter can have its default specified **only once**
  - defaults can be specified only if all parameters **to the right** already have defaults
  - **best practice**: Default arguments ordinarily should be specified with the function declaration in an appropriate header.
- changing the value of a default argument
  - you can **assign** a new default value to a default argument
  - do not try to change a default argument by "hiding" its name in a new scope (see the example in p. 237 and the corresponding explanation on p.238)
- "Function parameters are not allowed in default arguments (...).", [cppreference](https://en.cppreference.com/w/cpp/language/default_arguments)

```cpp
int f(int a, int b = a);          // Error: the parameter a used in a default argument
```

## Return Values

ignoring return values:
- "calling a function and ignoring the return result is *very* common", [stackoverflow](https://stackoverflow.com/a/38919156)
- eg. `printf("hello\n");` ignores the return value, [stackoverflow](https://stackoverflow.com/a/38919103)

## Static Functions

- see "objects.md" &rarr; "static members"
- see [stackoverflow](https://stackoverflow.com/a/15235626)
  - a `static` function is 
    - often used as a class **member** function
    - only very rarely used for a **free-standing** function

## Member Functions

- "member function bodies may use other members of their class regardless of where in the class those members appear" (see "compile order" in [Classes](#classes))
- "code is interpreted as being inside the scope of the class" (ie. eg. no need to use `this` to access members)

### Implicit "this" Parameter

- a `total.isbn()` call is translated like a `Sales_data::isbn(&total)` call
  - "the compiler passes the address of `total` to the implicit `this` parameter" (as if `Sales_data::isbn(Sales_data *const this)` were the function definition, see `passing_by_reference.c` - Listing 9.6)

### const Member Functions

"const member functions cannot change the object on which they are called."
- "The purpose of that `const` is to **modify the type of** the implicit `this` pointer." (see [this](#this))
  - **phth**: `this` is a `const` pointer, but the `const` following the parameter list makes it a pointer to `const` (see [pointer to const](#pointer-to-const))
  - "A `const` following the parameter list indicates that `this` is a pointer to `const`"

We can think of the `const` member 
```cpp
std::string isbn() const { return bookNo; }
```

as if it were written as
```cpp
// this code is illegal: we may not explicitly define the this pointer ourselves
std::string Sales_data::isbn(const Sales_data *const this) { return this->bookNo; }
```

- A "`const` function", denoted with the keyword `const` after a function declaration, makes it a compiler error for this class function to change a member variable of the class. However, reading of a class variables is okay inside of the function, but writing inside of this function will generate a compiler error., [stackoverflow](https://stackoverflow.com/a/3141107)

### inline member functions

- member functions ...
  - defined inside the class: automatically `inline`
  - defined outside the class: need to specify as `inline`
  - declared inside the class: need to specify as `inline`
- `inline` member functions should be defined in the same header as the corresponding class definition (for the same reason as described for `inline` functions)
- it is legal to specify `inline` on both the declaration and the definition. 
  - **Best practice**: specifying `inline` only on the definition outside the class can make the class easier to read.

### Move-enabled Members

- Overloaded functions that distinguish between moving and copying a parameter typically have one version that takes a `const T&` and one that takes a `T&&.`

```cpp
// - can pass any object that can be converted to type X to this version
// - copies data from its parameter
void push_back(const X&); // copy: binds to any kind of X
// - can pass only an rvalue that is not const to this version
// - an exact match (and a better match) for nonconst rvalues and will be run when we pass a modifiable rvalue
void push_back(X&&);      // move: binds only to modifiable rvalues of type X

// These push_back functions could be eg. members of class StrVec:
class StrVec {
public:
  void push_back(const std::string&); // copy the element
  void push_back(std::string&&);      // move the element
  // other members as before
};

// When we call push_back the type of the argument determines whether 
// the new element is copied or moved into the container:
StrVec vec;             // empty StrVec
string s = "some string or another";
vec.push_back(s);       // calls push_back(const string&)
vec.push_back("done");  // calls push_back(string&&)
```

- Ordinarily, there is no need to define versions of the operation that take a `const X&&` or a (plain) `X&`.
  - Usually, we pass an **rvalue reference** when we want to "steal" from the argument. In order to do so, the argument must not be `const`.
  - Similarly, **copying** from an object should not change the object being copied. As a result, there is usually no need to define a version that take a (plain) `X&` parameter.

## Lambdas

From [learn.microsoft](https://learn.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-170):
- In C++11 and later, a **lambda expression** - often called **a lambda** - is a convenient way of defining an anonymous function object (a closure) right at the location where it's invoked or passed as an argument to a function. Typically lambdas are used to encapsulate a few lines of code that are passed to algorithms or asynchronous functions.

## Passing Call Parameters in and out of Functions

phth:

- "in" parameters are only passed to a function
- "out" parameters are only returned from a function
- "in-out" parameters are passed **and** also returned from a function

From [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#fcall-parameter-passing):

- F.15: Prefer simple and conventional ways of passing information
  - when you use `move` semantics, "measure to ensure that it really is an improvement, and document/comment because the improvement might not be portable"
  - The following table summarizes the advice in the following Guidelines, F.16-21. (see notes below the table)

![summary](https://isocpp.github.io/CppCoreGuidelines/param-passing-advanced.png)

- F.16: For "in" parameters, pass cheaply-copied types by value and others by reference to `const`
- F.17: For "in-out" parameters, pass by reference to non-`const`
- F.18: For "will-move-from" parameters, pass by `X&&` and `std::move` the parameter
- F.19: For "forward" parameters, pass by `TP&&` and only `std::forward` the parameter
- F.20: For "out" output values, prefer return values to output parameters
- F.21: To return multiple "out" values, prefer returning a `struct` or `tuple`

## Return Value Optimization, RVO

### Return using Move instead of Copy

TODO

### Copy Elision

- for `return` statements: copy elision happens **in addition to** the "Move instead of Copy Trick" mentioned above

Initialization:
- a **copy** or **move constructor invocation** is often optimized away by constructing the object used to initialize right in the target object
  - among other things, this happens when copy or move constructors are invoked when functions `return` values (see [Copy Constructor](#copy-constructor), "called whenever ...")
- in the following example a compiler will typically construct the `X` from `make()` directly in `x`; thus eliminating ("eliding") a copy:

```cpp
X make(Sometype);   // create an "X" object named "make", calls the "X(Sometype)" constructor (defined in BS6.1.1)
X x = make(value);  // direct initialize "make" AND 
                    // copy initialize "x" (copying is not necessary because "make" was already created in "x")
```

Avoids Move Constructor Invocations:
- compiler is obliged (by the C++ standard) to eliminate most copies associated with initialization, so **move constructors** are not invoked
- eliminates (even) the very minor overhead of a move
- cppreference: "When the initializer is a prvalue, the move constructor call is often optimized out (...), see copy elision."

Does **not** avoid Move-Assignment Operator Invocations:
- The same is usually not possible for **assignments**.
  - ie. it is usually not possible to implicitly eliminate copy or move operations from assignments

**Prvalue semantics ("guaranteed copy elision")**:
- **phth:** "prvalues" sind im Prinzip die "rvalues", die bisher besprochen wurden
- Since C++17, a prvalue is not materialized until needed, and then it is constructed directly into the storage of its final destination. 
  - This sometimes means that even when the language syntax visually suggests a copy/move (e.g. copy initialization), no copy/move is performed
  - examples:

```cpp
// from: https://en.cppreference.com/w/cpp/language/copy_elision
// phth: brackets "[]" by me

// Example 1: Initializing the returned object in a return statement, when the operand is a prvalue of the same class type (ignoring cv-qualification) as the function return type:
T f()           // [return type of f is nonreference, ie. a prvalue]
{
    return U(); // constructs a [nameless] temporary of type U,
                // then [copy] initializes the returned T from the temporary [using T's copy/move constructor]
}
T g()           // [return type of g is nonreference, ie. a prvalue]
{
    return T(); // constructs the returned T directly; no move
}
// The destructor of the type returned must be accessible at the point of the return statement and non-deleted, even though no T object is destroyed. 
```

```cpp
// Example 2: In the initialization of an object, when the initializer expression is a prvalue of the same class type (ignoring cv-qualification) as the variable type:
// phth: 
// - f() (from Example 1) returns a nameless temporary object of nonreference type T, ie a prvalue
// - T() returns a nameless temporary object of nonreference type T, ie a prvalue
T x = T(T(f())); // x is initialized by the result of f() directly; no move
```

## Functions with Varying Parameters

- Sometimes we do not know in advance **how many arguments** we need to pass to a function
- **two primary ways** to write a function that takes a varying number of arguments
  1. if all the arguments have the same type
    - pass a library type named `initializer_list`
  2. if the argument types vary
    - write a special kind of function, known as a **variadic template**
- **ellipsis**
  - special parameter type
  - can be used to pass a varying number of arguments
  - should be used only in programs that need to interface to C functions

### `initializer_list`

- **primary use**:
  - to write a function that takes a **varying number of arguments** (all of the same type)
    - although you could use a function template with call parameter `int const (&arr)[N]` (= `const int (&arr)[N]`) instead, see [C-style Array vs `initializer_list`](#c-style-array-vs-initializer_list) and "templates.md" &rarr; "Passing Variable Size Arrays"
      - but this is **less convenient** because it introduces **(1)** the nontype template parameter `N` and **(2)** the type parameter for the array elements (here: `int`), whereas `intializer_list<int>` needs only **(2)** the type parameter for the elements
  - "**Initializer-list constructors**" (BJ5.2.3)
    - like [constructor (10) for `vector`](https://en.cppreference.com/w/cpp/container/vector/vector)

Lippman:

- defined in the `<initializer_list>` header
- a library type 
- a class template

#### Backing Array

- an `initializer_list` object "represents an **array** of values of the specified type", (Lip)
  - Why "array"?
    - "An object of type `std::initializer_list<T>` is a lightweight **proxy object** ("Zwischenobjekt") that provides **access to** an **array** of objects of type `const T`.", [cppreference](https://en.cppreference.com/w/cpp/utility/initializer_list)
      - **"backing array"**: "An object of type `std::initializer_list<E>` is constructed from an initializer list as if the compiler generated (...) a prvalue of type "array of `N` `const E`", where `N` is the number of elements in the initializer list; this is called the initializer list's **backing array**.", [cppreference](https://en.cppreference.com/w/cpp/language/list_initialization#List-initializing_std::initializer_list)
        - "Copying a `std::initializer_list` does not copy the **backing array** of the corresponding initializer list.", [cppreference](https://en.cppreference.com/w/cpp/language/list_initialization#List-initializing_std::initializer_list)

**Example 1:** "Each element **of the backing array** is **copy-initialized** with the corresponding element of the initializer list, and the `std::initializer_list<E>` object is constructed to **refer to** that array.", [cppreference](https://en.cppreference.com/w/cpp/language/list_initialization#List-initializing_std::initializer_list)

```cpp
// "backing array"

// phth: what WE see
void f(std::initializer_list<double> il);
void g(float x)
{
   f({1, x, 3});
}

// phth: what THE COMPILER sees
void g(float x)
{
    const double __a[3] = {double{1}, double{x}, double{3}}; // backing array
    f(std::initializer_list<double>(__a, __a + 3));
}
```

#### Immutability

Lip:

- elements are always `const` values
  - there is **no way to change** the value of an **element**

#### Initialization

```cpp
// Lip
initializer_list<T> lst; // Default initialization; an empty list of elements of type T.
initializer_list<T> lst{a,b,c...};
                // lst has as many elements as there are initializers; elements are copies of
                // the corresponding initializers. Elements in the list are const.
```

#### Copy, Assignment

- "Copying a `std::initializer_list` does not copy the **backing array** of the corresponding initializer list.", see ["Backing Array"](#backing-array)
  - phth: like a `shared_ptr` (where `shared_ptr`'s "handled resource" corresponds to `initializer_list`'s "backing array")

```cpp
// Lip
// Copying or assigning an initializer_list does not copy the elements
// in the list. After the copy, the original and the copy share the elements.
lst2(lst)
lst2 = lst
```

#### Iterators, Capacity

```cpp
// Lip
// Capacity
lst.size()  // Number of elements in the list.
// Iterators
lst.begin() // Returns a pointer to the first and one past the last element in lst.
lst.end()
```

#### As Function Parameter

- phth: `initializer_list` is often used to write a function that takes a **varying number of arguments**
  - phth: this is useful (and often used), e.g. when passing a **constructor parameter** for a sequential container, see [stackoverflow](https://stackoverflow.com/a/56169251/12282296) and "**Initializer-list constructors**" (BJ5.2.3)
- When we pass a sequence of values to an `initializer_list` parameter, we must enclose the sequence in curly braces

```cpp
void error_msg(initializer_list<string> il)
{
  for (auto beg = il.begin(); beg != il.end(); ++beg)
    cout << *beg << " ";
  cout << endl;
}

// expected, actual are strings
if (expected != actual)
  error_msg({"functionX", expected, actual});
else
  error_msg({"functionX", "okay"});
```

#### range for

- we can use a **range for** to process the elements

```cpp
void error_msg(ErrCode e, initializer_list<string> il)
{
  cout << e.msg() << ": ";
  for (const auto &elem : il)
    cout << elem << " " ;
  cout << endl;
}
```

#### C-style Array vs `initializer_list`

From [stackoverflow](https://stackoverflow.com/a/56169251/12282296):

Plain and simple: `initializer_list` isn't a container. It's an immutable view onto externally allocated elements. It's utterly unsuitable for any scenario a container would be useful in - consider the needless indirection (no resizability), the immutability, the idiomacy of its name. On top of that, it has no proper interface.

A situation where both seem adequate is a **constructor parameter** for a sequence ("**Initializer-list constructors**" (BJ5.2.3)). If the length is fixed (or template-parametrized), then `int const (&arr)[N]` is possible, although `initializer_list` is far simpler and more flexible. After all, **that's what it was designed and intended for**..

Comments:

- (...) why do you say that `intializer_list` is "far simpler and more flexible"? - **max66** `May 16, 2019 at 13:19`
- **@max66** It's simpler and more flexible as it doesn't require you to think of, or constrain **length** the same way an array parameter would. - **Columbo** `May 16, 2019 at 15:38`
  - phth: see example in ["As Function Parameter"](#as-function-parameter), this is why we are in section **"Functions with Varying Parameters"** here

### Variadic Templates

see "templates.md"

### Ellipsis

- special parameter type
- can be used to pass a varying number of arguments
- should be used only in programs that need to interface to C functions

## Return

Values are returned in exactly the same way as variables and parameters are initialized: 
- The **return value** is used to **initialize** a **temporary** at the call site, and that temporary is the result of the function call.
  - if we return **by value**, the return value is **copied** to the call site
  - if we return **by reference**, the return value is **not copied**

### Return Statement

1. terminates the function that is currently executing 
2. returns control to the point from which the function was called

2 forms:
- `return;`
- `return expression;`

### void functions

- a `return` with no value may be used **only in** a function that has a return type of `void`
- functions that return `void` are not required to contain a `return`
  - in a `void` function, an **implicit return** takes place after the function's last statement
- the `return expression;` form may be used **only to** return the result of calling another function that returns `void`
  - returning any other expression from a `void` function is a **compile-time error**

### Return Value

- must have
  - the same type as the function return type, or 
  - a type that can be implicitly converted to the function return type
- every non-`void` function **must** return a value
- a `return` with no value may be used **only in** a function that has a return type of `void`

### Common Errors

```cpp
// incorrect return values, this code will not compile
bool str_subrange(const string &str1, const string &str2)
{
  // same sizes: return normal equality test
  if (str1.size() == str2.size())
    return str1 == str2;      // ok: == returns bool
  // find the size of the smaller string; conditional operator, see § 4.7 (p. 151)
  auto size = (str1.size() < str2.size())
              ? str1.size() : str2.size();
  // look at each element up to the size of the smaller string
  for (decltype(size) i = 0; i != size; ++i) {
    if (str1[i] != str2[i])
      return; // error #1: no return value; compiler should detect this error
  }
  // error #2: control might flow off the end of the function without a return
  // the compiler might not detect this error -> what happens at run time is undefined
}
```

### Return a Reference

- "Reference Returns Are Lvalues" (p226)

```cpp
// Return a reference to the shorter of two strings:

// The strings are not copied when the function is called or when the result is returned
const string &shorterString(const string &s1, const string &s2)
{
  return s1.size() <= s2.size() ? s1 : s2;
}
// "Reference Returns Are Lvalues" (p226) -> if the return type was not const, we 
// could ASSIGN TO the result of shorterString() like so:
//shorterString("hi", "bye") = "X";     // error: return value is const
```

- "Never Return a Reference or Pointer to a Local Object" (Lippman)

```cpp
// disaster: this function returns a reference to a local object
const string &manip()
{
  string ret;
  // transform ret in some way
  if (!ret.empty())
    return ret;       // WRONG: returning a reference to a local object!
  else
    return "Empty";   // WRONG: "Empty" is a local temporary string
}
```

### Return an Array

Lippman:
- a function **cannot** return an array
  - because we cannot copy an array (see "Arrays" &rarr; "Copy")
- however, a function can return a pointer or a reference to an array
  - see "cpp-pointers-memory.md" &rarr; "Returning a Pointer to an Array"

### Return a Function

- we can't return a function type but **can** return a **pointer to** a function type (see [Return a Pointer to Function](#return-a-pointer-to-function))

## Pointers to Functions

- a pointer that denotes a function rather than an object
- points to a particular type (like any other pointer)
- a **function's type** is determined by 
  - its return type and 
  - the types of its parameters
  - note: the function's name is **not** part of its type

```cpp
// compares lengths of two strings
bool lengthCompare(const string &, const string &);
```

### Declare

```cpp
// pf points to a function returning bool that takes two const string references
bool (*pf)(const string &, const string &); // uninitialized
```

```cpp
// important: the parentheses around *pf are necessary:
// because the following line declares a function named pf that returns a bool*
bool *pf(const string &, const string &);
```

### Call a Function via a Pointer

- when we use the name of a function as a value, the function is automatically converted to a pointer

```cpp
// this is the pf declared above
pf = lengthCompare; // pf now points to the function named lengthCompare
pf = &lengthCompare; // equivalent assignment: address-of operator is optional
```

- to call the function to which the pointer points (no need to dereference the pointer!)

```cpp
bool b1 = pf("hello", "goodbye"); // calls lengthCompare
bool b2 = (*pf)("hello", "goodbye"); // equivalent call
bool b3 = lengthCompare("hello", "goodbye"); // equivalent call
```

### Assignment

- there is **no implicit conversion** between pointers to one function type and pointers to another function type
- as usual, we can **assign** 
  - `nullptr` or 
  - a zero-valued integer constant expression

```cpp
string::size_type sumLength(const string&, const string&);
bool cstringCompare(const char*, const char*);
pf = 0;               // ok: pf points to no function
pf = sumLength;       // error: return type differs
pf = cstringCompare;  // error: parameter types differ
pf = lengthCompare;   // ok: function and pointer types match exactly
```

### Overloading

The compiler uses the **type of the pointer** to determine which overloaded function to use
- the type of the pointer **must match** one of the overloaded functions **exactly**!

```cpp
// declare a pointer to an overloaded function
void ff(int*);
void ff(unsigned int);
void (*pf1)(unsigned int) = ff; // pf1 points to ff(unsigned)

// the type of the pointer must match one of the overloaded functions exactly
void (*pf2)(int) = ff;      // error: no ff with a matching parameter list
double (*pf3)(int*) = ff;   // error: return type of ff and pf3 don't match
```

### Return a Function

- we can't return a function type but **can** return a **pointer to** a function type

### Return a Pointer to Function

- we can't return a function type but **can** return a **pointer to** a function type
- the easiest way to declare a function that returns a pointer to function is by using a **type alias**:

```cpp
// alias declarations for function types
using F = int(int*, int);       // F is a function type, not a pointer
using PF = int(*)(int*, int);   // PF is a pointer type

PF f1(int);   // ok: PF is a pointer to function; f1 returns a pointer to function
F f1(int);    // error: F is a function type; f1 can’t return a function
F *f1(int);   // ok: explicitly specify that the return type is a pointer to function
```

- other ways to return a pointer to function:

```cpp
// declare f1 directly
int (*f1(int))(int*, int);      // f1 returns a pointer, pointer points to a function, that function returns an int

// using a trailing return
auto f1(int) -> int (*)(int*, int);
```

- we can use `decltype` to simplify writing a function pointer return type
  - remember, since `decltype` returns a function type, not a **pointer to** function type, we must add a `*`:

```cpp
string::size_type sumLength(const string&, const string&);
string::size_type largerLength(const string&, const string&);
// depending on the value of its string parameter,
// getFcn returns a pointer to sumLength or to largerLength
decltype(sumLength) *getFcn(const string &);
```

### Pointers to Member Functions

#### Example

```cpp
// https://stackoverflow.com/a/1779703
//
// Just like .*, ->* is used with pointers to members.

#include <iostream>

struct foo {
    void bar(void) { std::cout << "foo::bar" << std::endl; }
    void baz(void) { std::cout << "foo::baz" << std::endl; }
};

int main(void) {
    foo *obj = new foo;
    void (foo::*ptr)(void);

    ptr = &foo::bar;
    (obj->*ptr)();
    ptr = &foo::baz;
    (obj->*ptr)();
    return 0;
}
```

#### Best practice

From [isocpp](https://isocpp.org/wiki/faq/pointers-to-members#typedef-for-ptr-to-memfn):

**Always** use a `typedef`:

```cpp
// sample class
class Fred {
public:
  int f(char x, float y);
  int g(char x, float y);
  int h(char x, float y);
  int i(char x, float y);
  // ...
};

// - FredMemFn is the type name
// - a pointer of that type points to any member of Fred that takes (char,float), such as Fred's f, g, h and i.
typedef  int (Fred::*FredMemFn)(char x, float y);  // Please do this!
```

With that you can write:

```cpp
// declare a member-function pointer
int main()
{
  FredMemFn p = &Fred::f;
  // ...
}

// declare functions that receive member-function pointers
void userCode(FredMemFn p)
{ /*...*/ }

// declare functions that return member-function pointers
FredMemFn userCode()
{ /*...*/ }
```

#### `.*` vs `->*`

Member access operators: [cppreference](https://en.cppreference.com/w/cpp/language/operator_member_access)

From [isocpp](https://isocpp.org/wiki/faq/pointers-to-members#dotstar-vs-arrowstar):

- use `.*` when the left-hand argument is a **reference to an object**, and `->*` when it is a **pointer to an object**

```cpp
class Fred { /*...*/ };

// this typedef is explained in section "Pointers to Member Functions"
typedef  int (Fred::*FredMemFn)(int i, double d);  // use a typedef!!! please!!!

void sample(Fred x, Fred& y, Fred* z, FredMemFn func)
{
  x.*func(42, 3.14);
  y.*func(42, 3.14);
  z->*func(42, 3.14);
}
```

**Examples:**

```cpp
// https://stackoverflow.com/a/1779703
// "What is ->* operator in C++?"

// Just like .*, ->* is used with pointers to members

#include <iostream>

struct foo {
    void bar(void) { std::cout << "foo::bar" << std::endl; }
    void baz(void) { std::cout << "foo::baz" << std::endl; }
};

int main(void) {
    foo *obj = new foo;         // we need an object (or pointer to an object) to call the member function later
    void (foo::*ptr)(void);     // declare a pointer-to-member-function

    ptr = &foo::bar;            // assign bar() to the pointer
    (obj->*ptr)();              // call the member function "bar" on the object to which "obj" points
    ptr = &foo::baz;
    (obj->*ptr)();
    return 0;
}
```

```cpp
// https://stackoverflow.com/a/6586248
// "What are the pointer-to-member operators ->* and .* in C++?"

//we have a class
struct X
{
   void f() {}
   void g() {}
};

typedef void (X::*pointer)();
//ok, let's take a pointer and assign f to it.
pointer somePointer = &X::f;
//now I want to call somePointer. But for that, I need an object
X x;
//now I call the member function on x like this
(x.*somePointer)(); //will call x.f()
//now, suppose x is not an object but a pointer to object
X* px = new X;
//I want to call the memfun pointer on px. I use ->*
(px ->* somePointer)(); //will call px->f();
```

# Function Objects

- one kind of [callable object](#callable-objects)

Lippman 14.8:

- Function Objects are "objects of classes that define the **call operator**"
  - Such objects "act like functions" because we can call them (see `absObj(i)`)
  - Calling a function object runs its overloaded call operator

```cpp
struct absInt {
  int operator()(int val) const {
    return val < 0 ? -val : val;
  }
};

int i = -42;
absInt absObj;        // object that has a function-call operator
int ui = absObj(i);   // passes i to absObj.operator()
```

## Usage

- Function-object classes often contain data members that are used to **customize the operations in the call operator**:

```cpp
class PrintString {
public:
  PrintString(ostream &o = cout, char c = ' '):
  os(o), sep(c) { }
  void operator()(const string &s) const { os << s << sep; }
private:
  ostream &os;  // stream on which to write
  char sep;     // character to print after each output
};

// Then we can
// either use the defaults 
// or supply our own values for the separator or output stream
PrintString printer;  // uses the defaults; prints to cout
printer(s);           // prints s followed by a space on cout
PrintString errors(cerr, '\n');
errors(s);            // prints s followed by a newline on cerr
```

- Function objects are most often used **as arguments to the generic algorithms**:

```cpp
for_each(vs.begin(), vs.end(), PrintString(cerr, ’\n’));
```

## Lambdas

- lambdas are function objects

## Library-defined Function Objects

- a set of class templates that represent the arithmetic, relational, and logical operators
- each class defines a **call operator** that applies the named operation
- defined in the `functional` header

```cpp
plus<int> intAdd;         // function object that can add two int values
negate<int> intNegate;    // function object that can negate an int value
// uses intAdd::operator(int, int) to add 10 and 20
int sum = intAdd(10, 20);         // equivalent to sum = 30
sum = intNegate(intAdd(10, 20));  // equivalent to sum = -30
// uses intNegate::operator(int) to generate -10 as the second parameter
// to intAdd::operator(int, int)
sum = intAdd(10, intNegate(10));  // sum = 0
```

### For Overriding the Default Algorithms

- eg. to sort a vector in **descending** order (instead of ascending order)

```cpp
// passes a temporary function object that applies the > operator to two strings
sort(svec.begin(), svec.end(), greater<string>());
```

- unlike their built-in operator counterparts, the library function objects will work for pointers, too
  - eg. to sort a vector of pointers based on their addresses in memory
    - this is not possible with the `<` operator (because comparing two unrelated pointers using `<` is undefined)

```cpp
// "<" operator VS. "less<T>" function object

vector<string *> nameTable; // vector of pointers
// error: the pointers in nameTable are unrelated, so < is undefined
sort(nameTable.begin(), nameTable.end(),
      [](string *a, string *b) { return a < b; });
// ok: library guarantees that less on pointer types is well defined
sort(nameTable.begin(), nameTable.end(), less<string*>());
```

- similarly, the **associative containers** use `less<key_type>` to order their elements
  - as a result, we can
    - define a `set` of pointers
    - use a pointer as the key in a `map`

# Callable Objects

- examples:
  - functions
  - pointers to functions
  - lambdas
  - objects created by `bind`
  - classes that overload the function-call operator ([Function Objects](#function-objects))
- a callable object has a **type**
  - eg.
    - each **lambda** has its own unique (unnamed) class type
    - **function** and **function-pointer** types vary by their return type and argument types

## Call Signature

- specifies
  - the type returned by a call to the object
  - the argument type(s) that must be passed in the call
- two callable objects with different **types** may share **the same** call signature
- a call signature corresponds to a **function type**

```cpp
int(int, int)
```

## Different Types Can Have the Same Call Signature

```cpp
// all of these callables share the same call signature: int(int, int)

// ordinary function
int add(int i, int j) { return i + j; }
// lambda, which generates an unnamed function-object class
auto mod = [](int i, int j) { return i % j; };
// function-object class
struct divide {
  int operator()(int denominator, int divisor) {
    return denominator / divisor;
  }
};
```

## `function` Class Template

### Motivation: Function Table

**Example:**
- build a simple desk calculator
  - implement a **function table** using a `map`
    - use a `string` as the **key**
    - use the function that implements each operator as the **value**
    - index the `map` to call a specific operator

```cpp
// maps an operator to a pointer to a function taking two int s and returning an int
map<string, int(*)(int,int)> binops;

// ok: add is a pointer to function of the appropriate type
binops.insert({"+", add}); // {"+", add} is a pair

// error: mod is a lambda, and each lambda has its own class type
binops.insert({"%", mod}); // error: mod is not a pointer to function
```

### `function`

- is a [class template](https://en.cppreference.com/w/cpp/utility/functional/function)
- creates a `function` type
- defined in the `functional` header
- must specify the **call signature** of the objects that this particular function type can represent:

```cpp
// to represent callable objects that return an int result and have two int parameters
function<int(int, int)>
```

```cpp
// use that type
function<int(int, int)> f1 = add;               // function pointer
function<int(int, int)> f2 = divide();          // object of a function-object class
function<int(int, int)> f3 = [](int i, int j)   // lambda
                             { return i * j; };
cout << f1(4,2) << endl; // prints 6
cout << f2(4,2) << endl; // prints 2
cout << f3(4,2) << endl; // prints 8
```

- this solves the problem in section ["Motivation: Function Table"](#motivation-function-table)

```cpp
// table of callable objects corresponding to each binary operator
// all the callables must take two ints and return an int
// an element can be a function pointer, function object, or lambda
map<string, function<int(int, int)>> binops;

map<string, function<int(int, int)>> binops = {
    {"+", add},                                 // function pointer
    {"-", std::minus<int>()},                   // library function object
    {"/", divide()},                            // user-defined function object
    {"*", [](int i, int j) { return i * j; }},  // unnamed lambda
    {"%", mod} };                               // named lambda object

// note: underlying callable objects all have different types from one another

// When we index binops, we get a reference to an object (not a copy of an 
// object) of type function. The function type OVERLOADS the call operator.
binops["+"](10, 5); // calls add(10, 5)
binops["-"](10, 5); // uses the call operator of the minus<int> object
binops["/"](10, 5); // uses the call operator of the divide object
binops["*"](10, 5); // calls the lambda function object
binops["%"](10, 5); // calls the lambda function object
```

### Overloaded Functions and `function`

**Problem:**

- we cannot (directly) store the name of an **overloaded** function in an object of type `function`

```cpp
// "add" is an overloaded function
int add(int i, int j) { return i + j; }
Sales_data add(const Sales_data&, const Sales_data&);

map<string, function<int(int, int)>> binops;
binops.insert( {"+", add} ); // error: which add? (-> ambiguous name)
```

**Solution:**

```cpp
// Option 1:
// ok: store a function pointer
int (*fp)(int,int) = add;     // pointer to the version of add that takes two ints
binops.insert( {"+", fp} );   // ok: fp points to the right version of add

// Option 2:
// ok: use a lambda to disambiguate which version of add we want to use
binops.insert( {"+", [](int a, int b) {return add(a, b);} } );
```
