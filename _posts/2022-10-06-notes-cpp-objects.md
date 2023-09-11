---
title: "C++ Notes - Objects"
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
    - objects
    - notes

---

# Variables, Objects

"A region of memory that can contain data and has a type." (Lippman)

- cppreference: "A **variable** is an object or a reference that is not a non-static data member, that is introduced by a declaration."

Lippman
- C++ programmers use the terms "variable" and "object" interchangeably.
- variable expressions are **lvalues**
- a variable is an expression with one operand and no operator

## Construction Order

- construction order
  - base classes (first to last)
  - data members (in declaration order)
  - constructor body
- destruction order
  - reverse of construction order (due to stack memory)

## Initialization

- in C++: initialization != assignment
  - **Initialization** happens when a variable is given a value **when it is created**. 
  - **Assignment** obliterates an object's current value and replaces that value with a new one.
- sources
  - see [learn.microsoft.com](https://learn.microsoft.com/en-us/cpp/cpp/initializers?view=msvc-170)

### Default Initialization

- "the initialization performed when an object is constructed with **no** initializer", i.e. `T object;` or `new T`
- "performed in three situations"
  - 1) when a variable (...) is declared with no initializer `T object;`
  - 2) when an object (...) is created by a `new`-expression with no initializer `new T`
  - 3) when a (...) data member is **not mentioned** in a **constructor initializer list** and that constructor is called
- **precisely:**
  - if `T` is a **class type**, 
    - the constructors are considered and subjected to overload resolution against the empty argument list. 
    - The constructor selected (which is **one of** the **default constructors**) is called to provide the initial value for the new object; 
  - if `T` is an **array type**, 
    - every element of the array is default-initialized; 
  - otherwise, eg. **built-in type**, no initialization is performed
    - this includes raw pointers, eg. `MyClass *a;` does not initialize `a`, [stackoverflow](https://stackoverflow.com/a/37638617)

Examples:
- object of **built-in type**, **compound type** (eg. pointer to built-in type)
  - outside a block (global scope): **0**, **nullptr**
  - inside a block (local scope): **uninitialized/undefined** 
    - **best practice**:
      - every built-in type object, compound type object should be (in-class) initialized
      - provide a default constructor
- object of **class type**
  - value defined by the class
    - some classes **require** that every object be explicitly initialized

### Value Initialization

- the initialization performed when an object is constructed with an **empty** initializer, i.e. 
  - 1) `T()`, `T{}` (nameless temporary)
  - 4) `T object{};` (named object)
- performed in these situations
  - 1) when a **nameless temporary** object is created with the initializer consisting of an empty pair of parentheses `T()` or braces `T{}`;
  - 2) when an object with dynamic storage duration is created by a **new-expression** with the initializer consisting of an empty pair of parentheses `new T()` or braces `new T{}`;
  - 3) when a non-static data member or a base class is initialized using a **member initializer** with an empty pair of parentheses `Class::Class(...) : member () { ... }` or braces `Class::Class(...) : member {} { ... }`;
  - 4) when a named object (automatic, static, or thread-local) is declared with the initializer consisting of a pair of braces. (since C++ 11)
- **roughly**: 
  - default initializes class types, 
  - zero-initializes + default initializes all others
  - therefore, safer than default initialization
- **precisely**:
  - if `T` is a **class type** with a ...
    - "user-provided default constructor" **or** "no default constructor": the object is [default-initialized](#default-initialization);
    - "implicitly-defined" **or** "defaulted default constructor": all of the following 3 steps are performed in the following order:
      - 1) the object is [zero-initialized](#zero-initialization),
      - 2) the semantic constraints for default-initialization are checked, and 
      - 3) if `T` has a non-trivial default constructor, the object is default-initialized;
  - if `T` is an **array type**, 
    - each element of the array is value-initialized;
  - otherwise, eg. **built-in type**, the object is [zero-initialized](#zero-initialization).
- "References cannot be value-initialized"
- "All standard containers (`std::vector`, `std::list`, etc.) value-initialize their elements when constructed with a single `size_type` argument"
- Lippman p.132: "Initialization in which **built-in types** are initialized to zero and **class types** are initialized by the class's default constructor."
  - "Objects of a **class type** can be value initialized *only if* the class has a default constructor."
  - "Used to initialize a **container**'s elements when a size, but not an element initializer, is specified."
    - "Elements are initialized as **a copy of** this compiler-generated value."

### Zero-initialization

- "Sets the initial value of an object to zero."
- zero-initialization "does not have a dedicated syntax in the language", but other types of initialization perform zero-initialization
- "If `T` is a **scalar type**, the object is initialized to the value obtained by **explicitly converting** the integer literal `0` (zero) to `T`"
  - scalar types: "object types that are not array types or class types"
    - eg. pointers, arithmetic types (integral types, floating-point types), etc.

```cpp
// explicit conversions of "0":
// https://stackoverflow.com/a/21387175
long *a = 0;           // ok, 0 is a null pointer constant
long *b = (long *)0;   // ok, (long *)0 is a null pointer with appropriate type
// invalid in C++:
long *c = (void *)0;   // ok in C, invalid conversion in C++
// In both C and C++, (int *)0 is a constant expression whose value is a null pointer. It is not, however, a "null pointer constant".
long *d = (int *)0;    // invalid conversion in both C and C++
```

- merke: "null pointer constant" != "constant expression whose value is a null pointer"
  - this difference usually only matters for the assignments in the code above, [stackoverflow](https://stackoverflow.com/a/21387175))
  - **Null pointer constant**: "Null pointer constant is either `nullptr` (or any other prvalue of type `std::nullptr_t`), or integer literal of value `0`.", [stackoverflow](https://stackoverflow.com/a/59990291)

### Copy Initialization

```cpp
T object = other; 	// (1) 	
T object = {other}; 	// (2) 	(until C++11)
f(other) 	        // (3) 	
return other; 	        // (4) 	
```

- "Initializes an object **from another object**.", i.e. `T object = other;`
- we can supply only a **single initializer**
- happens when using `=`

```cpp
string s1;            // default initialization; s1 is the empty string
string s2 = s1;       // s2 is a copy of s1
string s3 = "hiya";   // s3 is a copy of the string literal
int units_sold = 0;
// not "Copy Initialization"
int units_sold = {0}; // since C++11 this is classified as "copy-list-initialization"
```

- copy initialization usually uses the **copy constructor**, but sometimes it uses the **move constructor**
  - cppreference: "If `other` is an **rvalue expression**, a move constructor will be selected by overload resolution and called during copy-initialization. This is still considered copy-initialization; there is no special term (e.g., move-initialization) for this case."
- happens when
  - when we define variables using an `=`
  - functions
    - (Syntax 3) **pass** an object as **an argument to a parameter** of nonreference type ("pass by value")
    - (Syntax 4) **return** an object **from a function** that has a nonreference return type ("return by value")
  - brace initialization of
    - arrays
    - members of an aggregate class
  - container 
    - initialization
    - when we `insert` or `push` a container member

### Direct Initialization

- "Initializes an object from explicit set of constructor arguments."
- Syntax: see "Case 1, 2, etc." below
- uses the **Copy Constructor** (sometimes)
  - proof: see [Copy Constructor](#copy-constructor) &rarr; "called whenever an object is initialized (by **direct-initialization** ... ) ..."
  - for members of class type, not for built-in types (which do not have constructors)
  - direct initialization selects by overload resolution, therefore, **any** other constructor (eg. move constructor, one of the converting constructors, etc.) may be selected as well!

**Difference**: Direct vs Copy Initialization:
- **direct initialization**: asks the compiler to "use ordinary function matching to **select the constructor** that best matches the arguments we provide"
- **copy initialization**: asks the compiler to "copy the right-hand operand into the object being created, converting that operand if necessary"

**Difference**: Direct vs Direct-list Initialization:
- only for **single non-class type initializer**: is both **direct-list-initialization** and **direct initialization**
  - common example: `int object{3}`
  - the difference is well explained here: [stackoverflow](https://stackoverflow.com/a/71994272)

#### Case 1

Happens when we omit the `=`

```cpp
// Examples
string s4(10, 'c');   // s4 is cccccccccc
int units_sold(0);
int units_sold{0};    // - special case of "direct initialization" according to Case 4 (as long as the initializer is non-class type)
                      // - special case of "direct-list-initialization" with one element
```

#### Case 2

Happens when initializing with a nonempty **parenthesized** list:
- "1) Initialization with a nonempty **parenthesized** list of expressions or braced-init-lists" (ie. `arg` can be a braced-init-list, but it must be enclosed by parentheses `()`!).

```cpp
T object ( arg );               // (1)

T object ( arg1, arg2, ... );   // (1)
```

#### Case 3

Happens in **constructor initializer lists**:
- "6) Initialization of a **base** or a **non-static member** by constructor initializer list."
  - better explanation: [learn.microsoft.com](https://learn.microsoft.com/en-us/cpp/cpp/initializers?view=msvc-170#direct-initialization)
  - here, **"base"** refers to the `BaseClass` when using inheritance (see **Example 2**)
    - because when using inheritance `BaseClass(initializer)` is in the constructor initializer list (like a member that is initialized)

```cpp
// "member" is direct initialized
Class::Class() : member( args, ... ) { ... } 	// (6)
```

**Example 2**:

```cpp
class DerivedClass : public BaseClass{
public:
    // BaseClass and m_char are direct initialized
    DerivedClass(int n, char c) : BaseClass(n), m_char(c) {}

    ...
}
```

#### Case 4

**Single** brace-enclosed initializer for **built-in** objects:
- "2) Initialization of an object of **non-class type** with a single brace-enclosed initializer"
  - **Note:** if the object is of class type, this would be **list-initialization**, or more precisely **direct-list-initialization**

```cpp
T object { arg };
```

### In-class Initialization

When we create objects, the in-class initializers will be used to initialize the data members. 

- "Initializer provided as part of the declaration of a class data member."
- C++11
- **Members without an initializer** are default initialized
- When we provide an in-class initializer, we must do so 
  - following an `=` sign (copy initialization) **or**
  - inside curly braces `{}` (direct initialization)
  - **not:** initializer in parentheses

### List Initialization

- C++11
- "Initializes an object from **braced-init-list**.", "that is, a possibly empty brace-enclosed list of expressions or nested braced-init-lists"
- 2 syntaxes:
  - **Direct-list-initialization**
    - 1) `T object { arg1, arg2, ... };`, "initialization of a named variable with a braced-init-list (that is, a possibly empty brace-enclosed list of expressions or nested braced-init-lists)"
      - thus, "Direct Initialization" &rarr; "Case 4" is a special case of "Direct-list-initialization"
  - **Copy-list-initialization**
    - 6) `T object = { arg1, arg2, ... };`, "initialization of a named variable with a braced-init-list after an equals sign"

ADVANTAGES:
- The compiler will not let us list initialize variables of **built-in type** if the initializer might lead to the loss of information

```cpp
long double ld = 3.1415926536;

// a: both "direct-list-initialization" and "direct initialization -> Case 4"
// b: both "copy-list-initialization" and "copy initialization"
int a{ld}, b = {ld}; // error: narrowing conversion required
// c: direct initialization
// d: copy initialization
int c(ld), d = ld;   // ok: but value will be truncated
```

VECTORS:
- when we **supply a list of (vector) element values** we can only do this by using list initialization
  - We cannot supply a list of initializers using parentheses:
```cpp
vector<string> v1{"a", "an", "the"}; // list initialization
vector<string> v2("a", "an", "the"); // error
```

### Aggregate Initialization

C++11:
- a form of **list initialization** (since C++11)
- "Initializes an aggregate from an initializer list."
  - `T object = { arg1, arg2, ... };`,
  - `T object{ arg1, arg2, ... };` (since C++11)
- An **aggregate** is one of the following types:
  - array type
  - class type (typically, `struct` or `union`), that has
    - no user-provided, inherited, or explicit constructors
    - no private or protected non-static data members
    - no base classes
    - no virtual member function
    - no default member initializers (in-class initializers)

### Reference Initialization

- "Binds a reference to an object."
- "A reference to T can be initialized with an object of type T, a function of type T, or an object implicitly convertible to T."
- "Once initialized, a reference **cannot be reseated (changed)** to refer to another object."

see [cppreference](https://en.cppreference.com/w/cpp/language/reference_initialization):

1) When a named **lvalue reference** variable is declared with an initializer
```cpp
T& ref = target ;
T& ref = { arg1, arg2, ... }; 
T& ref ( target );
T& ref { arg1, arg2, ... }; 
```

2) When a named **rvalue reference** variable is declared with an initializer
```cpp
T&& ref = target ;
T&& ref = { arg1, arg2, ... }; 
T&& ref ( target );
T&& ref { arg1, arg2, ... }; 
```

3) In a function call expression, when the function parameter has reference type
4) In the `return` statement, when the function returns a reference type
5) When a non-static data member of reference type is initialized using a member initializer

- "If the initializer is a braced-init-list, rules of **list initialization** are followed."
- "Otherwise, if the reference (`ref`) is an **lvalue reference**: ..." 
  - If `target` is an **lvalue expression**, and its **type** is `T` or derived from `T`, and is equally or less cv-qualified, 
    - then the reference is bound to the object identified by the lvalue or to its base class subobject.
  - Otherwise, if the **type** of `target` is **not** same or derived from `T`, and `target` **has conversion function** to an lvalue whose type is either `T` or derived from `T`, equally or less cv-qualified, 
    - then the reference is bound to the object identified by the lvalue returned by the conversion function (...)

## Declaration vs Definition
 
- **"separate compilation"**: split programs into several files, each of which can be compiled independently.
- To support separate compilation, C++ distinguishes between declarations and definitions. 
  - A **declaration** makes a name known to the program. A file that wants to use a name defined elsewhere includes a declaration for that name. 
  - A **definition** creates the associated entity.
- A **variable declaration** specifies the type and name of a variable. 
- A **variable definition** is a declaration. 
  - In addition to specifying the name and type, a definition also **allocates storage** and may provide the variable with an **initial value**.
- Variables must be **defined exactly once** but can be **declared many times**.
- To use the same variable in multiple files, we must define that variable in one - and only one - file. Other files that use that variable must declare - but not define - that variable

### extern

- `extern`: To obtain a declaration that is **not** also a definition, we add the `extern` keyword and must not provide an explicit initializer.
- inside a function
  - It is an error to provide an initializer on an `extern` 
- outside a function 
  - we can initialize an `extern`, however, this overrides the `extern`. The `extern` becomes a definition in this case.

## Static typing

- C++ is a statically typed language, which means that **types are checked at compile time**. 
- The process by which types are checked is referred to as **type checking**.
- consequence of static typing: **must declare the type** of a variable before we can use that variable

## this

like `self` in Python:
- [stackoverflow](https://stackoverflow.com/questions/22526153/self-of-python-vs-this-of-cpp-c), inside the class block `self` must be written **explicitly** each time, whereas `this` can be dropped
- [stackoverflow](https://stackoverflow.com/a/61240703), `this->member = 4;` equals `(*this).member = 4;` equals `member = 4;`

p.257-258: "Defining Member Functions", "Introducing this", "Introducing `const` Member Functions"
- member functions access the object on which they were called through an implicit parameter `this`
  - when a member function is called (eg. `total.isbn()`), `this` is initialized with the address of the object on which the function was invoked (like `Sales_data::isbn(&total)`, where `isbn()` is defined as `Sales_data::isbn(Sales_data *const this)`)
- the `this` parameter is defined implicitly. Thus, it is legal, although unnecessary, to define

```cpp
struct Sales_data {
  ...
  std::string isbn() const { return bookNo; }
  ...
}
```

as

```cpp
struct Sales_data {
  ...
  std::string isbn() const { return this->bookNo; }
  ...
}
```

Type of `this`:
- default:
  - "`this` is a `const` pointer to the non`const` version of the class type" 
    - **phth**: eg. `this` is the "const pointer" `Sales_data *const` (it is not a "pointer to const" `const Sales_data *const`)
- in `const` member function blocks:
  - the keyword `const` that follows the parameter list modifies the type of the implicit `this` pointer:
    - A `const` following the parameter list indicates that `this` is a pointer to `const` (see [pointer to const](#pointer-to-const)).

## const objects

- may call only `const` member functions
- a `const` object does not assume its "constness" until after the constructor completes the object's initialization

## pointer to const

```cpp
const double pi = 3.14;
double *ptr = &pi;          // error: ptr is a plain pointer (see (2))
const double *cptr = &pi;
*cptr = 42;                 // error: cannot assign to *cptr (see (1))
```

- (1) a pointer to `const` may not be used to change the object to which the pointer points
- (2) we may store the address of a `const` object **only** in a pointer to `const`
- we can use a pointer to `const` to point to a non`const` object (exception to rule: "types of a pointer and the object to which it points must match")
- a pointer to `const` says nothing about whether the object to which the pointer points is `const` (like a [reference to const](#reference-to-const))
  - think of them as pointers or references "that *think* they point or refer to `const`"

## const pointer

```cpp
int errNumb = 0;
int *const curErr = &errNumb;   // curErr will always point to errNumb
```

- its value (i.e., the address that it holds) may not be changed
- The fact that a pointer is itself `const` says nothing about whether we can use the pointer to change the underlying object
- "unlike references, pointers are objects" &rarr; pointers can be `const`, references cannot

## static

- [stackoverflow](https://stackoverflow.com/a/15235626)

### static members

- members that are associated with the class, rather than with individual objects of the class type
- basically, like any other member
  - can be `public` or `private`
  - type of a `static` data member can be `const`, reference, array, class type
  - etc ...
- unlike other members
  - exist outside any object
    - objects do **not** contain data associated with static data members
  - static members are **shared by all** the objects of the class type
  - static member functions
    - do not have a `this` pointer
    - may not be declared as `const`

**Declaration:**

```cpp
class Account {
  public:
    void calculate() { amount += amount * interestRate; }
    static double rate() { return interestRate; }
    static void rate(double);
  private:
    std::string owner;
    double amount;
    static double interestRate;
    static double initRate();
};
```

**Definition:**

```cpp
// static member functions:
// - do not repeat the "static" keyword outside the class body
void Account::rate(double newRate)
{
  interestRate = newRate;
}

// static data members:
// - not defined when we create objects
// - not initialized by the class' constructors
// - must be 
//   - defined and initialized OUTSIDE the class body
//   - declared, but NOT initialized inside the class body!
//   - defined outside ANY function (like global variables)
double Account::interestRate = initRate();   // define and initialize a static class member
// Once the class name is seen, the remainder of the definition is in the scope of the class
// -> we can use "initRate" without qualification (even though it is private)
```

**best practice:**
- put the definition of `static` data members in the same file that contains the definitions of the class noninline member functions 
  - avoids double definitions

**in-class initialization:**

Usually you do **not** do this for static members, but
- we can provide in-class initializers for static members that have `const` integral type
- must do so for static members that are `constexpr`s of literal type
  - the initializers must be constant expressions
  - such members are themselves constant expressions
- **important:** if an initializer is provided inside the class, the member's definition must not specify an initial value:

```cpp
// definition of a static member with no initializer
constexpr int Account::period; // initializer provided in the class definition
```

**best practice:**
- Even if a `const` static data member is initialized in the class body, that member ordinarily should be defined outside the class definition.

### Usage

```cpp
double r;
r = Account::rate();  // access a static member using the scope operator

Account ac1;
Account *ac2 = &ac1; 
// equivalent ways to call the static member rate function
r = ac1.rate();       // through an Account object or reference
r = ac2->rate();      // through a pointer to an Account object

class Account {
  public:
    // Member functions can use static members directly, without the scope operator
    void calculate() { amount += amount * interestRate; }
  private:
    static double interestRate;
    // remaining members as before
};
```

Can be used in ways that would be illegal for nonstatic members:
```cpp
// static members can have incomplete types
class Bar {
  public:
    // ...
  private:
    static Bar mem1;  // ok: static member can have incomplete type
    Bar *mem2;        // ok: pointer member can have incomplete type
    // error:
    Bar mem3;         // error: data members must have complete type
 };

// static members can be used as a default argument
class Screen {
  public:
    // bkground refers to the static member
    // declared LATER in the class definition
    Screen& clear(char = bkground);
  private:
    static const char bkground;
};
```

## Temporaries

- aka **"temporary object"**
- "Unnamed object created by the compiler while evaluating an expression." (Lippman)
  - "A temporary exists until the end of the largest expression that encloses the expression for which it was created."
- All temporary objects are **destroyed** as the last step in evaluating the **full-expression** that (lexically) contains the point where they were created, [cppreference](https://en.cppreference.com/w/cpp/language/lifetime#Temporary_object_lifetime)
  - and if multiple temporary objects were created, they are destroyed in the order opposite to the order of creation.
  - This is true even if that evaluation ends in throwing an exception.
  - **phth:** ie. in "expression statements" (where the "full-expression" is the **outermost** expression **without** the semicolon) temporaries are destroyed right before the outermost ";"

### Created When ...

Examples when temporaries are **created** automatically by the compiler: [cppreference](https://en.cppreference.com/w/cpp/language/lifetime#Temporary_object_lifetime)

### In Implicit Conversions

```cpp
double dval = 3.14;
const int &ri = dval;
```

is translated by the compiler like

```cpp
double dval = 3.14;
const int temp = dval;
const int &ri = temp;   // bind "ri" to the temporary "temp"
```

- This happens not only for type `int` temporaries but also for **class type** temporaries (given the class has a corresponding [converting constructor](#converting-constructor))
  - see `Sales_data::combine(const Sales_data &rhs)` call on p.295, 7.5.4
    - a temporary of type `const Sales_data &rhs` is created from a `string`

### Unnamed Temporaries

- unnamed temporaries are **value initialized**
  - cppreference "value initialization": "1,5) when a nameless temporary object is created with the initializer consisting of an empty pair of parentheses or braces (since C++11);"

Creating **unnamed temporary objects**, [stackoverflow](https://stackoverflow.com/a/18892056):
- the object is destroyed immediately after ";".
- memory is allocated (automatically, on the stack) for this temp object and it's freed (again automatically) after ";". 
- the **constructor** and **destructor** are called, as expected.

```cpp
// note: this does not explicitly call the constructor, instead this line creates a temporary unnamed object with type "Demo", which is destroyed immediately after ";"

#include<iostream.h>

class Demo
{
    public :

    Demo()
    {
        cout<<"\nIn Demo const";
    }
    ~Demo()
    {
        cout<<"\nin demo dest";
    }
};

void main() {
    Demo();
}
```

## Lifetimes

From `[basic.life]`:

The lifetime of an object of type T **begins** when:
- storage with the proper alignment and size for type T is obtained, **and**
- its initialization (if any) is complete (including vacuous initialization), (...)

The lifetime of an object o of type T **ends** when:
- if T is a **non-class type**, the object is destroyed, or
- if T is a **class type**, the destructor call starts, or
- the storage which the object occupies is 
  - released, or is 
  - reused by an object that is not nested within o.

**Resources** are bound to the **lifetime** of an object in [RAII](#raii)
- "RAII is a C++ programming technique which **binds** the **life cycle** of a **resource** that must be acquired before use to the **lifetime** of an object." (cppreference)
  - eliminating **"naked new"** expressions

- **Global objects:** 
  - allocated at program start-up and 
  - destroyed when the program ends
- **Local, automatic objects:** 
  - created and destroyed when the block in which they are defined is entered and exited
- **Local static objects:**
  - [cppreference](https://en.cppreference.com/w/cpp/language/storage_duration#Static_local_variables)
  - allocated before their first use ("are initialized the first time control passes through their declaration")
  - and are destroyed when the program ends ("The destructor for a block-scope static variable is called at program exit, but only if the initialization took place successfully.")
- **Dynamically allocated objects:** 
  - allocated at run time, ie. the program controls their lifetimes
  - they exist until they are explicitly freed
  - lifetime is independent of where they are created
- **Temporaries:**
  - "A temporary exists until the end of the largest expression that encloses the expression for which it was created." (Lippman)
    - **phth:** ie. gets destroyed before the ";"

## Anonymous Objects

- aka **nameless temporary** object

From [anonymous objects](https://www.learncpp.com/cpp-tutorial/anonymous-objects/):
- In C++, anonymous objects are primarily used either **to pass or return values** without having to create lots of **temporary** variables to do so.
  - can make the program shorter, cleaner, and generally easier to follow
- Memory allocated dynamically is also done so anonymously 
  - (which is why its address must be assigned to a pointer, otherwise we'd have no way to refer to it).
- It is also worth noting that because anonymous objects have expression scope, they can only be used once (unless bound to a constant l-value reference, which will extend the lifetime of the temporary object to match the lifetime of the reference). 
- If you need to reference a value in multiple expressions, you should use a **named** variable instead.

N3337: in `[class.temporary]`:

Example: Consider the following code:

```cpp
class X {
  public:
    X(int);
    X(const X&);            // X has no move constructor
    X& operator=(const X&);
    ~X();
};
class Y {
  public:
    Y(int);
    Y(Y&&);                 // Y has no copy constructor
    ~Y();
};

X f(X);
Y g(Y);

void h() {
  X a(1);
  X b = f(X(2));    // pass X(2) to f() using copy constructor
  Y c = g(Y(3));    // pass Y(3) to g() using move constructor
  a = f(a);
}
```

- An implementation might use a temporary in which to construct `X(2)` before passing it to `f()` using `X`'s **copy constructor**;
  - alternatively, `X(2)` might be constructed in the space used to hold the argument. ("copy elision")
- Likewise, an implementation might use a temporary in which to construct `Y(3)` before passing it to `g()` using `Y`'s **move constructor**; 
  - alternatively, `Y(3)` might be constructed in the space used to hold the argument. ("copy elision")
- Also, a temporary might be used to hold the result of `f(X(2))` before copying it to b using `X`'s **copy constructor**; 
  - alternatively, `f()`’s result might be constructed in `b`. ("copy elision")
- Likewise, a temporary might be used to hold the result of `g(Y(3))` before moving it to c using `Y`'s **move constructor**; 
  - alternatively, `g()`’s result might be constructed in `c`. ("copy elision")
- On the other hand, the expression `a=f(a)` requires a temporary for the result of `f(a),` which is then assigned to `a`.

