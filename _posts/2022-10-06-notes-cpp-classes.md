---
title: "C++ Notes - Objects, Classes"
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
    - classes
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

- "the initialization performed when an object is constructed with an **empty** initializer", i.e. `T()` or `T{}` or `T object{};`
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
- uses the **Copy Constructor**
  - proof: see [Copy Constructor](#copy-constructor) &rarr; "called whenever an object is initialized (by **direct-initialization** ... ) ..."
  - for members of class type, not for built-in types (which do not have constructors)

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
- usually you do **not** do this for static members, but
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

# Classes

- "Classes are user-defined types, defined by class-specifier"
- The **class specifier** has the following syntax: 
  - `class-key attr(optional) class-head-name final(optional) base-clause(optional) { member-specification }`
- `class-key`
  - one of `class`, `struct` and `union`. 
    - The keywords `class` and `struct` are identical except for 
      - the default member access and
      - the default base class access.
    - If it is `union`, the declaration introduces a union type. 
- declaration of **objects**:
  - `class ClassName item1;` (inherited from C) is equivalent to `ClassName item1;`
- **forward declaration:** declare a class without defining it
  - **incomplete type:** After a declaration and before a definition is seen, the class type is an **incomplete type** - it's known that the type is a class type but not known what members that type contains
  - A class must be **defined** - not just declared - before
    - we can write code that **creates objects** of that type
    - a reference or pointer is used to **access a member** of the type
- a class 
  - **cannot** have data members of its own type
  - **can** have data members that are pointers or references to its own type

## Compile Order

compile order: **two steps** 

1. **member declarations**
2. **member function bodies**, if any

Therefore, data members can be used inside member function bodies, **even if** the data members are declared **after** the member function bodies.

## Class Types

### struct

- `struct` is a `class-key` (see above)
- when we define a class intending for all of its members to be `public`, we use `struct`. 
  - If we intend to have `private` members, then we use `class`.

From [stackoverflow](https://stackoverflow.com/a/1127406):
- In C++ the only difference between a `class` and a `struct` is that members and base classes are `private` by default in classes, whereas they are `public` by default in structs.
- So structs can have **constructors**, and the syntax is the same as for classes.

Why use `typedef struct`?

From [Why should we typedef a struct so often in C? - Stack Overflow](https://stackoverflow.com/questions/252780/why-should-we-typedef-a-struct-so-often-in-c):
- no longer have to write `struct` all over the place
- can make the code cleaner since it provides a smidgen **more abstraction**

### union

- `union` is a `class-key` (see above)
- "A `union` is a special **class type** that can hold only one of its non-static data members at a time."
- like a "room in a hotel", [stackoverflow](https://stackoverflow.com/a/2313676)

## Constructors

- "Constructors do not have names" (`[class.ctor]`, N3337)
- "Classes control object **initialization** by defining one or more **special member functions** known as **constructors**."
- order:
  - 1) base members
  - 2) members are initialized in the [constructor initializer list](#constructor-initializer-list)
    - members **omitted** in the constructor initializer list are **default initialized** 
      - proofs: cppreference: 
        - in "Default Initialization": "3) when a base class or a non-static data member is not mentioned in a constructor initializer list and that constructor is called."
        - in "Constructors and member initializer lists": "The **member initializer list** is the place where **non-default initialization** of these (data member) objects can be specified.", implies that default initialization is the default behavior (for omitted data members)
  - 3) the constructor body is executed
    - members can be **assigned** in the constructor body, but they cannot be **initialized** there!
- Constructors 
  - have the same **name** as the class.
  - control 
    - object **initialization**
    - what happens when we **copy**, **assign**, or **destroy** objects of the class type
      - **copy**: 
        - when we initialize a variable or 
        - when we pass or return an object by value
      - **assign**: when we use the assignment operator
      - **destroy**: 
        - local object: destroyed on exit from the block in which it was created
        - Objects stored in a `vector` (or an array): destroyed when that `vector` (or array) is destroyed
      - If we do **not** define these operations, the compiler will **synthesize** them for us. 
        - the versions that the compiler generates for us execute by copying, assigning, or destroying **each member** of the object
        - **Warning:** "classes that manage dynamic memory, generally **cannot** rely on the synthesized versions of these operations"
          - but if you use `vector` and `string` to manage dynamic memory the synthesized versions for copy, assignment and destruction will work
  - are special member functions, but
    - have no return type
    - must not be declared as `const`
- a class may have multiple constructors (&rarr; overloading)
- can write to `const` objects during their construction

### Default Constructor

- "a constructor which can be called with no arguments" (cppreference)
- "called during 
  - **default initializations** and 
  - **value initializations**"
- 1) **explicitly defined default constructor**
- 2) **synthesized default constructor** (implicitly defined default constructor)
  - the compiler **implicitly** defines this constructor, if we do not explicitly define one
    - but **only if** a class declares **NO** constructors
  - member initialization
    - if there is an **in-class initializer**, use it to initialize the member
    - else **default initialize** the member
- situations in which we **must** explicitly define a default constructor:
  - 1. we have defined at least one constructor, and therefore, the compiler will not generate a default constructor
  - 2. to make sure that **class members** of **built-in types** or **compound types** cannot end up **uninitialized**
    - **in blocks**: objects of built-in or compound type that are defined inside a block have **undefined value** when they are default initialized (see ["Default Initialization"](#default-initialization))
  - 3. when the compiler cannot synthesize a default constructor
- defaulted default constructor
  - C++11
  - we can ask the compiler to generate the constructor for us by writing `= default` after the parameter list
    - this constructor does exactly the same work as the synthesized default constructor
- **best practice:**
  - "it is almost always right to provide a default constructor if other constructors are being defined."
- see [default arguments](#default-arguments):
  - "A constructor that supplies default arguments **for all its parameters** also defines the default constructor." (7.5.1 "Default Arguments and Constructors")

### Constructor Initializer List

In cppreference: **"member initializer list"**
- "The body of a function definition of any constructor, before the opening brace of the compound statement (see [statements](#statements)), may include the **member initializer list**, whose syntax is the colon character `:`, followed by the comma-separated list of one or more member-initializers, each of which has the following syntax ..."
- "Before the compound statement that forms the function body of the constructor begins executing, initialization of all direct bases, virtual bases, and non-static data members is finished. The **member initializer list** is the place where **non-default initialization** of these objects can be specified."
  - thus, all members omitted in the member initializer list are **default initialized**

Omitted Members:
- When a member is omitted from the constructor initializer list, it is implicitly initialized **using the same process as is used by the synthesized default constructor** (7.1.4 "Constructors" &rarr; "Constructor Initializer List")
  - **best practice:** 
    - constructors should "use an in-class initializer if one exists and gives the member the correct value", i.e. "constructors should not override in-class initializers"
    - **phth**: if there are no in-class initializers for a class **or** your compiler does not support in-class initializers, then **built-in types** must be explicitly initialized in the constructor initializer list (because otherwise they will be uninitialized, see "synthesized default constructor")
- "If we do not explicitly initialize a member in the constructor initializer list, that member is **default initialized** before the constructor body starts executing" (7.5.1 "Constructors Revisited" &rarr; "Constructor Initializer List")

Initialization vs Assignment:
- if you do not use constructor initializers, you do not **"initialize"** the members, but you **"assign"** values to the members (recall, in C++: assign != initialize) (7.5.1 "Constructors Revisited" &rarr; "Constructor Initializer List")

When constructor initializer is **required**:
- "We **must** use the constructor initializer list to provide values for members that are ...
  - `const`, 
  - reference, or 
  - of a `class` type that does not have a default constructor."
- **best practice**: By routinely using constructor initializers, you can avoid being surprised by compile-time errors when you have a class with a member that **requires** a constructor initializer

**best practice**:
- order of member initialization
  - write constructor initializers in the same order as the members are declared. 
  - when possible, avoid using members to initialize other members.
  - write member initializers to use the constructor's parameters rather than another data member from the same object (see example in 7.5.1)

### Delegating Constructor

- "In a delegating constructor, the **member initializer list** has a single entry that is **the name of the class itself**. Like other member initializers, the name of the class is followed by a parenthesized list of arguments. The argument list must match another constructor in the class."

### Converting Constructor

- **Implicit Class-Type Conversion**
- cppreference: "Implicit conversion is defined in terms of copy-initialization: if an object of type T can be copy-initialized with expression E, then E is **implicitly convertible** to T."
- "A constructor that can be called with a **single argument** defines an implicit conversion **from** the constructor's parameter type **to** the class type."
- **only one** implicit class type conversion is allowed (p.295)

#### explicit

- **disable** the implicit conversion by declaring the converting constructor as `explicit`
  - you can use `explicit` on constructors with **a single argument** only
  - use `explicit` only on the constructor declaration inside the class
  - `explicit` constructors can be used only with **direct initialization** (see Example 1)
    - because copy initialization triggers an implicit class type conversion which the `explicit` does not allow
- "use `explicit` for constructors that take a single argument unless there is a good reason not to" (BS6.1.2)

**Example 1**:

```cpp
Sales_data item1(null_book); // ok: direct initialization
// error: cannot use the copy form of initialization with an explicit constructor
Sales_data item2 = null_book;
```

**Example 2**:
- the `vector` constructor that takes a single size parameter is `explicit`

```cpp
void f(vector<int>); // f’s parameter is copy initialized
f(10); // error: can’t use an explicit constructor to copy an argument
f(vector<int>(10)); // ok: directly construct a temporary vector from an int
```

## Copy Control

- controlled by 5 **special member functions** ("copy-control members"):
  - copy constructor, 
  - copy-assignment operator, 
  - move constructor, 
  - move-assignment operator, and 
  - destructor
- "If a class does not define all of the copy-control members, the compiler **automatically defines** the missing operations"
  - **member-wise**: "the versions that the compiler generates for us execute by copying, assigning, or destroying **each member** of the object."

### Copy Constructor

- used for **initialization** (direct and copy forms), whereas the Copy-Assignment Operator is used for **assignment**

Lippman, 13.1.1 "The Copy Constructor"
- "A constructor is the copy constructor if 
  - its **first parameter** is a **reference to the class type** (eg. `T&`, `const T&`) and 
  - any **additional parameters** have default values"
- "almost always a reference to `const`" (although it can be a reference to nonconst)
- "the copy constructor usually should **not** be `explicit`" 
  - because it is often used for **implicit conversion** (see [converting constructor](#converting-constructor))

```cpp
class Foo {
public:
  Foo();             // default constructor
  Foo(const Foo&);   // copy constructor
  // ...
};
```

- "called whenever an object is initialized (by **direct-initialization** or **copy-initialization**) from another object of the same type, which includes"
  - **initialization**: `T a = b;` or `T a(b);`, where `b` is of type `T`;
  - **function argument passing**: `f(a);`, where `a` is of type `T` and `f` is `void f(T t);`
  - **function return**: `return a;` inside a function such as `T f()`, where `a` is of type `T`, which has no move constructor. 
- "the compiler is permitted (but not obligated) to skip the copy/move constructor and create the object directly" (see Example 1)

Example 1:

```cpp
string null_book = "9-999-99999-9"; // copy initialization
// may be rewritten to
string null_book("9-999-99999-9"); // compiler omits the copy constructor
```

Defined as deleted (see p.508, 538), if
- the class has a member whose own copy constructor is deleted or inaccessible. 
- the class has a member with a deleted or inaccessible destructor.
- see "synthesized as deleted" in section ["delete"](#delete)
  - in essence, if a class has a **data member** that **cannot** be default constructed, copied, assigned, or destroyed, then the corresponding copy-control member will be a deleted function
- we define a move constructor

#### Synthesized Copy Constructor

- for some classes disallows copying objects of that class type
- copies the members of its argument into the object being created **memberwise**
- unlike the synthesized default constructor, it is synthesized **even if** we define other constructors
- the synthesized constructor is **equivalent to** using a **constructor initializer list**, see example on p.497
- **always** uses **direct initialization**
  - if the synthesized copy constructor belongs to a `class` or `struct`, but not if it belongs to a `union`
  - **cppreference**: "For non-union class types (`class` and `struct`), the constructor performs full member-wise copy of the object's bases and non-static members, in their initialization order, using **direct initialization**."
- type of member determines how the member is copied
  - **class type**: copied by the class' copy constructor (using direct initialization)
  - **built-in type**: copied directly (direct initialization)
  - **array**: copied elementwise (using direct initialization)
- problem with pointer members:
  - see [Memberwise Assignment Example](https://www.cs.mtsu.edu/~xyang/2170/copyconstructor.html)
    - In class `StudentTestScores` the member `testScores` is a pointer. In `StudentTestScores student2 = student1;` `student2`'s `testScroes` member simply gets a **copy of the address** stored in `student1`'s `testScores` member. Both pointers will point to the same address.

### Copy-Assignment Operator

- "a (...) member function with the name `operator=` that takes **exactly one parameter** of type `T`, `T&`, `const T&`"

```cpp
class-name & class-name ::operator= ( const class-name & )   // (2)
```

- used for **assignment**, whereas the Copy Constructor is used for **initialization**
- "called whenever selected by overload resolution, e.g. when an object appears on the left side of an assignment expression"

Defined as deleted (see p.508, 538), if
- a member has a deleted or inaccessible copy-assignment operator, or 
- the class has a `const` or reference member.
- see "synthesized as deleted" in section ["delete"](#delete)
  - in essence, if a class has a **data member** that **cannot** be default constructed, copied, assigned, or destroyed, then the corresponding copy-control member will be a deleted function
- we define a move-assignment operator

#### Synthesized Copy-Assignment Operator

```cpp
T& T::operator=(const T&)
// or:
T& T::operator=(T&)
```

- for some classes disallows assignment
- analogue to synthesized copy constructor
- memberwise: **assign** each member of rhs object to lhs object (using copy-assignment operator for the type of that member)
  - arrays: elementwise
- returns a **reference to** its lhs object (via `return *this`)

#### Valuelike Copy-Assignment Operator

- **combine** the actions of the **destructor** and the **copy constructor**
- must handle self-assignment
  - always create a local temporary before using `delete` (see example below)
- must be exception safe
  - ie. will leave the left-hand operand in a sensible state should an exception occur
    - If an exception occurs, it will happen **before** we have changed the left-hand operand.
- a good pattern to use:

```cpp
HasPtr& HasPtr::operator=(const HasPtr &rhs)
{
  auto newp = new string(*rhs.ps);  // copy the underlying string (into a local temporary)
  delete ps;                        // free the old memory
  ps = newp;                        // copy data from rhs (now in the temporary) into this object
  i = rhs.i;
  return *this;                     // return this object
}
```

#### Pointerlike Copy-Assignment Operator

Ways to implement:
1. easiest way: use `shared_ptrs` to manage the resources in the class.
2. Sometimes we want to manage a resource directly. In such cases, it can be useful to use a **reference count**.

Reference Counting:
1. In addition to initializing the object, each constructor (other than the copy constructor) creates a counter (which is in stored in dynamic memory).
2. The copy constructor increments the **shared** counter.
3. The destructor decrements the counter.
4. The copy-assignment operator increments the right-hand operand's counter and decrements the counter of the left-hand operand. If the counter for the left-hand operand goes to zero, the copy-assignment operator must destroy the state of the left-hand operand.

```cpp
HasPtr& HasPtr::operator=(const HasPtr &rhs)
{
  ++*rhs.use;                   // increment the use count of the right-hand operand
  if (--*use == 0) {            // then decrement this object's counter
    delete ps;                  // if no other users
    delete use;                 // free this object's allocated members
  }
  ps = rhs.ps;                  // copy data from rhs into this object
  i = rhs.i;
  use = rhs.use;
  return *this;                 // return this object
}
```

#### Copy-and-Swap Assignment Operator

- classes **that manage resources** often also define a function named `swap`
  - particularly important for classes that we plan to use with **algorithms** that reorder elements
    - If a class defines its own `swap`, then the algorithm uses that class-specific version.
    - Otherwise, it uses the `swap` function defined by the library.
- the library `swap` makes unnecessary copies, so define a custom version for your class

The idea:

```cpp
// the library's "swap":
// will make three copy operations
// (none of these 3 memory allocations is necessary)
HasPtr temp = v1;     // make a temporary copy of the value of v1 (1st copy of the string that was originally in v1)
v1 = v2;              // assign the value of v2 to v1 (a copy of the string that was originally in v2)
v2 = temp;            // assign the saved value of v1 to v2 (2nd copy of the string that was originally in v1)

// better, our "swap":
// only "swap the pointers"
string *temp = v1.ps;   // make a temporary copy of the pointer in v1.ps
v1.ps = v2.ps;          // assign the pointer in v2.ps to v1.ps        
v2.ps = temp;           // assign the saved pointer in v1.ps to v2.ps  
```

A `swap` function and Copy-and-Swap Assignment Operator for the `HasPtr` class:

```cpp
class HasPtr {
  friend void swap(HasPtr&, HasPtr&);
  // other members as in § 13.2.1 (p. 511)
};
inline
void swap(HasPtr &lhs, HasPtr &rhs)
{
  using std::swap;        // does NOT hide the declarations for the HasPtr version of swap
                          // - type-specific version of swap will be a better match
                          // - library version of swap will be used, if there is no type-specific version
  swap(lhs.ps, rhs.ps);   // swap the pointers, not the string data
  swap(lhs.i, rhs.i);     // swap the int members
}

// note rhs is passed by value, which means the HasPtr copy constructor
// copies the string in the right-hand operand into rhs
HasPtr& HasPtr::operator=(HasPtr rhs)
{
  // swap the contents of the left-hand operand with the local variable rhs
  swap(*this, rhs);   // rhs now points to the memory this object had used
  return *this;       // rhs is destroyed, which deletes the pointer in rhs
}
```

Assignment operators that use copy and swap ...
- are automatically **exception safe**
  - ie. will leave the left-hand operand in a sensible state should an exception occur
    - The only code that might throw is the `new` expression inside the copy constructor.
    - If an exception occurs, it will happen **before** we have changed the left-hand operand.
- correctly handle **self-assignment**

### Destructor

```cpp
~ class-name ();
```

- "is the complement of a constructor" (BS5.2.2)
- "called when the lifetime of an object ends."
- "The purpose of the destructor is to free the resources that the object may have acquired during its lifetime."
- no return value
- takes no parameters
- order:
  - 1) function body is executed
    - typically, frees resources an object allocated during its lifetime
  - 2) (nonstatic) members are destroyed
    - in reverse order from the order in which they were initialized
- destruction part is **implicit**
  - cannot control how members are destroyed
    - whereas **for constructors** constructor initializer lists **can** control how members are initialized
- what happens when a member is destroyed:
  - **class type**: 
    - running the member's own destructor
  - **built-in types**:
    - do not have destructors, so nothing is done
  - **built-in pointer type**: 
    - does not `delete` the object to which that pointer points
      - **Note:** if you want to `delete` the object use **smart pointers** (which are class types)
- called when
  - **variables**: when they go out of scope
  - **members** of an object: when the object is destroyed
  - **elements of a container**: when a container is destroyed
  - **dynamically allocated objects**: `delete` is applied to a pointer to the object
  - **temporaries**: at the end of the creating expression
- not called when 
  - a **reference** to an object goes out of scope
  - a **pointer** to an object goes out of scope

Defined as deleted (p. 508), if
- the class has a member whose own destructor is deleted or is inaccessible (e.g., `private`).
- see "synthesized as deleted" in section ["delete"](#delete)
  - in essence, if a class has a **data member** that **cannot** be default constructed, copied, assigned, or destroyed, then the corresponding copy-control member will be a deleted function

#### Synthesized Destructor

- is automatically defined for any class that does not define its own destructor
- for some classes disallows destruction
- otherwise, the synthesized destructor has an empty function body
- members are automatically destroyed **AFTER** the (empty) destructor body is run
  - **Important**: Members are destroyed as part of the implicit destruction phase **that follows** the destructor body
- example
```cpp
// no work to do other than destroying the members, which happens automatically AFTER the destructor body
~Sales_data() { }   // equivalent to the synthesized `Sales_data` destructor
```

#### Destructors throwing Exceptions

- destructors should never throw exceptions that the destructor itself does not handle
  - **best practice:** if a destructor does an operation that might throw, it should wrap that operation in a `try` block and handle it **locally to** the destructor.
- in practice, because destructors free resources (and do not allocate &rarr; `bad_alloc` exception), it is **unlikely** that they will throw exceptions
  - All of the standard library types guarantee that their destructors will not raise an exception

### Rule of Zero/Three/Five

- **Rule of Zero/Three/Five:** define all of the copy-control members or none (using the default for all)
  - it is unusual to need one without needing to define them all
- **rule 1**: "Classes That Need Destructors Need Copy and Assignment"
  - decide first whether the class needs a destructor (often more obvious than the need for copy constructor or assignment operator)
  - if the class needs a destructor, it almost surely needs a copy constructor and copy-assignment operator as well
- **rule 2**: "Classes That Need Copy Need Assignment, and Vice Versa"
- why these rules? &rarr; read examples in the book
- **rule 3**: "When a class has a pointer member, it is usually a good idea to be explicit about copy and move operations" (BS6.1.1)
  - "Classes that define their own copy constructor and copy-assignment operator generally also benefit by defining the move operations" (Lippman)

### Moving Objects

- useful if
  - an object is immediately destroyed after it is copied
    - here, moving can provide a significant performance boost compared to copying
  - for classes that have a **resource** that may not be shared
    - Hence, objects of these types **cannot** be copied but can be moved
    - examples 
      - the IO classes (resource: an IO buffer)
      - `unique_ptr` class (resource: a pointer)
- copying is expensive if the objects
  - are large
  - themselves require memory allocation (e.g., `strings`)
- examples of classes that support 
  - move as well as copy: `string`, `shared_ptr`
  - move, but not copy: IO classes, `unique_ptr`
- [Rvalue References](#rvalue-references) were introduced by the new standard to support move operations

```cpp
// Rvalues Are Moved, Lvalues Are Copied
// - constructor is chosen by ordinary function matching
StrVec v1, v2;
v1 = v2;                      // v2 is an lvalue; copy assignment
StrVec getVec(istream &);     // getVec returns an rvalue
v2 = getVec(cin);             // getVec(cin) is an rvalue; move assignment (because StrVec&& is an exact match)

// But Rvalues Are Copied If There Is No Move Constructor
// - even if we attempt to move them by calling std::move
class Foo {
  public:
    Foo() = default;
    Foo(const Foo&); // copy constructor
    // other members, but Foo does not define a move constructor
};
Foo x;
Foo y(x);               // copy constructor; x is an lvalue
Foo z(std::move(x));    // copy constructor, because there is no move constructor (no problem, this is safe!)
```

### Both Move Operations

The move constructor and move-assignment operator are defined as **deleted**, if
- in general, a move operation is never implicitly defined as a **deleted** function
- if we explicitly ask the compiler to generate a move operation by using `= default`, and the compiler is unable to move all the members, then the move operation will be defined as **deleted**
- rules (see p.538) for when a **synthesized** move operation is defined as deleted are analogous to those for the copy operations with **one exception**
  - **the exception**: if the class has a member that 
    - defines its own copy constructor but does not also define a move constructor, or 
    - doesn't define its own copy operations and for which the compiler is unable to synthesize a move constructor.
  - otherwise, see "synthesized as deleted" in section ["delete"](#delete)
    - in essence, if a class has a **data member** that **cannot** be default constructed, copied, assigned, or destroyed, then the corresponding copy-control member will be a deleted function

```cpp
// assume Y is a class that defines its own copy constructor but not a move constructor
struct hasY {
  hasY() = default;
  hasY(hasY&&) = default;
  Y mem; // hasY will have a deleted move constructor
};
hasY hy, hy2 = std::move(hy); // error: move constructor is deleted
```

### Move Constructor

- "a non-template constructor whose first parameter is `T&&`, `const T&&`, (...) and either there are no other parameters, or the rest of the parameters all have default values"
- "The move constructor is typically called when an object is initialized (by **direct-initialization** or **copy-initialization**) from rvalue (xvalue or prvalue) (...) of the same type, including
  - **initialization**: `T a = std::move(b);` or `T a(std::move(b));`, where `b` is of type `T`;
  - **function argument passing**: `f(std::move(a));`, where a is of type `T` and `f` is `void f(T t);`
  - **function return**: `return a;` inside a function such as `T f()`, where a is of type `T` which has a move constructor."
- "Move constructors typically "steal" the resources held by the argument (e.g. pointers to dynamically-allocated objects, file descriptors, TCP sockets, I/O streams, running threads, etc.) rather than make copies of them, and leave the argument in some **valid** but otherwise indeterminate state."
  - **valid object**: see [move assignment operator](#move-assignment-operator)

Lippman
- 1st parameter is an **rvalue reference** to the class type
- additional parameters must all have default arguments (like for the copy constructor)
- **destructible state:** must ensure that the moved-from object is left in a state such that destroying that object will be "harmless"
  - original object must no longer point to those moved resources
  - **phth:** eg. by resetting pointer members of the **moved-from** object (= the original object) to `nullptr`, otherwise, when the **moved-from** object gets destroyed, the destructor of the **moved-from** object will `delete` the memory to which the **moved-to** object's (= the copied object's) pointer members point
- unlike a copy constructor, the move constructor **does not** itself **allocate** any resources, it **takes over** resources
  - thus, will not throw any exceptions

```cpp
StrVec::StrVec(StrVec &&s) noexcept // move won't throw any exceptions
  // member initializers take over the resources in s
  : elements(s.elements), first_free(s.first_free), cap(s.cap)
{
  // leave s in a state in which it is safe to run the destructor
  s.elements = s.first_free = s.cap = nullptr;
}
```

- `noexcept`
  - a way to promise that a function does not throw any exceptions
  - move constructors and move assignment operators that cannot throw exceptions **should** be marked as `noexcept`
  - why is `noexcept` needed?
    - unless the library knows that our move constructor will not throw, it will do **extra work** to cater to the possibliity that moving an object of our class type might throw.
    - without `noexcept` (specified on the element type's move constructor), in circumstances such as `vector<elementType>` reallocation, eg. when we call `push_back`, `vector<elementType>` **MUST** use `elementType`'s copy constructor instead of `elementType`'s move constructor
      - why "MUST"? - because `vector` guarantees that if an exception happens when we call `push_back`, the `vector` itself will be left unchanged (however, using a move constructor changes the `vector` itself and may possibly throw an exception)
        - (similarly, other **library containers** also provide guarantees as to what they do if an exception happens)
      - `vector` reallocation happens 
      - more detailed explanation (see p.536)

```cpp
// must specify on both:
// - the declaration in the class header
// - the definition if that definition appears outside the class
class StrVec {
  public:
    StrVec(StrVec&&) noexcept; // move constructor
    // other members as before
};
StrVec::StrVec(StrVec &&s) noexcept : /* member initializers */
{ /* constructor body */ }
```

BS
- a move constructor is supposed to remove ("steal") the value from its argument.
- Examples
  - an integer returned by a function call is never used again, so you can safely "steal" its value/resources/state
- After a move, the moved-from object should be in a state that allows a **destructor** to be run

#### std::move

- calls the move constructor
  - eg. "initialization: `T a = std::move(b);` or `T a(std::move(b));`, where `b` is of type `T`"
- **explicitly casts** an lvalue to its corresponding rvalue reference type
- in the `utility` header
- returns an rvalue reference to its given object
  - "It is exactly equivalent to a `static_cast` to an rvalue reference type." (cppreference)
- **problem:** we cannot directly bind `rr1` to `&&rr2`
  - **solution:** we can explicitly cast `rr1` using `std::move`:

```cpp
int &&rr1 = 42;             // ok: literals are rvalues
int &&rr2 = rr1;            // error: the expression rr1 is an lvalue!
int &&rr3 = std::move(rr1); // ok
```

- after a call to `std::move` 
  - we **cannot** use 
    - `rr1`
    - the value of a moved-from object
  - we **can** 
    - assign to it
    - destroy it
- **best practice:** 
  - always call `std::move` not `move`
  - do not use `std::move` casually:
    - Casually used in ordinary user code (as opposed to class implementation code), moving an object is more likely to lead to mysterious and hard-to-find bugs than to any improvement in the performance of the application.
  - Outside of class implementation code such as **move constructors** or **move-assignment operators**, use `std::move` only when you are certain 
    - that you need to do a move and 
    - that the move is guaranteed to be safe
      - we must be absolutely certain that there can be no other users of the moved-from object

BS
- `std::move`
- doesn't actually move anything
- returns an **rvalue reference** (a reference to its argument from which we may move)
- it is a kind of cast

#### Synthesized Move Constructor

Conditions under which the compiler synthesizes:
- **copy**:
  - recall, **always** synthesized (if we do not declare our own **copy constructor** or **copy-assignment operator**)
- **move**:
  - are **not** synthesized, if
    - a class defines its own copy constructor, copy-assignment operator, or destructor
      - thus, some classes do not have a move constructor or a move-assignment operator &rarr; copy is used in place of move
  - are synthesized, **only** if
    - the class doesn't define any of its own copy-control members **and** every nonstatic data member of the class **can be moved**
      - The compiler **can move**
        - members of built-in type.
        - members of a class type **if** the member's class has the corresponding move operation

```cpp
// the compiler will synthesize the move operations for X and hasX
struct X {
  int i;          // built-in types can be moved
  std::string s;  // string defines its own move operations
};
struct hasX {
  X mem;          // X has synthesized move operations
};
X x, x2 = std::move(x);         // uses the synthesized move constructor
hasX hx, hx2 = std::move(hx);   // uses the synthesized move constructor
```

### Move-Assignment Operator

- "a non-template non-static member function with the name `operator=` that takes exactly one parameter (...) of type `T&&,` `const T&&`, `volatile T&&`, or `const volatile T&&`."

```cpp
class-name& class-name::operator=(class-name&&) 	// (1)
```

- **called when the rhs is an rvalue:**
  - "is called whenever it is selected by overload resolution, e.g. when an object appears on the left-hand side of an assignment expression, where the **right-hand side is an rvalue** of the same or implicitly convertible type."

Lippman:
- A move assignment is defined similarly (as the move constructor)
  - it is supposed to remove the value from its argument
- does the same work as the **destructor** and the **move constructor**
- if it will not throw any exceptions, we **should** make it `noexcept` 
  - to prevent library containers from automatically using the copy constructor in circumstances such as `vector` reallocation (see section ["Move Constructor"](#move-constructor))
- like a copy-assignment operator
  - must guard against self-assignment

```cpp
StrVec &StrVec::operator=(StrVec &&rhs) noexcept
{
  // direct test for self-assignment "if (this != &rhs)":
  // why is this test necessary?
  // - when "a = a;", then the move assignment operator would NOT be called because the rhs "a" is an lvalue.
  // - when "a = std::move(a);", then the move assignment operator would be called because the rhs "std::move(a)" is an rvalue. 
  //   - in this case, "this == &r" would be true, so that the if condition "if (this != &rhs)" is necessary to prevent this form of self-assignment
  if (this != &rhs) {
    free();                     // free existing elements
    elements = rhs.elements;    // take over resources from rhs
    first_free = rhs.first_free;
    cap = rhs.cap;
    // leave rhs in a destructible state
    rhs.elements = rhs.first_free = rhs.cap = nullptr;
  }
  return *this;
}
```

**requirements** (like the move constructor):
1. **destructible state:** must ensure that the moved-from object is in a state in which the destructor can be run
  - eg. by setting the pointer members of the moved-from object to `nullptr`
2. **valid:** must guarantee that the object remains valid
  - **valid object:** one that can safely be given a new value or used in other ways that do not depend on its current value
  - example: 
    - move from a `string` or container object: 
      - we know that the moved-from object remains valid. 
        - As a result, we can run operations such as as `empty` or `size` on moved-from objects.
      - However, we **don't know** what result we'll get.
      - therefore, programs should never depend on the value of a moved-from object!

#### Copy-and-Swap Assignment and Move

The assignment operator has a nonreference parameter, which means the parameter is **copy initialized**.
- Depending on the type of the argument, copy initialization uses either the copy constructor or the move constructor; lvalues are copied and rvalues are moved. 
- As a result, this single assignment operator **acts as both** the copy-assignment and move-assignment operator.

```cpp
class HasPtr {
  public:
    // added move constructor
    HasPtr(HasPtr &&p) noexcept : ps(p.ps), i(p.i) {p.ps = 0;}
    // assignment operator is both the move- and copy-assignment operator
    HasPtr& operator=(HasPtr rhs)
                { swap(*this, rhs); return *this; }
    // other members as in § 13.2.1 (p. 511)
};
```

## Access Control and Encapsulation

- "encapsulate" the implementation = "hide" the implementation

### Access Specifiers

access Specifiers (to enforce encapsulation, i.e. hiding of implementation details)
- `public` members: defines the interface
- `private` members: encapsulate (i.e., hide) the implementation

The only difference between `struct` and `class` is the **default access level**:
- If we use the `struct` keyword, the members defined before the first access specifier are `public`
- if we use `class`, then the members are `private`

Best practice:
- When we define a class intending for all of its members to be `public`, we use `struct`. 
- If we intend to have `private` members, then we use `class`.

Benefits of Encapsulation:
- User code cannot inadvertently corrupt the state of an encapsulated object.
- The implementation of an encapsulated class can change over time without requiring changes in user-level code.

### Friends

- A class can allow another class or function to **access its nonpublic members** by making that class or function a `friend`
  - `friend` declarations 
    - may appear only inside a class definition
    - may appear anywhere in the class
  - `friend` function definitions
    - `friend` function can be **defined** inside the class body (7.3.4)
      - such functions are implicitly `inline`
- friends
  - are not members
  - are not affected by the access control of the section in which they are declared
- A friend declaration **is not a general declaration** of the function. 
  - we must also declare the function **separately** from the friend declaration (some compilers do not require this, but this is best practice)
  - **best practice**:
    - group `friend` declarations together at the beginning or end of the class definition
    - in addition to the `friend` declaration, declare each friend (outside the class) in the same header as the class
- friendship is not transitive

## Member Function

### Special Member Function

source: cppreference

**special member function:** Some member functions are **"special"**: under certain circumstances they are defined by the compiler even if not defined by the user. They are:
- Default constructor
- Copy constructor 
- Move constructor 
- Copy assignment operator 
- Move assignment operator 
- Destructor 

**defaulted function:** Special member functions are the only functions that can be **"defaulted"**, that is, defined using `= default` instead of the function body. (from: [cppreference: member functions](https://en.cppreference.com/w/cpp/language/member_functions))

### Functions that return *this

Such functions 
- return a **reference** to the object on which they are called
- are **lvalues**, which means that they return the object itself, **not a copy** of the object.

**Motivation**: Without such functions we cannot execute a **sequence** of operations **on the same object**, such as

```cpp
// move the cursor to a given position, and set that character
myScreen.move(4,0).set('#');
```

- you can also call a member function based on whether 

## Data Member

## Type Member

- a class can define its own (local) types
  - this type names may be either `public` or `private`
- unlike ordinary members, members that define types **must appear before they are used**
  - **best practice**: as a result, type members usually appear **at the beginning of the class**

## ::ClassName

From [stackoverflow](https://stackoverflow.com/questions/4269034/what-is-the-meaning-of-prepended-double-colon):

This ensures that resolution occurs from the global namespace, instead of starting at the namespace you're currently in. For instance, if you had two different classes called `Configuration` as such:

```cpp
class Configuration; // class 1, in global namespace
namespace MyApp
{
    class Configuration; // class 2, different from class 1
    function blah()
    {
        // resolves to MyApp::Configuration, class 2
        Configuration::doStuff(...) 
        // resolves to top-level Configuration, class 1
        ::Configuration::doStuff(...)
    }
}
```

Basically, it allows you to traverse up to the global namespace since your name might get clobbered by a new definition inside another namespace, in this case `MyApp`.

## Constructors throwing Exceptions

From [isocpp](https://isocpp.org/wiki/faq/exceptions#selfcleaning-members):
- Every data member inside your object should clean up its own mess
- If a constructor throws an exception, the object's destructor is not run
  - Therefore, if your object has already done something that needs to be undone (such as allocating some memory, opening a file, or locking a semaphore), this "stuff that needs to be undone" must be remembered by a data member inside the object.

From [isocpp](https://isocpp.org/wiki/faq/exceptions#ctors-can-throw):
- Note: if a constructor finishes by throwing an exception, the memory associated with the object itself is cleaned up — there is no memory leak. 
- For example:

```cpp
void f()
{
  X x;             // If X::X() throws, the memory for x itself will not leak
  Y* p = new Y();  // If Y::Y() throws, the memory for *p itself will not leak
}
```

related: [Handle a Constructor that fails](#handle-a-constructor-that-fails)

