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

## scope

cppreference:
- **scope**: Each **name** that appears in a C++ program is only visible in some possibly discontiguous portion of the source code called its scope.
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

## inline functions

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

Parameter names declared in function declarations are usually for only self-documenting purposes. They are used (but remain optional) in function definitions.

The type of each function parameter in the parameter list is determined according to the following rules:
1) First, `decl-specifier-seq` and the `declarator` are combined as in any **declaration** (&rarr; see "Definitions") to determine the type.
2) If the type is "array of T" or "array of unknown bound of T", it is replaced by the type "pointer to T".
3) If the type is a function type F, it is replaced by the type "pointer to F".
4) **Top-level cv-qualifiers** are dropped from the parameter type (This adjustment only affects the function type, but doesn't modify the property of the parameter: `int f(const int p, decltype(p)*);` and `int f(int, const int*);` declare the same function).

Because of these rules, the following function declarations declare exactly the same function:

```cpp
int f(char s[3]);
int f(char[]);
int f(char* s);
int f(char* const);
int f(char* volatile s);
```

The following declarations also declare exactly the same function:

```cpp
// case 3): function type
int f(int());
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

## return values

ignoring return values:
- "calling a function and ignoring the return result is *very* common", [stackoverflow](https://stackoverflow.com/a/38919156)
- eg. `printf("hello\n");` ignores the return value, [stackoverflow](https://stackoverflow.com/a/38919103)

## static functions

- see [static members](#static-members)
- see [stackoverflow](https://stackoverflow.com/a/15235626)

## member functions

- "member function bodies may use other members of their class regardless of where in the class those members appear" (see "compile order" in [Classes](#classes))
- "code is interpreted as being inside the scope of the class" (ie. eg. no need to use `this` to access members)

### implicit "this" parameter

- a `total.isbn()` call is translated like a `Sales_data::isbn(&total)` call
  - "the compiler passes the address of `total` to the implicit `this` parameter" (as if `Sales_data::isbn(Sales_data *const this)` were the function definition, see `passing_by_reference.c` - Listing 9.6)

### const member functions

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

[stackoverflow](https://stackoverflow.com/a/3141107)
- A "`const` function", denoted with the keyword `const` after a function declaration, makes it a compiler error for this class function to change a member variable of the class. However, reading of a class variables is okay inside of the function, but writing inside of this function will generate a compiler error.

### inline member functions

- member functions ...
  - defined inside the class: automatically inline
  - defined outside the class: need to specify as `inline`
  - declared inside the class: need to specify as `inline`
- inline member functions should be defined in the same header as the corresponding class definition (for the same reason as described for inline functions)
- it is legal to specify `inline` on both the declaration and the definition. 
  - **Best practice**: specifying `inline` only on the definition outside the class can make the class easier to read.

## Lambdas

From [learn.microsoft](https://learn.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-170):
- In C++11 and later, a **lambda expression** - often called **a lambda** - is a convenient way of defining an anonymous function object (a closure) right at the location where it's invoked or passed as an argument to a function. Typically lambdas are used to encapsulate a few lines of code that are passed to algorithms or asynchronous functions.

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
  - if all the arguments have the same type
    - pass a library type named `initializer_list`
  - if the argument types vary
    - write a special kind of function, known as a [variadic template](#variadic-templates)
- ellipsis
  - special parameter type
  - can be used to pass a varying number of arguments
  - should be used only in programs that need to interface to C functions

### `initializer_list`

- a library type 
- a template type
- represents an array of values of the specified type
- defined in the `initializer_list` header
- elements are always `const` values
  - there is no way to change the value of an element
- operations:

```cpp
initializer_list<T> lst; // Default initialization; an empty list of elements of type T.
initializer_list<T> lst{a,b,c...};
                // lst has as many elements as there are initializers; elements are copies of
                // the corresponding initializers. Elements in the list are const.
lst2(lst)   // Copying or assigning an initializer_list does not copy the elements
lst2 = lst  // in the list. After the copy, the original and the copy share the elements.
lst.size()  // Number of elements in the list.
lst.begin() // Returns a pointer to the first and one past the last element in lst.
lst.end()
```

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

## return

- Values are returned in exactly the same way as variables and parameters are initialized: 
  - The **return value** is used to **initialize** a **temporary** at the call site, and that temporary is the result of the function call.

### Return a Reference

Examples:

```cpp
// Return a reference to the shorter of two strings:

// The strings are not copied when the function is called or when the result is returned
const string &shorterString(const string &s1, const string &s2)
{
  return s1.size() <= s2.size() ? s1 : s2;
}

// Never Return a Reference or Pointer TO A LOCAL OBJECT:

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
