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

- templates are **not** functions or classes (Lip)
- templates "can be thought of as instructions to the compiler for generating classes or functions" (Lip)
- **instantiation**: the process that the compiler uses to create classes or functions from templates (Lip)
- are **recipes** to make things, e.g. classes or functions, with information to be specified later (slides)
- are a **parametrized** description (slides)

## Why use Templates?

- templates protect from **code duplication**
  - thus, templates can make the code shorter and more manageable
- by using templates, we can avoid the use of **type erasure** or **macros** for generic programming
- templates provide support for **parameterized types** (see "Definitions" &rarr; "Parameterized Types")

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

### Template Parameter Name

cppreference:

- The name of the parameter is **optional**:

```cpp
// Declarations of the templates shown above:
template<class>
class My_vector;
template<class = void>
struct My_op_functor;
template<typename...>
class My_tuple;
```

- In the body of the template declaration, the name of a type parameter is a **typedef-name** which aliases the type supplied when the template is instantiated. 

## Definition

- **template definition**: 
  - the declaration of a class template or function template is called a **definition** if it has a body (VJ10.2)
  - declaration and the definition of a given template must have the same number and kind (i.e., type or nontype) of parameters

## typename

- to explicitly tell the compiler that the name is a type

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

### typename vs class

- `class` can be misleading (not only class types can be substituted for `T`), you should prefer the use of `typename`
- `typename` came relatively late in the evolution of the C++98 standard

## Default Template Arguments

- possible for both function and class templates
- a template parameter may have a default argument only if all of the parameters **to its right** also have default argument
  - **for function templates:** default value can be anywhere (given the rest can be deduced)
- Whenever we use a **class template**, we must **always** follow the template's name with angle brackets (unlike for function templates)
  - because TAD works only for function templates
    - though **since C++17** there is "Class template argument deduction (CTAD)"
  - if we want to use the defaults, we must put an **empty bracket pair** following the template's name

### Examples

#### Function Template

- **Goal:** use the type of a **callable object** as default template argument for `compare()`, so that users may supply their own comparison operation but are not required to do so
- `F` represents the type of a **callable object**
- function parameter `f` will be bound to a **callable object**
- `f` will be a default-initialized object of type `F`
- type of `T` is deduced as `Sales_data`
- `F` is deduced as the type of `compareIsbn`
- when `compare` is called with three arguments, 
  - the type of the third argument must be a **callable object** that
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
```

#### Class Template

```cpp
// class templates with default args:

template <class T = int>      // by default T is int
class Numbers {
public:
  Numbers(T v = 0): val(v) { }
  // various operations on numbers
private:
  T val;
};
Numbers<long double> lots_of_precision;
Numbers<> average_precision;  // empty <> says we want the default type
```

## Return Type

### Explicit Template Argument

Lippman:

- useful, **if** we want to let the user control the type of the return

```cpp
// T1 cannot be deduced: it doesn't appear in the function parameter list
template <typename T1, typename T2, typename T3>
T1 sum(T2, T3);

// T1 is explicitly specified; T2 and T3 are inferred from the argument types
auto val3 = sum<long long>(i, lng); // long long sum(int, long)
```

### Trailing Return Type

- since C++11

Lippman:

- useful, **if** we want to determine the return type automatically (eg. from the function's parameters)
- it **can use the function's parameters**
  - because a trailing return appears after the parameter list

Example:

```cpp
// we want to write a function that
// - takes a pair of iterators denoting a sequence and 
// - returns a reference to an element in the sequence
template <typename It>
??? &fcn(It beg, It end)
{
  // process the range
  return *beg; // return a reference to an element from the range
}

vector<int> vi = {1,2,3,4,5};
Blob<string> ca = { "hi", "bye" };
// goal: the user should be able to call fcn without having to 
// specify any explicit template argument (like eg. fcn<double>()):
auto &i = fcn(vi.begin(), vi.end()); // fcn should return int&
auto &s = fcn(ca.begin(), ca.end()); // fcn should return string&
```

**Problem:**

```cpp
// in principle, this is what we want, BUT
// this approach does not work
// because `beg` doesn't exist until the parameter list has been seen
template <typename It>
decltype(*beg) fcn(It beg, It end)
{
  // process the range
  return *beg; // return a reference to an element from the range
}
```

**Solution:**

```cpp
// a trailing return lets us declare the return type after the parameter list is seen
template <typename It>
auto fcn(It beg, It end) -> decltype(*beg)
{
  // process the range
  return *beg; // return a reference to an element from the range
}
// string -> the return type will be string&
// int -> the return will be int&
```

- Here, we returned **by reference**.
- But what if we want to return **by value**?
  - in this case, we must use the type trait `remove_reference`
  - see `fcn2()` in examples in section "`remove_reference` Class Template"

#### `auto` Without Trailing Return Type

- the use of `auto` for the return type **without a corresponding trailing return type** (which would be introduced with a `->` at the end) indicates that the actual return type must be deduced from the **return statements** in the function body (VJ1.3.2)

## Two-Phase Lookup

Templates are "compiled" in two phases:
1. Without instantiation **at definition time** (in 1. `T` is not substituted!), the template code itself is checked for correctness ignoring the template parameters. This includes:
  - **Syntax errors** are discovered, such as missing semicolons.
  - Using **unknown names** (type names, function names, ...) that don't depend on template parameters are discovered.
  - **Static assertions that don't depend on template parameters** are checked.
2. **At instantiation time**, the template code is checked (again) to ensure that all code is valid. That is, now especially, all parts that depend on template parameters are double-checked.

```cpp
template<typename T>
void foo(T t)
{
  undeclared();       // first-phase compile-time error if undeclared() unknown
  undeclared(t);      // second-phase compile-time error if undeclared(T) unknown
  static_assert(sizeof(int) > 10,   // always fails if sizeof(int)<=10, first-phase compile-time error
                "int too small");
  static_assert(sizeof(T) > 10,     // fails if instantiated for T with size <=10, second-phase compile-time error
                "T too small");
}
```

## Type Traits

- aka type transformation
- are class templates (`struct` or `class`)
- in the `type_traits` header
- used for template metaprogramming
- cppreference "Metaprogramming library": "Type traits define compile-time template-based interfaces to query the properties of types."

### `remove_reference` Class Template

- has 
  - one template type parameter
  - a (`public`) type member named `type`
- If we instantiate `remove_reference` with a reference type, then type will be **the referred-to type**

```cpp
remove_reference<int&>        // the `type` member will be int
remove_reference<string&>     // the `type` member will be string
```

**Example:**

(continuation of the example in section "Trailing Return Type")

Given that `beg` is an **iterator** (for some sequential container):
- `remove_reference<decltype(*beg)>::type` will be the type of the element to which `beg` refers, where
  1. `decltype(*beg)` returns the reference type of the element type (eg. `int&`)
    - "The dereference operator returns an **lvalue**, so the type deduced by `decltype` is a reference to the type of the element that `beg` denotes." (Lip, p684)
  2. `remove_reference<>::type` strips off the reference and returns the type without the `&` (eg. `int`)

```cpp
// Lip:
// return a copy of an element's value:

// must use typename to use a type member of a template parameter;
template <typename It>
auto fcn2(It beg, It end) ->
  typename remove_reference<decltype(*beg)>::type
{
  // process the range
  return *beg; // return a copy of an element from the range
}
```

### Suffix `_t`

- since C++14
- an alias template for a [type member](#for-type-member-shortcuts)

```cpp
std::add_const_t<T>               // since C++14
typename std::add_const<T>::type  // since C++11

namespace std {
  template<typename T> using add_const_t = typename add_const<T>::type;   // since C++14
}
```

## Function Templates

see [stackoverflow](https://www.cplusplus.com/doc/oldtutorial/templates/)

### Terminology

- function templates have 2 sets of parameters:
  - **template parameters**: declared in angle brackets before the function template name
  - **Call parameters**: declared in parentheses after the function template name

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

### Order of Execution

From [cppreference](https://en.cppreference.com/w/cpp/language/template_argument_deduction):

"Template argument deduction takes place **after** the function template name lookup (which may involve argument-dependent lookup) and **before** template argument substitution (which may involve SFINAE) and overload resolution."

ie.

1. function template name lookup (which may involve argument-dependent lookup)
2. template argument deduction
3. template argument substitution (which may involve SFINAE)
4. overload resolution

(this is also the order in which they are presented in [cppreference](https://en.cppreference.com/w/cpp/language/function_template))

### SFINAE

- "SFINAE'ing out function templates"
- aka "disabling function templates for certain constraints"
- aka "ignoring function templates during overload resolution"
- aka "function template shall not participate in overload resolution if" (phrase often used in the standard)

### Non-type Template Parameters

Lip 16.1.1, 14.8.3:

- represent a value rather than a type
- are specified by using a specific type name instead of the `class` or `typename` keyword
  - VJ12.2.2: but it can in some cases also start with the keyword `typename` (see examples below)
- may be 
  - an **integral type**
    - arguments bound to a nontype integral parameter must be a **constant expression**
      - eg. `3`, `32`, but **not** `3.14`
    - eg. `constexpr int`, `constexpr unsigned`, but **not** `constexpr double`
  - a **pointer type** (to an object or to a function)
    - arguments bound to it must have `static` lifetime
      - ie. we may **not** use an ordinary (non`static`) local object or a dynamic object as a template argument
    - a pointer parameter can also be instantiated by `nullptr` or a zero-valued **constant expression**
  - an **lvalue reference** (to an object or to a function)
    - arguments bound to it must have `static` lifetime
      - ie. we may **not** use an ordinary (non`static`) local object or a dynamic object as a template argument
- is a constant value inside the template definition
  - thus, can be used when constant expressions are required, eg. to specify the size of an array

VJ12.2.2:

- "stand for constant values that can be determined at compile or link time" 
- "are declared much like variables, but they cannot have nontype specifiers like `static`, `mutable`, and so forth."
- "can have `const` and `volatile` qualifiers, but if such a qualifier appears at the outermost level of the parameter type, it is simply ignored"

#### Examples

VJ12.2.2:

Examples for nontype parameter of pointer type:

```cpp
template<typename T,                        // a type parameter
         typename T::Allocator* Allocator>  // a nontype parameter
class List;

template<class X*>      // a nontype parameter of pointer type
class Y;
```

Examples for function and array types (they are implicitly adjusted to the pointer type to which they decay):

```cpp
template<int buf[5]> class Lexer;     // buf is really an int*
template<int* buf> class Lexer;       // OK: this is a redeclaration

template<int fun()> struct FuncWrap;  // fun really has pointer to
                                      // function type
template<int (*)()> struct FuncWrap;  // OK: this is a redeclaration
```

Examples for other specifiers:
- nontype parameters cannot have nontype specifiers like `static`, `mutable`, and so forth
- top-level cv qualifiers are ignored:

```cpp
template<int const length> class Buffer;  // const is useless here
template<int length> class Buffer;        // same as previous declaration
```

Examples for lvalue reference type nontype parameters:
- nonreference nontype parameters are always prvalues
  - ie. their address cannot be taken, and they cannot be assigned to
- lvalue reference type nontype parameter can be used to denote an lvalue
  - ie. they CAN be assigned to

```cpp
template<int& Counter>
struct LocalIncrement {
  LocalIncrement() { Counter = Counter + 1; } // OK: reference to an integer
  ~LocalIncrement() { Counter = Counter - 1; }
};
```

#### Passing Variable Size Arrays

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

- but **for constructors** using an `initializer_list` is more convenient, see "functions.md" &rarr; "`initializer_list`"

#### Deducible Nontype Parameters (auto Nontype Parameters)

- note: there are
  1. `auto` nontype parameters
    - discussed in this section
  2. `decltype(auto)` nontype parameters
    - VJ "think that such nontype template parameters are likely to cause surprise and do not anticipate that they will be widely used"
      - so, **best practice:** do not use `decltype(auto)` nontype parameters

VJ p296:

```cpp
// prior to C++17
// having to specify the type of the nontype template 
// argument - that is, specifying int in addition to 42 - can be tedious
template<typename T, T V> struct S;
S<int, 42>* ps;

// since C++17
// type of V for S<42> is deduced to be int because 42 has type int
template<auto V> struct S;
S<42>* ps;

// general constraints on the type of nontype template parameters remain in effect
S<3.14>* pd; // ERROR: floating-point nontype argument
```

VJ15.10.3: 

- "the type of the parameter `N` of function template `f<>()` is deduced from the type of the nontype parameter of `S`."
- "That's possible because a name of the form `X<...>` where `X` is a class template is a deduced context"

```cpp
template<auto N> struct S {};
template<auto N> int f(S<N> p);   // S<N> is a deduced context
S<42> x;
int r = f(x);
```

However, there are also many patterns that cannot be deduced that way:

```cpp
// deduced_nontype_params.cpp
template<auto V> int f(decltype(V) p);    // V is a nontype template parameter
int r1 = f<42>(42);  // OK
int r2 = f(42);      // ERROR: decltype(V) is a nondeduced context
```

- There is no unique value of `V` that matches the argument `42` (e.g., `decltype(7)` produces the same type as `decltype(42)`).
  - Therefore, the nontype template parameter must be specified **explicitly** to be able to call this function

### Instantiation (Implicit Specialization)

From [cppreference](https://en.cppreference.com/w/cpp/language/function_template#Function_template_instantiation): 
- A function template by itself is **not a type, or a function, or any other entity**. 
- No code is generated from a source file that contains only template definitions. 
- In order for any code to appear, a template must be **instantiated**: 
  - the template arguments must be determined so that the compiler can generate an actual function (or class, from a class template).

There are two forms of instantiation:
1. **explicit** instantiation
2. **implicit** instantiation

phth: remember:
- an explicit instantiation is easily **visible in the code** (it always contains `template`)
- an implicit instantiation is **not visible in the code** because it is generated by the compiler (it contains `template`, too, like an explicit instantiation, but you will not see it in the code)
  - this is similar to a synthesized constructor which is also "there", but you do not see it in the code
  - when a class or function template is used in the code and has not been explicitly instantiated yet, then it is implicitly instantiated 

#### Implicit Instantiation

cppreference:
- **implicit instantiation** occurs when
  - **code refers to a function** in context that requires the function definition to exist **AND** this particular **function has not been explicitly instantiated**
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
    f<double>(1); // implicitly instantiates and calls f<double>(double)
    f<>('a');     // implicitly instantiates and calls f<char>(char)
    f(7);         // implicitly instantiates and calls f<int>(int)
    void (*pf)(std::string) = f; // implicitly instantiates f<string>(string)
    pf("∇");                     // calls f<string>(string)
}
```

#### Explicit Instantiation

Lip:

- **motivation**
  - when two or more separately compiled source files use the same template with the same template arguments, there is an **instantiation** of that template **in each of those files**. 
  - in large systems, the overhead of instantiating the same template in multiple files can become significant. 
  - since C++11, we can avoid this overhead through an **explicit instantiation**
- declaration vs definition
  - WITH `extern` it is an explicit instantiation **declaration**
  - WITHOUT `extern` it is an explicit instantiation **definition**
    - **note**: for a class template, this instantiates **all** the members of that template including `inline` member functions
      - consequently, we can use explicit instantiation only for types that can be used with all the members of that template
    - see "Class Templates" &rarr; "Member Functions" &rarr; "Instantiation"
- When the compiler sees an `extern` template declaration, it will **not** generate code for that instantiation in that file
  - `extern` is a promise that there will be a non`extern` use of that instantiation elsewhere in the program
  - cppreference: "An explicit instantiation declaration (an `extern` template) prevents implicit instantiations"
- For a given instantiation 
  - there may be several `extern` declarations 
  - there must be **exactly one** definition

**Syntax:**

cppreference:

```cpp
// Explicit Instantiation Definition
template return-type name < argument-list > ( parameter-list ) ; 	        // (1) 	
template return-type name ( parameter-list ) ; 	                                // (2) 	
// Explicit Instantiation Declaration
extern template return-type name < argument-list > ( parameter-list ) ; 	// (3) 	(since C++11)
extern template return-type name ( parameter-list ) ; 	                        // (4) 	(since C++11)
```

- (1) and (3): without template argument deduction if every non-default template parameter is explicitly specified
- (2) and (4): with template argument deduction for all parameters

Lip:

```cpp
// below, "declaration" refers to a class or function declaration
// in which ALL the template parameters are replaced by the template arguments
extern template declaration;  // instantiation declaration
template declaration;         // instantiation definition
```

**Examples**:

Lip:

```cpp
// instantiation declaration and definition
extern template class Blob<string>;             // declaration
template int compare(const int&, const int&);   // definition
```

cppreference:

```cpp
template<typename T>
void f(T s)
{
    std::cout << s << '\n';
}
 
// phth note: the word "template" helps to distinguish explicit and implicit instantiation
template void f<double>(double); // explicitly instantiates f<double>(double)
template void f<>(char);         // explicitly instantiates f<char>(char), template argument deduced
template void f(int);            // explicitly instantiates f<int>(int), template argument deduced
```

Lip:

```cpp
// Application.cc
// these template types must be instantiated elsewhere in the program
extern template class Blob<string>;
extern template int compare(const int&, const int&);
Blob<string> sa1, sa2; // instantiation will appear elsewhere
// Blob<int> and its initializer_list constructor instantiated in this file
Blob<int> a1 = {0,1,2,3,4,5,6,7,8,9}; // will use the initializer_list constructor
Blob<int> a2(a1); // copy constructor instantiated in this file
int i = compare(a1[0], a2[0]); // instantiation will appear elsewhere

// Application.o will contain instantiations for Blob<int>, along with
// the initializer_list and copy constructors for that class. The compare<int>
// function and Blob<string> class will not be instantiated in that file.
```

```cpp
// templateBuild.cc
// instantiation file must provide a (nonextern) definition for every
// type and function that other files declare as extern
template int compare(const int&, const int&);
template class Blob<string>; // instantiates all members of the class template

// templateBuild.o will contain the definitions for
// compare instantiated with int and for the Blob<string> class. When we build
// the application, we must link templateBuild.o with the Application.o files.
```

### Specialization

- aka **"Explicit Specialization"**
- a separate definition of the template in which one or more template parameters are specified to have particular types
  - phth: unlike an explicit instantiation (which looks similar, but has no empty `<>` behind `template`), this does not instantiate anything, it is only a function template that has not been used in the code yet
- useful, when 
  - the general definition might not compile or might do the wrong thing
  - we want to take advantage of some specific knowledge to write more efficient code than would be instantiated from the template
- When we define a specialization, the function parameter type(s) **must match** the corresponding types in a previously declared template
- cppreference: "When template arguments are provided, or, for function (and class (since C++17)) templates only, **deduced**, they are substituted for the template parameters to obtain a **specialization** of the template, that is, a specific type or a specific function lvalue."

#### Declaration Order

- Templates and their specializations should be declared in the same header file. 
- **declaration order**:
  1. declarations for all the templates with a given name should appear **first**
  2. any specializations of those templates
- **Warning**:
  - if an **ordinary class or function declaration** is missing, the compiler won't be able to process our code
  - whereas if a **specialization declaration** is missing, the compiler will usually generate code using the original template
  - therefore, errors in declaration order between a template and its specializations are easy to make but hard to find

#### T.144: Don't specialize function templates

Overloading > Specialization

From [Core Guidelines](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rt-specialize-function):

- Reason
  - You can't partially specialize a function template per language rules. 
  - You can fully specialize a function template but **you almost certainly want to overload instead**
    - because function template specializations don't participate in overloading, they don't act as you probably wanted. (see **"Explanation 1"**)
  - Rarely, you should actually specialize by delegating to a class template that you can specialize properly. (see **"Explanation 2"**)

**Explanation 1**: "function template specializations don't participate in overloading"

From [modernescpp](https://www.modernescpp.com/index.php/full-specialization-of-function-templates/):

- "Overload resolution **IGNORES** function template **specializations**"
- "Overload resolution operates on **primary templates**"
- "Overload resolution only selects a primary template (...). **Only after it's been decided which primary template** is going to be selected, and that choice is locked in, will the compiler look around to see if there happens to be a suitable specialization of that template available, and if so that specialization will get used.", [gotw](http://www.gotw.ca/publications/mill17.htm)

```cpp
// dimovAbrahams.cpp

#include <iostream>
#include <string>

// getTypeName

template<typename T>            // (1) primary template
std::string getTypeName(T){
    return "unknown";
}

template<typename T>            // (2) primary template that overloads (1)
std::string getTypeName(T*){
    return "pointer";
}

template<>                      // (3) explicit specialization of (2)  // phth: Genau lesen: "of (2)" (nicht "of (1)" !)
std::string getTypeName(int*){
    return "int pointer";
}

// getTypeName2

template<typename T>            // (4) primary template
std::string getTypeName2(T){
    return "unknown";
}

template<>                      // (5) explicit specialization of (4)  // phth: Genau lesen: "of (4)"
std::string getTypeName2(int*){
    return "int pointer";
}

template<typename T>            // (6) primary template that overloads (4)
std::string getTypeName2(T*){
    return "pointer";
}

int main(){
    
    std::cout << '\n';
    
    int* p;
    
    std::cout << "getTypeName(p): " << getTypeName(p) <<  '\n';  
    std::cout << "getTypeName2(p): " << getTypeName2(p) <<  '\n';
    
    std::cout <<  '\n';
    
}
```

**Output**:

```bash
getTypeName(p): int pointer
getTypeName2(p): pointer
```

**Explanation 2**: "specialize by delegating to a class template":

From [gotw](http://www.gotw.ca/publications/mill17.htm):

**Moral #2**: If you're writing a function primary template, prefer to write it as a single function template that should never be specialized or overloaded, and then implement the function template entirely as a simple handoff to a class template containing a static function with the same signature. Everyone can specialize that -- both fully and partially, and without affecting the results of overload resolution.

```cpp
// Example 4: Illustrating Moral #2 
// 
template<class T> 
struct FImpl;

template<class T> 
void f( T t ) { FImpl<T>::f( t ); } // users, don't touch this!

template<class T> 
struct FImpl 
{ 
  static void f( T t ); // users, go ahead and specialize this 
};
```

#### Full Specialization

- has **empty** angle brackets
- When we define a specialization, the function parameter type(s) **must match** the corresponding types in a previously declared template
  - Note: important for `const T` and `const T&` function parameters:
    - The const version of a pointer type (eg. `double*`) is a **constant pointer** (`double* const`) (as distinct from a pointer to const), see code example below

```cpp
// PRIMARY TEMPLATE:
// Assume the general definition of this "compare" template is not appropriate 
// for a particular type, namely, character pointers
template <typename T> int compare(const T&, const T&);

// OVERLOAD:
// an overloaded version of "compare" (which is not a specialization!) that
// will be called only when we pass a string literal or an array
template<size_t N, size_t M>
int compare(const char (&)[N], const char (&)[M]);

// SPECIALIZATION:
// we want: special version of "compare" to handle 
// pointers to character arrays, and therefore, we know "T" must be "const char*"
// Which Function Parameter to use?:
// "T" is "const char*" => so "const T&" is "const char* const&"
template <>
int compare(const char* const &p1, const char* const &p2)
{
  return strcmp(p1, p2);
}

// SPECIALIZATION:
// ... whereas a specialization to handle doubles would look like this:
// Which Function Parameter to use?:
// - "T" is "double" => so "const T&" is "const double&"
template <>
int compare(const double &p1, const double &p2)
{
  return strcmp(p1, p2);
}

// call "compare"
const char *p1 = "hi", *p2 = "mom";
compare(p1, p2);        // calls the specialization for "const char*"
                        // (if this specialization was missing, it would call the 
                        // primary template which cannot handle "const char*")
compare("hi", "mom");   // calls the overloaded template with two nontype parameters
```

#### Partial Specialization

cppreference:

- Specializations may also be provided explicitly:
  - **full specializations** are allowed for class(, variable (since C++14)) and function templates,
  - **partial specializations** are only allowed for class templates (and variable templates (since C++14)).

### Multiple Template Parameters

#### Choosing the Return Type

**Problem**: in the following example the return type depends on the call argument order

```cpp
template<typename T1, typename T2>
T1 max (T1 a, T2 b)
{
  return b < a ? a : b;
}
...
auto m = ::max(4, 7.2);         // OK, but type of first argument defines return type
```

C++ provides different ways to deal with this problem:
1. Introduce a third template parameter for the return type.
2. Let the compiler find out the return type. (`auto`, trailing return type syntax)
3. Declare the return type to be the "common type" of the two parameter types. (`std::common_type_t<T1,T2>`)

**Method 1:**

```cpp
// RT does not appear in the types of the function call parameters
// -> Therefore, RT cannot be deduced (TAD does not take return types into account)
// -> have to specify the template argument list explicitly
template<typename T1, typename T2, typename RT>
RT max (T1 a, T2 b);
...
::max<int,double,double>(4, 7.2);   // OK, but tedious
```

The same a bit shorter:

```cpp
// specify only the first arguments explicitly and allow the deduction process to derive the rest
// -> must specify all the argument types up to the last argument type that cannot be determined implicitly
// -> need to change the order of the template parameters, so that RT is double, and T1, T2 are deduced
template<typename RT, typename T1, typename T2>
RT max (T1 a, T2 b);
deduced as: int double
...
::max<double>(4, 7.2)               // OK: return type is double, T1 and T2 are deduced
```

**Method 2:**
- (the simplest and best approach to deduce the return type)

```cpp
// Since C++14
// - auto for the return type WITHOUT a corresponding trailing return type
// - indicates that the actual return type must be deduced from the return statements in the function body
// - condition: multiple return statements have to match
template<typename T1, typename T2>
auto max (T1 a, T2 b)
{
  return b < a ? a : b;
}
```

Using **trailing return type** syntax:

```cpp
// Since C++11
// - auto for the return type WITH a corresponding trailing return type
template<typename T1, typename T2>
auto max (T1 a, T2 b) -> decltype(b<a?a:b)
{
  return b < a ? a : b;
}

// Problem: It might happen that the return type is a reference type, 
//          because under some conditions T might be a reference
// Solution: use std::decay to return the type decayed from T

#include <type_traits>

template<typename T1, typename T2>
auto max (T1 a, T2 b) -> typename std::decay<decltype(b<a?a:b)>::type
{
  return b < a ? a : b;
}

// Note: must use "typename" because "type" is a type
```

**Method 3:**

```cpp
// choosing "the more general type".

#include <type_traits>

// Since C++11
template<typename T1, typename T2>
typename std::common_type<T1,T2>::type max (T1 a, T2 b)
{
  return b < a ? a : b;
}

// Since C++14
template<typename T1, typename T2>
std::common_type_t<T1,T2> max (T1 a, T2 b)
{
  return b < a ? a : b;
}

// Note: std::common_type<> decays so that the return value can't become 
// a reference (ie. drops & and top-level const)
```

### Default Template Arguments

- may even refer to previous template parameters

```cpp
// note: the default template argument for RT refers to the previous template parameters T1 and T2
#include <type_traits>

template<typename T1, typename T2,
         typename RT = std::decay_t<decltype(true ? T1() : T2())>>       // value initialization: calls default constructor
RT max (T1 a, T2 b)
{
  return b < a ? a : b;
}

// Note: this can be simplified by using declval (see notes "auto" -> "declval")
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

- **Inside** the scope of the class template itself, we may use the name of the template **without template arguments**.
  - see Notes "Classes" &rarr; "Scope" (**important!**)
  - eg. in the scope of the `BlobPtr` class template the compiler will treat `BlobPtr&` as `BlobPtr<T>&`

```cpp
template <typename T> class BlobPtr {
public:
  BlobPtr(): curr(0) { }
  BlobPtr(Blob<T> &a, size_t sz = 0):
          wptr(a.data), curr(sz) { }
  T& operator*() const
  { auto p = check(curr, "dereference past end");
    return (*p)[curr];            // (*p) is the vector to which this object points
  }
  // increment and decrement
  BlobPtr& operator++();          // prefix operators
  BlobPtr& operator--();
private:
  // check returns a shared_ptr to the vector if the check succeeds
  std::shared_ptr<std::vector<T>>
      check(std::size_t, const std::string&) const;
  // store a weak_ptr, which means the underlying vector might be destroyed
  std::weak_ptr<std::vector<T>> wptr;
  std::size_t curr;               // current position within the array
};
```

#### Example

- the **return type** of `operator++` is not in the scope of the class, but the **function body** and **parameter list** are!
  - see Notes "Classes" &rarr; "Scope" (**important!**)
  - therefore, inside the body of `operator++` the compiler will treat `BlobPtr` as `BlobPtr<T>`

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

- By default, a member function of a class template is implicitly instantiated **only if** the program uses that member function.
- But, **an explicit instantiation definition** for a class template instantiates **ALL** the members of that template including `inline` member functions.
  - Consequently, we can use explicit instantiation only for types that can be used with **ALL** the members of that template.

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
- eg. for instantiation of vectors (see "Container" &rarr; "Constraints on Types that a Container can hold")
- in this case, an explicit instantiation definition is not possible

```cpp
// assume noDefault is a type without a default constructor
vector<noDefault> v1(10, init); // ok: element initializer supplied
vector<noDefault> v2(10);       // error: must supply an element initializer
template class vector<noDefault>;   // instantiates all members of the class template
                                    // error: type "noDefault" cannot be used with ALL the members of the "vector" template
```

### Friends

For a **nontemplate** class:

```cpp
// forward declaration necessary to befriend a specific instantiation of a template
// (because, recall, a friend declaration is not a declaration)
template <typename T> class Pal;  // this "T" is not related to the "T" in nontemplate class "C"
class C {
  friend class Pal<C>;                      // specific friendship
  template <typename T> friend class Pal2;  // general friendship
};
```

For a class template:
- to allow **all** instantiations as friends, the `friend` declaration must use template parameter(s) that differ from those used by the class itself

```cpp
template <typename T> class Pal;  // this "T" is not related to the "T" in template class "C2"
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

#### For Type Member Shortcuts

```cpp
// After
struct MyType {
  typedef ... iterator;     // "iterator" is a type member
  ...
};

// or:
struct MyType {
  using iterator = ...;     // "iterator" is a type member
  ...
};

// a definition such as
template<typename T>
using MyTypeIterator = typename MyType<T>::iterator;      // abbreviates the type member "iterator"

// allows to use
MyTypeIterator<int> pos;

// instead of
typename MyType<T>::iterator pos;   // man spart sich den "member of" operator
```

### static members

- mostly like for any other (nontemplate) class (see "objects.md" &rarr; "static members")
  - a `static` member function is instantiated only if it is used in a program
  - access a `static` member of a class template 
    - through an object of the class type or 
    - by using the scope operator `::` to access the member directly
- there is a distinct object **for each instantiation** of a class template
  - eg. all objects of type `Foo<X>` share **the same** `ctr` object and `count` function, but there is a **distinct** `ctr` and `count` for objects of type `Foo<Y>`

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

### Specialization

#### Partial Specialization

- unlike function templates, a class template specialization does not have to supply an argument for **every** template parameter
- a partial specialization is itself a template
- a partial specialization has **the same name** as the template it specializes
- the **template parameter list** includes an entry for each template parameter whose type is **not** completely **fixed**
- **after the class name**, we specify **arguments** for the template **parameters we are specializing**
  - these arguments are listed **inside angle brackets** following the template name
  - the arguments correspond positionally to the parameters in the original template
  - *phth*: whereas for (full specializations of) function templates we fix the function **call parameter types** (which don't need angle brackets)

```cpp
// original, most general template (also called the "primary template")
template <class T> struct remove_reference
{ typedef T type; };      // "type" is a "member type"
// "partial specializations" that will be used for lvalue and rvalue references
template <class T> struct remove_reference<T&>  // lvalue references
{ typedef T type; };
template <class T> struct remove_reference<T&&> // rvalue references
{ typedef T type; };

int i;
remove_reference<decltype(42)>::type a;             // decltype(42) is int, uses the original template
remove_reference<decltype(i)>::type b;              // decltype(i) is int&, uses first (T&) partial specialization
remove_reference<decltype(std::move(i))>::type c;   // decltype(std::move(i)) is int&&, uses second (i.e., T&&) partial specialization
```

```cpp
// cppreference

template<class T1, class T2, int I>
class A {};             // primary template
 
template<class T, int I>
class A<T, T*, I> {};   // #1: partial specialization where T2 is a pointer to T1
 
template<class T, class T2, int I>
class A<T*, T2, I> {};  // #2: partial specialization where T1 is a pointer
 
template<class T>
class A<int, T*, 5> {}; // #3: partial specialization where
                        //     T1 is int, I is 5, and T2 is a pointer
 
template<class X, class T, int I>
class A<X, T*, I> {};   // #4: partial specialization where T2 is a pointer
```

#### Specialization of Specific Members but Not the Class

- we can specialize just specific member function(s)
- again, the arguments for the template parameters we are specializing are listed **inside angle brackets** following the class template name

```cpp
// Important:
// - The other members of Foo<int> will be supplied by the Foo template
template <typename T> struct Foo {
  Foo(const T &t = T()): mem(t) { }
  void Bar() { /* . . . */ }
  T mem;
  // other members of Foo
};
template<>
void Foo<int>::Bar()    // we're specializing the Bar member of Foo<int>
{
  // do whatever specialized processing that applies to ints
}

// When we use Foo with int, members other than Bar are instantiated as usual
Foo<int> fi;        // instantiates Foo<int>::Foo()
fi.Bar();           // uses our specialization of Foo<int>::Bar()
fi.otherMember();   // supplied by the Foo template

// When we use Foo with any type other than int, members are instantiated as usual
Foo<string> fs;     // instantiates Foo<string>::Foo()
fs.Bar();           // instantiates Foo<string>::Bar()
```

## Variadic Templates

- a function or class template that can take a varying number of parameters. (&rarr; "functions.md" &rarr; "Functions with Varying Parameters")
- *phth*: variadic functions are often recursive, but they don't *have to* be recursive
- The varying parameters are known as a **parameter pack**.
- There are two kinds of parameter packs:
  - A **template parameter pack** represents zero or more template parameters, 
  - a **function parameter pack** represents zero or more function parameters.
- We use an **ellipsis** to indicate that a template or function parameter represents a pack.

```cpp
// "Args" is a template parameter pack; "rest" is a function parameter pack
template <typename T, typename... Args>       // "Args" represents zero or more template type parameters
void foo(const T &t, const Args& ... rest);   // "rest" represents zero or more function parameters
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

### sizeof... Operator

- returns a constant expression
- does not evaluate its argument

```cpp
template<typename ... Args> 
void g(Args ... args) {
  cout << sizeof...(Args) << endl; // number of type parameters
  cout << sizeof...(args) << endl; // number of function parameters
}
```

### Variadic Function Templates

- used when we know neither the **number** nor the **types** of the arguments we want to process
- variadic functions are often **recursive**

**Example:**

```cpp
// function to end the recursion and print the last element
// this function must be declared before the variadic version of print is defined
template<typename T>
ostream &print(ostream &os, const T &t)
{
  return os << t; // no separator after the last element in the pack
}
// this version of print will be called for all but the last element in the pack
template <typename T, typename... Args>
ostream &print(ostream &os, const T &t, const Args&... rest)
{
  os << t << ", ";              // print the first argument
  return print(os, rest...);    // recursive call; print the other arguments
}
```

For `print(cout, i, s, 42);` the recursion will execute as follows:

Call | t | rest...
:---: | :---: | :---: |
print(cout, i, s, 42) | i | s, 42
print(cout, s, 42) | s | 42
print(cout, 42) calls the nonvariadic version of print | | 

- for the **last call** in the recursion, `print(cout, 42)`, **both** versions of `print` are viable
  - both functions provide an equally good match for the call. 
  - However, a nonvariadic template is **more specialized** than a variadic template (see "Overloading and Templates"), so the nonvariadic version is chosen for this call
- when the nonvariadic version of `print` is not declared **before** the variadic version, the variadic function will recurse indefinitely

### Pack Expansion

**In the example above:**

```cpp
// - the pattern is "const Args&"
// - the pattern is applied to each element in the template parameter pack "Args"
// - pattern expands to: comma-separated list of zero or more parameter types, each of which will have the form const type&
print(cout, i, s, 42); // two parameters in the pack

// This call is instantiated as 
ostream& print(ostream&, const int&, const string&, const int&);
```

```cpp
// - the pattern is the name of the function parameter pack (i.e., "rest")
// - pattern expands to: comma-separated list of the elements in the pack
print(os, rest...)

// this call is equivalent to
print(os, s, 42);
```

More complicated patterns are also possible:

```cpp
// - the pattern is "debug_rep(rest)"
// - call debug_rep on each argument in the call to print
// - pattern expands to: comma-separated list of calls to debug_rep
template <typename... Args>
ostream &errorMsg(ostream &os, const Args&... rest)
{
  // print(os, debug_rep(a1), debug_rep(a2), ..., debug_rep(an)
  return print(os, debug_rep(rest)...);
}

// Then
errorMsg(cerr, fcnName, code.num(), otherData, "other", item);  // note: errorMsg() calls print()

// ... will execute as if we had written
print(cerr, debug_rep(fcnName), debug_rep(code.num()),
            debug_rep(otherData), debug_rep("otherData"),
            debug_rep(item));
```

**Warning**: It is important where you put the ellipsis `...`:

```cpp
// passes the pack to debug_rep; print(os, debug_rep(a1, a2, ..., an))
print(os, debug_rep(rest...)); // error: no matching function to call

// expands to
print(cerr, debug_rep(fcnName, code.num(), otherData, "otherData", item));
```

### Forwarding

Variadic functions often forward their parameters to other functions. Such functions typically have a form similar to:

```cpp
// fun has zero or more parameters each of which is
// an rvalue reference to a template parameter type
template<typename... Args>
void fun(Args&&... args) // expands Args as a list of rvalue references
{
  // the argument to "work" expands both Args and args
  work(std::forward<Args>(args)...);
}

// Then
fun(10, 'c');

// ... will execute as if we had written
work(std::forward<int>(10), std::forward<char>(c))
```

## Fold Expressions

- since C++17

cppreference:

```cpp
( pack op ... )                // (1) unary right fold
( ... op pack )                // (2) unary left fold
( pack op ... op init )        // (3) binary right fold
( init op ... op pack )        // (4) binary left fold
```

VJ4.2:

Fold Expression         | Evaluation
:--- | :---
`( ... op pack )`         | `((( pack1 op pack2 ) op pack3 ) ... op packN )`
`( pack op ... )`         | `( pack1 op ( ... ( packN-1 op packN )))`
`( init op ... op pack )` | `((( init op pack1 ) op pack2 ) ... op packN )`
`( pack op ... op init )` | `( pack1 op ( ... ( packN op init )))`

related: [fluentcpp](https://www.fluentcpp.com/2021/03/12/cpp-fold-expressions/)

### Repeating Operations, Comma Operator

From [fluentcpp](https://www.fluentcpp.com/2021/03/19/what-c-fold-expressions-can-bring-to-your-code/):

- fold over the **comma operator**
- the default version of the comma operator
  1. executes the left operand
  2. then the right operand
  3. then returns the right operand

**Example:**

```cpp
// Adding several elements to a vector

auto v = std::vector<int>{1, 2, 3};

// bad: repeats code
v.push_back(4);
v.push_back(5);
v.push_back(6);
v.push_back(7);
v.push_back(8);
v.push_back(9);
v.push_back(10);

// unary right fold
// instead, we can call multiple "push_back"s in a single expression
template<typename T, typename... Ts>
void push_back(std::vector<T>& v, Ts&&... values)
{
    (v.push_back(std::forward<Ts>(values)), ...);
}

// then, call push_back
push_back(v, 4, 5, 6, 7, 8, 9, 10);
```

- The `,` operator in C++ is **associative** (see "cpp.md" &rarr; "Comma Operator").
  - it does **not** matter whether we use a **left fold** or a **right fold**, the result will be **the same**:
  
```cpp
// unary left fold
// will have the same result as the unary right fold above
template<typename T, typename... Ts>
void push_back(std::vector<T>& v, Ts&&... values)
{
    (..., v.push_back(std::forward<Ts>(values)));
}
```

## Constraints

### Requirement

- an ELEMENT OF of the set `requirement-seq` in a [requires expression](#requires-expression)
  - see `requirement-seq` in section "[requires expression](#requires-expression)"

#### Compound Requirement

```cpp
// Syntax
{ expression } noexcept(optional) return-type-requirement(optional) ;
```

`return-type-requirement` - `-> type-constraint`

3) If `return-type-requirement` is present, then:
- a) Template arguments are substituted into the `return-type-requirement;`
- b) `decltype((expression))` must satisfy the constraint imposed by the `type-constraint`. Otherwise, the enclosing **requires-expression** is `false`.

```cpp
// cppreference
template<typename T>
concept C2 = requires(T x)
{
    // the expression *x must be valid
    // AND the type T::inner must be valid
    // AND the result of *x must be convertible to T::inner
    {*x} -> std::convertible_to<typename T::inner>;
 
    // the expression x + 1 must be valid
    // AND std::same_as<decltype((x + 1)), int> must be satisfied
    // i.e., (x + 1) must be a prvalue of type int
    {x + 1} -> std::same_as<int>;
 
    // the expression x * 1 must be valid
    // AND its result must be convertible to T
    {x * 1} -> std::convertible_to<T>;
};
```

### requires clause

cppreference:

The keyword `requires` is used to introduce a **requires-clause**, which specifies constraints on template arguments or on a function declaration.

```cpp
template<typename T>
void f(T&&) requires Eq<T>; // can appear as the last element of a function declarator
 
template<typename T> requires Addable<T> // or right after a template parameter list
T add(T a, T b) { return a + b; }
```

In this case, the keyword `requires` must be followed by some constant expression (so it's possible to write `requires true`), but the intent is that 
- a named concept (as in the example above) or 
- a conjunction/disjunction of named concepts or 
- a **requires expression** is used.

### requires expression

cppreference:

Yields a prvalue expression of type `bool` that describes the constraints. 

```cpp
// Syntax
requires { requirement-seq } 		
requires ( parameter-list(optional) ) { requirement-seq } 		
```

- `parameter-list` - a comma-separated list of parameters like in a function declaration, except that default arguments are not allowed and it cannot end with an ellipsis
- `requirement-seq` - sequence of **requirements**, each requirement is one of the following:
  - simple requirement
  - type requirements
  - compound requirements
  - nested requirements
    - looks like a **requirement clause** inside a requirement expression (VJE.2, p743), but it is called "nested requirement"

### requires clause vs requires expression

- a requirement expression can appear inside a requirement clause, and vice versa
- **remember**: a requires **clause**
  - ~~has no braces~~
  - appears typically 
    - after a `template<typename>` or 
    - as the last element of a function declarator
    - inside a requirement expression ("nested requirement")

```cpp
// cppreference

template<typename T>
concept Addable = requires (T x) { x + x; }; // requires-expression
 
template<typename T> requires Addable<T> // requires-clause, not requires-expression
T add(T a, T b) { return a + b; }
 
// a requires-clause containing a requires-expression
template<typename T>
    requires requires (T x) { x + x; } // ad-hoc constraint, note keyword used twice
T add(T a, T b) { return a + b; }

// VJE.2

// a requires-expression containing a requires-clause
template<typename Seq>
concept Sequence = requires(Seq seq) {
  typename Seq::iterator;
  requires Iterator<typename Seq::iterator>;
  { seq.begin() } -> Seq::iterator;
  ...
};
```

## Concepts

cppreference:

- A concept is a named set of requirements. 

### Definition

- The definition of a concept must appear at namespace scope.

```cpp
// Syntax
template < template-parameter-list >
concept concept-name attr(optional) = constraint-expression;
```
