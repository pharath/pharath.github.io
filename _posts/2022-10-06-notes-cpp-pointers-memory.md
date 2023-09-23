---
title: "C++ Notes - Pointers and Dynamic Memory"
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
    - pointers
    - smartpointers
    - dynamicmemory
    - notes

---

# Pointer

- see my "C Notes"
- A pointer is a **compound type** that "points to" another type. 
- Like references,
  - pointers are used for **indirect access to other objects**. 
- Unlike a reference, 
  - a pointer is an object in its own right. 
  - a pointer need not be initialized at the time it is defined.
- Pointers can be **assigned** and **copied**
  - a single pointer can point to several different objects over its lifetime.
- pointers defined at **block scope** have **undefined value** if they are not initialized (like other built-in types)
- Each **pointer** is equal to the **address of the first byte** of the pointed-to variable (JA201)
  - The **address of a variable** is actually the address of the first (lowest) byte it occupies (JA201)
- **best practice:**
  - always initialize pointers

```cpp
double dval;
double *pd = &dval; // ok: initializer is the address of a double
// copy initialization of a pointer (more examples, see Lippman)
double *pd2 = pd;   // ok: initializer is a pointer to double
```

**How to memorize**:

From [stackoverflow](https://stackoverflow.com/a/2837853):
- The symbols `*` and `&` both have **two different meanings** that have to do with indirection.
  - `*` when used as part of a **type** indicates that the type is a pointer: 
    - `int` is a type, so `int*` is a pointer-to-int type, and `int**` is a pointer-to-pointer-to-int type.
  - `&` when used as part of a **type** indicates that the type is a reference. 
    - `int` is a type, so `int&` is a reference-to-int (there is no such thing as reference-to-reference). 
    - References and pointers are used for similar things, but they are quite different and not interchangable. 
    - A reference is best thought of as an alias, or alternate name, for an existing variable.
  - `*` when used as a **unary operator** performs an operation called "dereference" (which has nothing to do with reference types!).
    - This operation is only meaningful on pointers.
  - `&` when used as a **unary operator** performs an operation called "address-of".
- So remember, the type suffix `&` is for references, and has nothing to do with the unary operatory `&`, which has to do with getting addresses for use with pointers. The two uses are completely unrelated. And `*` as a type suffix declares a pointer, while `*` as a unary operator performs an action on pointers.

## delete

- Delete an array: `delete[] arrayName`
- Deleting a `nullptr` does not cause any change and no error.
- You cannot delete a pointer to a local stack allocated variable:

```cpp
// see delete.cpp (day 8)
int x;
int* ptr1 = &x;

// x is present on stack frame as
// local variable, only dynamically
// allocated variables can be destroyed
// using the delete operator
delete ptr1;

return 0;

// Output: Runtime error
```

## Nullpointer

### Do not use NULL

- `NULL` is defined as a [macro](https://gcc.gnu.org/onlinedocs/cpp/Macros.html) (i.e. a piece of code in a program that is replaced by the value of the macro; a macro is defined by `#define` directive; whenever a macro name is encountered by the [preprocessor](https://en.wikipedia.org/wiki/Preprocessor), it replaces the name with the definition of the macro):
  - unlike `nullptr`, `NULL` is **not** a built-in constant
  - defined in multiple headers, eg. `<cstddef>`, `<clocale>`, `<cstdio>`, etc.
  - the definition of this macro depends on the C++ implementation that is used
  - some **possible** implementations ([cppreference](https://en.cppreference.com/w/cpp/types/NULL)):
    ```cpp
    #define NULL 0
    //since C++11
    #define NULL nullptr
    // in g++
    #define NULL __null     // where "__null" is almost equivalent to the integer literal "0"
    ```
  - For **g++**, `NULL` is `#define`'d to be `__null`, a magic keyword extension of g++ that is slightly safer than a plain integer., [gcc.gnu.org](https://gcc.gnu.org/onlinedocs/libstdc++/manual/support.html#std.support.types.null)
- "Unless you need to be compatible with C++98/C++03 or C you should prefer to use `nullptr` instead of `NULL`." ([g++ doc](https://gcc.gnu.org/onlinedocs/libstdc++/manual/support.html#std.support.types.null))
- [Bjarne Stroustrup comment](https://www.stroustrup.com/bs_faq2.html#null)
  - I prefer to avoid macros, so I use `0`.
  - If you have to name the null pointer, call it `nullptr`.

### Implicit Cast of NULL and nullptr

#### To Pointer Types

- `NULL` und `nullptr` beide implicitly convertible to any **pointer** type
  - A **null pointer** constant (see `NULL`), can be converted to any **pointer** type (i.e. type with asterisk `*`), and the result is the null pointer value of that type. ([cppreference](https://en.cppreference.com/w/cpp/language/implicit_conversion))

#### To Integral Types

- Unlike `NULL`, `nullptr` is **not** implicitly convertible or comparable to integral types (e.g. `int`, `char`) ([geeksforgeeks](https://www.geeksforgeeks.org/understanding-nullptr-c/))
  - `int x = NULL` works; 
  - `int x = nullptr` does **not** work!
- from [C++11](https://en.wikipedia.org/wiki/C%2B%2B11#Null_pointer_constant):
  - `nullptr` is of type `nullptr_t`, which is implicitly convertible and comparable to any 
    - pointer type or
    - pointer-to-member type
  - It is **not** implicitly convertible or comparable to 
    - integral types (e.g. `int`, `char`), **except for** `bool`

### Type of NULL

The type of `NULL` depends on the C++ implementation.

- Some implementations define `NULL` as the compiler extension `__null` with following properties, [cppreference](https://en.cppreference.com/w/cpp/types/NULL):
  - `__null` is equivalent to a zero-valued integer literal (and thus compatible with the C++ standard) and has the same size as `void*`, e.g. it is equivalent to `0` (on ILP32 platforms) /`0L` (on LP64 platforms);
  - conversion from `__null` to an arithmetic type, including the type of `__null` itself, may trigger a warning.

~~In C++11 hat `NULL` den type `nullptr_t`~~ (depends on the C++ implementation)

"In C, the macro `NULL` may have the type `void*`, but that is not allowed in C++." ([cppreference](https://en.cppreference.com/w/cpp/types/NULL))
- how come?: [stackoverflow](https://stackoverflow.com/a/69057243)
- soll heißen: `NULL` hat in C++ **absichtlich nicht (wie in C)** den Type `void*`, weil "there is no **implicit cast** from `void*` to any other type in C++" (in C wäre das aber möglich!). 
  - Seit C++11 **kann** `NULL` den type `nullptr_t` haben, muss aber nicht ([cppreference](https://en.cppreference.com/w/cpp/types/nullptr_t)).

### Type of nullptr

"`nullptr_t` is the type of the null pointer literal, `nullptr`." ([cppreference](https://en.cppreference.com/w/cpp/types/nullptr_t))

`nullptr_t` ([cppreference](https://en.cppreference.com/w/cpp/types/nullptr_t)):
- is the type of the null pointer literal, `nullptr`.
- is a distinct type that is not itself a pointer type or a pointer to member type. 
- its values 
  - are **null pointer constants** and 
  - may be implicitly converted to any pointer and pointer to member type.
- `sizeof(std::nullptr_t)` is equal to `sizeof(void *)`.

```cpp
// Problem: there is no **implicit cast** from `void*` to any other type in C++
void* ptr = nullptr; 
int foo = *ptr;    // this implicit cast is not allowed in C++
```
gives a compiler error `error: ‘void*’ is not a pointer-to-object type`.

```cpp
// Fix: use a different type (a "pointer-to-object" type) instead of `void*` - `void*` is a "pointer-to-nothing"
char* ptr = nullptr; 
int foo = *ptr;
```
does **not** give a compiler error because `char*` is a pointer-to-object type.

### Type of `__null`

"`__null` is a g++ internal thing that serves roughly the same purpose as the standard `nullptr` added in C++11 (acting consistently as a pointer, never an integer).", [stackoverflow](https://stackoverflow.com/a/53963689)

From [cppreference](https://en.cppreference.com/w/cpp/types/NULL):
- `__null` is equivalent to a zero-valued integer literal (and thus compatible with the C++ standard) and has the same size as `void*`, e.g. it is equivalent to `0` (on ILP32 platforms) /`0L` (on LP64 platforms);
- conversion from `__null` to an arithmetic type, including the type of `__null` itself, may trigger a warning.

From [stackoverflow](https://stackoverflow.com/a/8783589):
- The implementation of `__null` is as a `g++` internal.
- You won't find it in a header file or anything like that.
- Basically, it works like `reinterpret_cast<void *>(0)`.

# Arrays

- a **compound type**
- the **dimension** must be a **constant expression**
- cannot use `auto` to deduce the type
- elements in an array are **default initialized**
  - thus, an array of built-in type that is defined inside a function will have undefined values.

## Initialization

- examples: see `array_init.cpp` and `array_init_basics.cpp`

### Default Initialization

```cpp
unsigned cnt = 42;          // not a constant expression
constexpr unsigned sz = 42; // constant expression

int arr[10];                  // array of ten ints
int *parr[sz];                // array of 42 pointers to int
string bad[cnt];              // error: cnt is not a constant expression
string strs[get_size()];      // ok if get_size is constexpr, error otherwise
```

### Value Initialization

```cpp
unsigned scores2[11]{};     // 11 buckets, all value initialized to 0
// Lip3.5.2
unsigned scores[11] = {};   // 11 buckets, all value initialized to 0
// actually, this is list initialization, but since in list initialization
// the remaining elements are value initialized this is effectively
// a value initialization of all elements
```

### List Initialization

- If the dimension is greater than the number of initializers
  - the initializers are used for the first elements 
  - remaining elements are **value initialized**

```cpp
const unsigned sz = 3;
int ia1[sz] = {0,1,2};          // array of three ints with values 0, 1, 2
int a2[] = {0, 1, 2};           // an array of dimension 3
int a3[5] = {0, 1, 2};          // equivalent to a3[] = {0, 1, 2, 0, 0}
string a4[3] = {"hi", "bye"};   // same as a4[] = {"hi", "bye", ""}
int a5[2] = {0,1,2};            // error: too many initializers
```

## Array Name

From [stackoverflow](https://stackoverflow.com/a/2351595):

It's not a pointer, const or otherwise, and it's not anything else, it's an array.

To see a difference:

```cpp
int a[10];
int *const b = a;

std::cout << sizeof(a); // prints "40" on my machine.
std::cout << sizeof(b); // prints "4" on my machine.
```

Clearly a and b are not the same type, since they have different sizes.

In most contexts, an array name "decays" to a pointer to its own first element. You can think of this as an automatic conversion. The result is an rvalue, meaning that it's "just" a pointer value, and can't be assigned to, similar to when a function name decays to a function pointer. Doesn't mean it's "const" as such, but it's not assignable.

From JA202:

An **array name without brackets** is a pointer to the array's first element
- You can also use the expression `&data[0]` to obtain the address of the array's first element

## Copy

- C-style arrays do **not** support Copy Initialization or Assignment (but `std::array` does!)

```cpp
int a[] = {0, 1, 2};  // array of three ints
int a2[] = a;         // error: cannot copy initialize one array with another
a2 = a;               // error: cannot copy assign one array to another
```

## Compiler Substitution of Array Name

Lippman:

**special property 1** of arrays:

In most places when we use an array, the compiler automatically substitutes a pointer to the first element

```cpp
string nums[] = {"one", "two", "three"}; // array of strings
string *p = &nums[0]; // p points to the first element in nums

// Because of special property 1:
string *p2 = nums; // equivalent to p2 = &nums[0]
```

## range for

```cpp
for (auto i : scores)
  cout << i << " ";
cout << endl;
```

## Subscript Operator

- distinct from the subscript operator defined by the `std::vector` class' 
- when we use a **variable to subscript** an array, we normally should define that variable to have type `size_t`
- **MUST ALWAYS check that the subscript value is in range**
  - Nothing stops a program from stepping across an array boundary
  - It is possible for programs to compile and execute yet still be fatally wrong.

### Buffer Overflow

- an error that the compiler is unlikely to detect
  - instead, the value we get at run time is undefined
- most common source of security problems are **buffer overflow bugs**
  - occur when a program fails to check a subscript and mistakenly uses memory **outside the range** of an array (or similar data structure)
- to ensure that subscripts are in range is **avoid subscripting altogether** by using a range `for` whenever possible
- occurs for arrays, `std::vector`

## Pointers are Iterators

pointers that address elements in an array have additional operations
- pointers to array elements support the same operations as **iterators** on `vectors` or `strings`

```cpp
int arr[] = {0,1,2,3,4,5,6,7,8,9};
int *p = arr; // p points to the first element in arr
++p; // p points to arr[1]
```

**special property 2** of arrays:

We can take the address of the nonexistent element one past the last element of an array ("off-the-end pointer")
- the only thing we can do with this element is take its address
- we may not dereference or increment an off-the-end pointer

```cpp
int *e = &arr[10]; // pointer just past the last element in arr
```

Print the elements in an array:

```cpp
// print the elements in arr
for (int *b = arr; b != e; ++b)
  cout << *b << endl; // print the elements in arr
```

Computing an off-the-end pointer is error-prone. Instead, use `begin` and `end` (defined in `<iterator>`):

```cpp
#include <iterator>

int ia[] = {0,1,2,3,4,5,6,7,8,9}; // ia is an array of ten ints
int *beg = begin(ia); // pointer to the first element in ia
int *last = end(ia); // pointer one past the last element in ia

// pbeg points to the first and pend points just past the last element in arr
int *pbeg = begin(arr), *pend = end(arr);
// find the first negative element, stopping if we’ve seen all the elements
while (pbeg != pend && *pbeg >= 0)
  ++pbeg;
```

## Pointer Arithmetic

- Computing a pointer more than one past the last element is an error, although the compiler is unlikely to detect such errors.
 
```cpp
constexpr size_t sz = 5;
int arr[sz] = {1,2,3,4,5};
int *ip = arr;              // equivalent to int *ip = &arr[0]
int *ip2 = ip + 4;          // ip2 points to arr[4], the last element in arr

// ok: arr is converted to a pointer to its first element; p points one past the end of arr
int *p = arr + sz;          // use caution -- do not dereference!
int *p2 = arr + 10;         // error: arr has only 5 elements; p2 has undefined value; compiler is unlikely to detect this error

// distance between two pointers (to elements of the same array):
// - return type: `ptrdiff_t` (a machine-specific type, in <cstddef> header)
auto n = end(arr) - begin(arr);     // n is 5, the number of elements in arr
```

- "compare pointers" using relational operators:

```cpp
constexpr size_t sz = 5;
int arr[sz] = {1,2,3,4,5};

// eg. traverse the elements in arr
int *b = arr, *e = arr + sz;
while (b < e) {                 // ok: b and e are related
  // use *b
  ++b;
}

// error: cannot use the relational operators on pointers to two unrelated objects:
int i = 0, sz = 42;
int *p = &i, *e = &sz;
// undefined: p and e are unrelated; comparison is meaningless!
while (p < e)
```

pointer arithmetic is also valid ...
- for null pointers
  - If `p` is a null pointer, we can add or subtract an integral constant expression whose value is `0` to `p`.
  - We can also subtract two null pointers from one another, in which case the result is `0`.
- for pointers that point to an object that is not an array
  - the pointers must point to the same object, or one past that object

## References and Pointers to Arrays

```cpp
int *ptrs[10];              // ptrs is an array of ten pointers to int
int &refs[10] = /* ? */;    // error: no arrays of references

int arr[10];
int (*Parray)[10] = &arr;   // Parray points to an array of ten ints
int (&arrRef)[10] = arr;    // arrRef refers to an array of ten ints -> to pass array "by reference"
```

## Pass an Array by Reference

Similarly, we can pass a parameter that is a reference to an array:

```cpp
void print(int (&arr)[10])   // the dimension is part of the type
{
  for (auto elem : arr)
    cout << elem << endl;
}
```

## Return an Array

Lippman:
- a function **cannot** return an array
  - because we cannot copy an array (see "Arrays" &rarr; "Copy")
- however, a function can return a pointer or a reference to an array
  - see "cpp-pointers-memory.md" &rarr; "Returning a Pointer to an Array"

## Returning a Pointer to an Array

- see also "functions.md" &rarr; "Return an Array"

**Option 1:** using a type alias

```cpp
typedef int arrT[10]; // arrT is a synonym for the type array of ten ints
using arrT = int[10]; // equivalent declaration of arrT
arrT* func(int i); // func returns a pointer to an array of ten ints
```

**Option 2:** function that returns a pointer to an array

```cpp
// Syntax: Type (*function(parameter_list))[dimension]
int (*func(int i))[10];
// compare with: "int (*array)[10];"
// -> aka a normal pointer-to-array (where "array" = "func(int i)"), like in section "References and Pointers to Arrays"
// -> ie. "int (*)[10]" is the return type (proof: see "Option 3")
```

**Option 3:** trailing return type syntax:

```cpp
// func takes an int argument and returns a pointer to an array of ten ints
auto func(int i) -> int(*)[10];
```

**Option 4:** using `decltype`

```cpp
int odd[] = {1,3,5,7,9};
int even[] = {0,2,4,6,8};
// returns a pointer to an array of five int elements
decltype(odd) *arrPtr(int i)
{
  return (i % 2) ? &odd : &even; // returns a pointer to the array
}
```

# Dynamic Memory

## Objects in Dynamic Memory

- managed through `new` and `delete`
- by default, dynamically allocated objects are **default initialized**
  - built-in or compound type: undefined value
  - class type: default constructor
- we can also use
  - direct initialization
  - value initialization
- **Best practice:**
  - always initialize dynamically allocated objects

## new

- allocates, constructs and optionally initializes, an object in dynamic memory
- returns a pointer to that object
- if the constructor throws an **exception**:
  - "If initialization terminates by throwing an exception (e.g. from the **constructor**), if new-expression allocated any storage, it calls the appropriate deallocation function: `operator delete` for non-array type, `operator delete[]` for array type.", [cppreference](https://en.cppreference.com/w/cpp/language/new#Construction)

```cpp
// default initialization
string *ps = new string; // initialized to empty string
int *pi = new int; // pi points to an uninitialized int

// (forms of) direct initialization:
// traditional:
int *pi = new int(1024); // object to which pi points has value 1024
string *ps = new string(10, '9'); // *ps is "9999999999"
// list initialization (direct-list-initialization)
vector<int> *pv = new vector<int>{0,1,2,3,4,5,6,7,8,9}; // vector with ten elements with values from 0 to 9

// value initialization:
string *ps1 = new string; // default initialized to the empty string
string *ps = new string(); // value initialized to the empty string
int *pi1 = new int; // default initialized; *pi1 is undefined
int *pi2 = new int(); // value initialized to 0; *pi2 is 0

// auto: only with a single initializer inside parentheses!
auto p1 = new auto(obj);    // p points to an object of the type of obj
                            // that object is initialized from obj
auto p2 = new auto{a,b,c};  // error: must use parentheses for the initializer

// const objects:
const int *pci = new const int(1024);   // allocate and initialize a const int
const string *pcs = new const string;   // allocate a default-initialized const empty string
// - recall, const objects MUST be initialized
// - pci and pcs are "pointers to const"
```

## Memory Exhaustion and Exceptions

Memory Exhaustion:
- by default, if `new` is unable to allocate the requested storage, it throws an **exception** of type `bad_alloc` (defined in the `new` header)

Prevent throwing an exception:
- **placement new:** a placement new lets us pass additional arguments to `new`
- to prevent `new` from throwing an exception pass an object named `nothrow` (defined in the `new` header):
  - if unable to allocate the requested storage, this form of `new` will return a null pointer

```cpp
// if allocation fails, new returns a null pointer
int *p1 = new int;              // if allocation fails, new throws std::bad_alloc
int *p2 = new (nothrow) int;    // if allocation fails, new returns a null pointer
```

- In order to prevent memory exhaustion, we must return dynamically allocated memory to the system once we are finished using it. We return memory through a `delete` expression.

## delete

- `delete`
  - takes a pointer to a dynamic object
  - destroys that object
  - frees the associated memory
- The pointer we pass to `delete` must either
  - point to dynamically allocated memory or
  - be a null pointer.
- **Warning:** Most compilers will accept the following situations, even though they are **undefined**:
  - deleting a pointer to memory that was not allocated by `new`: undefined
  - deleting the same pointer value more than once: undefined

```cpp
int i, *pi1 = &i, *pi2 = nullptr;
double *pd = new double(33), *pd2 = pd;
delete i;       // error: i is not a pointer
delete pi1;     // undefined: pi1 refers to a local
delete pd;      // ok
delete pd2;     // undefined: the memory pointed to by pd2 was already freed
delete pi2;     // ok: it is always ok to delete a null pointer

// const objects
const int *pci = new const int(1024);
delete pci;     // ok: deletes a const object
```

- cppreference: "If `expression` is not a null pointer (...), the `delete` expression invokes the **destructor** (if any) for the object that's being destroyed, or for every element of the array being destroyed (proceeding from the last element to the first element of the array)."

## Common Problems

1. Forgetting to delete memory &rarr; **"memory leak"**
  - Neglecting to delete dynamic memory is known as a memory leak, because the memory is never returned to the free store. 
  - Testing for memory leaks is difficult because they usually cannot be detected until the application is run for a long enough time to actually exhaust memory.
2. Using an object after it has been deleted &rarr; **"dangling pointer"**
  - after the `delete`, the pointer becomes what is referred to as a **dangling pointer** (Lip, p462)
  - This error can sometimes be detected by making the pointer null after the delete.
3. Deleting the same memory twice. &rarr; **"double disposal"**/**"double delete"**
  - This error can happen when two pointers address the same dynamically allocated object. 
  - If `delete` is applied to one of the pointers, then the object's memory is returned to the free store. 
  - If we subsequently `delete` the second pointer, then the free store may be corrupted.

### Dangling Pointers

- one that refers to memory that once held an object but no longer does so
- When we delete a pointer,
  - that pointer becomes **invalid**
  - on many machines the pointer continues to hold the address of the (freed) dynamic memory.
- dangling pointers have all the problems of **uninitialized pointers**
- avoid the problems by
  - deleting just before the pointer itself goes out of scope
  - if we need to keep the pointer around, we can assign `nullptr` after using `delete`
    - **Warning:** resetting a pointer has no effect on any of the other pointers

## Smart Pointers

- to implement the "no naked `new`" rule
- "Smart pointers enable automatic, exception-safe, object lifetime management."
- a smart pointer acts **like a regular pointer** with the important exception that it **automatically deletes** the object to which it points
- defined in `memory` header
- `unique_ptr`: "owns" the object to which it points
  - represents unique ownership (its destructor destroys its object) (BS)
- `shared_ptr`: allows multiple pointers to refer to the same object
  - represents shared ownership (the last shared pointer's destructor destroys the object) (BS)
  - `weak_ptr`: companion class, a weak reference to an object managed by a `shared_ptr`
- smart pointers are **templates** (ie must supply type in angle brackets)
- **default initialized** smart pointer hold a **null pointer**

### `shared_ptr` class

BS, p198

- similar to `unique_ptr` except that `shared_ptr`s are **copied** rather than moved 
  - `unique_ptr`s are "moved" &rarr; see section "Passing and Returning `unique_ptr`s"
- an object is destroyed when the last of its `shared_ptr`s is destroyed

Examples:

```cpp
shared_ptr<string> p1;  // shared_ptr that can point at a string
shared_ptr<list<int>> p2; // shared_ptr that can point at a list of ints
// if p1 is not null, check whether it's the empty string
if (p1 && p1->empty())
  *p1 = "hi"; // if so, dereference p1 to assign a new value to that string
```

slides:

- twice the size of a raw pointer

#### `make_shared`

- **best practice:** `make_shared` is the preferred method for constructing an object and returning an appropriate smart pointer
  - BS: Creating an object using `new` and passing it to a `shared_ptr` is
    - more verbose
      - therefore, less convenient
    - allows for mistakes
    - notably less efficient 
      - because it needs a separate allocation for the use count that is essential in the implementation of a `shared_ptr`
- `make_shared<type>(args)` (non-member function template of `std::shared_ptr`)
  - allocates and initializes an object in dynamic memory
  - returns a `shared_ptr` that points to that object
  - if we do not pass any arguments, then the object is **value initialized**

cppreference:

1) Constructs an object of type `T` and wraps it in a `std::shared_ptr` using `args` as the parameter list for the constructor of `T`.

```cpp
// note: only the 1st template parameter refers to the type "T" of the "shared_ptr<T>",
// the remaining template parameters "Args" refer to the type(s) of the 
// arguments "args" passed to one of the constructors of type "T" (see Lip examples below)
template< class T, class... Args >
shared_ptr<T> make_shared( Args&&... args );
```

Lip:

Examples:

```cpp
// shared_ptr that points to an int with value 42
shared_ptr<int> p3 = make_shared<int>(42);
// p4 points to a string with value 9999999999
shared_ptr<string> p4 = make_shared<string>(10, '9');
// p5 points to an int that is value initialized to 0
shared_ptr<int> p5 = make_shared<int>();

// ordinarily we use auto:
// p6 points to a dynamically allocated, empty vector<string>
auto p6 = make_shared<vector<string>>();
```

#### Copy and Assign `shared_ptr`s

- copyable and movable

Lippman:

- keeps track of how many other `shared_ptr`s point to the same object
- **reference count** (aka "use count"):
  - is **incremented** when we
    - copy a `shared_ptr`
    - use a `shared_ptr` to initialize another `shared_ptr`, 
    - use a `shared_ptr` as the right-hand operand of an assignment, 
    - pass it to a function by value
    - return it from a function by value
  - is **decremented** when
    - we assign a new value to the `shared_ptr`
    - the `shared_ptr` itself is destroyed
      - such as when a local `shared_ptr` goes out of scope
  - Once a `shared_ptr`'s counter goes to zero, the `shared_ptr` automatically frees the object that it manages
  - Note: The reference count does not have to be a counter. Some other data structure may be used. This is up to the implementation.

```cpp
auto p = make_shared<int>(42); // object to which p points has one user
auto q(p); // p and q point to the same object
// object to which p and q point has two users

auto r = make_shared<int>(42); // int to which r points has one user
r = q; // assign to r, making it point to a different address
// increase the use count for the object to which q points
// reduce the use count of the object to which r had pointed
// the object r had pointed to has no users; that object is automatically freed

// Returns the number of objects sharing with p; may be a slow
// operation, intended primarily for debugging purposes.
p.use_count()
// Returns true if p.use_count() is one; false otherwise.
p.unique()
```

#### Move

cppreference:

```cpp
shared_ptr( shared_ptr&& r ) noexcept; // (10) 

// 10) Move-constructs a `shared_ptr` from `r`. After the construction, `*this` contains 
//     a copy of the previous state of `r`, `r` is empty and its stored pointer is null.
```

Why moving instead of copying a `shared_ptr` is good: [stackoverflow](https://stackoverflow.com/questions/41871115/why-would-i-stdmove-an-stdshared-ptr)

#### Empty `shared_ptr`

- "A `shared_ptr` may also own no objects, in which case it is called **empty**", cppreference

What does `use_count` return, if a `shared_ptr` owns no object?

- "If there is no managed object, `0` is returned.", [cppreference](https://en.cppreference.com/w/cpp/memory/shared_ptr/use_count)

#### Destroy `shared_ptr`s

- automatically destroys the object to which that `shared_ptr` points via the destructor of the `shared_ptr` class, if the object's reference count goes to 0
- the destructor of the `shared_ptr` class
  - frees the resources that an object has allocated (all destructors do this)
  - reduce the use count of the object to which that `shared_ptr` points

#### Factory Functions

- `shared_ptr`s are often used for [factory functions](#factory)
  - which are called "factory" because they "produce" a **new** object ("a product")

```cpp
// factory returns a shared_ptr pointing to a dynamically allocated object
shared_ptr<Foo> factory(T arg)
{
  // process arg as appropriate
  // shared_ptr will take care of deleting this memory
  return make_shared<Foo>(arg);
}

void use_factory(T arg)
{
  shared_ptr<Foo> p = factory(arg);
  // use p
} // p goes out of scope => p is destroyed => its reference count is decremented; 
  // the memory to which p points is automatically freed

// - unlike above, here, the return statement in use_factory returns a copy of p to its caller
//   - Copying a shared_ptr adds to the reference count of that object
//   - therefore, the memory itself will not be freed (exactly, what we want!)
shared_ptr<Foo> use_factory(T arg)
{
  shared_ptr<Foo> p = factory(arg);
  // use p
  return p; // reference count is incremented when we return p
} // p goes out of scope; the local variable/object p is destroyed => its reference count is decremented;
  // the memory to which p points is not freed
```

Note: The same does not work with dynamic objects managed through **built-in pointers**:

```cpp
// factory returns a pointer to a dynamically allocated object
Foo* factory(T arg)
{
  // process arg as appropriate
  return new Foo(arg); // caller is responsible for deleting this memory
}

// "memory leak"
// - cannot be detected until the application is run for a long enough time to actually exhaust memory
void use_factory(T arg)
{
  Foo *p = factory(arg);
  // use p but do not delete it
} // p goes out of scope, but the memory to which p points is not freed!

// fix 1
void use_factory(T arg)
{
  Foo *p = factory(arg);
  // use p
  delete p; // remember to free the memory now that we no longer need it
}

// fix 2 (if we still need the allocated object)
Foo* use_factory(T arg)
{
  Foo *p = factory(arg);
  // use p
  return p; // caller must delete the memory
}
```

#### Factory Functions vs Constructors

read: [stackoverflow](https://stackoverflow.com/a/629006)

- "They both are there to create instance of an object."
- "So - for simple classes (value objects, etc.) constructor is just fine (you don't want to overengineer your application) but **for complex class hierarchies** factory method is a preferred way."

related: [When Is Factory Class Better Than Calling Constructor?](https://methodpoet.com/factory-method-vs-constructor/)
related: [List of "Design Patterns"](https://en.wikipedia.org/wiki/Design_Patterns#Patterns_by_type) (from the book written by the "Gang of Four")

#### Destroy `shared_ptr`s that are not needed

- make sure that `shared_ptr`s don't stay around after they are no longer needed
  - The program will execute correctly but **may waste memory** if you neglect to destroy `shared_ptr`s that the program does not need
  - example:
    - if you put `shared_ptr`s in a container, and you subsequently need to use some, but not all, of the elements, remember to `erase` the elements you no longer need

#### A Class with Resources That Have Dynamic Lifetime

**Problem:** by default, `vector` makes "deep copies"

```cpp
vector<string> v1; // empty vector
{ // new scope
  vector<string> v2 = {"a", "an", "the"};
  v1 = v2; // copies the elements from v2 into v1
} // v2 is destroyed, which destroys the elements in v2
  // v1 has three elements, which are copies (ie "deep" copies) of the ones originally in v2
```

**We want:** "shallow copies"

- define a class that uses dynamic memory in order to let several objects **share the same underlying data**

```cpp
Blob<string> b1; // empty Blob
{ // new scope
  Blob<string> b2 = {"a", "an", "the"};
  b1 = b2; // b1 and b2 share the same elements
} // b2 is destroyed, but the elements in b2 must not be destroyed
  // b1 points to the elements originally created in b2
```

**Solution:** store the `vector` in dynamic memory

```cpp
// a vector class that "shallow copies", unlike the std::vector class

class StrBlob {
  public:
    typedef std::vector<std::string>::size_type size_type;

    StrBlob();
    StrBlob(std::initializer_list<std::string> il);

    size_type size() const { return data->size(); }
    bool empty() const { return data->empty(); }

    // add and remove elements
    void push_back(const std::string &t) {data->push_back(t);}
    void pop_back();

    // element access
    std::string& front();
    std::string& back();

  private:
    std::shared_ptr<std::vector<std::string>> data;

    // throws msg if data[i] isn’t valid
    void check(size_type i, const std::string &msg) const;
};

// constructors
StrBlob::StrBlob(): data(make_shared<vector<string>>()) { }
StrBlob::StrBlob(initializer_list<string> il):
    data(make_shared<vector<string>>(il)) { }

// copy, assign, destroy:
// - use the default versions of the operations that copy, assign, and destroy objects of its type
//   - memberwise: the default versions copy, assign, and destroy the object's members
//     - StrBlob has only one data member, which is a shared_ptr
//       - the vector allocated by StrBlob constructors is automatically destroyed when the reference count goes to 0

// element access members
void StrBlob::check(size_type i, const string &msg) const
{
  if (i >= data->size())
    throw out_of_range(msg);
}
string& StrBlob::front()
{
  // if the vector is empty, check will throw
  check(0, "front on empty StrBlob");
  return data->front();
}
string& StrBlob::back()
{
  check(0, "back on empty StrBlob");
  return data->back();
}
void StrBlob::pop_back()
{
  check(0, "pop_back on empty StrBlob");
  data->pop_back();
}
```

#### Using `shared_ptr`s with `new`

```cpp
shared_ptr<double> p1; // shared_ptr that can point at a double
shared_ptr<int> p2(new int(42)); // p2 points to an int with value 42

// smart pointer constructors that take pointers are `explicit`
// (ie. cannot implicitly convert a built-in pointer to a smart pointer)
shared_ptr<int> p1 = new int(1024); // error: must use direct initialization
shared_ptr<int> p2(new int(1024)); // ok: uses direct initialization

// for the same reason, this return statement does not work
shared_ptr<int> clone(int p) {
  return new int(p); // error: implicit conversion to shared_ptr<int>
}

// instead, must explicitly bind a shared_ptr to the pointer
shared_ptr<int> clone(int p) {
  // ok: explicitly create a shared_ptr<int> from int*
  return shared_ptr<int>(new int(p));
}
```

#### Do Not Mix Ordinary Pointers and Smart Pointers

- **General Rule**: Once we give `shared_ptr` responsibility for a pointer, we should no longer use a built-in pointer to access the memory to which the `shared_ptr` now points.

```cpp
// ptr is created and initialized when process is called
void process(shared_ptr<int> ptr)
{
  // use ptr
} // ptr goes out of scope and is destroyed

// right way to use this function
shared_ptr<int> p(new int(42)); // reference count is 1
process(p);                     // copying p increments its count; in process the reference count is 2
int i = *p;                     // ok: reference count is 1

// WRONG way to use this function
int *x(new int(1024));        // dangerous: x is a plain pointer, not a smart pointer
process(x);                   // error: cannot convert int* to shared_ptr<int>
process(shared_ptr<int>(x));  // legal, but the memory will be deleted! => don't pass built-in pointer to temporary shared_ptr!
int j = *x;                   // undefined: x is a dangling pointer!
```

#### Do Not Use "get" to Initialize or Assign Another Smart Pointer

- `get`
  - returns a built-in pointer to the object that the smart pointer is managing
  - DO:
    - intended for cases when we need to pass a built-in pointer to **code that can't use a smart pointer**
  - DON'T:
    - code that uses the return from `get` **must not delete that pointer**
    - **must not bind another smart pointer** to the pointer returned by `get` (although the compiler will not complain)

```cpp
shared_ptr<int> p(new int(42)); // reference count is 1 (there is only ONE reference count for both p and q)
int *q = p.get();               // ok: but don't use q in any way that might delete its pointer
{ // new block
  // undefined: two independent "shared_ptr"s point to the same memory
  shared_ptr<int>(q);           // bind ANOTHER smart pointer to the pointer returned by `get`
  // note: this does not explicitly call the constructor, instead this line creates
  // a temporary unnamed object with type "shared_ptr", which is destroyed immediately
  // after ";" (see https://stackoverflow.com/a/18892056)
} // block ends, q is destroyed, and the memory to which q points is freed
int foo = *p;                   // undefined; the memory to which p points was freed
```

#### `reset`

- to assign a new pointer to a `shared_ptr`
- updates the reference counts
  - if appropriate, deletes the object to which the `shared_ptr` points

```cpp
shared_ptr<int> p(new int(42));
p = new int(1024);          // error: cannot assign a pointer to a shared_ptr
p.reset(new int(1024));     // ok: p points to a new object
```

```cpp
// from Table 12.3:
// If p is the only shared_ptr pointing at its object, reset frees
// p's existing object. If the optional built-in pointer q is passed,
// makes p point to q, otherwise makes p null.
p.reset()
p.reset(q)
```

- often used together with `unique` to control changes to the object shared among several `shared_ptr`s

```cpp
if (!p.unique())          // Before changing the underlying object, we check whether we're the only user.
  p.reset(new string(*p));    // we aren't alone; allocate a new "deep" copy (and let p point to it); 
                              // reduce the use count of the object to which p had pointed;
                              // other users can continue using the original object to which p had pointed
*p += newVal; // now that we know we're the only pointer, okay to change this object
```

#### Other Ways to Define `shared_ptr`s

```cpp
// from Table 12.3

// p manages the object to which the BUILT-IN POINTER q points;
// q must point to memory allocated by "new" and must be
// convertible to T*.
shared_ptr<T> p(q)

// p assumes ownership from the UNIQUE_PTR u; makes u null.
shared_ptr<T> p(u)
```

#### Special Problems

From lecture slides:

```cpp
int i = std::atoi(argv[1]);
// execution order: 
// 1. new X{}                       // allocate
// 2. do_something(i)               // may throw an exception!
// 3. std::shared_ptr<X>(...)       // memory allocated in 1. is bound to a shared_ptr
// 4. f()                           // where 2. and 3. are passed to f()
//
// -> will leak, if do_something() throws an exception in step 2.!
try { f(std::shared_ptr<X>(new X{}), do_something(i)); }
catch (int&) { std::cout << "exception main\n"; }
```

BS15.2.1:
- in code below: 
  - unlike `unique_ptr`, `shared_ptr` are **copied** rather than moved
  - in the code below `f()` or `g()` may spawn a task holding a copy of `fp` or in some other way store a copy that outlives `user()`
    - **problem**: makes the lifetime of the shared object hard to predict
  - **best practice:** use `shared_ptr` only if you actually need shared ownership

```cpp
// problem: lifetime of the shared object (an fstream) is hard to predict

void f(shared_ptr<fstream>);
void g(shared_ptr<fstream>);

// cppreference:
// ios_base is a multipurpose class that serves as the base class for all I/O stream classes.
void user(const string& name, ios_base::openmode mode)
{
  shared_ptr<fstream> fp {new fstream(name,mode)};
  if (!∗fp)         // make sure the file was properly opened
    throw No_file{};

  f(fp);
  g(fp);
  // ...
}
```

### `unique_ptr` class

- "owns" the object to which it points
- only one `unique_ptr` at a time can point to a given object
- movable but not copyable
- `release`
  - returns the pointer currently stored in the `unique_ptr` 
  - makes that `unique_ptr` null
- `reset`
  - takes an optional pointer 
  - repositions the `unique_ptr` to point to the given pointer
  - if the `unique_ptr` is not null, then the object to which the `unique_ptr` had pointed is deleted

BS15.2.1
- A `unique_ptr` is a **handle to an individual object** (or an array) 
  - in much the same way that a `vector` is a **handle to a sequence of objects**. 
  - Both control the lifetime of other objects (using RAII) and
  - both rely on **elimination of copying** (copy elision) or on **move semantics** to make `return` simple and efficient

#### Declare, Initialize

```cpp
// declaration
unique_ptr<double> p1; // unique_ptr that can point at a double
// must use the direct form of initialization
unique_ptr<int> p2(new int(42)); // p2 points to int with value 42
```

#### Copy Control

- movable but not copyable

```cpp
// does not support ordinary copy or assignment
unique_ptr<string> p1(new string("Stegosaurus"));
unique_ptr<string> p2(p1);  // error: no copy for unique_ptr
unique_ptr<string> p3;
p3 = p2;                    // error: no assign for unique_ptr

// transfers ownership from p1 (which points to the string Stegosaurus ) to p2
unique_ptr<string> p2(p1.release()); // release makes p1 null
unique_ptr<string> p3(new string("Trex"));
// transfers ownership from p3 to p2
p2.reset(p3.release()); // reset deletes the memory to which p2 had pointed

// problem with "release"
p2.release(); // WRONG: p2 won't free the memory and we’ve lost the pointer
auto p = p2.release(); // ok, but we must remember to delete(p)

// Deletes the object to which p1 points; makes p1 null. (from Table 12.4)
p1 = nullptr
// or equivalently:
p1.reset()          // option 1
p1.reset(nullptr)   // option 2
```

#### `make_unique`

- since C++14
- in C++11: there is no library function comparable to `make_shared` that returns a `unique_ptr`
  - instead, we bind it to a pointer returned by `new`

```cpp
struct S {
  int i;
  string s;
  double d;
  // ...
};

auto p1 = make_shared<S>(1,"Ankh Morpork",4.65); // p1 is a shared_ptr<S>
auto p2 = make_unique<S>(2,"Oz",7.62);           // p2 is a unique_ptr<S>
```

#### `release`

```cpp
u.release()
```

- relinquishes control of the pointer `u` had held
- returns the pointer `u` had held
- makes `u` null

#### `reset`

```cpp
// from Table 12.4:
// Deletes the object to which u points;
// If the built-in pointer q is supplied, makes u point to that object.
// Otherwise makes u null.
u.reset()         // makes "u" "equivalent to" nullptr (see Question 1 below)
u.reset(q)
u.reset(nullptr)
```

**Question 1:** What is the result of `u.reset()` without an argument?

cppreference:

```cpp
void reset( pointer ptr = pointer() ) noexcept;     // (1)
```

- only `(1)` is important
- `(2)` and `(3)` refer to `unique_ptr`s to **arrays**
- `(1)` says that
  - `reset`'s default argument is `pointer()`, ie. a nameless temporary of type `pointer`
    - `pointer` is a **"member type"** (see [cppreference](https://en.cppreference.com/w/cpp/memory/unique_ptr#Member_types)) of class template `std::unique_ptr`
      - `pointer` defaults to `T*` (see [cppreference](https://en.cppreference.com/w/cpp/memory/unique_ptr#Member_types))
  - `reset` is a normal member function (not a template itself!)
  - hence, if no argument is provided, `reset()` defaults to `reset(T*())` (reset to a **value initialized** nameless temporary of type `T*`)
    - value initialization always zero-initializes first, so that built-in pointer types `T*` are reset to `(T*)0`
    - since the C-style cast uses `static_cast` (here in this case), `(T*)0` returns `static_cast<T*>(0)`
    - from `static_cast` cppreference: "returns the imaginary variable `Temp` initialized as if by `target-type Temp(expression);`"
      - thus, `static_cast<T*>(0)` will return `T* Temp(0)`
    - since `unique_ptr` requires that its member type `pointer` must be a `NullablePointer`
      - thus, the type `T*` in `T* Temp(0)` must satisfy:

```cpp
// cppreference:
// The type must satisfy the following additional expressions, given 
// two values p and q that are of the type, and that np is a value 
// of std::nullptr_t type (possibly const qualified): 

Type p(np);   // Afterwards, p is equivalent to nullptr
Type p = np;  // Afterwards, p is equivalent to nullptr
```

- thus, `Temp` in `T* Temp(0)` is equivalent to `nullptr`
- thus, `u.reset()` without an argument will return `nullptr`

#### Prevents Memory Leaks

```cpp
// BS15.2.1: 
// most basic use of these "smart pointers" is to prevent memory leaks
void f(int i, int j)          // X* vs. unique_ptr<X>
{
  X∗ p = new X;               // allocate a new X
  unique_ptr<X> sp {new X};   // allocate a new X and give its pointer to unique_ptr
  // ...
  if (i<99) throw Z{};        // may throw an exception
  if (j<77) return;           // may return "early"
  // ... use p and sp ..
  delete p;                   // destroy *p
}
// - we delete p, BUT we "forgot" to delete p if i<99 or if j<77
// - only a "unique_ptr" ensures that its object is properly destroyed whichever way 
//   we exit f() (by throwing an exception, by executing return, or by "falling off the end")
```

#### Passing and Returning `unique_ptr`s

- one exception to the rule that we cannot copy a `unique_ptr`:
  - We can copy or assign a `unique_ptr` **that is about to be destroyed**
- in the following cases, the compiler does a special kind of "copy", a "move":

```cpp
unique_ptr<int> clone(int p) {
  // ok: explicitly create a unique_ptr<int> from int*
  return unique_ptr<int>(new int(p));
}

// alternatively: return a copy of a local object
unique_ptr<int> clone(int p) {
  unique_ptr<int> ret(new int (p));
  // . . .
  return ret;
}

// from BS:
// often used for passing free-store allocated objects in and out of functions
unique_ptr<X> make_X(int i)
// make an X and immediately give it to a unique_ptr
{
  // ... check i, etc. ...
  return unique_ptr<X>{new X{i}};
}
```

### `weak_ptr` class

- a companion class
- points to an object that is managed by a `shared_ptr`
  - binding a `weak_ptr` to a `shared_ptr` does not change the reference count of that `shared_ptr`
  - an object will be deleted even if there are `weak_ptrs` pointing to it
- hence, a smart pointer that does **not** control the lifetime of the object to which it points
- we cannot use a `weak_ptr` to access its object directly

```cpp
// initialize a weak_ptr from a shared_ptr
auto p = make_shared<int>(42);
weak_ptr<int> wp(p); // wp weakly shares with p; use count in p is unchanged
```

#### `lock()`

- to access a `weak_ptr`'s object, we must call `lock()`
  - the `lock()` function checks whether the object to which the `weak_ptr` points still exists
  - if so, `lock()` returns a `shared_ptr` to the shared object

```cpp
if (shared_ptr<int> np = wp.lock()) { // true if np is not null
  // inside the if, np shares its object with p
}
```

### Best practices

#### When to use Smart Pointers?

BS15.2.1

- try to not use them
  - smart pointers do not address eg. **data races**
  - smart pointers do not in themselves provide any **rules** for which of their owners can **read and/or write the shared object**
- first, try to use **containers** and other types that manage their resources
- in general, use them only when we really need pointer semantics:
  - when we share an object &rarr; `shared_ptr`
  - when there is an obvious single owner &rarr; `unique_ptr`
  - when we refer to a polymorphic object &rarr; `unique_ptr`
  - a shared polymorphic object &rarr; `shared_ptr`

#### typedef smart pointer types

From [isocpp](https://isocpp.org/wiki/faq/exceptions#selfcleaning-members):

By the way, if you think your `Fred` class is going to be allocated into a smart pointer, be nice to your users and create a `typedef` within your `Fred` class:

```cpp
#include <memory>
class Fred {
public:
  typedef std::unique_ptr<Fred> Ptr;
  // ...
};
```

That `typedef` simplifies the syntax of all the code that uses your objects: your users can say `Fred::Ptr` instead of `std::unique_ptr<Fred>`:

```cpp
#include "Fred.h"
void f(std::unique_ptr<Fred> p);  // explicit but verbose
void f(Fred::Ptr             p);  // simpler
void g()
{
  std::unique_ptr<Fred> p1( new Fred() );  // explicit but verbose
  Fred::Ptr             p2( new Fred() );  // simpler
  // ...
}
```

## RAII

- "Resource Acquisition Is Initialization"
  - **resources** ("anything that exists in limited supply")
    - allocated heap memory (here: acquisition = allocation)
    - thread of execution
    - open socket
    - open file
    - locked mutex
    - disk space
    - database connection
- aka
  - Constructor Acquires, Destructor Releases (CADRe)
  - Scope-Bound Resource Management (SBRM) (for the special case of automatic variables)
    - name because of "the basic use case where the lifetime of an RAII object ends due to scope exit"
  - Destruction is Resource Release, [stackoverflow](https://stackoverflow.com/a/4258097)
- read: [cppreference: raii](https://en.cppreference.com/w/cpp/language/raii)
- RAII guarantees that 
  - the resource is available to any function that may access the object (resource availability is a **class invariant**, eliminating redundant runtime tests). 
  - all resources are released when the lifetime of their controlling object ends, in reverse order of acquisition. 
  - Likewise, **if resource acquisition fails** (the constructor exits with an exception), all resources acquired by every fully-constructed member and base subobject are released in reverse order of initialization.

BS5.2.2
- The technique of 
  - acquiring resources in a constructor and 
  - releasing them in a destructor
- basis for most C++ general resource management techniques
  - very commonly used to manage **data that can vary in size** during the lifetime of an object
- allows us to eliminate 
  - "naked `new` operations"
    - that is, to avoid allocations in general code and keep them buried inside the implementation of well-behaved abstractions
  - "naked `delete` operations"
  - Avoiding naked `new` and naked `delete` makes code 
    - far less error-prone and 
    - far easier to keep free of resource leaks

Wikipedia
- In RAII
  - holding a resource is a class [invariant](#invariants), and is tied to object [lifetime](#lifetimes)
  - resource **allocation** (or acquisition) is done during object creation (specifically **initialization**), by the **constructor**
  - resource **deallocation** (release) is done during object destruction (specifically finalization), by the **destructor**
  - In other words, resource acquisition must succeed for initialization to succeed.

# Dynamic Memory and Arrays

**two ways** to allocate an array of objects at once:

1. a second kind of new expression that allocates and initializes an array of objects
2. a template class named `allocator` that lets us separate allocation from initialization
  - using an `allocator` generally provides better performance and more flexible memory management

- **Warning:** Classes that allocate dynamic arrays must define **their own** copy, assignment and destruction
  - **best practice:** use containers (easier, faster and safer)

## new

### Initialization

- by default, **default initialized**
- we can also
  - **value initialize**
  - **list initialize**
    - initializers are used to initialize the first elements in the array
    - If there are fewer initializers than elements, the remaining elements are value initialized
    - If there are more initializers than the given size, then the new expression fails and no storage is allocated
      - In this case, `new` throws an exception of type `bad_array_new_length` (in `new` header)
- we **cannot** use `auto` to allocate an array

```cpp
// value initialization
int *pia = new int[10];           // block of ten uninitialized ints
int *pia2 = new int[10]();        // block of ten ints value initialized to 0
string *psa = new string[10];     // block of ten empty strings
string *psa2 = new string[10]();  // block of ten empty strings

// list initialization
// block of ten ints each initialized from the corresponding initializer
int *pia3 = new int[10]{0,1,2,3,4,5,6,7,8,9};
// block of ten strings; the first four are initialized from the given initializers
// remaining elements are value initialized
string *psa3 = new string[10]{"a", "an", "the", string(3,’x’)};
```

### Empty Arrays

- it is **legal** to dynamically allocate an empty array
  - even though we cannot create an array variable of size 0

```cpp
char arr[0];              // error: cannot define a zero-length array
char *cp = new char[0];   // ok: but cp can't be dereferenced
```

### begin and end

- cannot call `begin` or `end`
  - because when we use `new` to allocate an array, we get a pointer to the element type of the array **and not an array**

### range for

- cannot use a range `for`
  - because when we use `new` to allocate an array, we get a pointer to the element type of the array **and not an array**

## allocator

- like `new`, but `allocator` decouples memory **allocation** from **object construction**
- allocates raw, **unconstructed** memory
- a template
- cppreference: "The `std::allocator` class template is the default *Allocator* **used by all standard library containers** if no user-specified allocator is provided."

```cpp
// define an allocator
allocator<string> alloc;            // object that can allocate strings
auto const p = alloc.allocate(n);   // allocate n unconstructed strings
```

### construct

We use this unconstructed memory from `allocate()` by constructing objects in that memory:

The `construct(p, args)` member takes:

1. a pointer `p`
  - `construct` constructs an element at the location given by the pointer
2. zero or more additional arguments `args`
  - are used to **initialize** the object being constructed
  - if `args` are of class type
    - `args` are passed to a constructor for type T

```cpp
auto q = p; // q will point to one past the last constructed element
alloc.construct(q++);               // *q is the empty string
alloc.construct(q++, 10, 'c');      // *q is cccccccccc
alloc.construct(q++, "hi");         // *q is hi!
```

Using unconstructed memory is undefined:

```cpp
cout << *p << endl; // ok: uses the string output operator
cout << *q << endl; // disaster: q points to unconstructed memory!
```

### destroy

After using the constructed elements they must be destroyed:

```cpp
while (q != p)
  alloc.destroy(--q); // free the strings we actually allocated
```

**Warning:** Destroy only elements that are actually constructed.

### deallocate

Once the elements have been destroyed, return the memory to the system:

```cpp
alloc.deallocate(p, n);
```

The pointer `p` 
- must not be null
- must point to memory allocated by `allocate`

The size argument `n` must be the same size as used in the call to `allocate` that obtained the memory.
