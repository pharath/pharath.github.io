---
title: "C++ Notes - Dynamic Memory"
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
    - smartpointers
    - dynamicmemory
    - notes

---

# References

## Lvalue References

- aka **lvalue references**
- from [isocpp](https://isocpp.org/wiki/faq/references#reseating-refs):
  - Unlike a pointer, once a reference is bound to an object, it can not be "reseated" to another object. 
  - The reference itself isn't an object 
    - it has no identity; 
    - taking the address of a reference gives you the address of the referent; 
    - remember: the reference **is** its referent
- idea
  - Normally, when you use a reference, you do not use the address-of operator. You simply use the reference as you would use the target variable.
    - references are **aliases** for their target
  - enables the function to change the object being referred to

```cpp
int intOne;
int &rSomeRef = intOne;
intOne = 5;

// intOne: 5
// rSomeRef: 5
// &intOne: 0x3500
// &rSomeRef: 0x3500
```

- references (unlike other variables)
  - must be initialized when they are declared
  - cannot be reassigned
- space **before** the address-of operator is **required**
- usually "the type of a reference must match the type of the object to which it refers"
  - 2 exceptions:
    - 1) "we can initialize a reference to const from any expression that can be converted to the type of the reference" (see [reference to const](#reference-to-const))

### reference to const 

- aka "`const` reference", "lvalue reference to a `const` value"
- `X const& x` is equivalent to `const X& x`, and `X const* x` is equivalent to `const X* x`. (see [isocpp.org](https://isocpp.org/wiki/faq/const-correctness#const-ref-nonsense))
- Unlike an ordinary reference, a reference to `const` cannot be used to change the object to which the reference is bound
- "we can **initialize** a reference to `const` from any expression that can be converted to the type of the reference"
  - we can bind a reference to `const` to 
    - a nonconst object, 
    - a literal, or 
    - a more general expression
    - an object of a different type (in this case, the reference is bound to a [temporary](#temporaries) object, see p.62)
- "Binding a reference to `const` to an object says nothing about whether the underlying object itself is `const`."
  - ie. the underlying object cannot be changed by using the reference to `const`, but it might be changed by other means
- binding a **non-const** reference to a [temporary](#temporaries) is illegal in C++, only **const** references may be bound to a temporary (see p.62)

```cpp
const int ci = 1024;
const int &r1 = ci;     // ok: both reference and underlying object are const
r1 = 42;                // error: r1 is a reference to const
int &r2 = ci;           // error: nonconst reference to a const object

// Initialization and References to const:
int i = 42;
const int &r1 = i;        // we can bind a const int& to a plain int object
const int &r2 = 42;       // ok: r1 is a reference to const
const int &r3 = r1 * 2;   // ok: r3 is a reference to const
int &r4 = r * 2;          // error: r4 is a plain, non const reference

// Temporaries and References to const:
// see examples in section "Temporaries"

// A Reference to const May Refer to an Object That Is Not const:
int i = 42;
int &r1 = i;          // r1 bound to i
const int &r2 = i;    // r2 also bound to i; but cannot be used to change i
r1 = 0;               // r1 is not const; i is now 0
r2 = 0;               // error: r2 is a reference to const
```

### const references

- usually refers to [reference to const](#reference-to-const)

From [isocpp.org](https://isocpp.org/wiki/faq/const-correctness#const-ref-nonsense):
- references are always `const`
- `X& const x` is functionally equivalent to `X& x`. Since you're gaining nothing by adding the `const` after the `&`, you shouldn't add it: it will confuse people — the `const` will make some people think that the `X` is const, as if you had said `const X& x`.

## Rvalue References

- introduced by the new standard to support move operations
- a reference that must be bound to an rvalue
- obtained by using `&&`
- may be bound only to an object that is about to be destroyed
  - so that we are free to "move" resources from an rvalue reference to another object
- rvalue and lvalue references have the opposite **binding properties**:

```cpp
int i = 42;
int &r = i;             // ok: r refers to i
int &&rr = i;           // error: cannot bind an rvalue reference to an lvalue
int &r2 = i * 42;       // error: i * 42 is an rvalue
const int &r3 = i * 42; // ok: we can bind a reference to const to an rvalue
int &&rr2 = i * 42;     // ok: bind rr2 to the result of the multiplication
```

Examples of expressions that return
- lvalues
  - Functions that return lvalue references
  - operators
    - assignment
    - subscript
    - dereference
    - prefix increment/decrement (`++i`)
  - Variable expressions
- rvalues (prvalues)
  - Functions that return a nonreference type
  - operators
    - arithmetic
    - relational
    - bitwise
    - postfix increment/decrement (`i++`)
  - literals

- rvalue references refer to objects that are about to be destroyed
- **ephemeral** vs **persistent**: 
  - lvalues have persistent state, whereas **rvalues** are either **literals** or **temporary objects**
- code that uses an rvalue reference is free to take over value/resources/state from the object to which the reference refers
  - "steal" value/resources/state from the object (rhs) bound to an rvalue reference (lhs)
- you **cannot** directly bind an rvalue reference **to a variable**, even if that variable was defined as an rvalue reference type

```cpp
int &&rr1 = 42;         // ok: literals are rvalues
// Error:
int &&rr2 = rr1;        // error: the expression rr1 is an lvalue!
```

## Reference to Pointer

From [stackoverflow](https://stackoverflow.com/a/1898556):
- a reference to a pointer is like a reference to any other variable

```cpp
void fun(int*& ref_to_ptr)
{
    ref_to_ptr = 0; // set the "passed" pointer to 0
    // if the pointer is not passed by ref,
    // then only the copy(parameter) you received is set to 0,
    // but the original pointer(outside the function) is not affected.
}
```

## Pointer to Reference

From [stackoverflow](https://stackoverflow.com/a/1898556):
- A pointer to reference is illegal in C++
  - because - unlike a pointer - a reference is just a concept that allows the programmer to make **aliases** of something else.
- A pointer is a **place in memory** that has the address of something else, but a reference is NOT.

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

- [Bjarne Stroustrup comment](https://www.stroustrup.com/bs_faq2.html#null)
- ([g++ doc](https://gcc.gnu.org/onlinedocs/libstdc++/manual/support.html#std.support.types.null))
- `NULL` ist definiert als [macro](https://gcc.gnu.org/onlinedocs/cpp/Macros.html) (i.e. a piece of code in a program that is replaced by the value of the macro; a macro is defined by `#define` directive; whenever a macro name is encountered by the [preprocessor](https://en.wikipedia.org/wiki/Preprocessor), it replaces the name with the definition of the macro): 
  - From [cppreference](https://en.cppreference.com/w/cpp/types/NULL):
    ```cpp
    #define NULL 0
    //since C++11
    #define NULL nullptr
    ```
  - dh `NULL` und `0` waren früher **dasselbe** [bis C++11] und jetzt sind `NULL` und `nullptr` **dasselbe** (aber how come [implicit cast difference](#implicit-cast-of-null-and-nullptr)?)
- "Unless you need to be compatible with C++98/C++03 or C you should prefer to use `nullptr` instead of `NULL`." ([g++ doc](https://gcc.gnu.org/onlinedocs/libstdc++/manual/support.html#std.support.types.null))

### Implicit Cast of NULL and nullptr

#### To Pointer Types

- `NULL` und `nullptr` beide implicitly convertible to any **pointer** type
  - A **null pointer** constant (see `NULL`), can be converted to any **pointer** type [i.e. type with asterisk `*`], and the result is the null pointer value of that type. ([cppreference](https://en.cppreference.com/w/cpp/language/implicit_conversion))

#### To Integral Types

- Unlike `NULL`, `nullptr` is **not** implicitly convertible or comparable to integral types [e.g. `int`, `char`] ([geeksforgeeks](https://www.geeksforgeeks.org/understanding-nullptr-c/))
  - `int x = NULL` geht; 
  - `int x = nullptr` geht **nicht**!
- `nullptr` is of type `nullptr_t`, which is implicitly convertible and comparable to any pointer type or pointer-to-member type. **It is not implicitly convertible or comparable to integral types** [e.g. `int`, `char`], **except for** `bool`. ([C++11](https://en.wikipedia.org/wiki/C%2B%2B11#Null_pointer_constant))

### Type of NULL and nullptr

In C++11 hat `NULL` den type `nullptr_t` 
- `nullptr_t` "is a distinct type that is not itself a pointer type or a pointer to member type." ([doc](https://en.cppreference.com/w/cpp/types/nullptr_t))

"In C, the macro `NULL` may have the type `void*`, but that is not allowed in C++." ([cppreference](https://en.cppreference.com/w/cpp/types/NULL))
- how come?: [stackoverflow](https://stackoverflow.com/a/69057243)
- soll heißen: `NULL` hat in C++ **absichtlich nicht (wie in C)** den Type `void*`, weil there is no **implicit cast** from `void*` to any other type in C++ (in C wäre das aber möglich!). Bis C++11 war `NULL` das [integer literal](https://en.cppreference.com/w/cpp/language/integer_literal) "`0`", konnte damit also einen der integer literal types (s. Tabelle in [integer literal](https://en.cppreference.com/w/cpp/language/integer_literal)) haben. Seit C++11 hat `NULL` den type `nullptr_t` ([doc](https://en.cppreference.com/w/cpp/types/nullptr_t)).

dh zB
```cpp
void* ptr = nullptr; 
int foo = *ptr;    // this implicit cast is not allowed in C++
```
gibt einen Compiler Error `error: ‘void*’ is not a pointer-to-object type` (**fix**: use a different type (a "pointer-to-object" type) instead of `void*` -  `void*` is a "pointer-to-nothing")

Aber 
```cpp
char* ptr = nullptr; 
int foo = *ptr;
```
gibt keinen Compiler Error, weil `char*` ein pointer-to-object type ist.

# Arrays

- are a compound type
- the **dimension** must be a **constant expression**
- cannot use `auto` to deduce the type
- elements in an array are **default initialized**
  - thus, an array of built-in type that is defined inside a function will have undefined values.
- An **array name without brackets** is a pointer to the array's first element (JA202)
  - You can also use the expression `&data[0]` to obtain the address of the array's first element (JA202)
  - The name of an array is a **pointer constant**; it can't be changed and remains fixed for the entire time the program executes (JA202)
    - You can, however, declare a pointer variable and initialize it to point at the array:

```cpp
int array[100], *p_array;
/* additional code goes here */
p_array = array;
```

- C-style arrays do not support Copy or Assignment (but `std::array` does!)

```cpp
int a[] = {0, 1, 2};  // array of three ints
int a2[] = a;         // error: cannot initialize one array with another
a2 = a;               // error: cannot assign one array to another
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

// "compare pointers" using relational operators:

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

- pointer arithmetic is also valid 
  - for null pointers
    - If `p` is a null pointer, we can add or subtract an integral constant expression whose value is `0` to `p`.
    - We can also subtract two null pointers from one another, in which case the result is `0`.
  - for pointers that point to an object that is not an array.
    - the pointers must point to the same object, or one past that object. 

## References and Pointers to Arrays

```cpp
int *ptrs[10];              // ptrs is an array of ten pointers to int
int &refs[10] = /* ? */;    // error: no arrays of references
int (*Parray)[10] = &arr;   // Parray points to an array of ten ints
int (&arrRef)[10] = arr;    // arrRef refers to an array of ten ints
```

Similarly, we can pass a parameter that is a reference to an array:

```cpp
void print(int (&arr)[10])   // the dimension is part of the type
{
  for (auto elem : arr)
    cout << elem << endl;
}
```

## Returning a Pointer to an Array

```cpp
// Option 1:
// Type (*function(parameter_list))[dimension]
int (*func(int i))[10];

// Option 2: trailing return type syntax:
// fcn takes an int argument and returns a pointer to an array of ten ints
auto func(int i) -> int(*)[10];
```

# Dynamic Memory

- managed through
  - `new`: 
    - allocates, and optionally initializes, an object in dynamic memory 
    - returns a pointer to that object
    - if the constructor throws an exception:
      - "If initialization terminates by throwing an exception (e.g. from the **constructor**), if new-expression allocated any storage, it calls the appropriate deallocation function: `operator delete` for non-array type, `operator delete[]` for array type.", [cppreference](https://en.cppreference.com/w/cpp/language/new#Construction)
  - `delete`
    - takes a pointer to a dynamic object,
    - destroys that object,
    - frees the associated memory
- by default, dynamically allocated objects are **default initialized**
  - built-in or compound type: undefined value
  - class type: default constructor
- we can also use
  - direct initialization
  - value initialization
- **Best practice:** always initialize dynamically allocated objects

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

- The pointer we pass to delete must either
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

## Common Problems

1. Forgetting to delete memory &rarr; **"memory leak"**
  - Neglecting to delete dynamic memory is known as a memory leak, because the memory is never returned to the free store. 
  - Testing for memory leaks is difficult because they usually cannot be detected until the application is run for a long enough time to actually exhaust memory.
2. Using an object after it has been deleted. 
  - This error can sometimes be detected by making the pointer null after the delete.
3. Deleting the same memory twice. 
  - This error can happen when two pointers address the same dynamically allocated object. 
  - If `delete` is applied to one of the pointers, then the object's memory is returned to the free store. 
  - If we subsequently `delete` the second pointer, then the free store may be corrupted.

## Dangling Pointers

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
- **best practice:**
  - try to not use them
  - try to use containers and other types that manage their resources
  - in general, only when we really need pointer semantics:
    - When we share an object
    - When we refer to a polymorphic object
    - A shared polymorphic object

### `shared_ptr` class

- smart pointers are **templates** (ie must supply type in angle brackets)
- **default initialized** smart pointer hold a **null pointer**
- similar to `unique_ptr` except that `shared_ptrs` are **copied** rather than moved (BS)

Examples:

```cpp
shared_ptr<string> p1;  // shared_ptr that can point at a string
shared_ptr<list<int>> p2; // shared_ptr that can point at a list of ints
// if p1 is not null, check whether it's the empty string
if (p1 && p1->empty())
  *p1 = "hi"; // if so, dereference p1 to assign a new value to that string
```

#### `make_shared`

- **best practice:** `make_shared` is the preferred method for constructing an object and returning an appropriate smart pointer
  - BS: Creating an object using `new` and passing it to a `shared_ptr` is
    - more verbose
    - allows for mistakes
    - notably less efficient 
      - because it needs a separate allocation for the use count that is essential in the implementation of a `shared_ptr`
- `make_shared<type>(args)` (non-member function template of `std::shared_ptr`)
  - allocates and initializes an object in dynamic memory
  - returns a `shared_ptr` that points to that object
  - if we do not pass any arguments, then the object is **value initialized**

1) Constructs an object of type `T` and wraps it in a `std::shared_ptr` using `args` as the parameter list for the constructor of `T`.

```cpp
template< class T, class... Args >
shared_ptr<T> make_shared( Args&&... args );
```

Examples:

```cpp
// shared_ptr that points to an int with value 42
shared_ptr<int> p3 = make_shared<int>(42);
// p4 points to a string with value 9999999999
shared_ptr<string> p4 = make_shared<string>(10, '9');
// p5 points to an int that is value initialized (§ 3.3.1 (p. 98)) to 0
shared_ptr<int> p5 = make_shared<int>();

// ordinarily we use auto:
// p6 points to a dynamically allocated, empty vector<string>
auto p6 = make_shared<vector<string>>();
```

- keeps track of how many other `shared_ptrs` point to the same object
- **reference count** (aka "use count"):
  - is **incremented** when we
    - copy a `shared_ptr`
    - use a `shared_ptr` to initialize another `shared_ptr`, 
    - use a `shared_ptr` as the right-hand operand of an assignment, 
    - pass it to a function
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
```

- automatically destroys the object to which that `shared_ptr` points via the `shared_ptr` **destructor** (the destructor of the `shared_ptr` class)
- the destructor of the `shared_ptr` class
  - frees the resources that an object has allocated (all destructors do this)
  - decrements the reference count of the object to which that `shared_ptr` points
- often used for [factory functions](#factory)

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
} // p goes out of scope; the memory to which p points is automatically freed

// - the return statement in use_factory returns a copy of p to its caller
//   - Copying a shared_ptr adds to the reference count of that object
//   - therefore, the memory itself will not be freed (exactly, what we want!)
shared_ptr<Foo> use_factory(T arg)
{
  shared_ptr<Foo> p = factory(arg);
  // use p
  return p; // reference count is incremented when we return p
} // p goes out of scope; the local variable/object p is destroyed; the memory to which p points is not freed
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

- make sure that `shared_ptrs` don't stay around after they are no longer needed
  - The program will execute correctly but **may waste memory** if you neglect to destroy `shared_ptrs` that the program does not need

#### A Class with Resources That Have Dynamic Lifetime

```cpp
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

// copy, assign, destroy:
// - use the default versions of the operations that copy, assign, and destroy objects of its type
//   - memberwise: the default versions copy, assign, and destroy the object's members
//     - StrBlob has only one data member, which is a shared_ptr
//       - the vector allocated by StrBlob constructors is automatically destroyed when the reference count goes to 0
```

#### using "new"

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

- General Rule: Once we give `shared_ptr` responsibility for a pointer, we should no longer use a built-in pointer to access the memory to which the `shared_ptr` now points.

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
  - intended for cases when we need to pass a built-in pointer to **code that can't use a smart pointer**
  - code that uses the return from `get` **must not delete that pointer**
  - must not bind another smart pointer to the pointer returned by `get` (although the compiler will not complain)

```cpp
shared_ptr<int> p(new int(42)); // reference count is 1 (there is only ONE reference count for both p and q)
int *q = p.get(); // ok: but don't use q in any way that might delete its pointer
{ // new block
  // undefined: two independent shared_ptrs point to the same memory
  shared_ptr<int>(q);           // bind ANOTHER smart pointer to the pointer returned by `get`
  // note: this does not explicitly call the constructor, instead this line creates a temporary unnamed object with type "shared_ptr", which is destroyed immediately after ";" (see https://stackoverflow.com/a/18892056)
} // block ends, q is destroyed, and the memory to which q points is freed
int foo = *p; // undefined; the memory to which p points was freed
```

#### `reset`

- to assign a new pointer to a `shared_ptr`
- updates the reference counts
  - if appropriate, deletes the object to which the `shared_ptr` points
- often used together with `unique` to control changes to the object shared among several `shared_ptrs`

```cpp
shared_ptr<int> p(new int(42));
p = new int(1024);          // error: cannot assign a pointer to a shared_ptr
p.reset(new int(1024));     // ok: p points to a new object

if (!p.unique())          // Before changing the underlying object, we check whether we're the only user.
  p.reset(new string(*p));    // we aren't alone; allocate a new "deep" copy (to which p points now); 
                              // other users can continue using the original object to which p pointed
*p += newVal; // now that we know we're the only pointer, okay to change this object
```

#### Problems

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
  - unlike `unique_ptr`, `shared_ptr` are copied rather than moved
  - in the code below `f()` or `g()` may spawn a task holding a copy of `fp` or in some other way store a copy that outlives `user()`
    - **problem**: makes the lifetime of the shared object hard to predict

```cpp
void f(shared_ptr<fstream>);
void g(shared_ptr<fstream>);

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
- there is no library function comparable to `make_shared` that returns a `unique_ptr`
  - instead, we bind it to a pointer returned by `new`
- `release`
  - returns the pointer currently stored in the `unique_ptr` 
  - makes that `unique_ptr` null
- `reset`
  - takes an optional pointer 
  - repositions the `unique_ptr` to point to the given pointer
  - if the `unique_ptr` is not null, then the object to which the `unique_ptr` had pointed is deleted

```cpp
// declaration
unique_ptr<double> p1; // unique_ptr that can point at a double
// must use the direct form of initialization
unique_ptr<int> p2(new int(42)); // p2 points to int with value 42

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

// Deletes the object to which p1 points; makes p1 null.
p1 = nullptr
```

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
// - only a "unique_ptr" ensures that its object is properly destroyed whichever way we exit f() (by throwing an exception, by executing return, or by "falling off the end")

// often used for passing free-store allocated objects in and out of functions
unique_ptr<X> make_X(int i)
// make an X and immediately give it to a unique_ptr
{
  // ... check i, etc. ...
  return unique_ptr<X>{new X{i}};
}
```

BS15.2.1
- A `unique_ptr` is a **handle to an individual object** (or an array) 
  - in much the same way that a `vector` is a **handle to a sequence of objects**. 
  - Both control the lifetime of other objects (using RAII) and
  - both rely on **elimination of copying** (copy elision) or on **move semantics** to make `return` simple and efficient

#### `make_unique`

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

### `weak_ptr` class

TODO

### Best practices

- Use `shared_ptr` only if you actually need shared ownership

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

