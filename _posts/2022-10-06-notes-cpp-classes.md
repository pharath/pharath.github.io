---
title: "C++ Notes - Classes"
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
    - classes
    - notes

---

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

## Scope

A class is a scope.
- therefore, when we define a member function **outside its class**, we must provide the **class name** as well as the function name
- Outside of the class, the names of the members are hidden
- Once the class name is seen, the remainder of the definition - including the **parameter list** and the **function body** - is **in** the scope of the class
  - As a result, we can refer to other class members **without qualification** (in the parameter list and in the function body)
  - Note: The member function name and its return type, however, must be qualified.

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

