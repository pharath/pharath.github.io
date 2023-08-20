---
title: "C++ Notes - Polymorphism"
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
    - polymorphism
    - notes

---

# Polymorphism

1. function **overloading**
2. templates
3. virtual functions ( **overriding** )

From [source](https://www.geeksforgeeks.org/templates-cpp/):
- "Both function overloading and templates are examples of **polymorphism** features of OOP."
- function templates vs overloading:
  - "Function overloading is used when multiple functions do quite similar (not identical) operations, templates are used when multiple functions do identical operations."

# Overloading vs Overriding

- overloading: the same function name, but with a different signature (i.e. name of the function, the parameter list, and the keyword `const`, if used)
- overriding: the same function name and the same signature

# Overloading

From [stackoverflow](https://www.tutorialspoint.com/cplusplus/cpp_overloading.htm):

**Function Overloading**, **Operator Overloading**: C++ allows you to specify more than one definition for a **function** name or an **operator** in the same scope, which is called **function overloading** and **operator overloading** respectively. 

**overloaded declaration**, **overload resolution**: An overloaded declaration is a declaration that is declared with the same name as a previously declared declaration in the same scope, except that both declarations have different arguments and obviously different definition (implementation). When you call an overloaded **function** or **operator**, the compiler determines the most appropriate definition to use, by comparing the argument types you have used to call the function or operator with the parameter types specified in the definitions. The process of selecting the most appropriate overloaded function or operator is called **overload resolution**.

# Overriding

- "**Overriding** a function means changing the implementation of a base class function in a derived class." (Day 12, J Liberty)

## "Normal" Overriding

- "When a derived class creates a function with the same return type and signature as a member function in the base class, but with a new implementation, it is said to be **overriding** that function. When you make an object of the derived class, the correct function is called." (Day 12, J Liberty)

## virtual

- C++ allows **pointers to base classes** to be assigned to derived class objects `BaseClass* pBase = new DerivedClass;` (Day 12, J Liberty)
  - `pBase` can invoke any method on `BaseClass`, but what we would like is for those methods that are overridden in `DerivedClass()` to call the correct function
- phth: "normal" overriding (cf. above) is possible **without** the `virtual` (see Day 12, J Liberty), but, if a pointer to the base class is assigned to the derived class object, and we use this pointer to call an overridden function, the overriding will not work

[source](https://www.programiz.com/cpp-programming/virtual-functions):
- A **virtual function** is a member function in the base class that we *expect* to redefine in derived classes.
- Basically, a virtual function is used in the base class in order to ensure that the function is **overridden**. This especially applies to cases where a pointer of base class points to an object of a derived class.

```cpp
// source: https://www.geeksforgeeks.org/virtual-functions-in-derived-classes-in-cpp/
// C++ Program to demonstrate Virtual
// functions in derived classes
#include <iostream>
using namespace std;

class A {
public:
	virtual void fun() { cout << "\n A::fun() called "; } 
};

class B : public A {
public:
	void fun() { cout << "\n B::fun() called "; }
};

class C : public B {
public:
	void fun() { cout << "\n C::fun() called "; }
};

int main()
{
	// An object of class C
	C c;

	// A pointer of class B pointing
	// to memory location of c
	B* b = &c;

	// this line prints "C::fun() called"
    // (Note: without the "virtual" in "class A" this line prints "\n A::fun() called ")
	b->fun();

	getchar(); // to get the next character
	return 0;
}
```

