---
title: "JavaScript Cheatsheet"
read_time: false
excerpt: "JavaScript Basics"
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - js
  - cheatsheet
---

# Tutorials

- [React State Management, Components](https://www.digitalocean.com/community/tutorials/how-to-manage-state-on-react-class-components#step-1-%E2%80%94-creating-an-empty-project)

# JavaScript

## Arrays

### Spread Syntax

[digitalocean](https://www.digitalocean.com/community/tutorials/understanding-destructuring-rest-parameters-and-spread-syntax-in-javascript#spread)

I.e. `...` notation, e.g.
```js
const cart = [...state.cart];
```

Use cases:
- instead of `const concatArray = array1.concat(array2)` use `const concatArray = [...array1, ...array2]`
  - both methods do not change the existing arrays, but instead return a new array
- instead of `const pushedArray = array1.push(array2[0])` (modifies `array1`) use `const pushedArray = [...array1, array2]` (does not modify `array1`)
- creating a new array from the existing one and adding a new item to the end **without changing the existing one**
- In JavaScript, when you create an object or array and assign it to another variable, you are not actually creating a new object - you are passing a reference.
  - spread allows to make a shallow copy of an array or object **without changing the existing one**
    - i.e. any top level properties will be cloned, but nested objects will still be passed by reference

### Array.prototype.slice

[stackoverflow](https://stackoverflow.com/questions/9050345/selecting-last-element-in-javascript-array)

Get the last element:
```js
var my_array = /* some array here */;
var last_element = my_array[my_array.length - 1];
```

- Do not use `my_array.slice(-1)[0]`, it is too slow.

### Array.prototype.splice

[mdn](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Array/splice)

**return**: An array containing the deleted elements.

The `splice()` method changes the contents of an array by removing or replacing existing elements and/or adding new elements **in place** (i.e. `splice()` modifies the original array). 
- To access part of an array without modifying it, see `slice()`.
- a **mutating method**: It may change the content of `this`.
- Although strings are also array-like, this method is **not** suitable to be applied on them, as strings are immutable.

```js
some_array.splice(start)   # remove all elements after index "start"
some_array.splice(start, deleteCount)   # remove "deleteCount" elements after index "start" and keep the rest
some_array.splice(start, deleteCount, item1)
some_array.splice(start, deleteCount, item1, item2, itemN)
```

### Array.prototype.reduce

[mdn](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Array/reduce)

The `reduce()` method executes a user-supplied **"reducer" callback function** on each element of the array, in order, passing in the return value from the calculation on the preceding element. The final result of running the reducer across all elements of the array is a single value.

**Initial value**: The first time that the callback is run there is no "return value of the previous calculation". If supplied, an **initial value** may be used in its place. Otherwise **the array element at index 0** is used as the initial value and iteration starts from the next element (index 1 instead of index 0).

Example:
```js
const array1 = [1, 2, 3, 4];

// 0 + 1 + 2 + 3 + 4
const initialValue = 0;
const sumWithInitial = array1.reduce(
  (accumulator, currentValue) => accumulator + currentValue,
  initialValue
);

console.log(sumWithInitial);
// Expected output: 10
```

# JSX

aka JavaScript Syntax Extension (sometimes referred to as **JavaScript XML**)
- looks like HTML
- React components are typically written using JSX (but can also be written in pure JavaScript)

## Using JavaScript inside of JSX

- wrap the code in curly braces

# React

- JavaScript **library** (not framework)
- **state management**: In React development, keeping track of how your application data changes over time is called state management.
  - common state management methods:
    - class-based state management
    - Hooks
    - Redux (third-party library)

## create-react-app

see [official doc](https://create-react-app.dev/)

```bash
npx create-react-app my-app
npm start
```

### Components

**root component**: `src/App.js`. This is the **root component** that is injected into the page. All components will start from here.

### Rename the Project

- First rename the project folder. 
- Then change the name(s) in `package.json` and `package-lock.json`.

### Without Boilerplate

```bash
nvim src/App.js   # Delete the line `import logo from './logo.svg';`. Then replace everything in the return statement to return a set of empty tags: <></>.
rm src/logo.svg 
mkdir src/components   # Each component will have its own directory in 'components/' to store the component file along with the styles, images, and tests.
mkdir src/components/App
mv src/App.* src/components/App/   # Move all of the App files into that directory.
nvim src/index.js   # change line to: `import App from './components/App/App';`
```

## Hooks

- Hooks are a broad set of tools that **run custom functions when a component's props change**. 
- Since this method of state management doesn't require you to use classes, developers can use Hooks to write **shorter**, **more readable** code that is **easy to share and maintain**. 
- One of the main differences between Hooks and class-based state management is that there is no single object that holds all of the state. Instead, you can break up state into multiple pieces that you can update independently.

## Redux Toolkit

- a method of state management
- a third-party library

## Best Practice

- It is best to only add information to `state` that you expect to change
  - This way, the information in `state` will be easier to keep track of as your application scales.

# Node.js

## nvm

Node Version Manager - POSIX-compliant bash script to manage multiple active node.js versions

| command | description |
| :--- | :--- |
`curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.3/install.sh | bash` | install nvm (see [github](https://github.com/nvm-sh/nvm))
`nvm ls` | Find out which versions of Node.js you may have installed and which one of those you're currently using
`nvm ls-remote` | List all versions of Node.js available for installation (for Windows: `nvm ls available`)
`nvm install 8.1.0` | install Node.js v8.1.0
`nvm install 8.1.0 --latest-npm` | install Node.js v8.1.0 and update `npm`
`nvm install-latest-npm` | update current `npm` (get the latest supported npm version on the current node version)
`nvm use 4.2` | Choose version. Here: set v4.2.0 as the active version
`nvm alias default 16.14.2` | set default node.js version 16.14.2

## npm

Node Package Manager.

| command | description |
| :--- | :--- |
`npm i -g npm` | **Do not run this! Use `nvm` instead.** Update `npm`.
`npm install pkg` | install a package
`npm Uninstall pkg` | Uninstall a package
`npm init` | this tells npm to make a file in your folder called `package.json` that is going to help you organise these dependencies.

## npx

```bash
npm install -g npx
```

# Next.js

Full-stack React framework.

# NextAuth.js


