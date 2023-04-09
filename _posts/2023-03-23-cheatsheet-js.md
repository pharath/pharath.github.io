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

React
- [Props](https://www.digitalocean.com/community/tutorials/how-to-customize-react-components-with-props) <!--- TODO -->
- [Class-based State Management, Components](https://www.digitalocean.com/community/tutorials/how-to-manage-state-on-react-class-components#step-1-%E2%80%94-creating-an-empty-project)
- [Hooks](https://www.digitalocean.com/community/tutorials/how-to-manage-state-with-hooks-on-react-components)
- [Web APIs](https://www.digitalocean.com/community/tutorials/how-to-call-web-apis-with-the-useeffect-hook-in-react) <!--- TODO -->
- [Login Authentication](https://www.digitalocean.com/community/tutorials/how-to-add-login-authentication-to-react-applications) <!--- TODO -->

# JavaScript (aka ECMAScript aka ES)

## Versions, Revisions

- ES6 = ECMAScript 6 = ECMAScript 2015
  - arrow functions

## Crash Course

see [YouTube](https://www.youtube.com/watch?v=hdI2bqOjy3c) ([Code](https://embed.plnkr.co/plunk/8ujYdL1BxZftGoS4Cf14))

## Implementations

- V8
  - from [Wikipedia](https://en.wikipedia.org/wiki/V8_(JavaScript_engine)):
    - developed by the Chromium Project for Google Chrome and Chromium web browsers
    - V8 compiles ECMAScript directly to native machine code using **just-in-time compilation** before executing it
      - **JIT compilation** aka dynamic compilation ( [source](https://www.freecodecamp.org/news/just-in-time-compilation-explained/) )
        - a method for improving the performance of **interpreted** programs
        - During execution the program may be compiled into native code to improve its performance. 
        - Advantages:
          - Dynamic compilation has some advantages over static compilation. 
          - When running `Java` or `C#` applications, the runtime environment can **profile** the application while it is being run. 
          - This allows for more optimized code to be generated.
          - If the behavior of the application changes while it is running, the runtime environment can recompile the code.
    - The compiled code is additionally **optimized** (and re-optimized) dynamically **at runtime**, based on heuristics of the code's execution profile. 
    - **Optimization techniques** used include 
      - inlining
      - elision of expensive runtime properties
      - inline caching.
    - used in
      - Chromium-based web browsers
      - Firefox - parts of V8 ported to the browser for regular expressions parsing
      - many more apps (see [Wikipedia](https://en.wikipedia.org/wiki/V8_(JavaScript_engine)))

## Formatting, Syntax Highlighting

- Install `javascript` syntax highlighting using `nvim-treesitter`.
- Install `javascript` formatting using `coc-prettier`.

## Numbers

### Number.prototype.toLocaleString

```js
const currencyOptions = {
  minimumFractionDigits: 2,
  maximumFractionDigits: 2,
}

total.toLocaleString(undefined, currencyOptions)
```
- Convert `total` from a number to a string with two decimal places. 
- this will also convert the number to a string according to the numerical conventions that match the browser's locale
- Use `undefined` as the first argument to `toLocaleString` to use the system locale rather than specifying a locale

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

### Array.prototype.indexOf

```js
array1.indexOf(x)
```
- index of the **first instance** of `x` in an `array1`
- return `-1` if `x` is not present

## Functions

### Anonymous Functions

From [javascripttutorial.net](https://www.javascripttutorial.net/javascript-anonymous-functions/):

An anonymous function is a function without a name.
```js
(function () {
   //...
});
```

An anonymous function is not accessible after its initial creation. Therefore, you often need to assign it to a variable:
```js
let show = function () {
    console.log('Anonymous function');
};

show();
```
In this example, the anonymous function has no name between the `function` keyword and parentheses `()`.

#### Arrow Function Notation

ES6 introduced **arrow function** expressions that provide a shorthand for declaring anonymous functions.
```js
let show = () => console.log('Anonymous function');
```

From [w3schools](https://www.w3schools.com/js/js_arrow_function.asp):
If you have **only one parameter** (here: `val`), you can skip the parentheses as well:
```js
hello = (val) => "Hello " + val;
// instead, you can use
hello = val => "Hello " + val;
```

#### Anonymous Functions as arguments

In practice, you often pass anonymous functions as arguments to other functions:
```js
setTimeout(function() {
    console.log('Execute later after 1 second')
}, 1000);
```

## Event Handler

- `<button onClick={function}>`

# JSX

aka JavaScript Syntax Extension (sometimes referred to as **JavaScript XML**)
- looks like HTML
- React components are typically written using JSX (but can also be written in pure JavaScript)

## Using JavaScript inside of JSX

- wrap the code in curly braces

# React

- JavaScript UI **library** (not framework)
- **state management**: In React development, keeping track of how your application data changes over time is called state management.
  - common state management methods:
    - class-based state management
    - Hooks
    - Redux (third-party library)
  - React may optimize code by calling actions asynchronously
    - make sure that your function has access to the most up-to-date state

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

### Templates

- [list of create-react-app (cra) templates](https://www.npmjs.com/search?q=cra-template-*)
- [official React doc source code](https://github.com/reactjs/react.dev)

### Docusaurus

Creates a documentation page in React that is easy to maintain.

Create a blank template:
```bash
npm init docusaurus@latest
```

E.g. the [create-react-app doc](https://create-react-app.dev/) was built using Docusaurus (see [source code on github](https://github.com/facebook/create-react-app/tree/main/docusaurus)).

## Hooks

- Hooks are functions
- Hooks **are triggered** 
  - by other actions
  - when a component's props change
- Hooks **are used to**
  - create data
  - to trigger further changes
- Since this method of state management doesn't require you to use classes, developers can use Hooks to write **shorter**, **more readable** code that is **easy to share and maintain**. 
- One of the main differences between Hooks and class-based state management is that there is no single object that holds all of the state. Instead, you can break up state into multiple pieces that you can update independently.

### Built-in Hooks

- **Built-in React Hooks**: see [API Reference](https://react.dev/reference/react)

### useState

- a function
- takes the initial state as an argument and returns an array with two items
- syntax: `const [variable, setFunction] = useState(defaultValue);`
  - event handler function: the event handler function (i.e. the `function` in `onClick={function}`) must have the same scope as the `setFunction`, therefore the event handler function must be defined inside the component function
    - if you define the arrow function **inside of a prop** (i.e. substitute `function` in `onClick={function}` with the definition of `function`), React will 
      1. create a new function in every re-render
      2. which triggers a prop change
      3. which causes the component to re-render
    - if you define the arrow function **outside of a prop**, you can use the `useCallback` Hook which will **memoize (cache) the function** 
      - this gives better performance than defining the function **inside of a prop**
    - as a rule, the higher a component is likely to be in a tree, the greater the need for memoization.

### useContext

### useReducer

- specially designed to update the state based on the current state, in a manner similar to the `.reduce` array method 
- The `useReducer` Hook is similar to `useState`, but when you initialize the Hook, you pass in **a function the Hook will run** when you change the state along with the initial data
- The function - referred to as the **reducer** - takes **two arguments**: 
  - the state
  - The other argument is what you will supply when you call the update function.
- A common pattern in reducer functions is to pass an **object as the second argument** that contains 
  - the **name of the action** and 
  - **the data** for the action. 
  - Inside the reducer, you can then update the total based on the action.

## Redux

- a method of state management
- a third-party library
- also follows the React **Hook naming convention**, i.e. Redux Hooks also start with the prefix `use...`
  - e.g. `useSelector`, `useStore`

## Best Practice

- It is best to only add information to `state` that you expect to change
  - This way, the information in `state` will be easier to keep track of as your application scales.
- minimize the different pieces of state by only saving related data in one place
  - In other words, try to avoid double references to the same data

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

[Wikipedia](https://en.wikipedia.org/wiki/Next.js)
- an open-source web development framework created by the private company Vercel providing React-based web applications with server-side rendering and static website generation.
- React documentation mentions Next.js among "Recommended Toolchains" advising it to developers when "building a server-rendered website with Node.js". 
- Where traditional React apps can only render their content in the **client-side** browser, Next.js extends this functionality to include applications rendered on the **server-side**. 

# NextAuth.js

# Deploy

## AWS

### SST

[github](https://github.com/serverless-stack/sst)

[docs.sst](https://docs.sst.dev/what-is-sst)
- SST is a framework that makes it easy to build modern full-stack applications **on AWS**.
- frontend:
  - **Deploy** a serverless **Next.js**, Remix, Astro, or Solid app **to your AWS account** and add any backend feature to it.
- backend:
  - SST has constructs for most backend features. And you can even **use any AWS service** in your app.

[Tutorials](https://sst.dev/guide.html#table-of-contents)
- [Overview of the tutorial's demo app](https://sst.dev/chapters/what-does-this-guide-cover.html)
- build the backend
  - [serverless](https://sst.dev/chapters/what-is-serverless.html)
    - **serverless (aka FaaS)**: Serverless computing (or serverless for short), is an execution model where the cloud provider (AWS, Azure, or Google Cloud) is responsible for executing a piece of code by dynamically allocating the resources. And only charging for the amount of resources used to run the code. 
    - **Events**: The code is typically run inside **stateless containers** that can be **triggered by** a variety of **events** including 
      - http requests, 
      - database events, 
      - queuing services, 
      - monitoring alerts, 
      - file uploads, 
      - scheduled events (cron jobs), 
      - etc. 
    - **FaaS**: The code that is sent to the cloud provider for execution is usually in the form of a **function**. Hence serverless is sometimes referred to as "*Functions as a Service*" or "*FaaS*". 
    - **Major offerings**: Following are the **FaaS offerings** of the **major cloud providers**:
      - AWS: AWS Lambda
      - Microsoft Azure: Azure Functions
      - Google Cloud: Cloud Functions
- build the frontend
  - [Setting up a React app](https://sst.dev/chapters/create-a-new-reactjs-app.html) (uses `create-react-app`)
  - [Login page](https://sst.dev/chapters/create-a-login-page.html)

# TypeScript

- a **strict syntactical superset** of JavaScript
- adds optional **static typing** to the language
- designed for the development of large applications
- transpiles to JavaScript
- As it is a superset of JavaScript, existing JavaScript programs are also valid TypeScript programs. 
- developed and maintained by Microsoft
