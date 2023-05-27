---
title: "React Notes"
read_time: false
excerpt: "React Basics"
toc: true
toc_sticky: true
categories:
  - Notes
tags:
  - react
  - notes
---

# React (Facebook)

- JavaScript UI **library** 
  - not a full fledged framework because React has no router
    - however, there is a package called `react-router-dom`
- **state management**: In React development, keeping track of how your application data changes over time is called state management.
  - common state management methods:
    - class-based state management
    - Hooks
    - Redux (third-party library)
  - React may optimize code by calling actions asynchronously
    - make sure that your function has access to the most up-to-date state
- Maintained by Facebook
- alternatives: 
  - **Angular** (2016, Google, full fledged framework)
    - complete rewrite of AngularJS
    - TypeScript based
  - **Vue.js** (2014, Evan You, gaining popularity, easy to learn)

# Tutorials

React
- [Props](https://www.digitalocean.com/community/tutorials/how-to-customize-react-components-with-props) <!--- TODO -->
- [Class-based State Management, Components](https://www.digitalocean.com/community/tutorials/how-to-manage-state-on-react-class-components#step-1-%E2%80%94-creating-an-empty-project)
- [Hooks](https://www.digitalocean.com/community/tutorials/how-to-manage-state-with-hooks-on-react-components)
- [Web APIs](https://www.digitalocean.com/community/tutorials/how-to-call-web-apis-with-the-useeffect-hook-in-react) <!--- TODO -->
- [Login Authentication](https://www.digitalocean.com/community/tutorials/how-to-add-login-authentication-to-react-applications) <!--- TODO -->

# create-react-app

- **CLI tool** to quickly spin up a React app
- loosing popularity because of Next.js

see [official doc](https://create-react-app.dev/)

```bash
npx create-react-app my-app
npm start
```

## Components

**root component**: `src/App.js`. This is the **root component** that is injected into the page. All components will start from here.

## Rename the Project

- First rename the project folder. 
- Then change the name(s) in `package.json` and `package-lock.json`.

## Without Boilerplate

```bash
nvim src/App.js   # Delete the line `import logo from './logo.svg';`. Then replace everything in the return statement to return a set of empty tags: <></>.
rm src/logo.svg 
mkdir src/components   # Each component will have its own directory in 'components/' to store the component file along with the styles, images, and tests.
mkdir src/components/App
mv src/App.* src/components/App/   # Move all of the App files into that directory.
nvim src/index.js   # change line to: `import App from './components/App/App';`
```

## Templates

- [list of create-react-app (cra) templates](https://www.npmjs.com/search?q=cra-template-*)
- [official React doc source code](https://github.com/reactjs/react.dev)

## Docusaurus

Creates a documentation page in React that is easy to maintain.

E.g. the [create-react-app doc](https://create-react-app.dev/) was built using Docusaurus (see [source code on github](https://github.com/facebook/create-react-app/tree/main/docusaurus)).

### Installation

For more details see [docusaurus doc](https://docusaurus.io/docs/installation#scaffold-project-website).

```bash
nvm install lts/hydrogen --latest-npm
nvm alias default lts/hydrogen
nvm use lts/hydrogen
```

### Generate a new Docusaurus Website

```bash
npx create-docusaurus@latest my-website classic
```

Or create a blank template:
```bash
npm init docusaurus@latest
```

### Local Development

```
$ cd my-website/
$ npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Local Build

Always build it locally before you push it to Gitlab.

```bash
npm run build
npm run serve
```

# setState()

[doc with examples](https://legacy.reactjs.org/docs/state-and-lifecycle.html#using-state-correctly)

[doc](https://react.dev/reference/react/Component#setstate)

3 things you should know about `setState()`

From [doc with examples](https://legacy.reactjs.org/docs/state-and-lifecycle.html#using-state-correctly):
1. Do not modify state directly
  - use [spread]({% post_url 2023-03-23-notes-js %}#spread) for copying arrays
  - [read](https://legacy.reactjs.org/docs/state-and-lifecycle.html#do-not-modify-state-directly)
  - "The only place where you can assign `this.state` is the constructor."
    - if you assign it somewhere else, the component will not re-render
2. State updates may be asynchronous
  - React may **batch** multiple `setState()` calls into a single update for performance.
  - Because `this.props` and `this.state` may be updated asynchronously, you should **not** rely on their values for calculating the next state.
3. State updates are **merged**
  - `setState()` merges the **specified** new variables into the current state and **does not modify the not-specified variables**

# Hooks

- since React 16
- Hooks are functions
- Hooks **are triggered** 
  - by other actions
  - when a component's props change
- Hooks **are used to**
  - create data
  - to trigger further changes
- Since this method of state management doesn't require you to use classes, developers can use Hooks to write **shorter**, **more readable** code that is **easy to share and maintain**. 
- One of the main differences between Hooks and class-based state management is that there is no single object that holds all of the state. Instead, you can break up state into multiple pieces that you can update independently.

## Built-in Hooks

- **Built-in React Hooks**: see [API Reference](https://react.dev/reference/react)

## useState

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

## useContext

- [doc](https://react.dev/reference/react/useContext)

## useReducer

- specially designed to update the state based on the current state, in a manner similar to the `.reduce` array method 
- The `useReducer` Hook is similar to `useState`, but when you initialize the Hook, you pass in **a function the Hook will run** when you change the state along with the initial data
- The function - referred to as the **reducer** - takes **two arguments**: 
  - the state
  - The other argument is what you will supply when you call the update function.
- A common pattern in reducer functions is to pass an **object as the second argument** that contains 
  - the **name of the action** and 
  - **the data** for the action. 
  - Inside the reducer, you can then update the total based on the action.

# Basics

## Lists and Keys

In JavaScript:
```js
const numbers = [1, 2, 3, 4, 5];
const doubled = numbers.map((number) => number * 2);
console.log(doubled);
```

The same in React:
```jsx
const numbers = [1, 2, 3, 4, 5];
const listItems = numbers.map((numbers) =>
  <li>{numbers}</li>
);

const root = ReactDOM.createRoot(document.getElementById('root')); 
root.render(<ul>{listItems}</ul>);
```

Render this inside a React component:
```jsx
function NumberList(props) {
  const numbers = props.numbers;
  const listItems = numbers.map((number) =>
    <li>{number}</li>  
  );
  return (
    <ul>{listItems}</ul>  
  );
}

const numbers = [1, 2, 3, 4, 5];
const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(<NumberList numbers={numbers} />);
```

This will give a warning that a `key` should be provided for list items.

Add a `key`:
```jsx
function NumberList(props) {
  const numbers = props.numbers;
  const listItems = numbers.map((number) =>
    <li key={number.toString()}>
      {number}
    </li>
  );
  return (
    <ul>{listItems}</ul>
  );
}
```

[Best Practices for keys](https://legacy.reactjs.org/docs/lists-and-keys.html#keys).
- "We don't recommend using **indexes** for keys if the order of items may change. This can negatively impact performance and may cause issues with component state."

# JSX

AKA **JavaScript Syntax Extension**, (**JavaScript XML**)
- looks like HTML
- **React components** are typically written using JSX (but can also be written in pure JavaScript)
  - JSX provides a way to **structure component rendering**
- JSX is created by Meta (formerly Facebook). 
- **XHP**: JSX is similar to another extension syntax created by Meta for PHP called XHP. 

## Using JavaScript inside of JSX

- wrap the code in curly braces

# Redux

- a method of state management
- a third-party library
- also follows the React **Hook naming convention**, i.e. Redux Hooks also start with the prefix `use...`
  - e.g. `useSelector`, `useStore`

# Best Practice

- It is best to only add information to `state` that you expect to change
  - This way, the information in `state` will be easier to keep track of as your application scales.
- minimize the different pieces of state by only saving related data in one place
  - In other words, try to avoid double references to the same data

# Performance

## Lazy Loading

From [Wiki](https://en.wikipedia.org/wiki/Lazy_loading):
- also known as **asynchronous loading**
- a **design pattern** commonly used in computer programming and **mostly in web design and development** to defer initialization of an object until the point at which it is needed. 
- It can contribute to efficiency in the program's operation if properly and appropriately used. 
- This makes it ideal in use cases where network content is accessed and initialization times are to be kept at a minimum, such as in the case of **web pages**. 
  - For **example**, deferring loading of images on a web page until they are needed can make the initial display of the web page faster. 
- The opposite of lazy loading is **eager loading**.
