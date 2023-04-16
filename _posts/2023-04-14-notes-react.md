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

Create a blank template:
```bash
npm init docusaurus@latest
```

E.g. the [create-react-app doc](https://create-react-app.dev/) was built using Docusaurus (see [source code on github](https://github.com/facebook/create-react-app/tree/main/docusaurus)).

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

