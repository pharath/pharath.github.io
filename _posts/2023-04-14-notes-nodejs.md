---
title: "Node.js Notes"
read_time: false
excerpt: "Node.js Basics"
toc: true
toc_sticky: true
categories:
  - Notes
tags:
  - nodejs
  - notes
---

# Node.js

Node.js is
- **cross-platform**: can run on Windows, Linux, Unix, macOS, and more. 
- a **back-end** JavaScript [runtime environment](https://en.wikipedia.org/wiki/Runtime_system)
  - runs on the V8 JavaScript Engine
  - executes JavaScript code outside a web browser.
- lets developers use JavaScript to 
  - write command line tools and 
  - for server-side scripting. 
    - The ability to run JavaScript code on the server is often used to generate **dynamic web page content** before the page is sent to the user's web browser.

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
`npm install` (in a package directory, no arguments) | Install the dependencies to the local `node_modules` folder. In global mode (ie, with `-g` or `--global` appended to the command), it installs the current package context (ie, the current working directory) as a global package. By default, `npm install` will install all modules listed as dependencies in `package.json`.
`npm install` | see [doc](https://docs.npmjs.com/cli/v9/commands/npm-install)
`npm Uninstall pkg` | Uninstall a package
`npm init` | this tells npm to make a file in your folder called `package.json` that is going to help you organise these dependencies.

### Semantic Versioning

- [watch](https://www.youtube.com/watch?v=kK4Meix58R4)
- [read](https://docs.npmjs.com/about-semantic-versioning)

New package versions may contain different "**kinds of updates**"
- "patch" 
- "minor release" 
- "major release"

SemVer is a way to communicate
- as a package **publisher**:
  - to indicate **in the SemVer version number** which of the above "kinds of updates" the new published package version contains
- as a package **user**:
  - to limit **via the SemVer version number** which "kind of package updates" will be accepted when we update dependencies

## npx

```bash
npm install -g npx
```

