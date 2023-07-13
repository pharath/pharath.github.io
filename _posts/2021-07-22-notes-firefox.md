---
title: "Firefox Setup"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Setup
tags:
  - firefox
  - setup
toc: true
toc_label: "Contents"

---

# Profiles

## Profile Directory

about:support &rarr; "Profile Directory" &rarr; "Open Directory" Button

## History

[source](https://support.mozilla.org/en-US/questions/1176169)

Firefox stores your history and bookmarks together in a database file named *places.sqlite* which is in your profile folder. 

- Just copy *places.sqlite* to another profile folder in order to transfer the history of one profile to another one.

# Tools

- about:about
- about:preferences
- about:performance
- about:config
- about:addons (**besser**: about:addons#extensions) 
- about:memory
- about:support -> "Profile Directory" -> "Open Directory" Button

## Developer Tools

| command | description |
| :---: | :---: |
ctrl + shift + i | [Toolbox](https://firefox-source-docs.mozilla.org/devtools-user/tools_toolbox/index.html)
ctrl + i | page info (i.e. general, media, permissions, security)

# Shortcuts

| command | description |
| :---: | :---: |
cmd + shift + n | open recently closed window
cmd + shift + t	| open recently closed tab
cmd + shift + b | hide/show bookmarks toolbar
alt + b (linux: ctrl + alt + b) | hide/show bookmark search plus pane
ctrl + shift + m | show mobile version of website
ctrl + u | view source
ctrl + shift + k | web console
ctrl + shift + j | [browser console](#browser-console) 
shift + esc | Process Manager

## Browser Console

[doc](https://firefox-source-docs.mozilla.org/devtools-user/browser_console/index.html)

The Browser Console is like the Web Console, but applied to the **whole browser** rather than a single content tab.

So it logs the same sorts of information as the Web Console - network requests, JavaScript, CSS, and security errors and warnings, and messages explicitly logged by JavaScript code. However, rather than logging this information for a single content tab, it logs information for **all** content tabs, for add-ons, and for the browser’s own code.

# Addons

- Adblock Plus
- Bookmark Serch Plus 2
- DownThemAll!
- Video Downloadhelper (by mig)
- Duplicate Tab Shortcut (by Stefan Sundin)
- iCloud Bookmarks
- Link to Text Fragment
- PDF Mage
- Raindrop.io
- Search with Wikipedia
- Textmarker
- TeXZilla
- Web Search Navigator
- Vimium-FF (by Stephen Blott, Phil Crosby)
- GNOME Shell integration
- Reload in address bar
- StickyNotes
- Tab Session Manager
- Grammarly: Grammar Checker and Writing App
- Fast Tab Switcher
- Auto Tab Discard by tlintspr
