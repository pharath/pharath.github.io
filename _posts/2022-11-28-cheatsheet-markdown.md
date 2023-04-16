---
title: "Markdown Cheatsheet"
read_time: false
toc: true
toc_label: "Contents"
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - markdown
  - cheatsheet

---

## Latex in vim

- vim italics issue (because Latex uses underscores):
    - vim will **display** italic text **as italics** as long as the environment variable `TERM` is set to `xterm-256color` (looks ugly!)
    - if `TERM` is not set to anything, vim will **highlight** the italic text (looks even more ugly!)

- **solution**: see [here]({% post_url 2021-07-05-cheatsheet-vim %}#latex)

## Syntax Highlighting

- see [list of languages 1](https://support.codebasehq.com/articles/tips-tricks/syntax-highlighting-in-markdown)
- see [list of languages 2](https://docs.readme.com/rdmd/docs/code-blocks#language-support)
