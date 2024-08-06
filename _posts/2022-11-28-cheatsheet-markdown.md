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

## Code Blocks

### Indentation

- If you want to add a block of code to a list item, you have to add an extra 4 spaces for every level of that list. You also have to make sure that you leave a blank line before the code block., [stackexchange](https://meta.stackexchange.com/a/7837)

## Common issues in vim

- the following characters must be escaped or in a code block:
  - `_`
  - `$`
  - asterisk `*`
  - hash `#`
- be careful with spaces
  - no additional space behind triple backticks!

## Markdown Viewer

### retext

- `pip install retext`
- [retext wiki](https://github.com/retext-project/retext/wiki)
- [config options](https://github.com/retext-project/retext/blob/master/configuration.md)
- config file: `nvim .config/ReText\ project/ReText.conf`
- [markdown extensions](https://github.com/retext-project/retext/wiki/Markdown-extensions)
- for github flavored markdown:
  - `pip install pymdown-extensions`
  - add this on top of the document: `<!-- Required extensions: pymdownx.betterem, pymdownx.tilde, pymdownx.emoji, pymdownx.tasklist, pymdownx.superfences -->`

**Note**: Install the old `markdown` package version `python3.8 -m pip install markdown==3.2` first.

| command                             | description                                                       |
| :---------------------------------- | :---------------------------------------------------------------- |
| `retext markdown_file.md`           | edit `markdown_file.md`                                           |
| `retext --preview markdown_file.md` | preview `markdown_file.md`                                        |
| **Tipp:**                           | Shortcuts: s. Menu &rarr; File und Edit                           |
| <kbd>ctrl</kbd> <kbd>e</kbd>        | preview on/off                                                    |
| <kbd>ctrl</kbd> <kbd>l</kbd>        | live preview on/off (die live updates brauchen manchmal bisschen) |

### grip

[github](https://github.com/joeyespo/grip)

- `pip install grip`
- see [manpages.ubuntu.com](https://manpages.ubuntu.com/manpages/focal/man1/grip.1.html)
- "Preview GitHub Markdown files like Readme locally"
- view in Firefox
- `grip file.md 6420` (to open a second file on a different port, here `6420`)
