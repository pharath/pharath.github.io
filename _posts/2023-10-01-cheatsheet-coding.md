---
title: "Coding Cheatsheet"
read_time: false
excerpt: "Some essential shortcuts"
header:
  teaser: /assets/images/Vim.png
  overlay_image: /assets/images/Vim.png
  overlay_filter: 0.5 
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - vim
  - coding
  - cheatsheet
---

# ubuntu

- ctrl + aboveTab (switch to last workspace)
  - like "`^`" in firefox or "alt + tab" in Ubuntu
  
# terminal

- ctrl - l (clear)

# firefox

## vimium

- T (switch tab)
- click on "vimium button in firefox" &rarr; "Options" (to see custom maps)
- yt (duplicate tab)
- gi: jump into the first search box (must be in the view, does not work if you have to scroll the box into the view!)
- gs: view source
- HL: tabs
- JK: history
- ctrl + shift + k (console) (click "..." to get side-by-side view)
- beside standard firefox DevTools tabs
  - React Components panel
  - React Profiler panel

## Textmarker

- Notes:
  - **Warning**: the "x" button deletes the note, use the "-" button to hide
- sidebar contains useful buttons, eg.
  - jump from one highlight to the next/previous
  - Notes: if a highlight has a Note attached to it, there is a small white box beside it in the "Marks and Notes" list
    - helps to quickly find all existing Notes (without clicking through all of them)

# godbolt

- f1 (command palette)
- c - x (cut line)
- c - d (duplicate line)

# markdown

- alt - i (next `# `)
- alt - o
- `:LspStop 1 (tailwindcss)` (because the treesitter markdown parser is sufficient)
- alt - f (insert arrow symbol)

## LaTex

- math mode
  - alt - b
  - alt - u
  - alt - t ($\[ \text{Let}\ x=\text{number of cats}. \]$)

# Lazy

- https://dev.to/vonheikemen/lazynvim-how-to-revert-a-plugin-back-to-a-previous-version-1pdp
  - restore specific plugin to previous state
  - revert all plugins to previous state
- :Lazy (must update regularly)
- C (check if a tag exists, after changing "tag =" or "version =" in init.lua)
- :checkhealth (debug broken plugins, shows a log)
  - tip: use `:set ft=markdown` for syntax highlighting
  - :checkhealth somePluginName
  
# LSP, Mason

- space ca (apply code fix)
- `:Mason` (manage all LSPs, DAPs, Linter, Formatter)
- install new language servers via `:LspInstall <tab>`
- `:LspInfo`
  - (Note: die "installierten" server sind in der Zeile ganz unten in der Ansicht! Die server, die oben in der Liste sind, sind die "aktiven" für aktuelle buffer!)
- `:LspStop serverName`
  - zB to stop tailwindcss server for markdown files

# clang, clangd

- clang is the compiler
- clangd is the language server

## compile commands json

- From `:LspInfo` -> press tab:
  - If `compile_commands.json` lives in a build directory, you should symlink it to the root of your source tree. `ln -s /path/to/myproject/build/compile_commands.json /path/to/myproject/`

## compile flags txt

- run `:LspRestart` to reload
- for simple projects (eg. compile one file only):
  - create a `compile_flags.txt`
    - in the folder where the source code is
    - eg. with the content `-std=c++20`

## .clangd (project configuration file, yaml)

- e.g. to suppress `In included file: main file cannot be included recursively when building a preamble` add 

```yml
Diagnostics:
  Suppress: 'pp_including_mainfile_in_preamble'
```

to `.clangd`

# autocomplete, completion

- 3 popup/drop-down Arten: 
  - vim default completion
  - `coc.nvim`
  - `nvim-cmp`
- ctrl - y (confirm + replace)
- ctrl - e (discard + do not do anything)
  - helpful eg. if two conflicting completion popups appear at the same time, ctrl - e will switch between these

# nvim-cmp

- requires a "snippet engine"
  - in use: "luasnip"
  - tab, shift-tab
    - select item in drop-down
      - if the item is a function a snippet is inserted
    - edit next/previous function parameter in the function snippet
  - luasnip config: [Example-mappings#luasnip](https://github.com/hrsh7th/nvim-cmp/wiki/Example-mappings#luasnip)
  - TODO:
    - maybe configure a loader: [LuaSnip#add-snippets](https://github.com/L3MON4D3/LuaSnip#add-snippets)
  
# coc.nvim

- das **completion popup** mit den eckigen Klammer Symbolen in der rechten Spalte (zB `completionVorschlag1 [A]`, `completionVorschlag2 [B]`, usw) wird von `coc.nvim` erzeugt, wobei jedes eckige Klammer Symbol für jeweils eine "source" steht, die in `:CocList` => "sources" registriert wurde
  - am besten alle "sources" deregistrieren (über `:CocList` => "sources"), weil sonst der `nvim-cmp` popup und der `coc.nvim` popup manchmal gleichzeitig erscheinen
  - wobei das source namens `File` nützlich ist (zeigt die files in cwd, wenn man `./` typet
- `:CocList` (select "sources" to configure which sources are used for autocompletion)

# nvim-tree

- c then p (duplicate file, automatically shows "rename" where you have to choose a new name)
- for Firefox-Bookmarks-like behavior use
  - init.lua: `on_attach` keymap: bind C-e to `api.tree.toggle`
  - keymaps.lua: normal keymap: bind C-e to `:NvimTreeFocus`
- alt - h, alt - l (focus/unfocus nvim-tree, when nvim-tree open)
- "/" at end (create dir)
- J (jump to last item in folder)
- K (jump to first item in folder)
- i (show gitignored folders and files)
- U (show hidden folders and files)
- q (quit)
- statt netrw
- `/` (search)
- c-d (collapse)

# toggleterm (for lazygit integration in nvim)

- scroll
  - `<c-\> <c-n>` to exit terminal mode, then you can use vim motions to move around
  - scroll with mouse first and then you can use pageUp/pageDown
- written in lua
- better than floaterm (written in vimscript)
- `1<c-\>, 2<c-\>, 7<c-\>,` etc. to create and access terminals
  - the prefixed number is the number of the terminal instance
  - pressing a non-existent number will create a new terminal

# gitsigns

This plugin is only active in git-tracked folders. Ie. `:map` (and, therefore, space sk) will not show any keymaps related to gitsigns in non-git-tracked folders.

- alt - , (next hunk)
- alt - . (prev hunk)
- space h b (blame line)
  - um schnell herauszufinden in welchem commit die line geaddet wurde
- diff
  - space h d (diff zu letztem commit)
  - space h D (diff zu vorletztem commit)
  - besser: space g f (see fugitive)

# lazygit

- each panel has its own help menu!
- W (groß W, diff menu)
- ctrl + r (switch repo)
- "@" (focus command log)
- "x" (help) (ins commit panel und auf x drücken, zeigt zB amend, reset, etc.)
- R (change multiline commit message)
- r (change commit message)
- "A" (amend)
  - erst space und dann "A"
- space gg (for normal repos)
- space gd (for dotfiles repo)
- H, L (scroll left, right) 
  - praktisch um kleine panels zu lesen, e.g. "commit" panel
- h, l
  - switch between the 5 panels (or 12345)
- `[`, `]`
  - tabs sind getrennt durch "-"
  - zB "Commit" - "Reflog" sind 2 tabs
- "+" (rotate through views, "-" dann nicht nötig)
- esc (go back, Achtung: "q" is for exit lazygit!)
- esc (go back to commit list after seeing a commit's files)
- pageup/pagedown to scroll
- `ctrl -/+` (zoom out/in, very useful to view the full main panel)
- um mehr vom file zu sehen: 
  - im "Commits" panel den commit fokussieren (aber nicht enter drücken) und dann mehrmals "`}`" drücken
    - dies sollte am Ende den ganzen file anzeigen

Git actions:
- add
  - a (git add all)
  - space (git add file, press again to undo)
- commit
  - c
- push
  - P (UPPERcase!)
- pull
  - p (lowercase!)

# vim

- ; (undo repeat)
- , (repeat)
- ga (show ascii code of letter under cursor)

## custom

- <Space>fc (keymaps.lua)
- <Space>fi (init.lua)
- <Space>fp (plugins/init.lua)

## motion (in a line)

- B (beginning of previous WORD, faster than b)
- E (end of WORD, faster than e)
- gE (end of previous WORD, "be" does not work if cursor currently in middle of a word)
- ge (end of previous word, "be" does not work if cursor currently in middle of a word)
- Ea (start writing at end of WORD)
- ea (start writing at end of word)

## motion

- `g_` (end of line, last non-blank character)
- `:changes`
  - `g;` and `g,` (jump to previous/next change location)
- `%` (jump out of parenthesized block to 1st parenthesis)
- `%%` (jump out of parenthesized block to 1st parenthesis)
- ` (list possible jumps and marks)
  - `backtick .` (jump to last change in current buffer)
  - `m + markLetter` (set jump mark)
  - `backtick + markLetter` (jump to mark)
- move
  - `)`, `(`
  - `*` fwd, `#` bwd

## edit

- `c-r %` (in insert mode: insert current file name)
- registers
  - `"` (show all registers)
  - `"ry` (yank text to register `"r`)
  - `"rp` (paste text from register `"r`)
  - record arbitrary normal mode commands:
    - `qa5jq` (`q`: start recording into register "a", `5j`: record motion "`5j`", `q`: stop recording)
      - `"a` (to use the recorded command `5j` in normal mode)
- delete
  - `D` (like `d$`)
  - `ciw` (statt `diwa`, `diwi`)
  - `de` (statt `dw`)
  - `0d$` (clear line without deleting it)
- find and replace
  - `:%s///gc` (find and replace in buffer; g for all occurrences; c for confirm)
  - `:s///gc` (find and replace in one line; g for all occurrences; c for confirm)
- replace one word:
  - `cw` statt `dwi<space>`
- replace more than one word:
  - vt letter, vf letter (if you need to paste) statt ve
  - ct letter (change till letter)
  - dt letter (delete till letter)
  - df letter (delete till and including letter)
- f ("find" in motions)
- t ("till" in motions)
- `rx` (replace character under cursor with `x`, where `x` can be any letter)
  - less useful:
    - `3rx` (replace the two characters to the right as well)
    - `Rxyz Esc` (replace multiple characters with `xyz`, where pressing `Esc` marks completion and `backspace` acts as an undo)
- `c` (delete + start insert, aka "change")
- `C` (delete to EOL + start insert)
- selection `u` (lowercase selection)
- selection `U` (uppercase selection)
- `~` (change case of letter under cursor)
- A (insert at end of line, besser als $a)
- selection `c"aString"` (replace selection with "aString")
  - see [how-to-block-replace-code-in-visual-mode](https://vi.stackexchange.com/questions/2036/how-to-block-replace-code-in-visual-mode)
- shift - j (append the line below)
- text objects
  - sometimes "word" works, whereas "WORD" doesn't and vice versa
  - W (WORD, delimited by whitespaces)
  - w (word, delimited by *non-keyword* characters, which are configurable)
  - iW (inner WORD)
  - iw (inner word)
  - aw (a word)
  - is, as (sentence)
  - ip, ap (paragraph)
  - [overview: text-objects](https://blog.carbonfive.com/vim-text-objects-the-definitive-guide/)
  - cib (change parenthesis block)
  - vib (select parenthesis block)
  - v2ib (select nested parenthesis block)
  - ciB (change curly braces block)
  - viB (select curly braces block)
  - v2iB (select nested curly braces block)

## control

- c-o (temporary normal mode for one command)
- `:set ft?` (show filetype)
  - eg. for denylist in vim-illuminate config
- autocomplete
  - c - y (confirm vim-complete)
  - c - space (show cmp-complete)
  - enter (confirm cmp-complete)
  - c - e (abort complete) (both cmp-complete and vim-complete)
- sessions
  - space fod (load default session)
  - space fsd (save default session)
  - space fon (load new session)
  - space fsn (save new session)
- tabs
  - next tab: gt or c - PageDown (by default in vim)
  - previous tab: gT or c - PageUp (by default in vim)
- alt-h and alt-l (switch viewports)
  - to go to `:AerialToggle` and back (Avoid this! Use `:AerialNavToggle` (leader bb) instead!)
  - to go to nvim-tree and back

# fugitive

- for normal git commands use exclamation mark "`:!git ...`"
- G (former "Gstatus", press g? to see what you can do)
- G `<tab><tab>` to see available commands
- G log
- G shortlog
- `Gdiffsplit HEAD~1` (horizontal view)
- `Gvdiffsplit HEAD~1` (vertical view)
  - space g f (diff zu weiter zurück liegenden commits)
- Gread `%` (git checkout) (use u to undo/go back)

# GV

- öffnet sich in neuem Tab, d.h. "H", "L" um mit aktuellem file zu vergleichen
- gg (only current file)
- ga (all commits)
- q (quit)

# telescope

- `:h` telescope.mappings
- `c-/` (insert mode), ? (normal mode) (show shortcuts)
- c-t (open file in new tab)
- `ctrl -/+` (see more of the preview pane/results pane)
- c-u c-d (scroll preview up/down)
- space sk (search **keymaps** of active plugins, ie keymaps of inactive plugins are not shown)
- space ? (recently opened)
- space `/` (current buffer find)

# Aerial

- ToC window mode:
  - space bb
  - space bt (Telescope Search)
    - faster than "bb" for jumping to sections
- sidebar mode:
  - g?
  - q
  - zM (collapse all)
  - c-b (fast markdown section select)

# Vista (Aerial ist besser!)

- q (quit)
- p (preview)

# vim illuminate

- a-n
- a-p

# Help

- `:nmap <some-key>` to list all bindings with `<some-key>`
- g? (help for Aerial, nvim-tree, etc)
- Vista: in "`:h Vista`": tag: `vista-key-mappings`

# treesitter

(note: treesitter highlighting is distinct from the [LSP-based "semantic highlighting"](https://gist.github.com/swarn/fb37d9eefe1bc616c2a7e476c0bc0316))

- `:set ft=markdown` (enable syntax highlighting for unwritten buffer, ie a "`No name`" buffer)
- from [supported-languages](https://github.com/nvim-treesitter/nvim-treesitter#supported-languages)
  - to support a specific feature for a specific language requires both 
    - a parser for that language and 
    - an appropriate language-specific query file for that feature
  - `:TSInstall parserName` (install a specific parser)
    - in use: `markdown`, `markdown_inline`
  - `:TSInstallInfo` (list all installed parsers)
  - `:TSUninstall parserName`
- ctrl + space multiple times (select nodes, von innen nach außen)
- press `]` (next) or `[` (previous) to see keybindings
  - jumps to
    - start of next function/class
    - end of next function/class
    
# Edit Code

- space ca (apply code fix)
- space rn (rename a variable)
- `_` (like `^`)
  - `[count]_` (1st non-blank char `count-1` lines downward)
- comments
  - repeat to uncomment
  - line: gcc (or gbc)
  - visual selection: gc (or gb)

## surround

- vSo (surround letter under cursor)
- ys3iw" (surround **ohne select**)
- ys3iW" (see word vs WORD)

# Help Code, Doc

Definition:
- shift + k (for objects: press when cursor is on object; press 2x to jump into def window)
- ctrl + k (for functions: press keys inside function brackets; press 2x to jump into def window)
- gd (go to def)
- gr (go to references)

Search ONE file:
- (symbols: classes, properties, methods)
  - space ds (native lsp)
  - space bb (aerial)
    - space bm (collapse all in aerial)
  - space bt (aerial in telescope)
- gr (e.g. cycle through all occurrences of a var in the file)

Search ALL files:
- space sw (search word under cursor; gives the same results as "space sg")
- space sg (search by grep all files in cwd)

`nvim-cmp` steuert scrolling in definition preview popup
- press ctrl-f to scroll down
- alle key mappings in `.config/nvim/init.lua` bei `cmp.setup { ..., mapping = ... }`
