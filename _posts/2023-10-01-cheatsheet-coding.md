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

# website

- <kbd>alt</kbd> + <kbd>shift</kbd> + <kbd>s</kbd> (Search) - (note: this is a html `AccessKey`)

# ubuntu

- <kbd>alt</kbd> + <kbd>aboveTab</kbd> (switch to last window)
- <kbd>ctrl</kbd> + <kbd>aboveTab</kbd> (switch to last workspace)
  - like <kbd>^</kbd> in firefox or <kbd>alt</kbd> + <kbd>tab</kbd> in Ubuntu
- window mgmt:
  - `wmctrl -l` (list)
  - `wmctrl -a window-name` (go to workspace and focus by window name)
    - map this to <kbd>alt</kbd> + <kbd>shift</kbd> + <kbd>1</kbd>, <kbd>2</kbd>, <kbd>3</kbd>, etc. (after renaming the windows properly, renaming command: see below)
  - `wmctrl -i -a 0x066f5d24` (go to workspace and focus by window ID)
  - `wmctrl -i -a 0x066f5d24 -T "documentation"` (rename)
  - `wmctrl -i -a 0x0667167d -T "main-tmux"` (rename)
  - [more complex commands](https://superuser.com/a/950287)
  
# terminal

- <kbd>ctrl</kbd> - l (clear)

# firefox

- <kbd>ctrl</kbd> <kbd>u</kbd> (view page source code)

## Page Info

- <kbd>ctrl</kbd> + i <kbd>alt</kbd> + g (page info)
- <kbd>ctrl</kbd> + i <kbd>alt</kbd> + m (page media)
- <kbd>ctrl</kbd> + i <kbd>alt</kbd> + p (page permissions)
- <kbd>ctrl</kbd> + i <kbd>alt</kbd> + s (page security, cookies, etc)

## DevTools

[official doc](https://firefox-source-docs.mozilla.org/devtools-user/index.html)

- <kbd>ctrl</kbd> + <kbd>shift</kbd> + i (or F12) (shows the tool that was last opened)

### Page Inspector

- <kbd>ctrl</kbd> + <kbd>shift</kbd> + c (pick element mode)

### Web Console

From [doc](https://firefox-source-docs.mozilla.org/devtools-user/web_console/):

The Web Console:

1. Logs information associated with a web page: network requests, JavaScript, CSS, security errors and warnings as well as error, warning and informational messages explicitly logged by JavaScript code running in the page context
2. Enables you to interact with a web page by executing JavaScript expressions in the context of the page

Shortcuts:

- <kbd>ctrl</kbd> + <kbd>shift</kbd> + k (console) (click "..." to get side-by-side view)

### JavaScript Debugger

[Shortcuts](https://firefox-source-docs.mozilla.org/devtools-user/keyboard_shortcuts/index.html#keyboard-shortcuts-debugger)

### React DevTools (Addon)

- only visible on webpages built with React
- beside standard firefox DevTools tabs
  - React Components panel
  - React Profiler panel

## vimium 

- T (switch tab)
- click on "vimium button in firefox" &rarr; "Options" (to see custom maps)
- yt (duplicate tab)
- gi (jump into the first search box (must be in the view, does not work if you have to scroll the box into the view!))
- gs (view source)
- HL (tabs)
- JK (history)

## Textmarker

- Notes:
  - **Warning**: the "x" button deletes the note, use the "-" button to hide
- sidebar contains useful buttons, eg.
  - jump from one highlight to the next/previous
  - Notes: if a highlight has a Note attached to it, there is a small white box beside it in the "Marks and Notes" list
    - helps to quickly find all existing Notes (without clicking through all of them)

# cpp

## godbolt

- f1 (command palette)
- <kbd>ctrl</kbd> - x (cut line)
- <kbd>ctrl</kbd> - d (duplicate line)

# markdown

- <kbd>alt</kbd> - i (next `# `)
- <kbd>alt</kbd> - o
- `:LspStop 1 (tailwindcss)` (because the treesitter markdown parser is sufficient)
- <kbd>alt</kbd> - f (insert arrow symbol)

## LaTex

- math mode
  - <kbd>alt</kbd> - b
  - <kbd>alt</kbd> - u
  - <kbd>alt</kbd> - t ($\[ \text{Let}\ x=\text{number of cats}. \]$)

# nvim Plugins

## Lazy

- [dev.to](https://dev.to/vonheikemen/lazynvim-how-to-revert-a-plugin-back-to-a-previous-version-1pdp)
  - restore specific plugin to previous state/version
  - revert all plugins to previous state/version
- `:Lazy` (must update regularly)
- C (check if a tag exists, after changing `tag =` or `version =` in `init.lua`)
- `:checkhealth` (debug broken plugins, shows a log)
  - hint: use `:set ft=markdown` for syntax highlighting
  - `:checkhealth somePluginName`
  
## LSP, Mason

- <kbd>space</kbd> ca (apply code fix)
- `:Mason` (manage all LSPs, DAPs, Linter, Formatter)
- **install new language servers** via `:LspInstall <tab>`
- `:LspInfo`
  - (Note: die "installierten" server sind in der Zeile ganz unten in der Ansicht! Die server, die oben in der Liste sind, sind die "aktiven" für aktuelle buffer!)
- `:LspStop serverName`
  - eg. to stop tailwindcss server for markdown files (&rarr; [markdown](#markdown))

## Diagnostics

- [definition in neovim doc](https://neovim.io/doc/user/diagnostic.html)
  - "Nvim provides a framework for displaying errors or warnings from external tools, otherwise known as **"diagnostics"**. These diagnostics can come from a variety of sources, such as **linters** or **LSP servers**. The diagnostic framework is an extension to existing error handling functionality such as the quickfix list."

## Formatter

- <kbd>leader</kbd><kbd>j</kbd><kbd>key-for-language</kbd>
- available **formatters**:
  - `:ClangFormat`
  - `coc-prettier`
  - `:Format`, alias for `vim.lsp.buf.format()` (defined in `init.lua`)

## clang, clangd

- **clang** is the **compiler**
- **clangd** is the **language server**

### compile commands json

- From `:LspInfo` &rarr; press tab:
  - If `compile_commands.json` lives in a build directory, you should symlink it to the root of your source tree. `ln -s /path/to/myproject/build/compile_commands.json /path/to/myproject/`

### compile flags txt

- run `:LspRestart` to reload
- for simple projects (eg. compile one file only):
  - create a `compile_flags.txt`
    - in the folder where the source code is
    - eg. with the content `-std=c++20`

### .clangd (project configuration file, yaml)

- e.g. to suppress `In included file: main file cannot be included recursively when building a preamble` add 

```yml
Diagnostics:
  Suppress: 'pp_including_mainfile_in_preamble'
```

to `.clangd`

## autocomplete, completion

- 3 popup/drop-down Arten:
  - vim builtin completion
  - `coc.nvim`
  - `nvim-cmp`
- <kbd>ctrl</kbd> - y (confirm + replace)
- <kbd>ctrl</kbd> - e (discard + do not do anything)
  - helpful eg. if two conflicting completion popups appear at the same time, <kbd>ctrl</kbd> - e will switch between these

## nvim-cmp

- requires a "snippet engine"
  - in use: "luasnip"
  - tab, <kbd>shift</kbd>-tab
    - select item in drop-down
      - if the item is a function a snippet is inserted
    - edit next/previous function parameter in the function snippet
  - luasnip config: [Example-mappings#luasnip](https://github.com/hrsh7th/nvim-cmp/wiki/Example-mappings#luasnip)
  - TODO:
    - maybe configure a loader: [LuaSnip#add-snippets](https://github.com/L3MON4D3/LuaSnip#add-snippets)
  
## coc.nvim

- das **completion popup** mit den eckigen Klammer Symbolen in der rechten Spalte (zB `completionVorschlag1 [A]`, `completionVorschlag2 [B]`, usw) wird von `coc.nvim` erzeugt, wobei jedes eckige Klammer Symbol für jeweils eine "source" steht, die in `:CocList` &rarr; "sources" registriert wurde
  - am besten alle "sources" deregistrieren (über `:CocList` &rarr; "sources"), weil sonst der `nvim-cmp` popup und der `coc.nvim` popup manchmal gleichzeitig erscheinen
  - wobei das source namens `File` nützlich ist (zeigt die files in cwd, wenn man `./` typet
- `:CocList` (select "sources" to configure which sources are used for autocompletion)

## nvim-tree

- <kbd>-</kbd> (show more (show parent folder))
- <kbd>ctrl</kbd> <kbd>\]</kbd> (show less (only show the folder on which the cursor is placed, ie. this works only if the cursor is placed on a folder))
- <kbd>c</kbd> then <kbd>p</kbd> (duplicate file, automatically shows "rename" where you have to choose a new name)
- for Firefox-Bookmarks-like behavior use
  - `init.lua`: `on_attach` keymap: bind <kbd>ctrl</kbd>-e to `api.tree.toggle`
  - `keymaps.lua`: normal keymap: bind <kbd>ctrl</kbd>-e to `:NvimTreeFocus`
- <kbd>alt</kbd> - h, <kbd>alt</kbd> - l (focus/unfocus nvim-tree, when nvim-tree open)
- "/" at end (create dir)
- J (jump to last item in folder)
- K (jump to first item in folder)
- i (show gitignored folders and files)
- U (show hidden folders and files)
- q (quit)
- statt netrw
- `/` (search)
- <kbd>ctrl</kbd>-d (collapse)
- gy (get absolute path)
- Y (get relative path)

## toggleterm (for lazygit integration in nvim)

- scroll
  - <kbd>ctrl</kbd>-\ <kbd>ctrl</kbd>-n to exit terminal mode, then you can use vim motions to move around
  - scroll with mouse first and then you can use pageUp/pageDown
- written in lua
- better than floaterm (written in vimscript)
- 1<kbd>ctrl</kbd>-\, 2<kbd>ctrl</kbd>-\, 7<kbd>ctrl</kbd>-\, etc. to create and access terminals
  - the prefixed number is the number of the terminal instance
  - pressing a non-existent number will create a new terminal

## gitsigns

This plugin is only active in git-tracked folders. Ie. `:map` (and, therefore, <kbd>space</kbd> sk) will not show any keymaps related to gitsigns in non-git-tracked folders.

- <kbd>alt</kbd> - <kbd>,</kbd> (next hunk)
- <kbd>alt</kbd> - <kbd>.</kbd> (prev hunk)
- <kbd>space</kbd> h b (blame line)
  - um schnell herauszufinden in welchem commit die line geaddet wurde
- diff
  - <kbd>space</kbd> h d (diff zu letztem commit)
  - <kbd>space</kbd> h D (diff zu vorletztem commit)
  - besser: <kbd>space</kbd> g f (see fugitive)

## lazygit

- each panel has its own help menu!
- <kbd>W</kbd> (**groß** W, diff menu)
- <kbd>ctrl</kbd> + <kbd>r</kbd> (switch repo)
- <kbd>@</kbd> (focus command log)
- <kbd>x</kbd> (help) (ins commit panel und auf x drücken, zeigt zB amend, reset, etc.)
- <kbd>R</kbd> (change multiline commit message)
- <kbd>r</kbd> (change commit message)
- <kbd>A</kbd> (amend)
  - erst <kbd>space</kbd> und dann <kbd>A</kbd>
- <kbd>space</kbd> gg (for normal repos)
- <kbd>space</kbd> gd (for dotfiles repo)
- <kbd>H</kbd>, <kbd>L</kbd> (scroll left, right) 
  - praktisch um kleine panels zu lesen, e.g. "commit" panel
- <kbd>h</kbd>, <kbd>l</kbd>
  - switch between the 5 panels (or 12345)
- <kbd>\[</kbd>, <kbd>\]</kbd>
  - tabs sind getrennt durch "-"
  - zB "Commit" - "Reflog" sind 2 tabs
- <kbd>+</kbd> (rotate through views, <kbd>-</kbd> dann nicht nötig)
- <kbd>esc</kbd> (go back, Achtung: <kbd>q</kbd> is for exit lazygit!)
- <kbd>esc</kbd> (go back to commit list after seeing a commit's files)
- <kbd>pageup</kbd>/<kbd>pagedown</kbd> to scroll
- <kbd>ctrl</kbd> <kbd>-/+</kbd> (zoom out/in, very useful to view the full main panel)
- um mehr vom file zu sehen: 
  - im "Commits" panel den commit fokussieren (aber nicht <kbd>enter</kbd> drücken) und dann mehrmals <kbd>\}</kbd> drücken
    - dies sollte am Ende den ganzen file anzeigen

### lazygit: Git actions

- add
  - <kbd>a</kbd> (git add all)
  - <kbd>space</kbd> (git add file, press again to undo)
- commit
  - <kbd>c</kbd>
- push
  - <kbd>P</kbd> (UPPERcase!)
- pull
  - <kbd>p</kbd> (lowercase!)

## fugitive

- for normal git commands use exclamation mark "`:!git ...`"
- :G (former "Gstatus", press g? to see what you can do)
- :G `<tab><tab>` to see available commands
- :G log
- :G shortlog
- `:Gdiffsplit HEAD~1` (horizontal view)
- `:Gvdiffsplit HEAD~1` (vertical view)
  - <kbd>space</kbd> g f (diff zu weiter zurück liegenden commits)
- :Gread `%` (git checkout) (use u to undo/go back)

## GV

- öffnet sich in neuem Tab, d.h. "H", "L" um mit aktuellem file zu vergleichen
- <kbd>space</kbd> gc (only commits of current file)
- <kbd>space</kbd> ga (all commits)
- q (quit)

## telescope

- `:h` telescope.mappings
- <kbd>ctrl</kbd>-/ (insert mode), <kbd>?</kbd> (normal mode) (show shortcuts)
- <kbd>ctrl</kbd>-t (open file in new tab)
- <kbd>ctrl</kbd> <kbd>-/+</kbd> (see more of the preview pane/results pane)
- <kbd>ctrl</kbd>-u <kbd>ctrl</kbd>-d (scroll preview up/down)
- <kbd>space</kbd> sk (search **keymaps** of active plugins, ie keymaps of inactive plugins are not shown)
- <kbd>space</kbd> ? (recently opened)
- <kbd>space</kbd> `/` (current buffer find)

## treesitter

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
- <kbd>ctrl</kbd> + <kbd>space</kbd> multiple times (select nodes, von innen nach außen)
- press `]` (next) or `[` (previous) to see keybindings
  - jumps to
    - start of next function/class
    - end of next function/class
    
## Aerial

- ToC window mode:
  - <kbd>space</kbd> bb
  - <kbd>space</kbd> bt (Telescope Search)
    - faster than "bb" for jumping to sections
  - also try: <kbd>ctrl</kbd>-n, <kbd>ctrl</kbd>-p (next / previous section)
- sidebar mode:
  - g?
  - q
  - zM (collapse all)
  - <kbd>ctrl</kbd>-b (fast markdown section select)

## Vista (Aerial ist besser!)

- Vista: in "`:h Vista`": tag: `vista-key-mappings`
- q (quit)
- p (preview)

## vim illuminate

- <kbd>alt</kbd>-n
- <kbd>alt</kbd>-p

# vim

- <kbd>;</kbd> (undo repeat)
- <kbd>,</kbd> (repeat)
- <kbd>g</kbd> <kbd>a</kbd> (show ascii code of letter under cursor)

## custom

- <Space>fc (keymaps.lua)
- <Space>fi (init.lua)
- <Space>fp (plugins/init.lua)

## motion (in a line)

- <kbd>f</kbd><kbd>letter</kbd><kbd>,,,,</kbd> (jump to <kbd>letter</kbd>, press <kbd>,</kbd> repeatedly to jump to the next <kbd>letter</kbd>)
- <kbd>B</kbd> (beginning of previous WORD, faster than <kbd>b</kbd>)
- <kbd>E</kbd> (end of WORD, faster than <kbd>e</kbd>)
- <kbd>gE</kbd> (end of previous WORD, <kbd>be</kbd> does not work if cursor currently in middle of a word)
- <kbd>ge</kbd> (end of previous word, <kbd>be</kbd> does not work if cursor currently in middle of a word)
- <kbd>Ea</kbd> (start writing at end of WORD)
- <kbd>ea</kbd> (start writing at end of word)

## motion

- search
  - <kbd>shift</kbd><kbd>N</kbd> (instead of <kbd>?</kbd> to search backwards)
- <kbd>g</kbd><kbd>_</kbd> (end of line, last non-blank character)
- `:changes` related motions (jumping between change locations)
  - <kbd>g;</kbd> and <kbd>g,</kbd> (jump to previous/next change location)
- <kbd>%</kbd> (jump out of parenthesized block to 1st parenthesis)
- <kbd>%%</kbd> (jump out of parenthesized block to 1st parenthesis)
- <kbd>backtick</kbd> (list possible jumps and marks)
  - <kbd>backtick</kbd> <kbd>.</kbd> (jump to last change in current buffer)
  - <kbd>m</kbd> + <kbd>markLetter</kbd> (set jump mark)
  - <kbd>backtick</kbd> + <kbd>markLetter</kbd> (jump to mark)
- move
  - <kbd>)</kbd>, <kbd>(</kbd>
  - <kbd>*</kbd> fwd, <kbd>#</kbd> bwd
- markdown sections
  - <kbd>ctrl</kbd><kbd>n</kbd>, <kbd>ctrl</kbd><kbd>p</kbd> (next / previous section)

## edit

- <kbd>ctrl</kbd>-r % (in insert mode: insert current file name)
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
- <kbd>shift</kbd> - j (append the line below)
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

- <kbd>ctrl</kbd>-o (temporary normal mode for one command)
- `:set ft?` (show filetype)
  - eg. for denylist in vim-illuminate config
- autocomplete
  - <kbd>ctrl</kbd> - y (confirm vim-complete)
  - <kbd>ctrl</kbd> - <kbd>space</kbd> (show cmp-complete)
  - enter (confirm cmp-complete)
  - <kbd>ctrl</kbd> - e (abort complete) (both cmp-complete and vim-complete)
- buffers
  - `:b partialName<tab>`
- sessions
  - <kbd>space</kbd> fod (load default session)
  - <kbd>space</kbd> fsd (save default session)
  - <kbd>space</kbd> fon (load new session)
  - <kbd>space</kbd> fsn (save new session)
- tabs
  - next tab: gt or <kbd>ctrl</kbd> - PageDown (by default in vim)
  - previous tab: gT or <kbd>ctrl</kbd> - PageUp (by default in vim)
- <kbd>alt</kbd>-h and <kbd>alt</kbd>-l (switch viewports)
  - to go to `:AerialToggle` and back (Avoid this! Use `:AerialNavToggle` (<kbd>leader</kbd> bb) instead!)
  - to go to nvim-tree and back

# Edit Code

- <kbd>space</kbd> ca (apply code fix)
- <kbd>space</kbd> rn (rename a variable)
- `_` (like `^`)
  - `[count]_` (1st non-blank char `count-1` lines downward)
- comments
  - repeat to uncomment
  - line: gcc (or gbc)
  - visual selection: gc (or gb)

## surround

- ysl" (surround letter under cursor)
- vSo (surround letter under cursor)
- ys3iw" (surround **ohne select**)
- ys3iW" (see word vs WORD)

# Help Code, Doc

Keybindings:
- `:nmap <some-key>` to list all bindings with `<some-key>`

Definition:
- <kbd>shift</kbd> + k (for <span style={{ color: "red" }}>**objects**</span>: press when cursor is on object; press 2x to jump into def window)
- <kbd>ctrl</kbd> + k (for <span style={{ color: "red" }}>**functions**</span>: press keys inside function brackets; press 2x to jump into def window)
- gd (go to def)
- gr (go to references)

Search through content of ONE file:
- (symbols: classes, properties, methods)
  - <kbd>space</kbd> ds (native lsp)
  - <kbd>space</kbd> bb (aerial)
    - <kbd>space</kbd> bm (collapse all in aerial)
  - <kbd>space</kbd> bt (aerial in telescope)
- gr (e.g. cycle through all occurrences of a variable in the file)

Search through content of ALL files:
- <kbd>space</kbd> sw (search word under cursor; gives the same results as <kbd>space</kbd> sg)
- <kbd>space</kbd> sg (search by grep all files in cwd)

`nvim-cmp` steuert scrolling in definition preview popup
- press <kbd>ctrl</kbd>-f to scroll down
- alle key mappings in `.config/nvim/init.lua` bei `cmp.setup { ..., mapping = ... }`
