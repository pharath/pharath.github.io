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

- <kbd>ctrl</kbd><kbd>l</kbd> (clear)

# firefox

- <kbd>ctrl</kbd> <kbd>u</kbd> (view page source code)

## Page Info

- <kbd>ctrl</kbd><kbd>i</kbd> <kbd>alt</kbd><kbd>g</kbd> (page info)
- <kbd>ctrl</kbd><kbd>i</kbd> <kbd>alt</kbd><kbd>m</kbd> (page media)
- <kbd>ctrl</kbd><kbd>i</kbd> <kbd>alt</kbd><kbd>p</kbd> (page permissions)
- <kbd>ctrl</kbd><kbd>i</kbd> <kbd>alt</kbd><kbd>s</kbd> (page security, cookies, etc)

## DevTools

[official doc](https://firefox-source-docs.mozilla.org/devtools-user/index.html)

- <kbd>ctrl</kbd> + <kbd>shift</kbd><kbd>i</kbd> (or F12) (shows the tool that was last opened)

### Page Inspector

- <kbd>ctrl</kbd> + <kbd>shift</kbd><kbd>c</kbd> (pick element mode)

### Web Console

From [doc](https://firefox-source-docs.mozilla.org/devtools-user/web_console/):

The Web Console:

1. Logs information associated with a web page: network requests, JavaScript, CSS, security errors and warnings as well as error, warning and informational messages explicitly logged by JavaScript code running in the page context
2. Enables you to interact with a web page by executing JavaScript expressions in the context of the page

Shortcuts:

- <kbd>ctrl</kbd> + <kbd>shift</kbd><kbd>k</kbd> (console) (click "..." to get side-by-side view)

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
- <kbd>ctrl</kbd><kbd>x</kbd> (cut line)
- <kbd>ctrl</kbd><kbd>d</kbd> (duplicate line)

# markdown

- <kbd>alt</kbd><kbd>i</kbd> (next `# `)
- <kbd>alt</kbd><kbd>o</kbd>
- `:LspStop 1 (tailwindcss)` (because the treesitter markdown parser is sufficient)
- <kbd>alt</kbd><kbd>f</kbd> (insert arrow symbol)

## vim-markdown

- ge (format a table)
  - when the table contains code with a pipe symbol "`|`" this won't work, but you can use the following workaround: 
    - first escape all pipe symbols with a backslash `\`, but not the `|`-symbols of the table
    - format the table with <kbd>ge</kbd>
    - manually remove all `\` again
  - this works only if all rows of the table start with a "`|`"-symbol
    - you can run something like ``:%s/\(^`.*`\s|\)/| \1/g`` to reformat all tables in your markdown files, so that <kbd>ge</kbd> works

## LaTex

- math mode
  - <kbd>alt</kbd><kbd>b</kbd>
  - <kbd>alt</kbd><kbd>u</kbd>
  - <kbd>alt</kbd><kbd>t</kbd> ($\[ \text{Let}\ x=\text{number of cats}. \]$)

# nvim

## Debugging

- `print(lua-variable)`
- `print'some-string'`

## Useful Help Pages

- `:h lua-guide` (basics of lua)
- `:h lua-vim-variables`
- `:h highlight-groups` (meaning of each highlight-group, ie. which vim objects each highlight-group affects)
- `:h 'runtimepath'`
- `:help lspconfig-all` (lsp server configuration: configuration options for each lsp server)
- `:h lspconfig-setup` (lsp server configuration: global)
- lua
  - [lua modules, what does init.lua do in a folder?](https://neovim.io/doc/user/lua-guide.html#_lua-modules)
- lazy
  - [plugin config table fields](https://lazy.folke.io/spec)
  - [keymap syntax](https://lazy.folke.io/spec/lazy_loading#%EF%B8%8F-lazy-key-mappings)

## Variables

- `:echo $SOME_ENV_VAR` (print the value of `SOME_ENV_VAR`)
- `:echo g:someGlobalVar` (print the value of `vim.g.someGlobalVar`)
- `:echo b:someBufferVar` (print the value of `vim.b.someBufferVar`)
  - eg. `:echo b:AutoPairs` shows the list of all currently set AutoPairs for the current buffer
- `:set someOption?` (print the value of `someOption`)
- `:lua =table` (print a lua table)

# nvim Plugins

## Lazy

- [plugin spec: list of properties when adding plugins to init.lua](https://lazy.folke.io/spec)
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

## lsp_signature.nvim

- shows the function signature in a floating window while typing
- highlights the current parameter in the signature
- <kbd>alt</kbd><kbd>p</kbd> (focus the floating window, put the cursor in the floating window)
  - <kbd>alt</kbd><kbd>u</kbd>, <kbd>alt</kbd><kbd>d</kbd> (while in the floating window: move cursor up/down)

## Diagnostics

- [definition in neovim doc](https://neovim.io/doc/user/diagnostic.html)
  - "Nvim provides a framework for displaying errors or warnings from external tools, otherwise known as **"diagnostics"**. These diagnostics can come from a variety of sources, such as **linters** or **LSP servers**. The diagnostic framework is an extension to existing error handling functionality such as the **quickfix list**."
- <kbd>leader</kbd><kbd>e</kbd> (open floating diagnostic message, useful if you cannot read the inline message)

## Formatter

- list
  - markdown: `prettier.formatfile`
  - lua: `:Format` (LSP format buffer)
  - cpp: `:ClangFormat` (rhysd/vim-clang-format)
- TODO: install [conform.nvim](https://github.com/stevearc/conform.nvim)
- <kbd>leader</kbd><kbd>j</kbd><kbd>key-for-language</kbd>
- available **formatters**:
  - `:ClangFormat`
  - `coc-prettier`, [github](https://github.com/neoclide/coc-prettier)
    - typical workflow:
      - To install prettier in your project and pin its version [as recommended](https://prettier.io/docs/en/install.html), run: `npm install prettier -D --save-exact`
      - Then, create an empty config file to let editors and other tools know you are using Prettier: `node --eval "fs.writeFileSync('.prettierrc','{}\n')"`
      - Next, create a .prettierignore file to let the Prettier CLI and editors know which files to not format. Here’s an example: `node --eval "fs.writeFileSync('.prettierignore','# Ignore artifacts:\nbuild\ncoverage\n')"`
      - Now, format all files with Prettier: `npx prettier . --write`
        - `prettier --write .` is great for formatting everything, but for a big project it might take a little while. You may run `prettier --write app/` to format a certain directory, or `prettier --write app/components/Button.js` to format a certain file.
  - `:Format`, alias for `vim.lsp.buf.format()` (defined in `init.lua`)

## clang, clangd

- **clang** is the **compiler**
- **clangd** is the **language server**

### clang

- [doc: all flags](https://clang.llvm.org/docs/ClangCommandLineReference.html#compilation-options)

### compile commands json

- From `:LspInfo` &rarr; press tab:
  - If `compile_commands.json` lives in a build directory, you should symlink it to the root of your source tree. `ln -s /path/to/myproject/build/compile_commands.json /path/to/myproject/`

### compile flags txt

- [doc](https://clangd.llvm.org/installation#compile_flagstxt)
- run `:LspRestart` to reload
- for simple projects (eg. compile one file only):
  - create a `compile_flags.txt`
    - in the folder where the source code is
    - eg. with the content `-std=c++20`

Example `compile_flags.txt`:

```txt
-nostdlibinc
-I/home/bra-ket/osbook/devenv/x86_64-elf/include
-I/home/bra-ket/osbook/devenv/x86_64-elf/include/c++/v1
-I.
-D__ELF__
-D_LIBCPP_HAS_NO_THREADS
-O2
-Wall
-g
--target=x86_64-elf
-fno-exceptions
-ffreestanding
-fno-rtti
-std=c++2a
```

### .clangd (project configuration file, yaml)

- e.g. to suppress `In included file: main file cannot be included recursively when building a preamble` add 

```yml
Diagnostics:
  Suppress: 'pp_including_mainfile_in_preamble'
```

to `.clangd`

## autocomplete, completion

- see also [Help Code, Doc](#documentation)
- 3 popup/drop-down Arten:
  - vim builtin completion (sieht so aus wie `completionVorschlag1`)
  - `coc.nvim` (sieht so aus wie `completionVorschlag1 [A]`)
  - `nvim-cmp` (sieht so aus wie `completionVorschlag1 Variable`)
- <kbd>ctrl</kbd> <kbd>y</kbd> (confirm + replace)
- <kbd>ctrl</kbd> <kbd>e</kbd> (discard + do not do anything)
  - helpful eg. if two conflicting completion popups appear at the same time, <kbd>ctrl</kbd> <kbd>e</kbd> will switch between these

### vim builtin completion

- <kbd>ctrl</kbd> <kbd>n</kbd> (next suggestion)
- <kbd>ctrl</kbd> <kbd>p</kbd> (previous suggestion)
- <kbd>ctrl</kbd> <kbd>x</kbd> followed by <kbd>ctrl</kbd> <kbd>f</kbd> (Path completion)
  - select with <kbd>ctrl</kbd> <kbd>n</kbd> then press <kbd>ctrl</kbd> <kbd>x</kbd> followed by <kbd>ctrl</kbd> <kbd>f</kbd> again to select the next subfolder etc.

### nvim-cmp, LuaSnip, friendly-snippets

- das **completion popup** mit den Kategorien (`Variable`, `Function`, `Keyword`, usw) in der rechten Spalte (zB `completionVorschlag1 Variable`, `completionVorschlag2 Keyword`, `completionVorschlag2 Function`, usw) wird von `nvim-cmp` erzeugt
- servers must be added to the LSP server list `local servers = {}` in `init.lua`, otherwise `nvim-cmp` will not autocomplete
- `luasnip` (snippet engine)
  - `:LuaSnipListAvailable`
  - `nvim-cmp` requires a snippet engine
  - <kbd>tab</kbd>, <kbd>shift</kbd><kbd>tab</kbd>
    - select item in drop-down
      - if the item is a function a snippet is inserted
    - edit next/previous function parameter in the function snippet
  - luasnip config: [Example-mappings#luasnip](https://github.com/hrsh7th/nvim-cmp/wiki/Example-mappings#luasnip)
  - TODO:
    - maybe configure a loader: [LuaSnip#add-snippets](https://github.com/L3MON4D3/LuaSnip#add-snippets)
- `friendly-snippets` (snippet collection)
  - requires `luasnip`
  - some snippets contained in this collection are disabled by default, but you can enable them
  - see [tips for using friendly-snippets](#friendly-snippets)
- `nvim-cmp` steuert scrolling im "documentation preview" popup
  - press <kbd>ctrl</kbd> <kbd>f</kbd> to scroll down
  - alle key mappings in `.config/nvim/init.lua` bei `cmp.setup { ..., mapping = ... }`
  - **problem**: you cannot jump into the documentation preview window in nvim-cmp (which is important to open links in the documentation preview window, eg. MDN links for JavaScript)
    - **solution**: instead, you can first autocomplete an expression and then press <kbd>ctrl</kbd> <kbd>k</kbd> or <kbd>shift</kbd> <kbd>k</kbd> while the cursor is on the completed expression
  
### friendly-snippets

<span style="color:red">**important**</span>: Each language has its own [Wiki](https://github.com/rafamadriz/friendly-snippets/wiki) page where you can find all <span style="color:red">**keymaps**</span> to trigger the snippets.

- [JavaScript](https://github.com/rafamadriz/friendly-snippets/wiki/Javascript,-Typescript,-Javascriptreact,-Typescriptreact)
- [React](https://github.com/rafamadriz/friendly-snippets/wiki/Javascript,-Typescript,-Javascriptreact,-Typescriptreact#react-snippets)
- [html](https://github.com/rafamadriz/friendly-snippets/wiki/HTML,-Pug,-Jade)

Other Tricks:

How to type the following code?:

```
<div className='wrapper'>
  |
</div>
```

- steps:
  - press <kbd>j</kbd> and select the "jsx element" snippet
  - write `div` into the first jsx tag and press <kbd>Esc</kbd> (or <kbd>jj</kbd>)
    - this will write another `div` into the second jsx tag while the cursor remains in its current position
  - press <kbd>a</kbd> and write `classN`, then press tab to select `className?~ Field`, press <kbd>Enter</kbd> on `className?~ Field`
    - this will put a `className='|'` attribute into the first jsx tag with the cursor positioned in between the single quotes (where the vertical bar `|` is)
  - write `wrapper`
  - after typing the last `r` of `wrapper` press <kbd>ctrl</kbd><kbd>o</kbd> and then <kbd>j</kbd>
    - this will place the cursor in the middle line between the first and second jsx tag (where the vertical bar `|` is) in insert mode
    - you can press <kbd>ctrl</kbd><kbd>o</kbd><kbd>j</kbd> even <span style="color:red">**after**</span> just typing `wra`, pressing tab and selecting `wrapper` and <span style="color:red">**before**</span> pressing <kbd>Enter</kbd> on `wrapper` in the popup (ie. once you select `wrapper`, the completed word `wrapper` will not disappear when you press <kbd>ctrl</kbd><kbd>o</kbd><kbd>j</kbd>)

### coc.nvim

- das **completion popup** mit den eckigen Klammer Symbolen in der rechten Spalte (zB `completionVorschlag1 [A]`, `completionVorschlag2 [B]`, usw) wird von `coc.nvim` erzeugt, wobei jedes eckige Klammer Symbol für jeweils eine "source" steht, die in `:CocList` &rarr; "sources" registriert wurde
  - am besten alle "sources" deregistrieren (über `:CocList` &rarr; "sources"), weil sonst der `nvim-cmp` popup und der `coc.nvim` popup manchmal gleichzeitig erscheinen
  - wobei das source namens `File` nützlich ist (zeigt die files in cwd, wenn man `./` typet)
- `:CocList` (select "sources" to configure which sources are used for autocompletion)
- `:CocConfig` (opens `coc-settings.json` where you can configure Coc, eg. you can turn off Coc's autocompletion by adding `"suggest.autoTrigger": "none"` to `coc-settings.json`)

## nvim-tree

- <kbd>P</kbd> (parent directory)
  - <kbd>P</kbd><kbd>tab</kbd> (collapse parent)
- <kbd>f</kbd> `somepattern` (filter the tree)
- <kbd>-</kbd> (show more (show parent folder))
- <kbd>ctrl</kbd><kbd>]</kbd> (show less (only show the folder on which the cursor is placed, ie. this works only if the cursor is placed on a folder))
- <kbd>c</kbd> then <kbd>p</kbd> (duplicate file, automatically shows "rename" where you have to choose a new name)
- for Firefox-Bookmarks-like behavior use
  - `init.lua`: `on_attach` keymap: bind <kbd>ctrl</kbd><kbd>e</kbd> to `api.tree.toggle`
  - `keymaps.lua`: normal keymap: bind <kbd>ctrl</kbd><kbd>e</kbd> to `:NvimTreeFocus`
- <kbd>alt</kbd><kbd>h</kbd>, <kbd>alt</kbd><kbd>l</kbd> (focus/unfocus nvim-tree, when nvim-tree open)
- <kbd>a</kbd> then put a "/" at the end of the written name (create dir)
- <kbd>J</kbd> (jump to last item in folder)
- <kbd>K</kbd> (jump to first item in folder)
- <kbd>i</kbd> (show gitignored folders and files)
- <kbd>U</kbd> (show hidden folders and files)
- <kbd>q</kbd> (quit)
- statt netrw
- <kbd>/</kbd> (search)
- <kbd>ctrl</kbd><kbd>d</kbd> (collapse)
- <kbd>g</kbd><kbd>y</kbd> (get absolute path)
- <kbd>Y</kbd> (get relative path)

## toggleterm (for lazygit integration in nvim)

- enter terminal mode: press <kbd>i</kbd>
- scroll (= enter normal mode)
  - <kbd>ctrl</kbd><kbd>\</kbd> <kbd>ctrl</kbd><kbd>n</kbd> to exit terminal mode, then you can use vim motions to move around
  - scroll with mouse first and then you can use <kbd>pageUp</kbd>/<kbd>pageDown</kbd>
  - press <kbd>i</kbd> to enter terminal mode again
- written in lua
- better than floaterm (written in vimscript)
- <kbd>1</kbd><kbd>ctrl</kbd><kbd>\</kbd>, <kbd>2</kbd><kbd>ctrl</kbd><kbd>\</kbd>, <kbd>7</kbd><kbd>ctrl</kbd><kbd>\</kbd>, etc. to create and access terminals
  - the prefixed number is the number of the terminal instance
  - pressing a non-existent number will create a new terminal

## git

### gitsigns

This plugin is only active in git-tracked folders. Ie. `:map` (and, therefore, <kbd>space</kbd> sk) will not show any keymaps related to gitsigns in non-git-tracked folders.

- <kbd>alt</kbd> <kbd>,</kbd> (next hunk)
- <kbd>alt</kbd> <kbd>.</kbd> (prev hunk)
- <kbd>space</kbd> <kbd>h</kbd> <kbd>b</kbd> (blame line)
  - um schnell herauszufinden in welchem commit die line geaddet wurde
  - press the keymap twice to jump into the preview window (useful if you want to copy the commit hash in the preview window in order to see the whole commit)
- diff
  - <kbd>space</kbd> <kbd>h</kbd> <kbd>d</kbd> (diff zu letztem commit)
  - <kbd>space</kbd> <kbd>h</kbd> <kbd>D</kbd> (diff zu vorletztem commit)
  - better use fugitive: <kbd>space</kbd> <kbd>g</kbd> <kbd>f</kbd> (see fugitive)

### lazygit

- [meaning of keymaps (official doc)](https://github.com/jesseduffield/lazygit/blob/master/docs/keybindings/Keybindings_en.md)
- <kbd>alt</kbd><kbd>enter</kbd> (when writing a commit message: starts a new line)
- when lazygit spins up the external hard drive so that lazygit lags, open `nvim ~/.config/lazygit/state.yml` and run `:g/^-\ \/media/d` to remove all "recent repo" paths to a git repo on the external hard drive
- start lazygit
  - <kbd>space</kbd> gg (for normal repos)
  - <kbd>space</kbd> gd (for dotfiles repo)
- each panel has its own help menu!
- commands that work in all panels
  - <kbd>R</kbd> (Refresh the git state (i.e. run `git status`, `git branch`, etc in background to update the contents of panels). This does not run `git fetch`.)
    - does not work in the "branches" panel because "rename branch" is also mapped to <kbd>R</kbd>
  - <kbd>W</kbd> (**groß** W, diff menu)
  - <kbd>ctrl</kbd> <kbd>r</kbd> (switch repo)
    - does not work when a commit is selected during cherry-picking (see commands under "commits" panel)
  - <kbd>@</kbd> (focus command log)
  - <kbd>x</kbd> (help) (ins commit panel und auf x drücken, zeigt zB amend, reset, etc.)
  - <kbd>H</kbd>, <kbd>L</kbd> (scroll left, right) 
    - praktisch um kleine panels zu lesen, e.g. "commit" panel
  - <kbd>h</kbd>, <kbd>l</kbd>
    - switch between the 5 panels (or 12345)
  - <kbd>\[</kbd>, <kbd>\]</kbd>
    - tabs sind getrennt durch "-"
    - zB "Commit" - "Reflog" sind 2 tabs
  - <kbd>+</kbd> (rotate through views, zoom-out <kbd>-</kbd> not necessary then)
  - <kbd>esc</kbd> (go back, Achtung: <kbd>q</kbd> is for exit lazygit!)
  - <kbd>esc</kbd> (go back to commit list after seeing a commit's files)
  - <kbd>pageup</kbd>/<kbd>pagedown</kbd> to scroll the main panel
  - <kbd>ctrl</kbd> <kbd>-/+</kbd> (zoom out/in, very useful to view the full main panel)
    - better: press <kbd>enter</kbd> on the file in the "files" panel, then press <kbd>+</kbd> to rotate to the view where the file spans the full width of the screen
- in "files" panel
  - <kbd>d</kbd> (git checkout, to discard changes made to a file)
  - <kbd>A</kbd> (amend)
    - erst <kbd>space</kbd> und dann <kbd>A</kbd>
- in "commits" panel
  - <kbd>ctrl</kbd><kbd>s</kbd> (show commits for a specific file only)
    - when you want to see the commits of a deleted file you have to checkout one of the commits before the commit where the file was deleted, then <kbd>ctrl</kbd><kbd>s</kbd> will work for this file (when you are done, you can checkout `master` in the "branches" panel to get back the latest commits)
  - <kbd>/</kbd> (search commits, <span style="color:red">**Warning**</span>: if a commit message has more than one line the search will only search in the first line of the commit message, [doc](https://github.com/jesseduffield/lazygit/blob/master/docs/Searching.md))
  - <kbd>+</kbd> (view awesome [commit graph](https://github.com/jesseduffield/lazygit#commit-graph), or just to make the <span style="color:red">**commit messages**</span> more readable)
  - <kbd>ctrl</kbd><kbd>o</kbd> (copy commit SHA to clipboard; useful eg. when writing commit messages that refer to other commits)
  - um mehr vom file zu sehen:
    - im "Commits" panel den commit fokussieren (aber nicht <kbd>enter</kbd> drücken) und dann mehrmals <kbd>\}</kbd> drücken, sodass am Ende der ganze file sichtbar ist
    - better: press <kbd>enter</kbd> on the file in the "files" panel, then press <kbd>+</kbd> to rotate to the view where the file spans the full width of the screen
  - reword commit messages
    - <kbd>R</kbd> (change multiline commit message)
    - <kbd>r</kbd> (change commit message)
  - cherry-pick
    - <kbd>c</kbd> (select cherry-pick)
    - <kbd>v</kbd> (apply cherry-pick to the current branch)
    - <kbd>ctrl</kbd> <kbd>r</kbd> (unselect cherry-pick)
- in "stash" panel (first you have to stash changes in the "files" panel)
  - <kbd>space</kbd> (apply)
  - <kbd>g</kbd> (pop)

#### lazygit: Git actions

- scroll
  - <kbd>H</kbd> / <kbd>L</kbd> (left/right) (useful when committing hunks with the "stage individual hunks" function)
- add
  - <kbd>a</kbd> (git add all)
  - <kbd>space</kbd> (git add file, press again to undo)
- commit
  - <kbd>c</kbd>
- push
  - <kbd>P</kbd> (UPPERcase!)
- pull
  - <kbd>p</kbd> (lowercase!)

### fugitive

- for normal git commands use exclamation mark "`:!git ...`"
- `:G` (former "Gstatus", press g? to see what you can do)
- `:G <tab><tab>` to see available commands
- `:G log`
- `:G shortlog`
- `:Gdiffsplit HEAD~1` (horizontal view)
- `:Gvdiffsplit HEAD~1` (vertical view)
  - <kbd>space</kbd> <kbd>g</kbd> <kbd>f</kbd>
  - in the right buffer select the changes you want to add with <kbd>v</kbd> and then press <kbd>d</kbd><kbd>p</kbd> (so that the changes are added to the left buffer), then `:write` the left buffer to stage (aka git add) these changes, see `:h :Gdiffsplit`
- `:Gread %` (git checkout) (use u to undo/go back)

### GV

- öffnet sich in neuem Tab, d.h. "H", "L" um mit aktuellem file zu vergleichen
- <kbd>space</kbd> gc (only commits of current file)
- <kbd>space</kbd> ga (all commits)
- q (quit)

## telescope

- `'` (prefix `'` to the search string to get an exact match instead of a fuzzy match, eg. useful when searching keymaps by writing the key combination)
- `:h` (telescope.mappings)
- <kbd>ctrl</kbd><kbd>/</kbd> (in insert mode), <kbd>?</kbd> (in normal mode) (show shortcuts)
- <kbd>space</kbd><kbd>space</kbd> (recent files)
  - based on `:oldfiles` (`:h :o`)
    - eg. you can filter the `:oldfiles` list using `:filter pattern o` (where `:o` is the short version of `:oldfiles`, see `:h :o`)
  - to remove entries in `:oldfiles` edit the "ShaDa" file <span style="color:red">**using nvim**</span> `nvim .local/state/nvim/shada/main.shada` (see [vi.stackexchange](https://vi.stackexchange.com/a/17260))
    - eg. you can install `michaeljsmith/vim-indent-object` and then use `:g/pattern/norm dai`
    - note: `nvim` has a different `:oldfiles` system than `vim` (see [vi.stackexchange](https://vi.stackexchange.com/a/17260))
    - useful if you accidentally put some files in the `:oldfiles` list that are on an external hard drive (the hard drive has to spin up each time you press this telescope shortcut while the frozen telescope window is blocking `nvim`)
      - in this case you can run `:g/\/media\/bra-ket\/INTENSO\//norm dai` to clear the `:oldfiles` list from all entries that are causing this behavior
- ignoring files ([comment](https://github.com/nvim-telescope/telescope.nvim/issues/2471#issuecomment-1513758675)):
  - put them in `.ignore`
  - use the `file_ignore_patterns` telescope option (`:h telescope.defaults.file_ignore_patterns`)
  - use `.gitignore`
- <kbd>ctrl</kbd><kbd>t</kbd> (open file in new tab)
- <kbd>ctrl</kbd><kbd>-/+</kbd> (see more of the preview pane/results pane)
- <kbd>ctrl</kbd><kbd>u</kbd>, <kbd>ctrl</kbd><kbd>d</kbd> (scroll preview up/down)
- <kbd>space</kbd><kbd>sk</kbd> (search **keymaps** of active plugins, ie keymaps of inactive plugins are not shown)
- <kbd>space</kbd><kbd>sa</kbd> (search **autocommands**, my own custom autocommands have a `ph_` prefix in their `augroup` so that I can find them more easily in telescope or via `:autocmd ph_<tab>`)
  - [what is augroup?](https://www.reddit.com/r/neovim/comments/xhtr1p/nvim_autocmd_filetype_option/)
  - **better**: use `:autocmd ph_<tab>` to find autocommands and their doc (specified in the `desc = "..."` field of `nvim_create_autocmd`)
- <kbd>space</kbd><kbd>?</kbd> (recently opened)
- <kbd>space</kbd><kbd>/</kbd> (current buffer find)

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
  - `:TSModuleInfo` (colorful overview of installed features `:h :TSModuleInfo`)
  - `:TSUninstall parserName`
- <kbd>v</kbd> multiple times (select nodes, von innen nach außen)
  - <kbd>alt</kbd><kbd>v</kbd> (decrement selection)
  - <kbd>shift</kbd><kbd>v</kbd> ("select line" works as usual)
- press <kbd>]</kbd> (next) or <kbd>[</kbd> (previous) to see keybindings
  - jumps to
    - start of next function/class
    - end of next function/class

### treesitter-textobjects

- an external plugin for the main treesitter plugin
- <kbd>i</kbd><kbd>f</kbd> and <kbd>a</kbd><kbd>f</kbd> (inner/outer function)
  - eg. <kbd>cif</kbd> to delete the inner part of a function
    
## Aerial

- ToC window mode:
  - <kbd>space</kbd> bb
  - <kbd>space</kbd> bt (Telescope Search)
    - faster than "bb" for jumping to sections
  - also try: <kbd>ctrl</kbd><kbd>n</kbd>, <kbd>ctrl</kbd><kbd>p</kbd> (next / previous section)
- sidebar mode:
  - g?
  - q
  - zM (collapse all)
  - <kbd>ctrl</kbd><kbd>b</kbd> (fast markdown section select)

## Vista (Aerial ist besser!)

- Vista: in "`:h Vista`": tag: `vista-key-mappings`
- <kbd>q</kbd> (quit)
- <kbd>p</kbd> (preview)

## vim illuminate

- [advantages of using vim-illuminate in nvim](https://www.reddit.com/r/neovim/comments/z6s4qo/is_vimilluminate_useful_for_neovim_users/)
- <kbd>alt</kbd><kbd>n</kbd>
- <kbd>alt</kbd><kbd>p</kbd>

## auto-session

- while in the session-lens telescope picker:
  - <kbd>c-s</kbd> **restores** the previously opened session. This can give you a nice flow if you're constantly switching between two projects.
  - <kbd>c-d</kbd> will **delete** the currently highlighted session. This makes it easy to keep the session list clean.
- <kbd>space</kbd> <kbd>cs</kbd> (**save** session)

# nvim Profiling

[reddit.com](https://www.reddit.com/r/neovim/comments/ht6mk4/neovim_starts_being_really_slow_when_working_on/)

Try run nvim with `nvim -u NONE`. Is it still slow?

Try profiling when inside that file.

```
:profile start profile.log
:profile func *
:profile file *.
```

Now start doing what is slow

```
:profile pause.
```

Quit vim and open `profile.log`.

At the end you should see sum of functions exec times and count of usage. Should point you.

bot summon `:help profile` 

# Finding Keymaps

- `:nmap <some-key>` to list all bindings with `<some-key>`
- <kbd>space</kbd> <kbd>sk</kbd> (show keybindings in Telescope)

# Understanding Code

## Documentation

- LSP Documentation:
  - <kbd>shift</kbd> <kbd>k</kbd> (hover doc, for <span style="color:red">**objects**</span>: press when cursor is on object)
    - press 2x to jump into def window
    - press <kbd>q</kbd> to jump out
  - <kbd>ctrl</kbd> <kbd>k</kbd> (function signature doc, for <span style="color:red">**functions**</span>: press when cursor inside function brackets)
    - press 2x to jump into def window
    - press <kbd>q</kbd> to jump out
- `nvim-cmp` documentation preview window
  - **problem**: you cannot jump into the documentation preview window in nvim-cmp (which is important to open links in the documentation preview window, eg. MDN links for JavaScript)
    - **solution**: instead, you can first autocomplete an expression and then press <kbd>ctrl</kbd> <kbd>k</kbd> or <kbd>shift</kbd> <kbd>k</kbd> while the cursor is on the completed expression

## Definitions

- LSP Definitions:
  - gd (go to def)
  - gr (go to references)

## Search for Words

- Search through content of ONE file:
  - (symbols: classes, properties, methods)
    - <kbd>space</kbd> ds (native lsp)
    - <kbd>space</kbd> bb (aerial)
      - <kbd>space</kbd> bm (collapse all in aerial)
    - <kbd>space</kbd> bt (aerial in telescope)
  - gr (e.g. cycle through all occurrences of a variable in the file)

- Search through content of ALL files:
  - <kbd>space</kbd> sw (search word under cursor; gives the same results as <kbd>space</kbd> sg)
  - <kbd>space</kbd> sg (search by grep all files in cwd)

# vim

- <kbd>;</kbd> (undo repeat)
- <kbd>,</kbd> (repeat)
- <kbd>g</kbd><kbd>a</kbd> (show ascii code of letter under cursor)

## custom

- <Space>fc (`keymaps.lua`)
- <Space>fi (`init.lua`)
- <Space>fp (`plugins/init.lua`)

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
- blocks
  - <kbd>[%</kbd> (jump to beginning of block) &rarr; <kbd>alt</kbd> <kbd>h</kbd>
  - <kbd>]%</kbd> (jump to end of block) &rarr; <kbd>alt</kbd> <kbd>l</kbd>
    - sometimes when there are multiple brackets and parentheses <kbd>[{</kbd> or <kbd>[(</kbd> can be faster than <kbd>[%</kbd>
    - see [vi.stackexchange](https://vi.stackexchange.com/a/16854)
  - <kbd>%</kbd> (jump out of parenthesized block to 1st parenthesis)
  - <kbd>%%</kbd> (jump out of parenthesized block to 2nd parenthesis)
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

- <kbd>ctrl</kbd><kbd>r</kbd> % (in insert mode: insert current file name)
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
- <kbd>shift</kbd><kbd>j</kbd> (append the line below)
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

- `:mes[sages]` (Show all messages.)
- <kbd>ctrl</kbd><kbd>o</kbd> (temporary normal mode for one command)
- `:set ft?` (show filetype)
  - eg. for denylist in vim-illuminate config
- autocomplete
  - <kbd>ctrl</kbd><kbd>y</kbd> (confirm vim-complete)
  - <kbd>ctrl</kbd><kbd>space</kbd> (show cmp-complete)
  - enter (confirm cmp-complete)
  - <kbd>ctrl</kbd><kbd>e</kbd> (abort complete) (both cmp-complete and vim-complete)
- buffers
  - `:b partialName<tab>` (open buffer)
  - `:ls` (list buffers)
- sessions
  - <kbd>space</kbd><kbd>fod</kbd> (load default session)
  - <kbd>space</kbd><kbd>fsd</kbd> (save default session)
  - <kbd>space</kbd><kbd>fon</kbd> (load new session)
  - <kbd>space</kbd><kbd>fsn</kbd> (save new session)
- tabs
  - next tab: <kbd>gt</kbd> or <kbd>ctrl</kbd><kbd>P</kbd>ageDown (by default in vim)
  - previous tab: <kbd>gT</kbd> or <kbd>ctrl</kbd><kbd>P</kbd>ageUp (by default in vim)
  - last accessed tab: <kbd>g</kbd><kbd>tab</kbd> (by default in vim, `:h ctrl-<tab>`)
  - list tabs: `:tabs` (also shows which ones are modified, `:h :tabs`)
- <kbd>alt</kbd><kbd>h</kbd> and <kbd>alt</kbd><kbd>l</kbd> (switch viewports)
  - to go to `:AerialToggle` and back (Avoid this! Use `:AerialNavToggle` (<kbd>leader</kbd><kbd>bb</kbd>) instead!)
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

- dst (delete surrounding tag)
- ds) (delete surrounding parentheses)
- ds" (delete surrounding ")
- ds' (delete surrounding ')
- ysl" (surround letter under cursor)
- vSo (surround letter under cursor)
- ys3iw" (surround **ohne select**)
- ys3iW" (see word vs WORD)

# Troubleshooting

- auto-session errors can often be resolved by simply deleting the session (with <kbd>ctrl</kbd><kbd>d</kbd>) in the session list (press <kbd>leader</kbd><kbd>s</kbd><kbd>s</kbd>) and then restarting nvim
