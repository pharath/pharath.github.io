---
title: "Vim Cheatsheet"
read_time: false
excerpt: "Some essential vim commands"
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
  - cheatsheet
---

# Plugins

- `VundleVim/Vundle.vim`
    - **install**: add `Plugin 'github_repo_name'` to `.vimrc`, `PluginInstall`
    - **uninstall**: remove `Plugin 'github_repo_name'` from `.vimrc`, `PluginUpdate`, restart vim, `PluginClean`
- `rip-rip/clang_complete` (install: see [below](#install-autocomplete-clang_complete))
- for `junegunn/fzf`: when this plugin asks for downloading the executable, confirm with "y"

```vim
" braket Plugins

Plugin 'NLKNguyen/papercolor-theme'

" c++ autocomplete
Plugin 'rip-rip/clang_complete'

" markdown editing in vim
"Plugin 'godlygeek/tabular' " required for vim-markdown plugin
"Plugin 'plasticboy/vim-markdown'

" markdown previewing in a browser
Plugin 'iamcco/markdown-preview.nvim'

" vscode dark theme
Plugin 'tomasiser/vim-code-dark'
Plugin 'octol/vim-cpp-enhanced-highlight'

" fuzzy file search
Plugin 'junegunn/fzf'
Plugin 'junegunn/fzf.vim'

" Viewer & Finder for LSP symbols and tags 
" (built to replace "tagbar" plugin which does not support LSP)
" (LSP: https://en.wikipedia.org/wiki/Language_Server_Protocol)
Plugin 'liuchengxu/vista.vim'

" Delete/change/add parentheses/quotes/XML-tags/much more with ease
Plugin 'tpope/vim-surround'

" copying to the system clipboard with text-objects and motions
Plugin 'christoomey/vim-system-copy'
```

## Install Autocomplete `clang_complete` 

1. Download [https://apt.llvm.org/llvm.sh](https://apt.llvm.org/llvm.sh)
2. `chmod u+x ~/Downloads/llvm.sh`
3. `bash -c ~/Downloads/llvm.sh`
    - (einige dependencies - output sagt welche - müssen evtl. manuell nachinstalliert werden) installs LLVM + Clang + compiler-rt + polly + LLDB + LLD + libFuzzer + libc++ + libc++abi + openmp + libclc (see [here](https://apt.llvm.org/))
4. Install [https://github.com/xavierd/clang_complete](https://github.com/xavierd/clang_complete)
    - with minimum Configuration (Achtung: hier im `.vimrc` stattdessen `:let g:clang_library_path='/usr/lib/llvm-12/lib'` einsetzen, dh. die richtige llvm Version) (optional: make `.clang_complete` file for setting Compiler options)
5. `cd /usr/lib/llvm-12/lib`
    - look for `libclang.so.1` file
6. `ln -s libclang.so.1 libclang.so`
    - make a symlink (=symbolic link) called `libclang.so` to `libclang.so.1` in this directory

# help

## navigation

| command | description |
| :--- | :--- |
`gO` | show **table of contents**
`:h tag` | vim tags
`ctrl + ]` (während cursor auf einem tag)	| springe zu selected tag
`ctrl + t` | jump back after pressing `ctrl + ]`
`/pattern` | zB `/^i` springt zur nächsten Zeile, die mit i anfängt
`/|.\{-}|` | jump to next tag on the same help page

## vim commands

| command | description |
| :--- | :--- |
`:h` | help
`:h :u`	| Manual zu `:u` Befehl (im Manual kann man zB via Befehl `:v` zu dem `:v` springen)
`:h Q_<tab>` |
`:h quickref`	| und dann eg `_sc` um auf `Q_sc` tag zu springen

![Screenshot_table](https://i.ibb.co/jw1X1nH/Screen-Shot-2021-06-10-at-3-51-04-AM-2.png)

## keymaps

| command | description |
| :--- | :--- |
`:h key-notation` | vim key notation
`:h index` | list of all vim **default** keymaps
`:map`, `:nmap`, `:vmap`, `:imap` | list all **custom** keymaps
`:map <someKey>`, `:nmap <someKey>` | list all keymaps with `<someKey>`
`:filter pattern imap` |
`:h map-which-keys` | to see which keys are **not** mapped
`:h i_CTRL-R` | lookup meaning of CTRL-R in insert mode (deshalb das `i_` prefix; `v_` prefix für visual mode usw. s. Tabelle unten drunter)

## Plugins

| command | description |
| :--- | :--- |
`:h coc-nvim.txt` | 
`:h telescope.<tab>` |
`:h lsp<tab>` |
`:h comment.config` |

# Highlight Groups

- setting hi group in lua: [link](https://www.reddit.com/r/neovim/comments/sihuq7/psa_now_you_can_set_global_highlight_groups_ie/)
- change the color of folders in the output of the `ls` command: [link](https://askubuntu.com/a/466203)
  - meaning of the `LS_COLORS` variable: "The first number is the style (1=bold), followed by a semicolon, and then the actual number of the color."
- terminal `xterm` highlight groups [source](https://unix.stackexchange.com/a/172674): 
```bash
# get the terminal's background color rgb
printf '\033]11;?\007'
# get the terminal's foreground color rgb
printf '\033]10;?\007'
```

# regex/regexp

(Achtung: in Windows manchmal bisschen anders s. [https://superuser.com/a/518231](https://superuser.com/a/518231)

## Trick

Use [regexr.com](https://regexr.com/) to ...
- **read regex**: paste the regex expression you want to understand into the "Expression" field and read the "Tools" field. This field will show what each symbol in the expression means.
- **write regex**: write something into the "Expression" field and see if it works by looking into the "Text/Tests" field with the example text.

## In Vim

| command | description |
| :--- | :--- |
`^`	| vim: jump to the first non-blank character of the line; regexp: beginning of line ( praktisch für netrw: zB für jump to nächstem Ordner der mit "i" anfängt: /^i )
`\n` oder `\r` | linebreak (man kann damit auch linebreaks suchen und mit einem whitespace (ie einfach 1x Leertaste) ersetzen)
`:%s/pattern/replace/g`	| find pattern (regexp) and replace with replace. The basic construct of the command is `s#search#replace#`. `%`: tells the regex to work on **all lines** in the vim buffer. `g`: match multiple times in a single line. **Achtung**: Sonderzeichen (eg. Klammern, Punkt, ...) muss ein `\` vorangestellt werden!
`:s///` | find and replace **just on the current line**. 
`:%s/  /    /g`	| replace two spaces with four spaces
`:g/pattern/d` | remove all lines containing "pattern"
`^\R`  | blank line (exact empty line)
`^\h\*\R` | for empty lines with blanks, only

# Von Shell aus

| command | description |
| :--- | :--- |
vim .  | öffne netrw in current dir
vim -r *file.swp* | restore *file.swp*
vim -u vimrc-file file | use another `.vimrc` (to specify other `.vim` location `set runtimepath` in `.vimrc`, further details below in section "General facts/Change default `.vim` `.vimrc` location")
vim --clean file | start clean without loading `.vimrc`

# netrw

| command | description |
| :--- | :--- |
F1	|			help
i	|			change listing style
d	|			new directory
%	|			new file
s	|			cycle through sort order
r	|			reverse current sort order
:Lexplore	 (in vim)	|	öffnet netrw links und jeder gewählte file wird rechts geöffnet (gut zum Browsen)

# Compile, Run

| command | description |
| :--- | :--- |
:!g++ -g -Wall % |	compile current file (% steht für current file)
:!./a.out	|		(after :!g++ -g -Wall %) execute compiled file a.out

# In COMMAND mode

| command | description |
| :--- | :--- |
a | gehe in INSERT modus
:terminal | open a terminal in Vim
ctrl+w capital N | in terminal mode: enable scrolling (move around with cursor) in vim terminal mode (press i or a to resume using terminal as before)

## Vim Settings

| command | description |
| :--- | :--- |
:syntax on |
:set number |
:set nonumber |
:set hlsearch | 
:set nohlsearch |
:filetype indent plugin on |
:so $VIMRUNTIME/syntax/python.vim | aktiviere Python Syntax
:echo $VIMRUNTIME | show the content of the `VIMRUNTIME` variable (e.g. syntax files are in the `VIMRUNTIME/syntax/` directory)

### Syntax highlighting

| command | description |
| :--- | :--- |
|`:set filetype=cmake` | wenn Syntax nicht automatisch aktiviert wird, diesen Befehl ausführen
|`:scriptnames` | To find out which files Vim has actually loaded. Generally, only the last one listed is "active", and even it may not be if you've turned syntax highlighting off. 
|`:echo b:current_syntax` | To see the syntax currently in effect. 
|`:syntax list` | To see the syntax items currently in effect. This may help if syntax items from more than one syntax file are in effect.
|`:setlocal syntax?` | The syntax for the current buffer can be queried via this command. It usually (but not necessarily) corresponds to the buffer's filetype (`:setlocal filetype?`).

## Vim Sessions

| command | description |
| :--- | :--- |
`:mks ~/.vim-sessions/some-project-session.vim` | save vim session
`:so ~/.vim-sessions/some-project-session.vim` | restore vim session

## Tabs

| command | description |
| :--- | :--- |
:gt | next Tab
:gT | previous Tab
:tabe . | open netrw in a new Tab
:tabe *file* | open *file* in a new Tab
:4gt | switch to Tab number 4
:tab ter | open terminal in new Tab
ctrl + w N | scroll in a terminal tab

## Buffers

| command | description |
| :--- | :--- |
:ls | get the buffer number of all files edited in this session
:files | see `:ls`
:buffers | see `:ls`
:tabnew +Nbuf | re-open closed tab (where N is the buffer number of the closed tab which you can get via `:ls`), e.g. `:tabnew +18buf` in order to reopen buffer 18
:buffer very/very/long/path/to/a/veryVeryLongFileName.txt | switch to buffer `very/very/long/path/to/a/veryVeryLongFileName.txt`, where a **buffer** is a file in the `:ls` list
:b LongFileName\<hit tab to find a match\> | short form of `:buffer very/very/long/path/to/a/veryVeryLongFileName.txt` (**note**: `:b` command can also take a **substring** of the name of the file, thus `LongFileName` instead of `veryVeryLongFileName`!)
:b SubstringOfFilename\<hit tab to find a match\> |
:b 5 | switch to buffer 5 (see file-number map in the `:ls` list)
`:bd [N]` | Unload buffer `[N]` (default: current buffer) and delete it from the buffer list. If the buffer was changed, this fails, unless when `[!]` is specified, in which case changes are lost. The file remains unaffected.

## File Open, Close, Suspend, Name, Refresh

| command | description |
| :--- | :--- |
shift + z q	| wie `:q!`
ctrl + w q	| wie `:q` (ohne `!`), schließe aktiven split window
shift + z z	| wie `:x` ( dasselbe wie `:wq`, aber `:wq` überschreibt auch wenn keine modification [nur wichtig, falls modification times matter] )
ctrl + z | suspend, i.e. pause and switch to terminal (sends SIGTSTP to a process, like `kill -TSTP [processid]`, see notes on Bash under section "job control")
in terminal: fg | resume, i.e. go back to vim (a shell **builtin command**)
ctrl + g | show current file name
1 + ctrl + g | show current file name + path
:e | reload/refresh file
:e! | discard local changes and reload/refresh file
:e /path/to/other/file	| öffne anderen file in vim

## File Saving

| command | description |
| :--- | :--- |
`:w` |
`:sav(eas) new_file_name` | save as `new_file_name` and set `new_file_name` as current file (`:sav` and `:saveas` are the same)

## Navigation

| command | description |
| :--- | :--- |
ctrl + e	| 		scroll window one line down
ctrl + y	|		scroll window one line up 
ctrl + + | zoom in (anschließend ctrl + w, =)
ctrl - - | zoom out (anschließend ctrl + w, =)

### Viewports

| command | description |
| :--- | :--- |
vi -o /path/to/file1 /path/to/file2	|	öffne 2 files in split screen
ctrl + w, s	|	öffne neuen split window horizontal
ctrl + w, v	|	öffne neuen split window vertical (oder besser: `:Lexplore`)
ctrl + w J |    change two vertically split windows to horizonally split
ctrl + w H |    change two horizonally split windows to vertically split
ctrl + w, &lt;h j k l&gt;|	change active viewport
ctrl + r		|	rotate viewport (zum Anordnen der viewports)
ctrl + R |
ctrl + w, q	|	wie `:q` (ohne `!`), schließe aktiven split window
ctrl + w, =	|	resize viewports to be of equal size
ctrl + w, &lt;	|	decrease active viewport size (für 8 Einheiten Verkleinerung: ctrl + w, 8, &lt;)
ctrl + w, o     |       close all the other splits except the active one (same as `:only`)
`:qa`, `:qa!`   |       close all splits ("quit all")

### Jump

| command | description |
| :--- | :--- |
h j k l |			links hoch runter rechts
`/irgend_ein_wort` | suche `irgend_ein_wort` vorwärts (springt zum ersten solchen Wort, drücke n für nächstes Wort und N für previous occurrence)
`?irgend_ein_wort` | suche `irgend_ein_wort` rückwärts
0	|			spring zu Zeilenanfang
$	|			spring zu Zeilenende
H   |           spring zu top of page ("**H**igh")
L   |           spring zu bottom of page ("**L**ow")
b	|			spring zu Wortanfang
e	|			spring zu Wortende
`*` |				jump to next occurrence of the word under the cursor (then navigate back and forth with "n" and "shift + n")
f x |				spring zum nächsten "x" in der Zeile (repeat mit ";", reverse mit ",")
ctrl + d	|		spring 1/2 window nach unten
ctrl + u	|		spring 1/2 window nach oben
ctrl + f       |		spring 1 window nach unten (Merke: "f" für forward)
ctrl + b       |		spring 1 window nach oben (Merke: "b" für backward)
shift + g | 		Jump to end of file
"line number" + shift + g | Jump to line
g + g |			Jump to first line of file
ctrl + o | Jump to previous cursor position
ctrl + i | Jump to next cursor position
ctrl + ] | Jump to definition (if `ctags` is installed)
`%` | Jump to a matching opening or closing parenthesis, bracket or curly brace, when the cursor is positioned on one of them
`[{` or `]}` | Jump to outer curly brackets, see [vi.stackexcange](https://vi.stackexchange.com/a/16854)
`[(` or `])` | Jump to outer parenthesis, see [vi.stackexcange](https://vi.stackexchange.com/a/16854)
`[%` or `]%` | Jump to outer bracket, see [vi.stackexcange](https://vi.stackexchange.com/a/16854) (works only if the plugins "matchit" and "match-up" are installed) 
`{` or `}` | jump forwards or backwards between blank lines which should be enough to move between closed blocks (e.g. jump to the next function).
`ma` and `<backtick>a` | set mark and jump to the set mark (why useful?: A mark is persistent no matter how long you are in the file, as opposed to `<C-o>`.)

### URLs, links

| command | description |
| :--- | :--- |
gx | in normal mode: open the URL/link under the cursor in Firefox
gx | in visual mode: open the (manually) selected URL/link in Firefox

## Write

| command | description |
| :--- | :--- |
v	|			select
Shift + v	|		select line
x	|			cut
p		|		paste
:r !xsel | paste from clipboard
o	|			insert new line below
d w     |			delete (=cut) to the start of next word
d i w		|		delete (=cut) current word
d 3 i w		|		delete (=cut) next 3 `iw` objects ("space" also counts as an object!)
d 3 a w         |               delete (=cut) next 3 `aw` objects ("space" does **not** count as an object!)
5 d w		|	        delete (=cut) next 5 words
d d	|			delete (=cut) current line
d %	|			delete (=cut) betw matching brackets `{}`, `[]`, `()`
d i (   |                       delete (=cut) betw matching brackets `()`, see [stackoverflow](https://stackoverflow.com/a/405450)
d $	|			delete (=cut) to end of line
y l     |                       yank the symbol under the cursor
y w	|			yank to the start of next word
y i w		|		yank current word
y 3 i w		|		yank next 3 `iw` objects ("space" also counts as an object!)
y 3 a w		|		yank next 3 `aw` objects ("space" does **not** count as an object!)
y y	|			yank current line
y %		|		yank to the matching character (useful to copy text betw matching brackets {}, [], () )
`> >`		|		indent (in Insert mode: ctrl + t)
`< <`		|		unindent (in Insert mode: ctrl + d)
`10 < <`	|		unindent 10 lines
`< %`		|		unindent betw matching brackets {}, [], ()
u oder :u	|		undo last change
ctrl + r	|		redo
ctrl - k *digraph_id* | to type special characters that are not on the keyboard
:dig | list all digraphs (see :h digraph)

### Copy, Paste

#### Using Vim Registers

Use `"+y` to copy the selection to the system clipboard.

#### Using a Plugin

```bash
" copying to the system clipboard with text-objects and motions
Plugin 'christoomey/vim-system-copy'
```

| command | description |
| :--- | :--- |
cp | copy
cv | paste
cpiw | copy word
cP | copy line
cV | paste line
cpi' | copy inside single quotes to system clipboard
cvi' | paste inside single quotes from system clipboard

### Mehrere Zeilen auskommentieren

[how-to-comment-and-uncomment-multiple-line-vi-terminal-editor](https://discuss.devopscube.com/t/how-to-comment-and-uncomment-multiple-line-vi-terminal-editor/64)

1. ctrl + v	(Block markieren)
2. Shift + i (enter Insert mode (while in Block mode))
3. " (Kommentarsymbol am Anfang der ersten Zeile eingeben (while in Block mode))
4. Esc (drücken und 1 sec warten (bis das Kommentarsymbol vor allen Zeilen im Block auftaucht))

# In INSERT mode

| command | description |
| :--- | :--- |
Esc | gehe in command mode

## Avoid the escape key: 
[vim.fandom.com](https://vim.fandom.com/wiki/Avoid_the_escape_key#Avoiding_the_Esc_key)

| command | description |
| :--- | :--- |
ctrl + c | gehe in command mode **<span style="color:red">Achtung:</span>** abbreviations funktionieren hier nicht im Ggs. zu dem "Esc" command mode
alt + normal mode command | führt "normal mode command" im INSERT mode aus
ctrl + o + (cmd) | switch to command mode for one command (gut für zB ctrl + o + `$` oder andere Jump-commands) **<span style="color:red">Note:</span>** cmd nur bei Mac

## Schreiben

| command | description |
| :--- | :--- |
ctrl + x und danach ctrl + o | Omnicompletion (navigiere in Dropdown hoch/runter mit ctrl + p/ctrl + n)
ctrl + p | completion with previous matching pattern
ctrl + n | completion with next matching pattern
ctrl + t | indent
ctrl + d | unindent
ctrl + r dann `%` | insert current file name

# General facts

## Reasons for swap files (.swp)

Swap files store changes you've made to the buffer. If Vim or your computer crashes, they allow you to recover those changes.
Swap files also provide a way to avoid multiple instances of Vim from editing the same file. This can be useful on multi-user systems or just to know if you have another Vim already editing a file.
[s. stackexcange question](https://vi.stackexchange.com/questions/177/what-is-the-purpose-of-swap-files)

## Freeze/Freezing

Press CTRL + S to freeze vim and press CTRL + Q to unfreeze. 

## Change default .vim .vimrc location

- `:set runtimepath?`
- copy the displayed runtimepath
- in new `.vimrc` which is in an arbitrary location write `set runtimepath=` and paste the copied paths behind it
- add path of new `.vim` behind it
- start vim with `vim -u path/to/new/.vimrc some_other_file` 
    - or create alias e.g. `vvim` for `vim -u path/to/new/.vimrc`
    - or create `.bash_aliases` file with content `alias vvim="vim -u .vimrc"` and source it on startup using `source .bash_aliases` command

# Neovim

## Opinion

- [vim9script problem](https://www.youtube.com/watch?v=p0Q3oDY9A5s&t=172s)

## Install

Install the Debian package `nvim-linux64.deb` from `https://github.com/neovim/neovim/releases`:

```bash
sudo apt install ./nvim-linux64.deb
```

**Note**: Do **not** install from the `apt` package repository because there are only older `nvim` versions.

## Configuration

### Nvim

```bash
mkdir $HOME/.cfg/
git clone --bare https://github.com/pharath/dotfiles.git $HOME/.cfg/
alias config='/usr/bin/git --git-dir=$HOME/.cfg/ --work-tree=$HOME'
config checkout   # before running this command, remove all the existing dotfiles that would be overwritten otherwise
config config core.excludesFile '~/.gitignore'
```

**Note**: `~/.config/nvim/` is a modified version of `https://github.com/nvim-lua/kickstart.nvim`.

### Coc

Install `coc-prettier`: in neovim run:
```bash
:CocInstall coc-prettier
```

Under `:CocList` &rarr; `sources` deregister all `sources` (otherwise Coc's autocompletion will show a popup, but we only want to use the vim default completion popup and the `nvim-cmp` popup).

### ripgrep

Install `ripgrep` on Ubuntu: 
- e.g. for ripgrep 13.0.0 run:
```bash
$ curl -LO https://github.com/BurntSushi/ripgrep/releases/download/13.0.0/ripgrep_13.0.0_amd64.deb
$ sudo dpkg -i ripgrep_13.0.0_amd64.deb
```

### LSP

coc lsp vs native lsp: 
- reddit: [reddit](https://www.reddit.com/r/neovim/comments/rr6npy/question_coc_vs_lsp_whats_exactly_the_difference/)
- `chris@machine`: [youtube](https://www.youtube.com/watch?v=190HoB0pVro&t=22s)

For **coc-tsserver** (javascript, jsx, TypeScript, etc):
- Note: this will give 2 hover popups because nvim kickstart already has a TypeScript LSP

```bash
# :CocInstall coc-json   # required for :CocConfig
# :CocConfig
# # :CocConfig will open "coc-settings.json", look if "suggest.completionItemKindLabels" is set
# # (if not copy https://www.chiarulli.me/Neovim/26-lsp-symbols/)
# :CocInstall coc-tsserver
```

For **clangd**:
- configuration: see [github](https://github.com/neovim/nvim-lspconfig/blob/master/doc/server_configurations.md#clangd)
- open a `.cpp` file, run `:LspInstall` (without arguments) and select `clangd`
- each project needs a `compile_commands.json` (JSON compilation database) in the root of your source tree ([compile_commands.json, clangd doc](https://clangd.llvm.org/installation#compile_commandsjson))
  - if your project has a `Makefile` only, then the `compile_commands.json` can be generated with `bear` (`sudo apt install bear`)
    - to generate the `compile_commands.json` run `make clean; bear make` 
      - **Note**: the `--` in `make clean; bear -- make` is deprecated (see [issue](https://github.com/rizsotto/Bear/issues/202))
    - [bear github](https://github.com/rizsotto/Bear)
- project configuration file: `.clangd` (in the root of the project)

For **spectral** (json LSP), **tailwindcss**, **bashls**:
- open a json, css and bash file, run `:LspInstall` and select `spectral`, `tailwindcss` and `bashls` respectively

## Font

Install [nerd-fonts](https://github.com/ryanoasis/nerd-fonts#linux):
```bash
mkdir -p ~/.local/share/fonts
cd ~/.local/share/fonts
```

In this directory download [Hack Nerd Font](https://github.com/ryanoasis/nerd-fonts/releases/download/v2.3.3/Hack.zip), then run
```bash
unzip Hack.zip
rm Hack.zip LICENSE.md readme.md
```

In the Ubuntu terminal profile settings set `Hack Nerd Font Regular` as the terminal font.

## Plugins

`tpope/vim-surround`
- to delete/change/add parentheses/quotes/XML-tags
- `ysiw"`: surround the word under the cursor with `"` (without selecting this word)

`nvim-cmp`
- settings: see `cmp.setup` in `init.lua`
- changes: in `cmp.setup` add
```bash
['<C-n>'] = cmp.mapping(cmp.mapping.select_next_item(), {'i','c'}),
['<C-p>'] = cmp.mapping(cmp.mapping.select_prev_item(), {'i','c'}),
```

`nvim-tree/nvim-tree.lua`
- [doc](https://docs.rockylinux.org/books/nvchad/nvchad_ui/nvimtree/)
- press `g ?` to show all shortcuts
- Git Integration
  - Icon indicates when a file is:
    - ✗  unstaged or folder is dirty
    - ✓  staged
    - ★  new file
    - ✓ ✗ partially staged
    - ✓ ★ new file staged
    - ✓ ★ ✗ new file staged and has unstaged modifications
    - ═  merging
    - ➜  renamed
- does NOT show folders and files that are in `.gitignore`
  - press "I" to show them

`toggleterm`
- use `3<c-\>` to create and toggle terminal "3"

`lazygit`
- install:
  - `wget https://github.com/jesseduffield/lazygit/releases/download/v0.37.0/lazygit_0.37.0_Linux_x86_64.tar.gz`
  - untar and `mv -iv lazygit /usr/local/bin/`
- set a custom pager: [doc](https://github.com/jesseduffield/lazygit/blob/master/docs/Custom_Pagers.md)
  - install `delta` (Note: use an older release, if `sudo apt install ./git-delta_0.14.0_amd64.deb` shows [errors](https://github.com/dandavison/delta/issues/1250); my currently used version: [link](https://github.com/dandavison/delta/releases/download/0.14.0/git-delta_0.14.0_amd64.deb))
- how to handle the `dotfiles` repo: [issue](https://github.com/jesseduffield/lazygit/discussions/1201)
  - `lazygit --git-dir=$HOME/.cfg/ --work-tree=$HOME`
  - note: `<leader>gd` triggers this command in my config

## Syntax Highlighting

### nvim-treesitter

In order to install a language just add the language to the `ensure_installed` list in the `init.lua` file and close and re-open the `init.lua` file. E.g. to install `javascript` syntax highlighting:
```lua
-- [[ Configure Treesitter ]]
-- See `:help nvim-treesitter`
require('nvim-treesitter.configs').setup {
  -- Add languages to be installed here that you want installed for treesitter
  ensure_installed = { 'c', 'cpp', 'go', 'lua', 'python', 'rust', 'tsx', 'typescript', 'help', 'vim', 'javascript' },

  -- Autoinstall languages that are not installed. Defaults to false (but you can change for yourself!)
  auto_install = false,

  highlight = { enable = true },
  indent = { enable = true, disable = { 'python' } },
  incremental_selection = {
    enable = true,
    keymaps = {
      init_selection = '<c-space>',
      node_incremental = '<c-space>',
      scope_incremental = '<c-s>',
      node_decremental = '<M-space>',
    },
  },
  textobjects = {
    select = {
      enable = true,
      lookahead = true, -- Automatically jump forward to textobj, similar to targets.vim
      keymaps = {
        -- You can use the capture groups defined in textobjects.scm
        ['aa'] = '@parameter.outer',
        ['ia'] = '@parameter.inner',
        ['af'] = '@function.outer',
        ['if'] = '@function.inner',
        ['ac'] = '@class.outer',
        ['ic'] = '@class.inner',
      },
    },
    move = {
      enable = true,
      set_jumps = true, -- whether to set jumps in the jumplist
      goto_next_start = {
        [']m'] = '@function.outer',
        [']]'] = '@class.outer',
      },
      goto_next_end = {
        [']M'] = '@function.outer',
        [']['] = '@class.outer',
      },
      goto_previous_start = {
        ['[m'] = '@function.outer',
        ['[['] = '@class.outer',
      },
      goto_previous_end = {
        ['[M'] = '@function.outer',
        ['[]'] = '@class.outer',
      },
    },
    swap = {
      enable = true,
      swap_next = {
        ['<leader>a'] = '@parameter.inner',
      },
      swap_previous = {
        ['<leader>A'] = '@parameter.inner',
      },
    },
  },
}
```

## Formatting

### C/C++

```bash
sudo apt install clang-format
```

Then use [vim-clang-format](https://github.com/rhysd/vim-clang-format).

### coc.nvim

Nodejs extension host for vim and neovim, load extensions like VSCode and host language servers.

[doc: coc.nvim](https://github.com/neoclide/coc.nvim)

### coc-prettier

Prerequisites: Install the `coc.nvim` plugin in neovim.

Install: Run `:CocInstall coc-prettier`.

[doc: coc-prettier](https://github.com/neoclide/coc-prettier)

[doc: prettier](https://prettier.io/docs/en/install.html)

Install locally:
```bash
npm install prettier -D --save-exact
echo {} > .prettierrc.json
cp -iv .gitignore .prettierignore
```

Format shortcuts: see `nvim/init.lua`

## Markdown

### LaTeX

To avoid that LaTeX equations surrounded by `$` symbols are displayed in italic font follow these steps:

In `/usr/share/nvim/runtime/syntax/markdown.vim` add:
```vim
97 exe 'syn region markdownLatex matchgroup=markdownLatexDelimiter start="\S\@<=\$\|\$\S\@=" end="\S\@<=\$\|\$\S\@=" skip="\\\$" contains=markdownLineStart,@Spell' . s:concealends
```

and
```vim
161 hi def link markdownLatexDelimiter        markdownLatex
```

## Swap Files

- location: in `~/.local/state/nvim/swap//`

# Vimium

`?` (help, click on "show advanced commands")

## Visual

see [doc](https://github.com/philc/vimium/wiki/Visual-Mode)

tip: use find `/` to locate and `v` to select
