---
title: "Tmux Cheatsheet"
read_time: false
excerpt: "Some essential tmux commands"
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - tmux
  - cheatsheet
---

# tmux

## Restart tmux Server

```bash
tmux kill-server && tmux
```

## Setup

```bash
# create a .tmux.conf
# https://askubuntu.com/a/280587
tmux show -g | cat > ~/.tmux.conf
```

Then, install [TPM](https://github.com/tmux-plugins/tpm):

```bash
# destination ~/.tmux/plugins/tpm will be created automatically
# (ie. neither the folder .tmux/ nor plugins/ needs to be created manually)
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
```

Follow the other instructions in the [TPM README](https://github.com/tmux-plugins/tpm).

Install these plugins using the TPM method (after installing TPM):
- [tmux-resurrect](https://github.com/tmux-plugins/tmux-resurrect)
  - press `c-b c-s` to store your first session (*must* do this after installing tmux-resurrect, otherwise you will get this [issue](https://github.com/tmux-plugins/tmux-continuum/issues/33))
- [tmux-continuum](https://github.com/tmux-plugins/tmux-continuum)
  - Put `set -g @continuum-restore 'on'` in `.tmux.conf` to enable ["Automatic Restore"](https://github.com/tmux-plugins/tmux-continuum#automatic-restore).

## Colors

Vim/Neovim will show wrong colors unless <span style="color:red">true color support</span> is configured properly.

**Check**:

The following script

```bash
awk 'BEGIN{
    s="/\\/\\/\\/\\/\\"; s=s s s s s s s s;
    for (colnum = 0; colnum<77; colnum++) {
        r = 255-(colnum*255/76);
        g = (colnum*510/76);
        b = (colnum*255/76);
        if (g>255) g = 510-g;
        printf "\033[48;2;%d;%d;%dm", r,g,b;
        printf "\033[38;2;%d;%d;%dm", 255-r,255-g,255-b;
        printf "%s\033[0m", substr(s,colnum+1,1);
    }
    printf "\n";
}'
```

shows a <span style="color:red">continuous</span> color band when run outside of Tmux (&rarr; true color is supported),

![truecolor-working.png](https://i.ibb.co/QCjdgfG/truecolor-working.png)

as opposed to when run inside Tmux (&rarr; true color is <span style="color:red">not</span> supported).

![truecolor-not-working.png](https://i.ibb.co/KbJPY2F/truecolor-not-working.png)

To properly set truecolor capability add the following snippet to `.tmux.conf`:

```bash
set -g default-terminal 'tmux-256color'
set -as terminal-overrides ",xterm*:Tc"
```

Then, restart tmux completely by exiting <span style="color:red">**all**</span> sessions and then opening a new session. Here, it is very important to really exit ALL existing tmux sessions!

source: [reddit.com](https://www.reddit.com/r/neovim/comments/11usepy/how_to_properly_set_tmux_truecolor_capability/)

"Adding `,xterm*:Tc` is pointless unless you are using additional terminals outside of Kitty that also support True Color but don't have the Tc flag set inside their terminfo (most likely the case as it's not an official flag) <span style="color:red">or if you aren't defining default-terminal as xterm-kitty and instead using a generic xterm like `xterm-*` or `xterm-256color`</span>, Tmux's generic `tmux-256color`, or any other terminfo that was not defined by kitty."

## tmux Shortcuts

| command | description |
| :--- | :--- |
`c-b ?` | list all keymaps (eg. all `c-b` keymaps have the word `prefix` in their 3rd column, scroll with pageUp/pageDown keys)
`c-b [` | copy-mode, or (if enabled) [copy-mode-vi](https://dev.to/iggredible/the-easy-way-to-copy-text-in-tmux-319g)
`c-b d` | detach
`c-b :` | command-prompt

| command | description |
| :--- | :--- |
`c-b $` | rename current session
`c-b ,` | rename current window
`c-b s` | list sessions
`c-b w` | list all sessions, windows and panes
`c-b (` | previous session
`c-b )` | next session
`c-b c` | create window
`c-b &` | close window
`c-b %` | split pane (left/right)
`c-b "` | split pane (top/bottom)
`c-b !` | convert pane into a window
`c-b arrowKey` | switch to pane to the direction
`c-b q 0-9` | select pane by number
`c-b x` | close pane
`c-b z` | Toggle pane zoom (pressing this twice may help if a pane is "messed up")

### Copy and Paste

| command | description |
| :--- | :--- |
`c-b [` | copy-mode, or (if enabled) [copy-mode-vi](https://dev.to/iggredible/the-easy-way-to-copy-text-in-tmux-319g)
`space` | start selection
`arrowkeys` | select (after pressing `space`)
`enter` | copy selection
`c-b ]` | paste selection

## tmux Commands

### in command-prompt

| command | description |
| :--- | :--- |
`new` | Start a new session
`new -s mysession` | Start a new session with the name `mysession`
`kill-session -t 3` | kill the session with index 3 (ie. this number is in the `c-b w` view the number **without** parentheses next to `# windows`)
`kill-session -t name` | kill the session with name `name`
`:set -g mouse on` | enable mouse support in tmux

### send-keys

`C-m` ist dasselbe wie `Enter` (in `send-keys` Befehlen).

## Plugins

### tmux-resurrect

- [Restoring previously saved environment](https://github.com/tmux-plugins/tmux-resurrect/blob/master/docs/restoring_previously_saved_environment.md)
  - all saved environments have timestamps

| command | description |
| :--- | :--- |
`c-b c-s` | save the tmux session (must do this after installing tmux-resurrect, see [issue](https://github.com/tmux-plugins/tmux-continuum/issues/33))
`c-b c-r` | restore an environment that was saved via `c-b c-s`
