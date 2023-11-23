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

## tmux Shortcuts

| command | description |
| :--- | :--- |
`c-b ?` | list all keymaps (eg. all `c-b` keymaps have the word `prefix` in their 3rd column, scroll with pageUp/pageDown keys)
`c-b [` | copy-mode, see also [copy-mode-vi](https://dev.to/iggredible/the-easy-way-to-copy-text-in-tmux-319g)
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

## tmux Commands

### in command-prompt

| command | description |
| :--- | :--- |
`new` | Start a new session
`new -s mysession` | Start a new session with the name `mysession`
`kill-session -t 3` | kill the session with index 3 (ie. this number is in the `c-b w` view the number **without** parentheses next to `# windows`)
`kill-session -t name` | kill the session with name `name`

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
