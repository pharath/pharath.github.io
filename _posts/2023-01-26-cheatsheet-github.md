---
title: "Github Cheatsheet"
read_time: false
excerpt: "Some essential Github tricks"
header:
  teaser: /assets/images/github_logo.png
  overlay_image: /assets/images/github_logo.png
  overlay_filter: 0.5
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - github
  - cheatsheet
---

# Shortcuts

see [docs.github.com](https://docs.github.com/en/get-started/using-github/keyboard-shortcuts)

Blacklist `github.com` in the vimium extension settings.

press the `?` key on the github.com site to show all shortcuts

# SSH Keys

- generate the ssh key on your local machine: `ssh-keygen -o -t rsa -C "ssh@github.com"`, where `-C "comment"` is just a comment/description
- `cat ~/.ssh/id_rsa.pub` and copy and paste the full output into the **key** field on **github.com** &rarr; **Account Settings** &rarr; **SSH Keys**

# Github Actions

## Compress Images

- [Calibre Image Actions](https://github.com/marketplace/actions/image-actions)
  - see demo on `feature/compressImage` branch

# Storage Limits

see [stackoverflow](https://stackoverflow.com/questions/38768454/repository-size-limits-for-github-com)

# Github CLI

see article ["Git"]({% post_url 2021-08-19-cheatsheet-git %}#create-a-new-repository-on-the-command-line)

## Setup

```bash
gh auth login
gh repo create
```

For quickly adding a `.gitignore` AFTER creating a repo, first run:

```bash
gh alias set --shell .gitignore 'gh api -X GET /gitignore/templates/"$1" --jq ".source"'
```

Then, `gh .gitignore Node >> .gitignore` generates a `.gitignore` for Node projects. You can also use `C, C++` etc. (TODO: find the list of possible languages in this API call).

## gh repo

### gh repo view

| command                                                 | description                                 |
| :------------------------------------------------------ | :------------------------------------------ |
| `gh repo view -h`                                       |
| `gh repo view dotfiles`                                 | no "pharath/" prefix needed for "dotfiles"! |
| `gh repo view -b some_branch`                           |
| `gh repo view --json name -q ".name"`                   | name of the repository                      |
| `gh repo view --json nameWithOwner -q ".nameWithOwner"` | owner/repository                            |
| `gh repo view --json url -q ".url"`                     | the complete url                            |

### gh repo edit

| command                                            | description                       |
| :------------------------------------------------- | :-------------------------------- |
| `gh repo edit <repository> --description <string>` | edit the description in your repo |
| `gh repo edit <repository> --<tab><tab>`           | show available edit options       |
