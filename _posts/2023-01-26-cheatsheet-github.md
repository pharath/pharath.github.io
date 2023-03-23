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

```bash
gh auth login
gh repo create
```
