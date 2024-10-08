---
title: "Git Cheatsheet"
read_time: false
excerpt: "Some essential git commands"
header:
  teaser: /assets/images/git_logo.png
  overlay_image: /assets/images/git_logo.png
  overlay_filter: 0.5
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - git
  - cheatsheet
---

# Terminology

- **remote** = remote-repository (e.g. in `git push *remote* *branch*`)
- **PAT** = Personal Access Token (see [youtube: PATs and their scopes](https://www.youtube.com/watch?v=SzrETQdGzBM&t=49s))
- **index** = Staging Directory (`.index/`) (see [explanation](https://www.javatpoint.com/git-index))
  - There are three places in Git where file changes can reside, and these are
    - working directory,
    - staging area,
    - the repository.

<p align="center">
  <img src="https://i.ibb.co/kmTgYqH/Screenshot2022-PM.png" alt="Screenshot2022-PM" border="0">
</p>

- **Plumbing and Porcelain** (see [git-scm.com](https://git-scm.com/book/en/v2/Git-Internals-Plumbing-and-Porcelain)):
  - This book covers primarily how to use Git with 30 or so subcommands such as checkout, branch, remote, and so on. But because Git was initially a toolkit for a version control system rather than a full user-friendly VCS, it has a number of <span style="color:green">**subcommands that do low-level work and were designed to be chained together UNIX-style or called from scripts**</span>. These commands are generally referred to as Git’s <span style="color:red">**“plumbing”**</span> commands, while the more <span style="color:green">**user-friendly commands**</span> are called <span style="color:red">**“porcelain”**</span> commands.
  - As you will have noticed by now, this book’s first nine chapters deal almost exclusively with porcelain commands. But in this chapter, you’ll be dealing mostly with the lower-level plumbing commands, because they give you access to the inner workings of Git, and help demonstrate how and why Git does what it does. Many of these commands aren’t meant to be used manually on the command line, but rather to be used as building blocks for new tools and custom scripts.

# Basics

## git status

| command                                            | description                                                                                                                                                                   |
| :------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `git status -uno`                                  | Show no untracked files (useful, if you have many untracked files), [stackoverflow](https://stackoverflow.com/a/46335093)                                                     |
| `git config --global status.showUntrackedFiles no` | makes `git status -uno` the default, ie. after running this command `git status` will show the same as `git status -uno` [stackoverflow](https://stackoverflow.com/a/2774461) |

## git rm, git add

| command                                                                                   | description                                                                                      |
| :---------------------------------------------------------------------------------------- | :----------------------------------------------------------------------------------------------- |
| `git rm file1.txt`                                                                        | remove the file from the Git repository **and the filesystem**                                   |
| `git rm --cached file1.txt`                                                               | remove the file only from the Git repository and not remove it from the filesystem               |
| `git add`                                                                                 | synonoym for `git stage` (since 2008), see [stackoverflow](https://stackoverflow.com/a/34175877) |
| `git add -u :/`                                                                           | `git rm` all files after accidentally `rm` those files                                           |
| `git add --all -- ':!./Gemfile' ':!./_config.yml' ':!./_layouts/*.bak' ':!./_sass/*.bak'` | **exclude files** from `git add`                                                                 |
| `git stage`                                                                               | synonoym for `git add` (since 2008), see [stackoverflow](https://stackoverflow.com/a/34175877)   |

## git commit

### Reword the last commit and force push

| command                                                | description                                                                                                                                                                                                                                                                   |
| :----------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `git commit --amend -m "message"`                      | reword/rephrase the last commit message, "This brings up the editor with the last commit message and lets you edit the message. (You can use `-m` if you want to wipe out the old message and use a new one.)", [stackoverflow](https://stackoverflow.com/a/8981216/12282296) |
| `git commit --allow-empty --amend --only -m "message"` | reword/rephrase the last commit message (the lazygit way)                                                                                                                                                                                                                     |
| `git push --force-with-lease remote <branch>`          | to push the reworded commit message to the remote                                                                                                                                                                                                                             |

### Reword a specific commit

[stackoverflow](https://stackoverflow.com/a/1186549/12282296)

| command                              | description                                                                    |
| :----------------------------------- | :----------------------------------------------------------------------------- |
| `git rebase --interactive bbc643cd~` | After this command `bbc643cd` is your last commit and you can easily amend it. |
| `git commit --all --amend --no-edit` | Make your changes and then `commit` them with this command                     |
| `git rebase --continue`              | to return back to the previous `HEAD` commit                                   |

<span style="color:red">**WARNING**</span>: Note that this will change the SHA-1 of that commit <span style="color:red">**as well as all children**</span> -- in other words, this rewrites the history from that point forward. You can break repos doing this if you push using the command `git push --force`.

## git push

| command                                     | description                                                                                                                                                                                                                                                                                                                                                                                                                         |
| :------------------------------------------ | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `git push remote my_branch:remote_branch`   |
| `git push --force remote branch`            | **do not** use this, better use `--force-with-lease`, see below                                                                                                                                                                                                                                                                                                                                                                     |
| `git push remote +branch`                   | **do not** use this, better use `--force-with-lease`, see below                                                                                                                                                                                                                                                                                                                                                                     |
| `git push --force-with-lease remote branch` | **best practice**: `--force-with-lease` is a safer option \[than `git push remote +branch` and `git push --force remote branch`\] that will not overwrite any work on the remote branch if more commits were added to the remote branch (by another team-member or coworker or what have you). It ensures you do not overwrite someone elses work by force pushing., [stackoverflow](https://stackoverflow.com/a/52823955/12282296) |
| `git push --set-upstream origin new_branch` | push to a locally newly created branch (see `git checkout -b new_branch`) that does not yet exist on remote (i.e. on github.com). `--set-upstream`: will make the local `new_branch` track the remote `remotes/origin/new_branch` (w/o this flag git does not know where to push the `new_branch`).                                                                                                                                 |

## git clone

| command                                                | description |
| :----------------------------------------------------- | :---------- |
| `git clone --recurse-submodules repo <Ziel directory>` |

## git branch

| command                                    | description                                                                                                                                             |
| :----------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `git branch -a`                            | show/list all branches (local and remote)                                                                                                               |
| `git branch -r`                            | show/list all branches (only remote)                                                                                                                    |
| `git branch -d *local_branch*`             | delete local branch `local_branch` in local repo                                                                                                        |
| `git branch -r -d remote/branch`           | delete remote branch in local repo (eg. after the branch has been deleted on the remote), [stackoverflow](https://stackoverflow.com/a/3184742/12282296) |
| `git push origin --delete *remote/branch*` | delete remote branch `remote/branch`                                                                                                                    |
| `git branch -m new_name`                   | rename branch (`--move`)                                                                                                                                |
| `git branch -M new_name`                   | force rename branch (`--move --force`), ie rename even if the branch name exists                                                                        |
| `git branch -vv`                           | show which **local branches** track which **remote branches**, useful e.g. when you are creating and setting up new branches                            |

### Track another branch, Untrack a branch

- for newly created GitHub repos:
  - When you just created an empty repo on GitHub the repo will not have any branches yet. You cannot track a remote branch with `git branch -u remote/branch` that does not exist on the remote.

| command                                                | description                                                                                   |
| :----------------------------------------------------- | :-------------------------------------------------------------------------------------------- |
| `git branch -u upstream/foo`                           | As of Git 1.8.0, make the local branch `foo` track remote branch `foo` from remote `upstream` |
| `git branch -u upstream/foo foo`                       | if local branch `foo` is not the current branch                                               |
| `git branch --set-upstream-to=upstream/foo`            | if you like to type longer commands, these are equivalent to the above two                    |
| `git branch --set-upstream-to=upstream/foo foo`        |
| `git branch --set-upstream foo upstream/foo`           | As of Git 1.7.0 (before 1.8.0)                                                                |
| `git branch --set-upstream my_branch origin/my_branch` | (similar to `git push --set-upstream origin <new_branch>`)                                    |
| `git branch --unset-upstream`                          | untrack a branch (eg. to update a branch that is still tracking a deleted branch)             |

related:

[stackoverflow](https://stackoverflow.com/a/2286030/12282296)

## git checkout, git switch

| command                                                      | description                                                                                                                                                                                                                                                                                               |
| :----------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `git checkout <existing_branch>`                             | switch to an existing branch                                                                                                                                                                                                                                                                              |
| `git switch *branch*`                                        | switch to an existing branch (since Git v2.23)                                                                                                                                                                                                                                                            |
| `git checkout -b <new_branch>`                               | switch to a non-existing branch; subsequently, `git push --set-upstream origin <new_branch>` to create `new_branch` in the remote (on github.com).                                                                                                                                                        |
| `git checkout -b <new_branch> <old_branch>`                  | create `<new_branch>` off `<old_branch>`                                                                                                                                                                                                                                                                  |
| `git switch -c *branch*`                                     | switch to a non-existing branch (since Git v2.23)                                                                                                                                                                                                                                                         |
| `git push --set-upstream origin <new_branch>`                | push to a locally newly created branch (see `git checkout -b <new_branch>`) that does not yet exist on remote (i.e. on github.com). `--set-upstream`: will make the local `<new_branch>` track the remote `remotes/origin/<new_branch>` (w/o this flag git does not know where to push the `new_branch`). |
| `git checkout c5f567 -- file1/to/restore file2/to/restore`   | get files from a specific commit `c5f567` (shortest SHA1: usually the first 7 hex digits are enough, [stackoverflow](https://stackoverflow.com/a/21015031))                                                                                                                                               |
| `git checkout c5f567~1 -- file1/to/restore file2/to/restore` | get files from the commit before a specific commit `c5f567`                                                                                                                                                                                                                                               |

## git show-branch

| command              | description                                                               |
| :------------------- | :------------------------------------------------------------------------ |
| `git show-branch -a` | show/list all branches **and the last commit message** (local and remote) |
| `git show-branch -r` | show/list all branches **and the last commit message** (only remote)      |

## Tags

| command                                                                          | description                                                                                                                                                                   |
| :------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `git tag <tag name>`                                                             | tag the current commit with `<tag name>`                                                                                                                                      |
| `git push origin v1.2`                                                           | push tag `v1.2`                                                                                                                                                               |
| `git push --tags`                                                                | push all tags                                                                                                                                                                 |
| `git tag -l`                                                                     | list all tags                                                                                                                                                                 |
| `git -c 'versionsort.suffix=-' ls-remote --tags --sort='v:refname' <repository>` | list all tags of `<repository>`                                                                                                                                               |
| `git checkout <tag name>`                                                        | checkout a specific tag                                                                                                                                                       |
| `git checkout tags/<tag name>`                                                   | checkout a specific tag                                                                                                                                                       |
| `git clone --depth 1 --branch <tag_name> <repo_url>`                             | clone a specific tag; `--depth 1` is optional but if you only need the state at that one revision, you probably want to skip downloading all the history up to that revision. |

### Undo Pushing a Tag

1. `git push origin :refs/tags/<tagname>` (Delete the tag on any remote before you push)
2. `git tag -f <tagname>` (Replace the tag to reference the most recent commit)
3. `git push origin --tags` (Push the tag to the remote origin)

## git remote

| command                                                                       | description                                 |
| :---------------------------------------------------------------------------- | :------------------------------------------ |
| `git remote get-url origin`                                                   | get only the URL of the remote `origin`     |
| `git remote set-url origin https://git.rwth-aachen.de/mygroup/myreponame.git` | set the URL of the remote `origin`          |
| `git remote show [remote-name] command`                                       | get more details about a particular remote  |
| `git remote show origin`                                                      | get more details about the current remote   |
| `git remote update origin --prune`                                            | To update the local list of remote branches |

## git reflog

| command      | description                               |
| :----------- | :---------------------------------------- |
| `git reflog` | view history of `git checkout` operations |

## What is the difference between git log and git show?

- [reddit: What is the difference between git log and git show?](https://www.reddit.com/r/git/comments/30qcpg/what_is_the_difference_between_git_log_and_git/)
  - `git show` is primarily for showing a <span style="color:red">**single**</span> commit. It defaults to a verbose display, including the entire diff. It can also show other, non-commit objects.
  - `git log` is primarily for showing a <span style="color:red">**range**</span> of commits. It defaults to only showing the commit message, and can be reduced to one line.
  - Both take similar options for the format. You can, with an appropriate command line, persuade `git log` to display a single commit with full diff just like `git show`.

## git log

- [What is the difference between git log and git show?](#what-is-the-difference-between-git-log-and-git-show)

| command                                                  | description                                                                                                                                                                                                                                                                                                                                                         |
| :------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `git log`                                                | view history of commits ([more commands](#git-log))                                                                                                                                                                                                                                                                                                                 |
| `git log --oneline`                                      | show SHA1 + commit messages                                                                                                                                                                                                                                                                                                                                         |
| `git log --oneline filename`                             | get the commits that contain a specific file                                                                                                                                                                                                                                                                                                                        |
| `git log -- filename`                                    | commit history of a file                                                                                                                                                                                                                                                                                                                                            |
| `git log --all`                                          | all commits of all branches, tags and other refs (why `--all` isn't the default: you normally won't want that. For instance, if you're on branch `master`, and you run `git log`, you typically aren't interested in the history of any feature branches, you typically want to see the history of `master`, [stackoverflow](https://stackoverflow.com/a/29756754)) |
| `git log --all --full-history -- filename`               | commit history of a (possibly deleted) file (in all branches which touched that file)                                                                                                                                                                                                                                                                               |
| `git log --all --full-history -- "**/thefile.*"`         | commit history of a (possibly deleted) file that matches `"**/thefile.*"` (useful if you do not know the exact path)                                                                                                                                                                                                                                                |
| `git log -p -- filename`                                 | Like `git log`, but shows the file content that changed, as well. Generates the patches for each log entry.                                                                                                                                                                                                                                                         |
| `git log -p -1 35e32b6a00dec02ae7d7c45c6b7106779a124685` | find a commit given the commit hash; `-1` flag: to show the specified commit only, otherwise all commits prior to the specified commit will be shown as well                                                                                                                                                                                                        |
| `git log -L110,110:/lib/client.js`                       | will return every commit which touched the line 110, [stackoverflow: Show all of the various changes to a single line in a specified file over the entire git history](https://stackoverflow.com/a/27108677/12282296)                                                                                                                                               |
| `git log -L150,+11:/lib/client.js`                       | log lines 150 to 150+11                                                                                                                                                                                                                                                                                                                                             |

## git shortlog

| command        | description                                                                               |
| :------------- | :---------------------------------------------------------------------------------------- |
| `git shortlog` | show the commit messages grouped by author and title (for creating release announcements) |

## git show

- [What is the difference between git log and git show?](#what-is-the-difference-between-git-log-and-git-show)

| command                                             | description                         |
| :-------------------------------------------------- | :---------------------------------- |
| `git show HEAD`                                     | just the diff for a specific commit |
| `git show 35e32b6a00dec02ae7d7c45c6b7106779a124685` | find a commit given the commit hash |

## gitk

| command           | description                    |
| :---------------- | :----------------------------- |
| `gitk [filename]` | To browse the changes visually |

## git blame

"`git blame` <span style="color:red">does not</span> show the per-line modifications history in the chronological sense. It only shows <span style="color:red">who was the last person</span> to have changed a line in a document up to the last commit in `HEAD`.", [What does 'git blame' do?](https://stackoverflow.com/a/31204980/12282296)

| command                        | description                                                                                     |
| :----------------------------- | :---------------------------------------------------------------------------------------------- |
| `git blame -L 150,+11 -- file` | look at the lines 150 to 150+11, [stackoverflow](https://stackoverflow.com/a/19757493/12282296) |

## git ls-tree

[superuser](https://superuser.com/a/429694)

If you want to **list all files for a specific branch**, e.g. `master`:

```bash
git ls-tree -r master --name-only
```

If you want to get a **list of all files that ever existed**:

```bash
git log --pretty=format: --name-only --diff-filter=A  | sort -u
```

## git init

How to use `main` instead of `master` as the initial branch?

```bash
git init
git checkout -b main
```

Note: After this the branch `main`/`master` does not actually exist--the branches don't get created until they have at least one commit.

Since Git 2.28.0 you can set `git config --global init.defaultBranch foo` if you want all new repos to have `foo` as the default branch

## git pull vs fetch vs update

[source](https://stackoverflow.com/a/17712553):

- `git remote update` will update all of your branches set to track remote ones, but not merge any changes in.
- `git fetch` will **update** only the branch you're on, but not merge any changes in.
- `git pull` will **update** and **merge** any remote changes of the current branch you're on. This would be the one you use to update a local branch.
  - like `git fetch` followed by `git merge`
- `git pull --rebase`
  - like `git fetch` followed by `git rebase`

## .gitignore

[source](https://git-scm.com/docs/gitignore)

Git normally checks `gitignore` patterns from multiple sources, with the following order of precedence, from highest to lowest

- Patterns read from the command line
- Patterns read from a `.gitignore` file
- Patterns read from `$GIT_DIR/info/exclude`
- Patterns read from the file specified by the configuration variable `core.excludesFile`

## git update-index

read: [how-to-stop-tracking-and-ignore-changes-to-a-file-in-git](https://stackoverflow.com/questions/936249/how-to-stop-tracking-and-ignore-changes-to-a-file-in-git)

- do use: `git update-index --skip-worktree path`: to stop tracking a certain file and have your own version of this file
- do not use: `git update-index --assume-unchanged path`: to stop tracking a certain file, <span style="color:red">**but**</span> file(s) are overwritten if there are upstream changes to the file/folder (when you pull)

# Reset/undo changes

## Undo git add

- <span style="color:red">**Best practice**</span>: Only use `git add` for adding new (ie. untracked) files. Do not use it for anything else!
  - In particular, do not add new contents (parts of files) via `git add`, since you cannot easily undo `git add`-ing parts of files _when you had other staged changes in the same files_ before the last `git add`., see "caveats" in [stackoverflow](https://stackoverflow.com/a/6049090/12282296)
  - Instead, update new contents via the commit, `git commit -a` command.

| command                   | description                                                                                                                                                                                                        |
| :------------------------ | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| git reset                 | undo `git add`                                                                                                                                                                                                     |
| git restore --staged file | discard a `file` **in the index** (= "added" file) without overwriting the same file **in the working tree**. `man git restore`: "Specifying `--staged` will only restore the index" and **not** the working tree! |

## Undo local Branch delete

see [stackoverflow](https://stackoverflow.com/a/4025983/12282296)

```bash
user@MY-PC /C/MyRepo (master)
$ git branch -D master2
Deleted branch master2 (was 130d7ba).    <-- This is the SHA1 we need to restore it!

user@MY-PC /C/MyRepo (master)
$ git branch master2 130d7ba
```

Use `git reflog` to find the `SHA1` of the last commit of the branch.

## Undo Commits

### git reset

| command                  | description                                                                                                                                                                                                                           |
| :----------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| git reflog               | get SHA-1 list of previous states                                                                                                                                                                                                     |
| git reset --soft HEAD~   | undo last commit **locally** (`--soft`: safe way)                                                                                                                                                                                     |
| git push origin +HEAD    | reset the **remote's** last commit to the **local's** last commit (**Note**: `HEAD` always points to the last commit. The `+` indicates a force-push, like the `-f` flag, see [stackoverflow](https://stackoverflow.com/a/25937833).) |
| git reset --soft HEAD~1  | `HEAD~` and `HEAD~1` are the same; `HEAD` and `HEAD~0` are the same                                                                                                                                                                   |
| git reset --soft _SHA-1_ | reset to a previous state **locally** (`--soft`: safe way)                                                                                                                                                                            |
| git reset --hard _SHA-1_ | reset to a previous state **locally** (**Warning**: `--hard`: All changes will be lost.)                                                                                                                                              |

### git revert

| command                    | description                                                                                                                                                                                                                                                                      |
| :------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `git revert <commit-hash>` | undo commit `<commit-hash>`, "`git revert` simply creates a new commit that is the opposite of an existing commit. It leaves the files in the same state as if the commit that has been reverted never existed.", [stackoverflow](https://stackoverflow.com/a/19032678/12282296) |

## Undo Changes to Specific Files

| command                                               | description                                                                                                 |
| :---------------------------------------------------- | :---------------------------------------------------------------------------------------------------------- |
| `git checkout -- some_file`                           | undo changes to `some_file` in the local repository and get the latest `some_file` version from git instead |
| `git checkout HEAD path/to/file path/to/another_file` | [stackoverflow](https://stackoverflow.com/a/8735590/12282296)                                               |

## Undo local changes

- E.g., if files were removed accidentally, but you did not `git add` them yet, you can get them back using the following commands.

| command                                | description                                                                                                                                                                                                      |
| :------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| git checkout -- .                      | see `git restore .`                                                                                                                                                                                              |
| git restore --worktree file            | `man git restore`: "If neither option (`--worktree` or `--staged`) is specified, **by default** the working tree is restored. Specifying `--staged` will only restore the index. Specifying both restores both." |
| git restore .                          | discard **all unstaged** files in current working directory                                                                                                                                                      |
| git checkout -- path/to/file/to/revert | see `git restore path/to/file/to/revert`                                                                                                                                                                         |
| git restore path/to/file/to/revert     | discard a specific unstaged file                                                                                                                                                                                 |

## Undo git reset --soft

To undo `git reset HEAD~`:

`git reset HEAD@{1}`

## Undo git reset --hard

`git fsck --lost-found`

For **uncommitted** changes, run this command and git will create a new dir in your repo at `./.git/lost-found` and you might be able to find them in an `other` directory under `lost-found`.

see

- [stackoverflow](https://stackoverflow.com/a/59846439/12282296)
- [blog post](https://blog.agchapman.com/recovering-lost-commits-from-a-git-reset/)
- [stackoverflow](https://stackoverflow.com/a/22370074/12282296)

For **committed** changes, see

- [stackoverflow](https://stackoverflow.com/a/21778/12282296)

## Remove modified `__pycache__/` and `.pyc` Files from `git status`

`git checkout path/to/__pycache__/`
`git checkout *.pyc`

# Rewriting History

## git rebase

- "sowas wie `git merge`"
- [stackoverflow](https://stackoverflow.com/a/13193922/12282296)
  - Rebase is most useful when pushing a single commit or a small number of commits developed in a short time frame (hours or minutes).
  - Before pushing to a shared server, one must first pull the commits made to the origin's HEAD in the meantime—failing to do so would create a non-fast-forward push. In doing so, one can choose between a merge (`git pull`) or a rebase (`git pull --rebase`) operation. The merge option, while technically more appealing, creates an additional merge commit. For a small commit, the appearance of two commits per change actually makes the history less readable because the merge operation distracts from the commit's message.
  - In a typical shared development tree, every developer ends up pushing to a shared branch by doing some variation of `git pull; <resolve conflicts>; git push`. If they are using git pull without `--rebase`, what happens is that almost every commit ends up being accompanied by a merge commit, even though no parallel development was really going on. This creates an intertwined history from what is in reality a linear sequence of commits. For this reason, `git pull --rebase` is a better option for small changes resulting from short development, while a merge is reserved for integration of long-lived feature branches.
  - All this applies to rebasing _local_ commits, or rebasing a short-lived feature branch shared by closely connected coworkers (sitting in the same room). Once a commit is pushed to a branch regularly followed by by others, it should <span style="color:red">**never**</span> be rebased.
- [atlassian.com](https://www.atlassian.com/git/tutorials/rewriting-history/git-rebase)
  - The primary reason for rebasing is to **maintain a linear project history**.
    - For **example**, consider a situation where the `main` branch has progressed since you started working on a `feature` branch. You want to get the latest updates to the `main` branch in your `feature` branch, but you want to keep your branch's history clean so it appears as if you've been working off the latest `main` branch. This gives the later benefit of a clean merge of your `feature` branch back into the `main` branch.
- history: **CVS** and **Subversion** users routinely rebase their local changes on top of upstream work when they update before commit. **Git** just adds explicit separation between the commit and rebase steps. [source](https://stackoverflow.com/a/2452610)

## git rebase vs git merge

- [Explanation 1](https://poanchen.github.io/blog/2020/09/19/what-to-do-when-git-branch-has-diverged)
- [Explanation 2](https://www.atlassian.com/git/tutorials/merging-vs-rebasing):
  - The first thing to understand about `git rebase` is that it solves the same problem as `git merge`. Both of these commands are designed to integrate changes from one branch into another branch - they just do it in very different ways.

**Problem:**

e.g.

```bash
Your branch and 'origin/master' have diverged,
and have 2 and 1 different commits each, respectively.
  (use "git pull" to merge the remote branch into yours)
```

**Solution:**

| command            | description                                                                                                                                 |
| :----------------- | :------------------------------------------------------------------------------------------------------------------------------------------ |
| git merge main     | advantage: non-destructive operation; disadvantage: non-linear git history                                                                  |
| git rebase main    | advantage: linear project history; disadvantage: destructive operation (remember ["Golden Rule of git rebase"](#golden-rule-of-git-rebase)) |
| git rebase -i main | interactive rebasing                                                                                                                        |

## Golden Rule of git rebase

The **golden rule of git rebase** is to never use it on _public_ branches.

- d.h. Branches auf denen andere Leute arbeiten.
- [https://www.atlassian.com/git/tutorials/merging-vs-rebasing#the-golden-rule-of-rebasing](https://www.atlassian.com/git/tutorials/merging-vs-rebasing#the-golden-rule-of-rebasing)

# git stash

- [git stash basics](https://opensource.com/article/21/4/git-stash)

| command                                          | description                                                                   |
| :----------------------------------------------- | :---------------------------------------------------------------------------- |
| git stash                                        | stash uncommitted changes (staged and unstaged files)                         |
| git stash -u                                     | stash untracked files                                                         |
| git stash -a                                     | stash untracked files and ignored files                                       |
| git stash -p                                     | stash specific files                                                          |
| git stash pop                                    | removes the changes from the stash **AND** reapplies them to the working copy |
| git stash pop stash@{1}                          | by default: `pop stash@{0}`                                                   |
| git stash apply                                  | only reapplies changes to the working copy                                    |
| git stash list                                   |
| `git stash save "remove semi-colon from schema"` | add a description to the stash                                                |
| git stash clear                                  | remove all stashes                                                            |
| git stash drop stash@{1}                         | remove `stash@{1}`                                                            |
| git stash show stash@{1}                         | view the diff of the stash                                                    |
| git stash show stash@{0} -p                      | view a more detailed diff of the stash                                        |

## Undo git stash pop

Git is smart enough not to drop a stash if it doesn't apply cleanly., [stackoverflow](https://stackoverflow.com/a/22207257/12282296)

So, `git stash list` should still show the stashed changes after running `git stash pop`.

## Undo git stash apply

[stackoverflow](https://stackoverflow.com/questions/1020132/how-to-reverse-apply-a-stash)

```bash
git stash show -p | git apply --reverse

# to remove ALL non-committed changes
git checkout -f
```

## git pull despite having uncommitted changes

- pull, but make sure that the local files are not overwritten by the remote
  - [Explanation](https://stackoverflow.com/questions/19216411/how-do-i-pull-files-from-remote-without-overwriting-local-files)
    - (1) `git stash`
    - (2) `git pull`
    - (3) `git stash pop`

## git reset despite having uncommitted changes

- If you have uncommitted changes, but `git reset HEAD` doesn't work, then save your uncommitted changes with `git stash` [stackoverflow](https://stackoverflow.com/a/2125738/12282296):

```bash
git stash
git reset --hard HEAD
git stash pop
```

## After working on the wrong Branch

```bash
git stash --all
git checkout correct_branch
git stash apply
```

**Note**: `git stash` by default does not stash untracked and ignored files. ([stackoverflow](https://stackoverflow.com/a/835561/12282296))

- Use `--all` to include untracked and ignored files.
- Use `--include-untracked` aka `-u` to include untracked files only.

# git checkout

- **Remember**: Commits save your changes to the **local** repository only!

## Checkout another branch after modifying files in current branch

- When files modified wrt how you checked them out, you will not be able to `git checkout` another branch.
  Therefore, either discard via

```bash
git reset --hard   # reset all tracked files to original states
git clean -f -d	   # delete all untracked/new files and dirs
```

or keep via

```bash
git status        # see which files are modified
git add -A        # add all new files
git add file1 file2	# add specific files (hier: file1, file2)
git status 		# should show added files in green (green = files are added)
git commit -m "message for saving my solution to exercise 2" #	commit added files to your local “exercise/2_foundations”
```

## Untracked Files / Files in .gitignore

- **Remember**: Commits save your changes to the **local** repository only!
- If you do not commit **all** files of branchA before switching to branchB (e.g. because some files are in `.gitignore`) and you `git checkout branchB`, the files which were not committed to branchA will show up in branchB. To avoid this:

```bash
git checkout branchA
git add -f .; git commit -m 'WIP' # stash untracked .gitignore files in branch A ("Work In Progress")
git checkout branchB # now the untracked files will NOT show up in branchB (which is exactly what we wanted)
# do s.th. in branchB
git checkout branchA
git reset --soft HEAD~; git reset # get all stashed untracked .gitignore files back to branchA
```

## detached HEAD

Detached head means you are no longer on a branch, you have checked out a single commit in the history.

Run `git checkout master` to checkout the latest commit in `master` again.

# git diff

## Side-by-Side View

For `nvimdiff` ( aka `nvim -d` ): Paste this into your `~/.gitconfig`:

```bash
[difftool]
    prompt = true
[diff]
    tool = nvimdiff
[difftool "nvimdiff"]
    cmd = "nvim -d \"$LOCAL\" \"$REMOTE\""
```

Then run `git config --global difftool.prompt false`.

For `vimdiff`:

```bash
git config --global diff.tool vimdiff
git config --global difftool.prompt false
```

After this you can use either `nvim -d` or `vimdiff` as the difftool by running

```bash
git difftool   # accepts the same arguments as "git diff"
```

## See changes in one file

If you want to see what you haven't git added yet:

```bash
git diff myfile.txt
```

or if you want to see already added changes

```bash
git diff --cached myfile.txt
```

### Last Change of a File

```bash
git log -p [-m] [--follow] [-1] <file>
```

where `[]` means "optional".

- `-1` shows **only the most recent** change to the specified file, otherwise, **all** non-zero diffs of that file are shown.
- `--follow` is required to see changes that occurred prior to a rename.
- `-m` causes merge commits to include the diff content (otherwise these just show the commit message, as if `-p` were not specified).
- See [stackoverflow](https://stackoverflow.com/a/22412252).

## See changes in all files

```bash
git log --stat   # like the summary shown by "git pull"
git whatchanged
```

```bash
git diff --stat
git diff --numstat
git diff --shortstat
git diff --dirstat
git diff --name-status
```

## Compare two Branches

Get list of files that are different between branches (here we compare `master` to `dev` branch):

```bash
git diff --name-status master..dev
```

Then inspect changes for a particular file:

```bash
git diff master..dev /path/to/file

# if e.g. vimdiff has been configured as the difftool (see section "Side-by-Side View"):
git difftool -y master..dev /path/to/file
```

The `-y` flag automatically confirms the prompt to open the diff in the editor. Use `git config --global difftool.prompt false` to turn off these prompts.

## Compare files outside a git repo, `--no-index`

`git diff oldFile newFile` works in all folders, not just in git repos.

```bash
git diff origin/exercise/4_ros_node_cpp..origin/solution/4_ros_node_cpp

# compare two arbitrary files on disk
# ("--no-index" is only necessary inside a git repo)
git diff --no-index file1.txt file2.txt
```

## Compare files between commits

Syntax:

```bash
git diff [--options] <commit> <commit> [--] [<path>...]
```

```bash
git diff HEAD^^ HEAD main.c
git diff HEAD^^..HEAD -- main.c
git diff HEAD~2 HEAD -- main.c

find . -iname *templates* # copy the (long) file path
git log ./docs/cpp/core/2022-10-06-notes-cpp-templates.md
commithash=SHA-1
git difftool $commithash~2 $commithash -- ./docs/cpp/core/2022-10-06-notes-cpp-templates.md
```

## Compare commits

```bash
# Last commits
git diff HEAD^ HEAD

# Syntax: git diff COMMIT~ COMMIT (vergleicht COMMIT mit dessen ancestor)
git diff 2326473e602be4b90b46f6b6afc7315ff1d09a17~ 2326473e602be4b90b46f6b6afc7315ff1d09a17
```

## Compare two different files between different commits

[stackoverflow](https://stackoverflow.com/a/16683184)

```bash
git diff HEAD:full/path/to/foo HEAD~:full/path/to/bar
```

## Compare with stash

```bash
git stash show -p stash@{2}
```

Why `-p`?: By default, the command shows the `diffstat`, but it will accept any format known to `git diff` (e.g., `git stash show -p stash@{1}` to view the second most recent stash in patch form).

Note: `stash@{0}` is the default

## Compare with an unmerged file

**problem**: `git diff <unmerged-path>` shows nothing, whereas `git diff --staged <successfully-merged-and-added-file>` shows a diff

[stackoverflow](https://stackoverflow.com/questions/22215651/why-doesnt-git-diff-show-anything-for-unmerged-paths)

"During a merge, the conflicting files are in a special state."

**Takeaway**: There is no easy way to do this. During a merge conflict, there are multiple blobs with the name of the conflicting file and you need to diff these blobs.

## `--cached`, `--staged`

To see already added changes: `git diff --cached`

`--staged` is a synonym of `--cached`

## `--name-status`

Show only names and status of changed files. See the description of the `--diff-filter` option on what the status letters mean.

## `--diff-filter`

From `man git-diff`:

`git diff --diff-filter=[ACDMRTUXB*]`

Select only files that are

- `A` Added
- `C` Copied
- `D` Deleted
- `M` Modified
- `R` Renamed
- `T` have their type (mode) changed
- `U` Unmerged
- `X` Unknown
- `B` have had their pairing Broken
- `*` All-or-none

Any combination of the filter characters may be used.

When `*` (All-or-none) is added to the combination, all paths are selected if there is any file that matches other criteria in the comparison; if there is no file that matches other criteria, nothing is selected.

Also, these upper-case letters can be **downcased** to **exclude**. E.g. `--diff-filter=ad` excludes added and deleted paths.

# Hunks

[gnu.org](https://www.gnu.org/software/diffutils/manual/html_node/Hunks.html)

"When comparing two files, `diff` finds sequences of lines common to both files, interspersed with groups of differing lines called **hunks**."

# git grep

[stackoverflow](https://stackoverflow.com/a/2929502)

| command                                             | description                                                                                                                                         |
| :-------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------- |
| `git grep <regexp>`                                 | Search working tree for text matching regular expression `regexp`                                                                                   |
| `git grep -e <regexp1> [--or] -e <regexp2>`         | Search working tree for lines of text matching regular expression `regexp1` or `regexp2`                                                            |
| `git grep -l -e <regexp1> --and -e <regexp2>`       | Search working tree for lines of text matching regular expression `regexp1` and `regexp2`, reporting file paths only                                |
| `git grep -l --all-match -e <regexp1> -e <regexp2>` | Search working tree for files that have lines of text matching regular expression `regexp1` and lines of text matching regular expression `regexp2` |
| `git diff --unified=0 \| grep <pattern>`            | Search working tree for changed lines of text matching `pattern`                                                                                    |
| `git grep <regexp> $(git rev-list --all)`           | Search all revisions for text matching regular expression `regexp`                                                                                  |
| `git grep <regexp> $(git rev-list <rev1>..<rev2>)`  | Search all revisions between `rev1` and `rev2` for text matching regular expression `regexp`                                                        |

# git merge

| command                  | description                                       |
| :----------------------- | :------------------------------------------------ |
| `git merge newdevBranch` | will merge `newdevBranch` into the current branch |

# Merge Conflicts

## Resolving a Merge Conflict

from: [docs.github.com](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/addressing-merge-conflicts/resolving-a-merge-conflict-using-the-command-line)

Generate a list of the files affected by the merge conflict. In this example, the file `styleguide.md` has a merge conflict.

```
$ git status
> # On branch branch-b
> # You have unmerged paths.
> #   (fix conflicts and run "git commit")
> #
> # Unmerged paths:
> #   (use "git add <file>..." to mark resolution)
> #
> # both modified:      styleguide.md
> #
> no changes added to commit (use "git add" and/or "git commit -a")
```

To see the beginning of the merge conflict in your file, search the file for the conflict marker `<<<<<<<`. When you open the file in your text editor, you'll see the changes from the HEAD or base branch after the line `<<<<<<< HEAD`. Next, you'll see `=======`, which divides your changes from the changes in the other branch, followed by `>>>>>>> BRANCH-NAME`. In this example, one person wrote "open an issue" in the base or HEAD branch and another person wrote "ask your question in IRC" in the compare branch or `branch-a`.

```
If you have questions, please
<<<<<<< HEAD
open an issue
=======
ask your question in IRC.
>>>>>>> branch-a
```

Decide if you want to keep only your branch's changes, keep only the other branch's changes, or make a brand new change, which may incorporate changes from both branches. Delete the conflict markers `<<<<<<<`, `=======`, `>>>>>>>` and make the changes you want in the final merge. In this example, both changes are incorporated into the final merge:

```
If you have questions, please open an issue or ask in our IRC channel if it's more urgent.
```

Add or stage your changes.

```bash
git add .
```

Commit your changes with a comment.

```bash
git commit -m "Resolve merge conflict by incorporating both suggestions"
```

You can now merge the branches on the command line or push your changes to your remote repository on GitHub and merge your changes in a pull request.

## "deleted by us: path/to/file.ext"

[stackoverflow](https://stackoverflow.com/a/42174625/12282296)

- "`us`" is the branch you are applying the changes/commits to
  - [stackoverflow](https://stackoverflow.com/a/21025695/12282296)
    - When you <span style="color:red">**merge**</span>, `us` refers to the branch you're merging into, as opposed to `them`, the branch to be merged.
    - When you <span style="color:red">**rebase**</span>, `us` refers the upstream branch, and `them` is the branch you're moving about. It's a bit counter-intuitive in case of a rebase.
- if you want to keep this file, run `git add file` or, if you wanted to accept the delete, `git rm file` to resolve the conflict

# git config

| command                                | description                                                                                                                         |
| :------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------- |
| `git config -l`                        | list the current git settings                                                                                                       |
| `git config --get remote.origin.url`   | get only the URL of the remote `origin`                                                                                             |
| `git config --global core.editor nvim` | set the default editor, see [git-scm](https://git-scm.com/book/en/v2/Customizing-Git-Git-Configuration#_basic_client_configuration) |

# Git Credential Helper, Storing Git Passwords

## Set credential helper

| command                                     | description                                                                                                |
| :------------------------------------------ | :--------------------------------------------------------------------------------------------------------- |
| git config credential.helper store          | store the next entered password in ~/.git-credentials (visible for anyone !) (for current repo only)       |
| git config --global credential.helper store | store the next entered password in ~/.git-credentials (visible for anyone !) (globally, ie. for all repos) |

## Unset/Reset credential helper

Unset credential helper

`git config --global --unset credential.helper`

and set pw again as usual or just use

`git config credential.helper store`

and you will be prompted to enter pw again.

# Create a new repository on the command line

Install gh CLI first: [installation instructions](https://github.com/cli/cli/blob/trunk/docs/install_linux.md)

```bash
# Read the man pages!
man gh
man gh-repo
man gh-repo-create
```

```bash
gh auth login
gh repo create   # do not add anything, create an empty repo!
gh repo list
gh repo view dotfiles   # no "pharath/" prefix needed for "dotfiles"!
# copy the repo url (needed for "git remote add")
```

```bash
echo "# documentation" >> README.md
git init
# Note: without this the initial commit will not be listed in "git log" and
# the branch will not be created ("git branch -a" will not show the branch)
git add README.md

# from https://stackoverflow.com/a/42871621:
# The branch "master" does not actually exist -- the branches don't get created until they have at least
# one commit (phth: this must be a non-empty commit!).
# Until the branch gets created, the branch only exists in ".git/HEAD", which explains why the "master" branch
# will disappear when you switch to "main".
# either:
git commit -m "first commit"
git branch -m main
# or:
git checkout -b main
git commit -m "first commit"

git remote add origin https://github.com/pharath/documentation.git
git push -u origin main
```

# Push an existing repository from the command line

```bash
git remote add origin https://github.com/pharath/documentation.git
git branch -M main
git push -u origin main
```

# Push to a fork

## After working (accidentally) on original Repo

```bash
git remote rename origin old-origin # rename old remote
git remote add origin https://github.com/pharath/TSR_PyTorch.git # add new remote
git fetch origin

# for git > v2.23
git switch -c local/origin/main origin/main # create a new LOCAL branch tracking the newly added remote https://github.com/pharath/TSR_PyTorch.git
# Notation:
git switch -c origin/main origin/main # notation: "local/" is usually not written

# for git < v2.23: statt "git switch"
git branch my2.6.14 v2.6.14
git checkout my2.6.14
# do both in one command:
git checkout -b local/origin/main origin/main

# Note: delete branches via "git branch -D branch_name", if necessary

# here "HEAD" is important!
git push origin HEAD:main
```

# submodules

## Populate a repository's submodules

1. `git submodule init`
2. `git submodule update --progress`

- use `--progress` to display a cloning progress report

## Add submodule

```bash
git submodule add <git repo url> path/to/submodule/folder/
```

# git gc

Generally speaking you shouldn't be running garbage collection manually. It is a bad habit to get into and git does garbage collection when needed anyways. [stackoverflow comment under an answer](https://stackoverflow.com/a/18515113)

## Dangling Objects

Run `git fsck` in your repo to see all dangling objects.

**Types** of dangling objects ( [stackoverflow](https://stackoverflow.com/a/22226373) ):

- **Dangling blob** = A change that made it to the staging area/index, but never got committed. One thing that is amazing with Git is that once it gets added to the staging area, you can always get it back because these blobs behave like commits in that they have a hash too!!
- **Dangling commit** = A commit that isn't directly linked to by any child commit, branch, tag or other reference. You can get these back too!

E.g. if you run `git add .` and it takes very long, so that you have to press `ctrl + c` and run `git reset`, then there will be many **dangling blobs** after running this `git reset`.

# Troubleshooting/Errors

```bash
 ! [remote rejected] origin/parking -> origin/parking (deny updating a hidden ref)
error: failed to push some refs to 'https://git.rwth-aachen.de/team_galaxis/carolo-cup-2021.git'
```

**Solution**: The branch name is wrong. Use `git push origin parking` instead of `git push origin origin/parking`.

```bash
$ git checkout local/origin/deepstream
error: The following untracked working tree files would be overwritten by checkout:
	README.md
Please move or remove them before you switch branches.
Aborting
```

**Solution**: The file `README.md` also exists in branch `local/origin/deepstream`. Remove `README.md` and run the command again. Make sure there are no other **uncommitted** files or folders in the current branch having the same name as one of the files and folders in the branch `local/origin/deepstream`.
