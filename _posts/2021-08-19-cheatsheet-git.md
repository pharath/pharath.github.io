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

**remote** = remote-repository (e.g. in `git push *remote* *branch*`)
**PAT** = Personal Access Token (see [youtube: PATs and their scopes](https://www.youtube.com/watch?v=SzrETQdGzBM&t=49s))

# Basics

| command | description |
| :--- | :--- |
`git rm file1.txt` | remove the file from the Git repository **and the filesystem**
`git rm --cached file1.txt` | remove the file only from the Git repository and not remove it from the filesystem
`git add -u :/` | `git rm` all files after accidentally `rm` those files

| command | description |
| :--- | :--- |
`git clone --recurse-submodules repo <Ziel directory>` | 
`git branch -a` | show/list all branches (local and remote)
`git branch -r` | show/list all branches (only remote)
`git branch -d *local_branch*` | delete local branch *local_branch*
`git push origin --delete *remote/branch*` | delete remote branch *remote/branch*
`git show-branch -a` | show/list all branches **and commits** (local and remote)
`git show-branch -r` | show/list all branches **and commits** (only remote)
`git checkout <existing_branch>` | switch to an existing branch (or: git switch *branch*)
`git checkout -b <new_branch>` | switch to a non-existing branch (or: `git switch -c *branch*`); subsequently, `git push --set-upstream origin <new_branch>` to create `new_branch` in the remote (on github.com), too. Here, the `--set-upstream` flag will make the local `<new_branch>` track the remote `remotes/origin/<new_branch>` (w/o this flag git does not know where to push the `new_branch`).
`git push --set-upstream origin <new_branch>` | push to a locally newly created branch (see `git checkout -b <new_branch>`) that does not yet exist on remote (i.e. on github.com)
`git branch -vv` | show which **local branches** track which **remote branches**, useful e.g. when you are creating and setting up new branches
`git branch --set-upstream my_branch origin/my_branch` | make the **local** `my_branch` track the **remote** `origin/my_branch` (similar to `git push --set-upstream origin <new_branch>`)
`git reflog` | view history of checkout operations
`git log`	| view history of commits ([more commands](#git-log))
`git revert <commit-hash>`	| commit `<commit-hash>` rückgängig machen
`git tag -l`	| list all tags
`git -c 'versionsort.suffix=-' ls-remote --tags --sort='v:refname' <repository>` | list all tags of `<repository>`
`git checkout tags/<tag name>`	| checkout a specific tag
`git clone --depth 1 --branch <tag_name> <repo_url>` | clone a specific tag; `--depth 1` is optional but if you only need the state at that one revision, you probably want to skip downloading all the history up to that revision.
`git fetch` |
`git checkout solution/2_foundations` |
`git submodule init` |
`git submodule update --progress` | `submodule update` by default does not show any cloning progress, use `--progress` to show the cloning progress
`git config --get remote.origin.url` | get only the URL of the current remote
`git remote show [remote-name] command` | get more details about a particular remote
`git remote show origin` | get more details about the current remote
`git remote update origin --prune` | To update the local list of remote branches

## git log

| command | description |
| :--- | :--- |
`git log -- filename` | commit history of a file
`git log -p -- filename` | Like `git log`, but shows the file content that changed, as well. Generates the patches for each log entry.
`git show HEAD` | just the diff for a specific commit
`gitk [filename]` | To browse the changes visually

## pull vs fetch vs update

[source](https://stackoverflow.com/a/17712553):

- `git remote update` will update all of your branches set to track remote ones, but not merge any changes in.
- `git fetch` will update only the branch you're on, but not merge any changes in.
- `git pull` will update and merge any remote changes of the current branch you're on. This would be the one you use to update a local branch.

# Reset/undo changes

## Undo git add

| command | description |
| :---: | :---: |
git reset | undo `git add`

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

| command | description |
| :---: | :---: |
git reflog | get SHA-1 list of previous states
git reset --soft HEAD~ | undo last commit **locally** (`--soft`: safe way)
git push origin +HEAD | reset the **remote's** last commit to the **local's** last commit (**Note**: `HEAD` always points to the last commit.)
git reset --soft HEAD~1 | HEAD~ and HEAD~1 are the same
git reset --soft *SHA-1* | reset to a previous state **locally** (`--soft`: safe way)
git reset --hard *SHA-1* | reset to a previous state **locally** (**Warning**: `--hard`: All changes will be lost.)

## Undo Changes to Specific Files

| command | description |
| :---: | :---: |
`git checkout -- some_file` | undo changes to `some_file` in the local repository and get the latest `some_file` version from git instead
`git checkout HEAD path/to/file path/to/another_file` | [stackoverflow](https://stackoverflow.com/a/8735590/12282296)

## Undo local changes

- E.g., if files were removed accidentally, but you did not `git add` them yet, you can get them back using the following commands.

| command | description |
| :---: | :---: |
git checkout -- . | see `git restore .`
git restore . | discard all unstaged files in current working directory
git checkout -- path/to/file/to/revert | see `git restore path/to/file/to/revert`
git restore path/to/file/to/revert | discard a specific unstaged file

## Remove modified `__pycache__/` and `.pyc` Files from `git status`

`git checkout path/to/__pycache__/`
`git checkout *.pyc`

# git rebase vs git merge

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

| command | description |
| :---: | :---: |
git merge main | advantage: non-destructive operation; disadvantage: non-linear git history
git rebase main | advantage: linear project history; disadvantage: destructive operation (remember ["Golden Rule of git rebase"](#golden-rule-of-git-rebase))
git rebase -i main | interactive rebasing

## Golden Rule of git rebase

The **golden rule of git rebase** is to never use it on *public* branches.
- d.h. Branches auf denen andere Leute arbeiten.
- [https://www.atlassian.com/git/tutorials/merging-vs-rebasing#the-golden-rule-of-rebasing](https://www.atlassian.com/git/tutorials/merging-vs-rebasing#the-golden-rule-of-rebasing)

# git stash

- use case 1: way to pull, but make sure that the local files are not overwritten by the remote
    - [Explanation](https://stackoverflow.com/questions/19216411/how-do-i-pull-files-from-remote-without-overwriting-local-files)
        - (1) `git stash`
        - (2) `git pull`
        - (3) `git stash pop`
- use case 2: If you have uncommitted changes, but the first command doesn't work, then save your uncommitted changes with `git stash` [stackoverflow](https://stackoverflow.com/a/2125738/12282296):
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

# git diff

## Side-by-Side View

For `nvim -d`: Paste this into your `~/.gitconfig`:
```bash
[difftool]
    prompt = true
[diff]
    tool = nvimdiff
[difftool "nvimdiff"]
    cmd = "nvim -d \"$LOCAL\" \"$REMOTE\""
```

For `vimdiff`:
```bash
git config --global diff.tool vimdiff
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

The `-y` flag automatically confirms the prompt to open the diff in the editor.

## Compare files on disk

```bash
git diff origin/exercise/4_ros_node_cpp..origin/solution/4_ros_node_cpp
# compare two arbitrary files on disk
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
```

## Compare commits

```bash
# Last commits
git diff HEAD^ HEAD

# Syntax: git diff COMMIT~ COMMIT (vergleicht COMMIT mit dessen ancestor)
git diff 2326473e602be4b90b46f6b6afc7315ff1d09a17~ 2326473e602be4b90b46f6b6afc7315ff1d09a17
```

# Git Credential Helper, Storing Git Passwords

## Set credential helper

| command | description |
| :---: | :---: |
git config credential.helper store | store the next entered password in ~/.git-credentials (visible for anyone !) (for current repo only)
git config --global credential.helper store | store the next entered password in ~/.git-credentials (visible for anyone !) (globally, ie. for all repos)

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
gh repo create
gh repo list
```

```bash
echo "# documentation" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
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
