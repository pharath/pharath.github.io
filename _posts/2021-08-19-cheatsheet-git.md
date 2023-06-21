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

**index** = Staging Directory (`.index/`) (see [explanation](https://www.javatpoint.com/git-index))
    - There are three places in Git where file changes can reside, and these are 
        - working directory, 
        - staging area, 
        - the repository.

# Basics

## git rm, git add

| command | description |
| :--- | :--- |
`git rm file1.txt` | remove the file from the Git repository **and the filesystem**
`git rm --cached file1.txt` | remove the file only from the Git repository and not remove it from the filesystem
`git add` | synonoym for `git stage` (since 2008), see [stackoverflow](https://stackoverflow.com/a/34175877)
`git add -u :/` | `git rm` all files after accidentally `rm` those files
`git add --all -- ':!./Gemfile' ':!./_config.yml' ':!./_layouts/*.bak' ':!./_sass/*.bak'` | **exclude files** from `git add`
`git stage` | synonoym for `git add` (since 2008), see [stackoverflow](https://stackoverflow.com/a/34175877)

## git clone

| command | description |
| :--- | :--- |
`git clone --recurse-submodules repo <Ziel directory>` | 

## git branch

| command | description |
| :--- | :--- |
`git branch -a` | show/list all branches (local and remote)
`git branch -r` | show/list all branches (only remote)
`git branch -d *local_branch*` | delete local branch *local_branch*
`git push origin --delete *remote/branch*` | delete remote branch *remote/branch*
`git branch -m new_name` | rename branch (`--move`)
`git branch -M new_name` | force rename branch (`--move --force`), ie rename even if the branch name exists
`git branch -vv` | show which **local branches** track which **remote branches**, useful e.g. when you are creating and setting up new branches
`git branch --set-upstream my_branch origin/my_branch` | make the **local** `my_branch` track the **remote** `origin/my_branch` (similar to `git push --set-upstream origin <new_branch>`)

## git checkout, git switch

| command | description |
| :--- | :--- |
`git checkout <existing_branch>` | switch to an existing branch
`git switch *branch*` | switch to an existing branch (since Git v2.23)
`git checkout -b <new_branch>` | switch to a non-existing branch; subsequently, `git push --set-upstream origin <new_branch>` to create `new_branch` in the remote (on github.com). 
`git checkout -b <new_branch> <old_branch>` | create `<new_branch>` off `<old_branch>`
`git switch -c *branch*` | switch to a non-existing branch (since Git v2.23)
`git push --set-upstream origin <new_branch>` | push to a locally newly created branch (see `git checkout -b <new_branch>`) that does not yet exist on remote (i.e. on github.com). `--set-upstream`: will make the local `<new_branch>` track the remote `remotes/origin/<new_branch>` (w/o this flag git does not know where to push the `new_branch`).

## git show-branch

| command | description |
| :--- | :--- |
`git show-branch -a` | show/list all branches **and the last commit message** (local and remote)
`git show-branch -r` | show/list all branches **and the last commit message** (only remote)

## Tags

| command | description |
| :--- | :--- |
`git tag -l` | list all tags
`git -c 'versionsort.suffix=-' ls-remote --tags --sort='v:refname' <repository>` | list all tags of `<repository>`
`git checkout tags/<tag name>`	| checkout a specific tag
`git clone --depth 1 --branch <tag_name> <repo_url>` | clone a specific tag; `--depth 1` is optional but if you only need the state at that one revision, you probably want to skip downloading all the history up to that revision.

## git remote

| command | description |
| :--- | :--- |
`git remote get-url origin` | get only the URL of the remote `origin`
`git remote set-url origin https://git.rwth-aachen.de/mygroup/myreponame.git` | set the URL of the remote `origin`
`git remote show [remote-name] command` | get more details about a particular remote
`git remote show origin` | get more details about the current remote
`git remote update origin --prune` | To update the local list of remote branches

## git reflog

| command | description |
| :--- | :--- |
`git reflog` | view history of `git checkout` operations
`git revert <commit-hash>` | commit `<commit-hash>` rückgängig machen

## git log, git shortlog

| command | description |
| :--- | :--- |
`git log` | view history of commits ([more commands](#git-log))
`git log -- filename` | commit history of a file
`git log -p -- filename` | Like `git log`, but shows the file content that changed, as well. Generates the patches for each log entry.
`git log --all` | all commits of all branches, tags and other refs (why `--all` isn't the default: you normally won't want that. For instance, if you're on branch `master`, and you run `git log`, you typically aren't interested in the history of any feature branches, you typically want to see the history of `master`, [stackoverflow](https://stackoverflow.com/a/29756754))
`git show HEAD` | just the diff for a specific commit
`gitk [filename]` | To browse the changes visually
`git shortlog` | show the commit messages only

## git pull vs fetch vs update

[source](https://stackoverflow.com/a/17712553):

- `git remote update` will update all of your branches set to track remote ones, but not merge any changes in.
- `git fetch` will update only the branch you're on, but not merge any changes in.
- `git pull` will update and merge any remote changes of the current branch you're on. This would be the one you use to update a local branch.

## .gitignore

[source](https://git-scm.com/docs/gitignore)

Git normally checks `gitignore` patterns from multiple sources, with the following order of precedence, from highest to lowest 
- Patterns read from the command line
- Patterns read from a `.gitignore` file
- Patterns read from `$GIT_DIR/info/exclude`
- Patterns read from the file specified by the configuration variable `core.excludesFile`

# Reset/undo changes

## Undo git add

| command | description |
| :---: | :---: |
git reset | undo `git add`
git restore --staged file | discard a `file` **in the index** (= "added" file) without overwriting the same file **in the working tree**. `man git restore`: "Specifying `--staged` will only restore the index" and **not** the working tree!

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
git push origin +HEAD | reset the **remote's** last commit to the **local's** last commit (**Note**: `HEAD` always points to the last commit. The `+` indicates a force-push, like the `-f` flag, see [stackoverflow](https://stackoverflow.com/a/25937833).)
git reset --soft HEAD~1 | `HEAD~` and `HEAD~1` are the same; `HEAD` and `HEAD~0` are the same
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
git restore --worktree file | `man git restore`: "If neither option (`--worktree` or `--staged`) is specified, **by default** the working tree is restored. Specifying `--staged` will only restore the index. Specifying both restores both."
git restore . | discard **all unstaged** files in current working directory
git checkout -- path/to/file/to/revert | see `git restore path/to/file/to/revert`
git restore path/to/file/to/revert | discard a specific unstaged file

## Remove modified `__pycache__/` and `.pyc` Files from `git status`

`git checkout path/to/__pycache__/`
`git checkout *.pyc`

# Rewriting History

## git rebase

- "sowas wie `git merge`"
- [atlassian.com](https://www.atlassian.com/git/tutorials/rewriting-history/git-rebase)
    - The primary reason for rebasing is to **maintain a linear project history**. 
        - For **example**, consider a situation where the `main` branch has progressed since you started working on a `feature` branch. You want to get the latest updates to the `main` branch in your `feature` branch, but you want to keep your branch's history clean so it appears as if you've been working off the latest `main` branch. This gives the later benefit of a clean merge of your `feature` branch back into the `main` branch.

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

- [git stash basics](https://opensource.com/article/21/4/git-stash)

| command | description |
| :---: | :---: |
git stash | stash uncommitted changes (staged and unstaged files)
git stash -u | stash untracked files
git stash -a | stash untracked files and ignored files
git stash -p | stash specific files
git stash pop | removes the changes from the stash **AND** reapplies them to the working copy
git stash pop stash@{1} | by default: `pop stash@{0}`
git stash apply | only reapplies changes to the working copy
git stash list |
`git stash save "remove semi-colon from schema"` | add a description to the stash
git stash clear | remove all stashes
git stash drop stash@{1} | remove `stash@{1}`
git stash show stash@{1} | view the diff of the stash
git stash show stash@{0} -p | view a more detailed diff of the stash

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

## Compare files on disk, `--no-index`

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

## Compare two different files between different commits

[stackoverflow](https://stackoverflow.com/a/16683184)

```bash
git diff HEAD:full/path/to/foo HEAD~:full/path/to/bar
```

## diff-filter

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

# git config

| command | description |
| :---: | :---: |
`git config -l` | list the current git settings
`git config --get remote.origin.url` | get only the URL of the remote `origin`

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
