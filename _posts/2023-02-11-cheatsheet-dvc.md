---
title: "DVC Cheatsheet"
read_time: false
excerpt: "Some essential dvc commands"
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - dvc
  - cheatsheet
---


# Important

TODO: Could not make it work for **private** git repos yet.

# Install with pip

- `python -m pip install --upgrade pip`
- `python3 -m venv env`
- `source env/bin/activate`
- `pip install dvc` 
    - if it gives `ERROR: Could not build wheels for pygit2 which use PEP 517 and cannot be installed directly`, **fix**: upgrade `pip` to the latest version!
- `pip install 'dvc[gdrive]'`
- `dvc completion -s bash | sudo tee /etc/bash_completion.d/dvc` to activate **bash autocompletion**, then restart your terminal

# Basics

Watch: [official YouTube tutorials](https://www.youtube.com/watch?v=kLKBcPonMYw&list=PL7WG7YrwYcnDb0qdPl9-KEStsL-3oaEjg)

```bash
# always update pip and dvc first 
# (otherwise gdrive access does not work sometimes)
pip install -U pip
pip install -U dvc
pip install -U 'dvc[gdrive]'

# 1. initialize a new git repo and dvc repo
git init 
dvc init

# 2. create dvc repo
mkdir data
# clone a file
dvc get https://github.com/iterative/dataset-registry get-started/data.xml -o data/data.xml
# add a directory
dvc add -R mydir/
# add a file
dvc add data/data.xml   # like "git add" + "git commit" together; use "dvc add --no-commit" flag to avoid committing
# track the changes with git
git add data/.gitignore data/data.xml.dvc
git commit -m "Add raw data"

# 3. add dvc remote (cloud storage)
dvc remote add -d myremote gdrive://<copy repo id from browser url>
# if necessary: authorize gdrive (see below in section "Authorization")
# (https://dvc.org/doc/user-guide/data-management/remote-storage/google-drive#authorization)
dvc remote modify myremote --local gdrive_user_credentials_file ~/.gdrive/myremote-credentials.json
# track the changes with git
git commit .dvc/config -m "Configure remote storage"

# 4. push data
git push   # always "git push" before "dvc push", otherwise you will forget "git push" because "dvc push" can take a long time
dvc push
cat data/.gitignore   # confirm that data is not tracked by git

# pull
dvc pull
```

## Pushing a new dataset version

```bash
# create a new dataset artificially by "doubling" the data.xml
cp -iv data/data.xml /tmp/data.xml
cat /tmp/data.xml >> data/data.xml
ls -lh data/

# push the new dataset version
dvc add data/data.xml
git add data/data.xml.dvc 
git commit -m "Dataset updates"
git push   # always "git push" before "dvc push", otherwise you will forget "git push" because "dvc push" can take a long time
dvc push
```

## Moving forward and backward in time with different versions of the dataset

```bash
git log --oneline
git checkout HEAD^1 data/data.xml.dvc
dvc checkout
l -lh data/   # confirms that the previous version has been restored
git commit data/data.xml.dvc -m "Revert dataset updates"   # to keep the changes
# we do not have to run another "dvc add" because we have saved this version of the dataset in the dvc repo already
```

# dvc add

Behaves like `git add` + `git commit` together.

To get a `git`-like behavior:
- use `dvc add --no-commit` flag to avoid committing.
- and then `dvc commit`

| command | description |
| :--- | :--- |
`dvc add -R mydir/` | adds subdirectories of `mydir/` recursively (by default, `dvc add` does not!)

# dvc remote

## Authorization

```bash
dvc remote modify myremote --local gdrive_user_credentials_file ~/.gdrive/myremote-credentials.json
```
- this is necessary e.g. if you are adding a remote from a different GDrive account than the first remote
- **IMPORTANT**: Add `~/.gdrive/myremote-credentials.json` to `.gitignore`! Don't commit it!
  - the `--local` flag writes to `.dvc/config.local` which is in `.dvc/.gitignore`
- [dvc doc - GDrive Authorization](https://dvc.org/doc/user-guide/data-management/remote-storage/google-drive#authorization)
