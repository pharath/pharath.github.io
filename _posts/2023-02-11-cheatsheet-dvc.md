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

```bash
# initialize a new git repo and dvc repo
git init 
dvc init

# create dvc repo
mkdir data
dvc get https://github.com/iterative/dataset-registry get-started/data.xml -o data/data.xml
dvc add data/data.xml
git add data/.gitignore data/data.xml.dvc
git commit -m "Add raw data"

# add dvc remote (cloud storage)
dvc remote add -d storage gdrive://<copy repo id from browser url>
git commit .dvc/config -m "Configure remote storage"
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
dvc add data/data.xml
git add data/data.xml.dvc 
git commit -m "Dataset updates"
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
