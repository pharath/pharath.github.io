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
dvc add mydir/
# add each file in a directory individually 
# (Don't use this! Not needed!) (see https://dvc.org/doc/command-reference/add#adding-entire-directories)
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

Undo `dvc add`: ([doc](https://dvc.org/doc/user-guide/how-to/stop-tracking-data#how-to-stop-tracking-data))
```bash
dvc remove data.dvc   # the .dvc file must be removed, not the data itself!
dvc gc -w   # clear the cache
```

Behaves like `git add` + `git commit` together.

To get a `git`-like behavior:
- use `dvc add --no-commit` flag to avoid committing.
- and then `dvc commit`

| command | description |
| :--- | :--- |
`dvc add mydir/` | adds directory `mydir/`
`dvc add -R mydir/` | adds each file in subdirectories of `mydir/` recursively (Don't use this! Not needed! See [doc](https://dvc.org/doc/command-reference/add#adding-entire-directories))

# dvc list url path

The following commands will only work when
- you are **inside a DVC repo** (outside a DVC repo you will only see the files and folders that have a corresponding `.dvc` file)
- a **default** remote (`dvc remote default`) containing the target is set for the current DVC repo ([doc](https://dvc.org/doc/command-reference/remote/default))

```bash
dvc list . .
dvc list . data/augmented/data_11_19.v4i.darknet/
dvc list -R . data/augmented/data_11_19.v4i.darknet/
```

# dvc pull

- Downloads tracked files or directories from remote storage based on the current `dvc.yaml` and `.dvc` files, and makes them visible **in the workspace** (i.e. in the git repo folder).
- Downloads tracked data from a dvc remote **to the cache**.

```bash
dvc pull data/raw_not_augmented/data_11_19.v6i.darknet/train/
```

# dvc get

- Syntax: `dvc get url path`
- Downloads a file or directory tracked by DVC or by Git into the current working directory.
- This file or directory must be found in a `dvc.yaml` or `.dvc` file of the repo.

`dvc get` is like `wget` or `curl`.

Get the folder `data/raw_not_augmented/data_11_19.v6i.darknet/`:
```bash
cd path/to/gitlab/dvc/repo/
dvc get . data/raw_not_augmented/data_11_19.v6i.darknet/
```

# dvc config

**Important:** This configuration is specific to the Git repository of the dataset (it is not set globally)!

Your **repo** configuration should look similar to this:

```bash
$ dvc config -l

remote.galaxisgdrive.url=gdrive://1-7B_Sg8rX0IjozvJy31j4La4xiGB_rrw
core.remote=galaxisgdrive
remote.galaxisgdrive.gdrive_user_credentials_file=/home/bra-ket/.gdrive/myremote-credentials.json
remote.galaxisgdrive.gdrive_acknowledge_abuse=true
```

If your configuration doesn't look like this, you have to go through the authorization process first (see [Authorization](#authorization)).

# dvc remote

**Important:** DVC remotes are set for each Git repository individually! I.e. the output of `dvc remote list` depends on your current working directory.

## Authorization

### Google Drive

**Important:** If you have any authorization issues, chances are that your GDrive token has expired. Re-run the following command followed by `dvc pull`. A browser window should pop up and GDrive will prompt you to log in again (also see [Troubleshooting](#problem-1)).

```bash
# Set "myremote" to your specific remote's name
dvc remote modify myremote --local gdrive_user_credentials_file ~/.gdrive/myremote-credentials.json
```
- this is necessary e.g. if 
  - you want to configure `remote.yourremote.gdrive_user_credentials_file` for your dvc repo
  - you are adding a remote from a different GDrive account than the first remote
  - you want to refresh your expired GDrive access token (see [Troubleshooting](#problem-1))
    - see `"token_expiry": "2023-05-17T04:00:13Z",` field in `~/.gdrive/myremote-credentials.json`
- **IMPORTANT**: Add `~/.gdrive/myremote-credentials.json` to `.gitignore`! Don't commit it!
  - the `--local` flag writes to `.dvc/config.local` which is in `.dvc/.gitignore`
- [dvc doc - GDrive Authorization](https://dvc.org/doc/user-guide/data-management/remote-storage/google-drive#authorization)

```bash
# Set "myremote" to your specific remote's name
dvc remote modify myremote gdrive_acknowledge_abuse true
```
- see [Troubleshooting](#problem-2)
- see [dvc doc](https://dvc.org/doc/user-guide/data-management/remote-storage/google-drive#authorization)

# dvc move

- creates the destination directory, if it does not exist (ie. `mkdir` is not necessary! See example in [doc](https://dvc.org/doc/command-reference/move#example-move-a-directory))

# Troubleshooting

## Problem 1

```bash
ERROR: unexpected error - : <HttpError 404 when requesting https://www.googleapis.com/drive/v2/files/1-7B_Sg8rX0IjozvJy31j4La4xiGB_rrw?fields=driveId&supportsAllDrives=true&alt=json returned "File not found: 1-7B_Sg8rX0IjozvJy31j4La4xiGB_rrw". Details: "[{'message': 'File not found: 1-7B_Sg8rX0IjozvJy31j4La4xiGB_rrw', 'domain': 'global', 'reason': 'notFound', 'location': 'file', 'locationType': 'other'}]">

Having any troubles? Hit us up at https://dvc.org/support, we are always happy to help!
```

**Problem**:

The `remote.yourremote.gdrive_user_credentials_file` field is probably not set for your repo (check with `dvc config -l`).

**Solution**:

See [Authorization GDrive](#google-drive)

## Problem 2

```bash
ERROR: failed to transfer '22a1a2931c8370d3aeedd7183606fd7f' - <HttpError 403 when requesting https://www.googleapis.com/drive/v2/files/1B1qrTj9XDcz5ywlZHnnbrghQIElgIImm?acknowledgeAbuse=false&alt=media returned "This file has been identified as malware or spam and cannot be downloaded". Details: "[{'message': 'This file has been identified as malware or spam and cannot be downloaded', 'domain': 'global', 'reason': 'abuse'}]">                                            
ERROR: failed to pull data from the cloud - 1 files failed to download
```

**Solution**:

Set `dvc remote modify myremote gdrive_acknowledge_abuse true`
- **Note**: If you set this, you should use the latest `dvc` version. Older `dvc` versions will start throwing strange errors. Also make sure you have installed `pip install dvc[gdrive]`.

## Problem 3

```bash
ERROR: configuration error - config file error: extra keys not allowed @ data['remote']['galaxisgdrive']['drive_acknowledge_abuse']
```

Solution:
- typo, you forgot "`g`" in front of "`drive_acknowledge_abuse`": use `['gdrive_acknowledge_abuse']` instead of `['drive_acknowledge_abuse']` 

## Problem 4

```bash
(env) bra-ket@braket-pc:~/git/Galaxis/object-detection-2023/training_code$ dvc list . data/
ERROR: failed to list '.' - Current operation was unsuccessful because '/home/bra-ket/git/Galaxis/object-detection-2023/training_code/test_dvc/dvc-det-2023/models/10feb23_v3tiny' requires existing cache on 'local' remote. See <https://man.dvc.org/config#cache> for information on how to set up remote cache.
```

Solution:
- You are probably not in a DVC repo. Check, if you are in a DVC repo.

## Problem 5

```bash
(env) bra-ket@braket-pc:~/git/Galaxis/object-detection-2023/dvc-det-2023/data$ dvc list . data/
ERROR: unexpected error - [Errno 2] No such file or directory: '/home/bra-ket/git/Galaxis/object-detection-2023/dvc-det-2023/data/data'
```

Solution:
- `data/` does not exist in the current directory. Run `dvc list . .` to list the contents of the current directory.
