---
title: "HPC Cheatsheet"
read_time: false
excerpt: "Some essential HPC commands"
header:
  teaser: /assets/images/linux_teaser.jpg
  overlay_image: /assets/images/linux_teaser.jpg
  overlay_filter: 0.5 
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - hpc
  - cheatsheet
---

# HPC

HPC Cluster

(ausführliche Dokumentation: [Link](https://capi.sabio.itc.rwth-aachen.de/documents/0962a984657bb2e901657bc9e2810004/1jifm42ljvvhw/c010e5a1bed64d79aa9dd1fc62838bb0/primer-8.4.0.pdf))

## Python

Setup Routine für jedes Mal bevor Python 3 benutzt wird:

```bash
module switch intel gcc
export PYTHONPATH=$PYTHONPATH:$HOME/SomeDirForPythonInstall/lib/python3.9.1/site-packages
module load python/3.9.1
module list
```

aus S. 32 in Dok:

```bash
man module

module avail # list all available modules

module list # list currently loaded module files

module help python/3.9.1 # Manual zu modul ‘python/3.9.1’

module unload modulename # NICHT BENUTZEN ! (stattdessen “module switch” s.u.)
```

If you want to use another version of a software (e.g., another compiler), we strongly recommend switching between modules:

```bash
module switch oldmodule newmodule
```

- Install python packages in `$HOME/SomeDirForPythonInstall`:
  - Python software often rely on GCC compilers and often do not work (out of the box) with Intel compilers; thus type `module switch intel gcc` before adding any Py-Modules.

```bash
$ module switch intel gcc
$ export PYTHONPATH=$PYTHONPATH:$HOME/SomeDirForPythonInstall/lib/python3.9.1/site-packages
$ pip3 install numpy
```

## SLURM on HPC

- SLURM is a workload manager / job scheduler.
  - [hpc-wiki](https://hpc-wiki.info/hpc/SLURM)
  - [RWTH: Basic Slurm compute batch job](https://help.itc.rwth-aachen.de/en/service/rhr4fjjutttf/article/13ace46cfbb84e92a64c1361e0e4c104/)
  - [RWTH: Slurm Commands](https://help.itc.rwth-aachen.de/en/service/rhr4fjjutttf/article/3d20a87835db4569ad9094d91874e2b4/)

```bash
top # show running processes
sbatch jobscript.sh # submitting a jupyter-notebook server as a slurm batch job
squeue -u$(whoami) # show currently running jobs
scancel JOBID # Jupyter Server schließen
```

```bash
nvidia-smi # GPU Usage Info
```

## Jupyter Lab on HPC

Quelle: [https://ljvmiranda921.github.io/notebook/2018/01/31/running-a-jupyter-notebook/](https://ljvmiranda921.github.io/notebook/2018/01/31/running-a-jupyter-notebook/)

```bash
export PYTHONPATH=$PYTHONPATH:$HOME/SomeDirForPythonInstall/lib/python3.9.1/site-packages
module switch intel gcc
module load python/3.9.1
pip3 install tensorflow
export PATH=/home/no411606/.local/bin:$PATH
pip3 install jupyterlab
```

```bash
# auf HPC server: 
jupyter lab --no-browser --port=8889
# auf meinem PC:
# 1. 
ssh -N -f -L localhost:8888:localhost:8889 -l no411606 login-g.hpc.itc.rwth-aachen.de (nur ausführen. Dieser command gibt keinen Output im Terminal !)
# 2. im Browser auf meinem PC:
# 2.1. localhost:8888 in der Adresszeile eingeben und starten
# 2.2. Als Passwort im Browser den Token aus der jupyterlab URL (im Output zum command “jupyter lab --no-browser --port=8889”) im HPC Terminal rauskopieren und einfügen
# 2.3. Passwort/Token bestätigen und jupyterlab startet.
# 3. Beim beenden: auf meinem PC: mit 
sudo netstat -lpn | grep :8888 
# pid suchen und mit 
kill some_pid # pid steht in der Spalte ganz rechts, zB. in "1239526/ssh" ist die pid vor dem slash
# beenden. Denn port auf HPC Seite (hier: 8889) wird automatisch frei beim Beenden von jupyter, aber port auf meinem PC (hier: 8888) nicht!
```

**Achtung**: Schau, dass die ports frei sind (mit `(sudo) netstat -lpn | grep :8889` die PID suchen und mit `kill ABCDEF` port frei machen)! Sonst anderen port probieren.
