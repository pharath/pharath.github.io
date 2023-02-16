---
title: "Docker Notes"
read_time: false
excerpt_separator: "<!--more-->"
toc: true
toc_sticky: true
categories:
  - Notes
tags:
  - docker
  - notes
---

# docker

## Compare images using `docker history`

```bash
# compare tag-1 and tag-2
docker history nvcr.io/nvidia/deepstream:tag-1 > hist_tag-1.txt
docker history nvcr.io/nvidia/deepstream:tag-2 > hist_tag-2.txt
vimdiff hist_tag-1.txt hist_tag-2.txt
# in vim press ctrl + w J to get horizonally split viewports
```

## Location on System

| command | description |
| :---: | :---: |
sudo ls /var/lib/docker/overlay2 | hier ist der Großteil aller docker image Daten
sudo du -sh $(ls /var/lib/docker/) | list size of all files and dirs in /var/lib/docker/

## X11 Forwarding

| :---: | :---: |
xhost + | enable GUI for docker
xhost +local:root |	enable GUI for docker

## docker login

| :---: | :---: |
docker login registry.git.rwth-aachen.de | do not forget to logout !
docker pull |

## docker logout

| :---: | :---: |
docker logout registry.git.rwth-aachen.de | 

## Images/Storage Info

| :---: | :---: |
sudo docker ps -a | -a flag: Show all containers (default shows just running)
sudo docker images | show all images
sudo docker system df | Show docker disk usage (size of all images together)

## Free up Storage

See
- `docker builder prune` (see [builder](#builder))
- [Remove dangling Images](#remove-dangling-images)

## docker commit

| :---: | :---: |
`sudo docker commit 308aeb468339 tensorflow/tensorflow:latest-gpu-jupyter_braket` | [Schritte](https://stackoverflow.com/a/64532554), i.e. `docker commit CONTAINER_ID NEW_IMAGE_NAME`
`docker commit -m "added test file" eloquent_lehmann` | commit with commit message
`docker history <image hash>` | view commit messages

## remove

| :---: | :---: |
sudo docker image rm 1ff0175b5104 | remove image with id 1ff0175b5104 
sudo docker rmi 1ff0175b5104 | alias for `docker image rm` [source](https://stackoverflow.com/a/63035352), see also [doc](http://manpages.ubuntu.com/manpages/bionic/man1/docker-rmi.1.html)
sudo docker rmi "image with more than 1 tag" | If your image is tagged with more than one tag, then `docker rmi` will remove the tag, but not the image.

## container

| :---: | :---: |
sudo docker container ls -a |
docker container inspect container_id | zeige container info (u.a. **Bindings** [= Ordner, deren Inhalte host und container sharen])
sudo docker container stop 1ff0175b5104 | stoppt den container nur (dh. container Status: "Exited"), aber `docker ps -a` zeigt den container noch!
sudo docker container rm 1ff0175b5104 | entfernt den container, dh. "docker ps -a" zeigt den container nicht mehr
sudo docker container kill 1ff0175b5104 | killt den container (Unterschied zu `docker container stop`: see [here](https://stackoverflow.com/a/66736836): "So ideally we always stop a container with the `docker stop` command in order to get the running process inside of it a little bit of time to shut itself down, otherwise if it feels like the container has locked up and it's not responding to the docker stop command then we could issue `docker kill` instead.")

## run

| :---: | :---: |
sudo docker run -d ... | start a container in detached mode [docs](https://docs.docker.com/engine/reference/run/#detached--d)
sudo docker run --rm ... | Automatically remove the container when it exits
docker run --name test -it *image_name* | This example runs a container named test using the image *image_name*. The `-it` instructs Docker to allocate a pseudo-TTY connected to the container's stdin; creating an interactive bash shell in the container.
docker run --rm --name ubuntu_phth -it --entrypoint=/bin/bash deep_braket:v4 | start deep_braket:v4 in bash shell instead of starting in Jupyter Lab.
docker run -e "TERM=xterm-256color" ... | enable color output in docker bash terminal

## exec

| :---: | :---: |
sudo docker exec -it 6b594d9d60cc bash | start bash in container 6b594d9d60cc 

## build

| :---: | :---: |
sudo docker build --no-cache -t deep\_braket:v1 . | `-t`: REPO name and TAG name of image; `--no-cache`: [explanation](https://stackoverflow.com/a/35595021), ohne diesen flag wird Layer Caching benutzt (image updated die alte image-Version sozusagen nur und hat dependencies zur alten image-Version; die alte image-Version kann also nicht gelöscht werden!); `.`: location of Dockerfile

## builder

| :---: | :---: |
docker builder prune | Remove build cache (phth: e.g. to free up space when using `docker compose` repeatedly) ([doc](https://docs.docker.com/engine/reference/commandline/builder_prune/))

## compose

| :---: | :---: |
docker compose build *SomeServiceName* | Build or rebuild services ([doc](https://docs.docker.com/engine/reference/commandline/compose_build/))
docker compose build *SomeServiceName* | Build or rebuild services ([doc](https://docs.docker.com/engine/reference/commandline/compose_build/))
docker compose build --no-cache *SomeServiceName* | Do not use cache when building the image

A `docker-compose.yaml` example:
```yaml
version: '3.8'
services:
  base:
    image: kitcar-sim:base
    build:
      context: ../
      dockerfile: docker/Dockerfile
  cml:
    image: ${IMAGE_URL}/ci:${CI_IMAGE_TAG_CML}
    depends_on:
    - base
    build:
      context: ../
      dockerfile: docker/DockerfileCML
      args:
        PARENT: ${IMAGE_URL}/ci:${CI_IMAGE_TAG}
  kitcar-ros:
    image: kitcar-sim:kitcar-ros
    depends_on:
    - base
    build:
      context: ../
      dockerfile: docker/DockerfileROS
      args:
        PARENT: kitcar-sim:base
```

## top

| :---: | :---: |
sudo docker top 6b594d9d60cc | see all processes (incl. pids) in container 6b594d9d60cc 

## attach/detach

| :---: | :---: |
docker attach *double-tab* | attach to running container (double-tab shows names of running containers or use container id)
ctrl-p ctrl-q | detach from container

## volume

| :---: | :---: |
[docker volume overview](https://www.baeldung.com/ops/docker-volumes) | 
docker volume create data_volume_name |
docker volume ls |
docker volume inspect volume_hash |
docker volume rm data_volume_name | remove one or more volumes individually
docker volume prune | remove all the unused volumes
docker run -v data-volume:/var/opt/project bash:latest bash -c "ls /var/opt/project" | start a container with a volume using the -v option. The -v option contains three components, separated by colons: 1. Source directory or volume name, 2. Mount point within the container, 3. (Optional) *ro* if the mount is to be read-only

## Remove dangling images

- **Dangling images** entstehen, wenn man ein neues image committet, das den Namen eines bereits existierenden images hat.
In `docker images` wird das alte image dann \<none\> genannt (sowohl REPOSITORY als auch TAG) [source](https://stackoverflow.com/a/40791752)

| command | description |
| :---: | :---: |
docker images --filter dangling=true | lists all images that are dangling and has no pointer to it
docker rmi `` `docker images --filter dangling=true -q` `` | Removes all those images.

## Gitlab Container Registry

| command | description |
| :---: | :---: |
docker login registry.git.rwth-aachen.de | login to Container Registry
docker image tag galaxis_simulation:phth-8 registry.git.rwth-aachen.de/pharath/gitlab_backups/galaxis_simulation:phth-8 | tag local image "galaxis_simulation:phth-8" (Note: the tag registry.git.rwth-aachen.de/pharath/gitlab_backups/galaxis_simulation:phth-8 must have this form!)
docker tag galaxis_simulation:phth-8 registry.git.rwth-aachen.de/pharath/gitlab_backups/galaxis_simulation:phth-8 | see `docker image tag`
docker push registry.git.rwth-aachen.de/pharath/gitlab_backups/galaxis_simulation:phth-8 | push image to Gitlab Container Registry
docker logout registry.git.rwth-aachen.de | logout

## Check if GUIs work in a Linux container

```bash
sudo apt update
sudo apt install x11-utils
xmessage -center hello!
```

# Welcher CMD wird per default beim image Start ausgeführt? 

Klicke auf den letzten der layers in der Liste links. Dann erscheint rechts der zugehörige vollständige CMD.

![image start CMD default config](https://i.ibb.co/QcRnxP8/Screenshot-from-2021-08-16-04-53-12.png)

# Beispiele

Run osrf/ros image with GUI support:
1. `xhost +local:root`
2. `docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix osrf/ros:eloquent-desktop`  

Start new bash shell in running container:
1. `xhost +local:root`
2. `docker exec -it <tab-tab>container bash` 


Start new bash shell in running container with color support:
1. `xhost +local:root`
2. `docker exec -it <tab-tab>container env TERM=xterm-256color bash`

# Webcam

- first enable x11 forwarding: `xhost +`
- `docker run --ipc=host --device=/dev/video0`
- Check the webcam: `sudo mplayer tv://device=/dev/video0`
    - or: `sudo mplayer tv:// -tv driver=v4l2:device=/dev/video0`

# Adding Users

- see [https://stackoverflow.com/a/49848507](https://stackoverflow.com/a/49848507)

# Check OS in container

- `lsb_release -sirc`
- `cat /etc/os-release`

# Troubleshooting

## Running GUI apps in a Linux container

```bash
$ sudo ./run.sh
xauth: (argv):1:  unable to read any entries from file "(stdin)"
```

**Solution**:
das Problem kann folgende Ursachen haben:
- es gibt keinen `/tmp/.docker.xauth`
- es gibt nur einen **FOLDER** `/tmp/.docker.xauth/` aber keinen **FILE** `/tmp/.docker.xauth`

From (vgl: [riptutorial](https://riptutorial.com/docker/example/21831/running-gui-apps-in-a-linux-container)):  
1. entferne **FOLDER** `/tmp/.docker.xauth/`, falls es existiert 
    - (**ACHTUNG**: falls ein gleichnamiger **FILE** `/tmp/.docker.xauth` (ohne "`/`" hinten) existiert, diesen FILE nicht entfernen)
2. `xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -` (creates `/tmp/.docker.xauth`)
3. nachschauen, ob `/tmp/.docker.xauth` kreiert wurde

```bash
$ ./run.sh
xauth:  /tmp/.docker.xauth not writable, changes will be ignored
```

**Solution**:
führe stattdessen `sudo ./run.sh` aus.
