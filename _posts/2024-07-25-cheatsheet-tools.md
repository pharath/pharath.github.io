---
title: "Tools Cheatsheet"
read_time: false
excerpt: "Cheatsheet for some handy tools."
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - tools
  - cheatsheet
---

# yt-dlp

- if the `path/to/destination` does not exist, it will be created automatically

```bash
yt-dlp -o "path/to/destination/%(title)s.%(ext)s" link
yt-dlp -o "path/to/destination/%(title)s.%(ext)s" link1 link2 link3

# selection
yt-dlp --flat-playlist link   # only list all available videos
yt-dlp -o "path/to/destination/%(title)s.%(ext)s" -I 1:40 link   # download video 1 to video 40

# quality
yt-dlp -F link   # the 'ID' field contains all available qualities for a video
yt-dlp -o "path/to/destination/%(title)s.%(ext)s" -f 720p link   # '-f someQuality': has to be one of the qualities shown by 'yt-dlp -F link'
```


