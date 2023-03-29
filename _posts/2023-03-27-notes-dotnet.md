---
title: ".NET Notes"
read_time: false
excerpt: ".NET Basics"
toc: true
toc_sticky: true
categories:
  - Notes
tags:
  - dotnet
  - notes
---

# .NET 5

## Install

source: [microsoft.com](https://learn.microsoft.com/en-us/dotnet/core/install/linux-ubuntu-2004)

Add the package repository:

```bash
wget https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
rm packages-microsoft-prod.deb
```

SDK:

```bash
sudo apt-get update && \
  sudo apt-get install -y dotnet-sdk-7.0
```

Runtime:

```bash
sudo apt-get update && \
  sudo apt-get install -y aspnetcore-runtime-7.0
```

# ASP.NET Core

see [Getting Started](https://learn.microsoft.com/en-us/aspnet/core/getting-started/?view=aspnetcore-7.0&tabs=linux)
(how to create and view a template website)
