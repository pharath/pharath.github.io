---
title: "Next.js Notes"
read_time: false
excerpt: "Next.js Basics"
toc: true
toc_sticky: true
categories:
  - Notes
tags:
  - nextjs
  - notes
---

# Next.js

Full-stack React framework.

[Wikipedia](https://en.wikipedia.org/wiki/Next.js)
- an open-source web development framework created by the private company Vercel providing React-based web applications with server-side rendering and static website generation.
- React documentation mentions Next.js among "Recommended Toolchains" advising it to developers when "building a server-rendered website with Node.js". 
- Where traditional React apps can only render their content in the **client-side** browser, Next.js extends this functionality to include applications rendered on the **server-side**. 

# NextAuth.js

# Deploy

## AWS

### SST

[github](https://github.com/serverless-stack/sst)

[docs.sst](https://docs.sst.dev/what-is-sst)
- SST is a framework that makes it easy to build modern full-stack applications **on AWS**.
- frontend:
  - **Deploy** a serverless **Next.js**, Remix, Astro, or Solid app **to your AWS account** and add any backend feature to it.
- backend:
  - SST has constructs for most backend features. And you can even **use any AWS service** in your app.

[Tutorials](https://sst.dev/guide.html#table-of-contents)
- [Overview of the tutorial's demo app](https://sst.dev/chapters/what-does-this-guide-cover.html)
- build the backend
  - [serverless](https://sst.dev/chapters/what-is-serverless.html)
    - **serverless (aka FaaS)**: Serverless computing (or serverless for short), is an execution model where the cloud provider (AWS, Azure, or Google Cloud) is responsible for executing a piece of code by dynamically allocating the resources. And only charging for the amount of resources used to run the code. 
    - **Events**: The code is typically run inside **stateless containers** that can be **triggered by** a variety of **events** including 
      - http requests, 
      - database events, 
      - queuing services, 
      - monitoring alerts, 
      - file uploads, 
      - scheduled events (cron jobs), 
      - etc. 
    - **FaaS**: The code that is sent to the cloud provider for execution is usually in the form of a **function**. Hence serverless is sometimes referred to as "*Functions as a Service*" or "*FaaS*". 
    - **Major offerings**: Following are the **FaaS offerings** of the **major cloud providers**:
      - AWS: AWS Lambda
      - Microsoft Azure: Azure Functions
      - Google Cloud: Cloud Functions
- build the frontend
  - [Setting up a React app](https://sst.dev/chapters/create-a-new-reactjs-app.html) (uses `create-react-app`)
  - [Login page](https://sst.dev/chapters/create-a-login-page.html)

# Build Tools

- old school:
  - Webpack
- a bit newer:
  - Parcel
- the future (faster, less bloated):
  - Vite
  - Snowpack
