---
title: "Latex in Jekyll Cheatsheet"
read_time: false
excerpt: "Some essential Latex commands and snippets"
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - latex
  - jekyll
  - cheatsheet
---

# Latex

- brackets

```
# parentheses (brit. round brackets):
\(\)

# braces (brit. curly brackets):
## either
\\{\\}
## or
\{\}

# brackets (brit. square brackets):
\[\]

# sizes:
\bigr)
## either
\bigl\{
## or
\bigl\\{
```

- spaces

```
\<space>
\,
\quad
```

- cases

```
# works only inside $$$$
$$
\begin{equation*}
X(\omega) = \begin{cases}
1 &\text{se $\omega\in A$}\\
1250 &\text{se $\omega \in A^c$}
\end{cases}
\end{equation*}
$$
```

- symbols

```
\circ   # degree, angle
```
