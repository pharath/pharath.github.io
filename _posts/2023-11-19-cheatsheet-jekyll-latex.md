---
title: "Latex in Jekyll Cheatsheet"
read_time: false
excerpt: "Some essential Jekyll, Latex and Ruby commands and snippets"
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - latex
  - jekyll
  - cheatsheet
---

# Ruby

[Ruby vs Ruby on Rails](https://www.netguru.com/blog/ruby-versus-ruby-on-rails)
- Rails: 
  - a **web application development framework** and 
  - is based on the **Model View Controller (MVC)** architecture
  - Ruby and Ruby on Rails sound similar because **Ruby on Rails was programmed in Ruby** and then released to the public in 2005, 10 years after Ruby.
  - Ruby on Rails is mostly used for 
    - building web applications and 
    - server side scripting, 
    - but it can also be used to develop interface scripts.
- Ruby: 
  - an open source, **object oriented scripting language** designed with simplicity as a core focus, and is known for its ability to build online applications quickly.
  - Ruby was **written in C language**
  - very popular choice amongst web application development professionals
  - can also be used on the **back end** for data science and memory management purposes

## gem

- [What is a gem?](https://guides.rubygems.org/what-is-a-gem/)
- like "packages" in Python
- can be installed with `bundle`

| command | description |
| :---: | :---: |
`gem list` | list gems
`gem query --local` | list all locally installed gems

## bundle

- like `apt` and `pip`, but for Ruby

| command | description |
| :---: | :---: |
`bundle install` | install all missing gems
`bundle info [gemname]` | to see where a bundled gem is installed

# Jekyll

- **Jekyll** is a gem.

Jekyll is written in Ruby.
- [Ruby 101](https://jekyllrb.com/docs/ruby-101/) explains:
  - **Gems**
    - Gems are code you can include in Ruby projects
    - [Ruby gems search](https://rubygems.org/)
    - [Ruby gems list](https://rubygems.org/gems)
  - **Jekyll**
    - **Jekyll** is a gem.
    - Many **Jekyll plugins** are also gems, including jekyll-feed, jekyll-seo-tag and jekyll-archives.
  - **Gemfile**
    - a list of gems used by your site. Every Jekyll site has a Gemfile in the main folder.
  - **Bundler**
    - Bundler is a gem that installs all gems in your `Gemfile`.

## Install

| command | description |
| :---: | :---: |
`sudo apt-get install ruby-full` |
`sudo apt install rubygems-integration` | nicht nötig, wenn ruby-full installiert wurde
`sudo gem install bundler` |
`sudo gem install jekyll` | Seite funktioniert nicht richtig, wenn dieses Gem **nicht** installiert wurde (z.B. bei Seitenvergrößerung über ctrl - + werden die einzelnen Teile der Seite nicht automatisch ausgerichtet)! 
`sudo bundle install` | im github-pages repo **root** folder
`bundle exec jekyll serve` | warten bis "Server running... press ctrl-c to stop." message und dann ctrl gedrückt halten und auf server address clicken (oder in Browser "http://localhost:4000" aufrufen)

## Create a new site

To create a new Jekyll site at `./myblog`:

```bash
// Create a new Jekyll site
jekyll new myblog

cd myblog

// Build the site and make it available on a local server
bundle exec jekyll serve
```

## Linking to posts

see [Jekyll documentation](https://jekyllrb.com/docs/liquid/tags/#linking-to-posts)

E.g. [this link]({% post_url 2022-09-22-notes-OS %}) is a link to the post "Operating Systems Notes".

## Troubleshooting

### Markdown Syntax

#### Code Blocks

```bash
Liquid Exception: Liquid syntax error (line 194): Variable 'double-{-without-whitespace' was not properly terminated with regexp: /\}\}/ in /home/bra-ket/git/pharath.github.io/_posts/2022-09-22-notes-c.md
             Error: Liquid syntax error (line 194): Variable 'double-{-without-whitespace' was not properly terminated with regexp: /\}\}/
             Error: Run jekyll build --trace for more information.
```
- do **not** use \{\{, use \{ \{, i.e. there **must** be a whitespace between the two curly brackets!

# Latex in Jekyll

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

- formula with cases

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

- non-standalone symbols (a symbol changing another symbol)

```
\overline{A}    # instead of "\bar"
```

- absolute value, norm

```
\vert A\vert
# versus:
\Vert A\Vert
```

# Github Pages

- **github-pages** is a gem.
- make sure you have the same github-pages gem version installed locally as github.com (check with `bundle list | grep github-pages`)
