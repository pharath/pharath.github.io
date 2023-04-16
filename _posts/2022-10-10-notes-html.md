---
title: "HTML Notes"
read_time: false
excerpt: "For learning html."
header:
    teaser: /assets/images/mario-question-block.jpeg
    overlay_image: /assets/images/mario-question-block.jpeg
    overlay_filter: 0.5 
toc: true
toc_label: "Contents"
toc_sticky: true
categories:
    - Notes
tags:
    - html
    - notes

---

# Terminology

- **Markup language**: a text-encoding system consisting of a set of symbols inserted in a text document to control its structure, formatting, or the relationship between its parts. [Wikipedia](https://en.wikipedia.org/wiki/Markup_language)
    - Markup is often used to control the display of the document or to enrich its content to facilitating automated processing.
    - e.g. HTML (HyperText Markup Language), TeX, LaTeX, XML

# Jekyll

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
sudo apt-get install ruby-full |
sudo apt install rubygems-integration | nicht nötig, wenn ruby-full installiert wurde
sudo gem install bundler |
sudo gem install jekyll | Seite funktioniert nicht richtig, wenn dieses Gem **nicht** installiert wurde (z.B. bei Seitenvergrößerung über ctrl - + werden die einzelnen Teile der Seite nicht automatisch ausgerichtet)! 
sudo bundle install | im github-pages repo **root** folder
bundle exec jekyll serve | warten bis "Server running... press ctrl-c to stop." message und dann ctrl gedrückt halten und auf server address clicken (oder in Browser "http://localhost:4000" aufrufen)

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

# `<p>` tag

Attributes: 
- id: `<p id="demo"></p>`, [explained]( https://www.dofactory.com/html/p/id )
    - assigns an identifier to the paragraph.
    - The identifier must be unique across the page.

# Anchor Points

from: [superuser](https://superuser.com/a/382083)

The vanilla way link to somewhere in-page is via an [anchor point](https://www.w3.org/TR/html4/struct/links.html#h-12.2) already present in the page.

This can be created using the `<a>…</a>` tag. Note that the link specified in "anchor point" (above) has `#h-12.2` at the end. This corresponds to `<a id="h-12.2">12.2</a>` embedded in the HTML forming the page, and when clicked will reposition the page view to this anchor.

Note that prior to HTML5, the name attribute was used in the anchor tag, but is no longer supported and the id attribute should be used in its place ([reference](https://www.w3schools.com/tags/tag_a.asp)). This also means that you can use any element for an anchor tag, you are not limited to the `<a>` element.

## Beispiel

"https://github.com/infokiller/web-search-navigator#keybindings" springt direkt auf **anchor point** "keybindings", der folgendermaßen aussieht:

```html
<a id="user-content-keybindings" class="anchor" aria-hidden="true" href="#keybindings">
	<svg class="octicon octicon-link" viewBox="0 0 16 16" version="1.1" width="16" height="16" aria-hidden="true">
		<path fill-rule="evenodd" d="M7.775 3.275a.75.75 0 001.06 1.06l1.25-1.25a2 2 0 112.83 2.83l-2.5 2.5a2 2 0 01-2.83 0 .75.75 0 00-1.06 1.06 3.5 3.5 0 004.95 0l2.5-2.5a3.5 3.5 0 00-4.95-4.95l-1.25 1.25zm-4.69 9.64a2 2 0 010-2.83l2.5-2.5a2 2 0 012.83 0 .75.75 0 001.06-1.06 3.5 3.5 0 00-4.95 0l-2.5 2.5a3.5 3.5 0 004.95 4.95l1.25-1.25a.75.75 0 00-1.06-1.06l-1.25 1.25a2 2 0 01-2.83 0z"></path>
	</svg>
</a>
```

# iframe

An inline frame (iframe) is a HTML element that loads another HTML page within the document. It essentially puts another webpage within the parent page. They are commonly used for advertisements, embedded videos, web analytics and interactive content. [source](https://www.techtarget.com/whatis/definition/IFrame-Inline-Frame)

"`<iframe>` tag specifies an inline frame. An inline frame is used to embed another document within the current HTML document." [w3schools](https://www.w3schools.com/tags/tag_iframe.ASP)


