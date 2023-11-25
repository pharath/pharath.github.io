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

# Tags

## Division - div

[Division](https://www.w3schools.com/tags/tag_div.ASP):
- `<div>`: division
- The `<div>` tag defines a division or a section in an HTML document.
- The `<div>` tag is used as a container for HTML elements - which is then styled with CSS or manipulated with JavaScript.
- The `<div>` tag is easily styled by using the class or id attribute.
- Any sort of content can be put inside the `<div>` tag! 
- **Note**: By default, browsers always place a line break before and after the `<div>` element.

## Description List - dl

[Description List](https://www.w3schools.com/tags/tag_dl.asp):
- `<dl>`: description list
- `<dt>`: description term/name
- `<dd>`: description

### Example: Firefox bookmarks

To indent firefox `.bookmarks` add this CSS style: 
- note: the `<meta>` tags must come **after** the `<style>` tags
  - otherwise the `Content-Security-Policy` (CSP) in the HTML head will prevent **all** CSS code in the HTML document from loading
    - however, you can still allow some CSS code, see examples in [CSP: style-src](https://developer.mozilla.org/en-US/docs/Web/HTTP/Headers/Content-Security-Policy/style-src)

```html
<head>
  <style type="text/css">
    dl dl {
      margin-left:0.5in;
      }
  </style>
  <meta>
    ...
  </meta>
</head>
<body>
  ...
</body>
```

## Horizontal Line - hr

[Horizontal Rule](https://www.w3schools.com/tags/tag_hr.asp)
- `<hr>`: horizontal rule
- thematic break in an HTML page (e.g. a shift of topic)
- used to separate content (or define a change)
- `<hr style="height:2px;border-width:0;color:gray;background-color:gray">`

## Paragraphs - p

Attributes: 
- id: `<p id="demo"></p>`, [explained]( https://www.dofactory.com/html/p/id )
  - assigns an identifier to the paragraph.
  - The identifier must be unique across the page.

## Anchor Points - a

from: [superuser](https://superuser.com/a/382083)

The vanilla way link to somewhere in-page is via an [anchor point](https://www.w3.org/TR/html4/struct/links.html#h-12.2) already present in the page.

This can be created using the `<a>…</a>` tag. Note that the link specified in "anchor point" (above) has `#h-12.2` at the end. This corresponds to `<a id="h-12.2">12.2</a>` embedded in the HTML forming the page, and when clicked will reposition the page view to this anchor.

Note that prior to HTML5, the name attribute was used in the anchor tag, but is no longer supported and the id attribute should be used in its place ([reference](https://www.w3schools.com/tags/tag_a.asp)). This also means that you can use any element for an anchor tag, you are not limited to the `<a>` element.

### Beispiel

"https://github.com/infokiller/web-search-navigator#keybindings" springt direkt auf **anchor point** "keybindings", der folgendermaßen aussieht:

```html
<a id="user-content-keybindings" class="anchor" aria-hidden="true" href="#keybindings">
	<svg class="octicon octicon-link" viewBox="0 0 16 16" version="1.1" width="16" height="16" aria-hidden="true">
		<path fill-rule="evenodd" d="M7.775 3.275a.75.75 0 001.06 1.06l1.25-1.25a2 2 0 112.83 2.83l-2.5 2.5a2 2 0 01-2.83 0 .75.75 0 00-1.06 1.06 3.5 3.5 0 004.95 0l2.5-2.5a3.5 3.5 0 00-4.95-4.95l-1.25 1.25zm-4.69 9.64a2 2 0 010-2.83l2.5-2.5a2 2 0 012.83 0 .75.75 0 001.06-1.06 3.5 3.5 0 00-4.95 0l-2.5 2.5a3.5 3.5 0 004.95 4.95l1.25-1.25a.75.75 0 00-1.06-1.06l-1.25 1.25a2 2 0 01-2.83 0z"></path>
	</svg>
</a>
```

## Inline Frame - iframe

An inline frame (iframe) is a HTML element that loads another HTML page within the document. It essentially puts another webpage within the parent page. They are commonly used for advertisements, embedded videos, web analytics and interactive content. [source](https://www.techtarget.com/whatis/definition/IFrame-Inline-Frame)

"`<iframe>` tag specifies an inline frame. An inline frame is used to embed another document within the current HTML document." [w3schools](https://www.w3schools.com/tags/tag_iframe.ASP)

## Button - button

### AccessKey

Keymap to activate access keys depends on the **Web Browser** and on the **OS**. In **Firefox** (on Windows and Linux) use <kbd>alt</kbd> + <kbd>shift</kbd> + <kbd>key</kbd>, [developer.mozilla.org](https://developer.mozilla.org/en-US/docs/Web/HTML/Global_attributes/accesskey)

```html
<button accesskey="s">Stress reliever</button>
```
