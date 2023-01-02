---
title: "Machine Learning (Part 4)"
excerpt: "Notes on Computer Vision theory and NLP. Based on Goodfellow, Bengio \"Deep Learning\", Stanford CS231n and RWTH Aachen University Machine Learning"
classes: wide
header:
  teaser: /assets/images/lenet.png
  overlay_image: /assets/images/lenet.png
  overlay_filter: 0.6
  caption: "Photo credit: [**Yann LeCun**](http://yann.lecun.com/)"
  actions:
    - label: "Some Content"
[//]: # (      url: "https://htmlpreview.github.io/?https://github.com/pharath/home/blob/master/_posts_html/2021-09-23-Databases.html")
categories:
  - Notes
tags:
  - lecture_notes
  - notes
  - ml
  - cv
toc: true
toc_label: "Contents"

---

# Transformer

## Scaled Dot-Product Attention

- watch: [Kilcher: Attention is all you need](https://www.youtube.com/watch?v=iDulhoQ2pro&t=17s)
- keys K, values V and queries Q
- **encoder**: 
    - e.g. in translation: takes the source sentence as **input**
    - Kilcher: selects ("queries") one of the keys $k$ by doing a dot product $q \cdot k$ with all keys $k$ and then applying a $\text{softmax}\(\frac{K \cdot q}{\sqrt{d}}\)$, where $K$ is the matrix of keys $k$ and $d$ is the length of $q$
    - **more precisely**: the softmax over the dot products gives us a **weighting factor** $w_i$ for each value in $V$, or to quote the [original paper](https://arxiv.org/pdf/1706.03762.pdf), the softmax "selects the weights on the values"
    - in practice, all "queries" are done in parallel, therefore the keys, queries and values are packed together into matrices $K$, $Q$ and $V$. Then, the dot product becomes $Q \cdot K^\top$.
- **decoder**: 
    - e.g. in translation: takes the target sentence that we produced so far as **input**
    - Self-Attention Block:
        - Kilcher: selects the value in vector $V$ corresponding to the selected ("queried") key $k$ from the encoder
        - **more precisely**: weighs each value in $V$ by its corresponding weighting factor $w_i$ calculated by the encoder
- **encoder-decoder**: 
    - [Masked Multi-Head Self-Attention Block](#masked-self-attention) (see below)
- **output**:
    - next word (more precisely: probability distribution over possible next words, where we pick the word with highest probability)

## Masked Self-Attention

- watch: [YouTube: masked self-attention](https://www.youtube.com/watch?v=piT1_k8b9uM&list=PLDw5cZwIToCvXLVY2bSqt7F2gu8y-Rqje&index=7)
- masked softmax
    - "masks" certain weights in the weight matrix W, so that at training time the self-attention block cannot "cheat" by looking at **future words** in the sequence. It must look at **previous words** only! Therefore, we **mask** (i.e. set to 0) the weights corresponding to the future words.
- multi-head (see [original paper: section "Multi-Head Attention": formula for 'MultiHead()'](https://arxiv.org/pdf/1706.03762.pdf))
    - in practice, multiple masked softmaxes are computed in parallel
    - instead of $Attention\(Q,K,V\)$ they compute $Attention\(Q W^Q,K W^K,V W^V\)$
    - then, the resulting features are concatenated ("stacked on top of each other")
    - finally, to make the output size be **the same as the input size** the concatenated features are multiplied by a **learnable** matrix $W^O$ (output projection matrix)

## Add & Norm Layer

- watch: [Encoder: at this time point](https://youtu.be/SsLzwRXH0UI?list=PLDw5cZwIToCvXLVY2bSqt7F2gu8y-Rqje&t=590)
