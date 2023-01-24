---
title: "CLIP: Learning Transferable Visual Models From Natural Language Supervision"
excerpt: "RWTH Computer Vision Seminar \"Current Topics in Computer Vision and Machine Learning\"."
classes: wide
header:
  teaser: /assets/images/clippretraining.svg
  overlay_image: /assets/images/clippretraining.svg
  overlay_filter: 0.6
  actions:
    - label: "Download the Seminar Report"
      url: "https://raw.githubusercontent.com/pharath/home/master/assets/pdfs/seminar_report_topic_05_CLIP_fourth.pdf"
categories:
  - computervision
tags:
  - seminar
  - computervision

---

# Abstract 

Although using natural language supervision for image representation learning has been rarely studied in the past due to the low performance on common benchmarks, <cite>[Radford et al. 2021][1]</cite> demonstrated that contrastive language-image pre-training is indeed an efficient and scalable way to learn competitive image representations from scratch that perform well on a wide range of tasks. This seminar report discusses the rationale behind the model CLIP and the authors' tests and findings. In particular, it discusses the core aspects of the CLIP pre-training framework, as well as its robustness to distribution shift and the quality of the learned representations. In addition, it explains many of the ideas of other authors which inspired CLIP.


# References

[1]: Alec Radford, Jong Wook Kim, Chris Hallacy, Aditya Ramesh, Gabriel Goh, Sandhini Agarwal, Girish Sastry, Amanda Askell, Pamela Mishkin, Jack Clark, Gretchen Krueger, and Ilya Sutskever. Learning transferable visual models from natural language supervision. In Marina Meila and Tong Zhang, editors, *Proceedings of the 38th International Conference on Machine Learning*, volume 139 of *Proceedings of Machine Learning Research*, pages 8748–8763. PMLR, 18–24 Jul 2021. [view on arxiv](https://arxiv.org/abs/2103.00020)
