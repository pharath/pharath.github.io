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

## Linear Projection

- see [original paper: section "Multi-Head Attention": formula for 'MultiHead()'](https://arxiv.org/pdf/1706.03762.pdf)
- instead of computing $\text{Attention}\(Q,K,V\)$ they **linearly project** Q,K and V: $\text{Attention}\(Q W^Q,K W^K,V W^V\)$
- $W^Q$, $W^K$ and $W^V$ are **learnable** parameter matrices

## Head

- each **head** computes **one** such attention function

## Multi-Head

- multi-head (see [original paper: section "Multi-Head Attention": formula for 'MultiHead()'](https://arxiv.org/pdf/1706.03762.pdf))
    - in practice, **multiple** heads are computed **in parallel**
    - then, the resulting features are concatenated ("stacked on top of each other")
    - finally, to make the output size be **the same as the input size** the concatenated features are also linearly projected, i.e. they are multiplied by a **learnable** matrix $W^O$ ("O" for output)

## Masked Self-Attention

- watch: [YouTube: masked self-attention](https://www.youtube.com/watch?v=piT1_k8b9uM&list=PLDw5cZwIToCvXLVY2bSqt7F2gu8y-Rqje&index=7)
- **masked softmax**
    - "masks" certain weights in the weight matrix $W$, so that at training time the self-attention block cannot "cheat" by looking at **future words** in the sequence. It must look at **previous words** only! Therefore, we **mask** (i.e. set to 0) the weights corresponding to the future words.

## Add & Norm Layer

- watch: [Encoder: at this time point](https://youtu.be/SsLzwRXH0UI?list=PLDw5cZwIToCvXLVY2bSqt7F2gu8y-Rqje&t=590)

## Positional Encodings

- watch: [Kilcher: Attention is all you need](https://www.youtube.com/watch?v=iDulhoQ2pro&t=17s)
- the positional encodings help the transformer to understand that the **order** of the words in a sequence **matters**

# Similarity Learning

## Unsupervised Learning

### Contrastive Learning

- SimCLR (image-only)
- ConVIRT (text-image)
    - CLIP

## Weakly Supervised Learning

### Word similarity

Flickr dataset: Joulin 2015:
- "The target for each image is a **bag of all the words** in the dictionary associated with that image, i.e., a multi-label vector $\mathbf{y} \in \{ 0,1 \}^K$."
    - "Specifically, we select a training example by **picking a word uniformly at random** and randomly selecting an image associated with that word. All the other words are considered negative for the corresponding image, *even words that are also associated with that image*"
- "The **weights in the last layer** of our networks can be viewed as an **embedding of the words**. This word embedding is, however, different from those learned by language models such as word2vec that learn embeddings based on word co-occurrence"
    - "recall that during training, we use a single, randomly selected word as target for an image"
    - "This means that structure in the word embedding can only be learned when the network notices that two words are assigned to images with a similar visual structure"
- "models learn **word embeddings** that are both grounded in vision and **capture** important semantic information, for instance, on **word similarity** and **analogies**." 
    - "Because our training data is **multilingual**, our models also relate words from different languages by observing that they are frequently assigned to similar visual inputs."

Predict n-grams given an image: Li 2017:
- zero-shot experiments:
    - "we simply apply the Flickr-trained models on a test set from a different dataset"
        - aYahoo
        - SUN
        - ImageNet
    - "The results reveal that, even without any finetuning or re-calibration, non-trivial performances can be obtained on generic vision tasks."
    - "The performance of our models is **particularly good on** common classes such as **those in the aYahoo dataset for which many examples are available in the YFCC100M dataset**. The performance of our models is worse on datasets that involve fine-grained classification such as ImageNet, for instance, because YFCC100M contains few examples of specific, uncommon dog breeds."

## NLP

### BERT (by Google)

from Wiki:
- pre-trained on **two tasks**:
    - **language modeling** (15% of tokens were masked and BERT was trained to predict them from context) 
    - **next sentence prediction** (BERT was trained to predict if a chosen next sentence was probable or not given the first sentence)
- As a result of the training process, BERT learns **contextual embeddings** for words. (like CLIP)
- After pretraining, which is computationally expensive, BERT can be **finetuned** with fewer resources on smaller datasets to optimize its performance on **specific tasks**. (like CLIP)
- The **architecture** is "almost identical" to the original transformer implementation in **Vaswani et al. (2017)**.
- In 2019, **Google** announced that it had begun leveraging BERT in its **search engine**, and by late 2020 it was using BERT in almost every English-language query.

#### ClinicalBERT

- [source](https://medium.com/nwamaka-imasogie/clinicalbert-using-deep-learning-transformer-model-to-predict-hospital-readmission-c82ff0e4bb03):
    - ClinicalBERT is a Bidirectional Transformer. 
    - ClinicalBERT is a **modified BERT model**: Specifically, the representations are learned using medical notes and further processed for downstream clinical tasks. 
    - ClinicalBERT is **pretrained on patient clinical notes/EHR** and then can be used for downstream predictive tasks.

### GPT-3 (by OpenAI)

from Wiki:
- The architecture is a **standard transformer network** (with a few engineering tweaks) with the unprecedented size of 2048-token-long context and 175 billion parameters (requiring 800 GB of storage). 
- The training method is "generative pretraining", meaning that it is **trained to predict what the next token is**. 
- The model demonstrated **strong few-shot learning** on many text-based tasks.

### GPT-3 vs. BERT

from [Kaggle](https://www.kaggle.com/code/residentmario/notes-on-gpt-2-and-bert-models):
- GPT-2 works like a traditional language model is that it takes word vectors and input and produces estimates for the probability of the next word as outputs. It is **auto-regressive** in nature: each token in the sentence has the context of the previous words. Thus GPT-2 works **one token at a time**. 
    - **autoregressive**: A statistical model is autoregressive if it predicts future values based on past values. For example, an autoregressive model might seek to predict a stock's future prices based on its past performance. [investopedia](https://www.investopedia.com/terms/a/autoregressive.asp)
- BERT, by contrast, is **not auto-regressive**. It uses the entire surrounding context **all-at-once**.

from [source](https://datascience.stackexchange.com/a/104655/115254):
- BERT is an **encoder-only** model trained with the masked language-modeling objective and operates **non-autoregressively**. 
- GPT-2 is a **decode-only** model trained using the left-to-right language objective and operates **autoregressively**. 
- Other than that, there are only technical differences in hyper-parameters, but no other conceptual differences.
- BERT (other masked LMs) could also be used for zero- or few-shot learning, but in a slightly different way. There is a method called PET (Pattern-Exploiting Training). It uses the language modeling abilities of BERT via templates.

## Multi-modal learning

### VirTex

- did not perform well on the captioning task (pre-text task)
    - it was well below SOTA
    - but getting high-quality captions was not VirTex's main goal, rather getting good image representations
    - however, VirTex performed better than humans at the captioning task (in this sense, the captioning task is actually pretty meaningless, see J. Johnson talk)

### ConVIRT

- pre-training
    - **initialize** the text encoder with ClinicalBERT weights
    - **initialize** the image encoder with ImageNet weights (they try several different weights and initialization strategies, e.g. a strategy they call "in-domain" initialization)
- hyperparameter experiments:
    - pre-training is most sensitive to **temperature parameter** choice

### CLIP

- WIT
    - constructed to cover as many visual concepts as possible
    - similar total word count as WebText (GPT-2)
- image encoder
    - ResNet-50
    - "attention pooling" layer (instead of global average pooling layer)
- text encoder
    - Vaswani transformer
    - 63M-parameter 12-layer 512-wide model with 8 attention heads
    - BBPE
        - The transformer operates on a lower-cased byte pair encoding (BPE) representation of the text with a 49,152 vocab size \cite{sennrich-etal-2016-neural}. By using BPE the model can represent any Unicode string. 
    - For computational efficiency, the **max sequence length** was capped at 76.
    - The text sequence is bracketed with **(SOS) and (EOS) tokens** and 
    - the activations of the highest layer of the transformer at the (EOS) token are treated as the **feature representation of the text** which is layer normalized and then **linearly projected** into the multi-modal embedding space.
    - **Masked self-attention** was used in the text encoder to preserve the ability to initialize with a pre-trained language model or add language modeling as an auxiliary objective, though exploration of this is left as future work.
- temperature parameter
    - log-parameterized multiplicative scalar (see pseudocode)
- When the CLIP authors **evaluated** the zero-shot performance **on all the datasets**, they stored (**cached**) the weights generated by the hypernetwork (text encoder), since the prompts do not change when iterating through all images in a specific dataset. This saves computational cost.
    - weiter unten: prompt ensembling: "**cache** a single set of averaged text embeddings so that the compute cost of the ensemble is the same as using a single classifier when amortized over many predictions"
- prompt engineering in GPT-3 is software 3.0
- prompt engineering specific examples in 3.1.4

#### datasets

- UCF101
    - CLIP better than ResNet at action recognition, why?: For instance, UCF101 contains Youtube video clips of people playing musical instruments or doing some kind of sport. Describing these activities without verbs is obviously harder than describing them with nouns and verbs.
- Stanford Cars
    - The Stanford Cars dataset consists of 196 classes of cars with a total of 16,185 images, taken from the rear
    - categories: Make, Model, Year
- Food101
    - 101 food categories: ice cream, pizza, donuts, steak, ...
- Flowers102
    - 102 flower categories
    - flower commonly occuring in the UK
- FGVCAircraft
    - 102 different aircraft model variants, most of which are airplanes
    - Label: Model, Variant, Family, Manufacturer
- CLEVR (authors use only its subset "count")
    - synthetic Visual Question Answering
    - e.g. counting objects with different colors in a scene
- EuroSAT
    - satellite image classification
- PatchCamelyon
    - scans of lymph node sections
    - tumor detection
    - binary label: presence of metastatic tissue
- MS-COCO
    - the resource most used for **image captioning** was the MS-COCO dataset, containing around 100k images and 5-way image-caption annotations (produced by paid annotators)
- Conceptual Captions (Google)
    - successor of MS-COCO
    - 3 million images, paired with natural-language captions. 
    - In contrast with the curated style of the MS-COCO images, Conceptual Captions images and their raw descriptions are harvested **from the web**
- Visual Genome
    - contains VQA data in a multi-choice setting. 
    - consists of 100k images from MSCOCO with 2 million QA pairs
    - 17 questions per image on average
    - 6 question types: What, Where, When, Who, Why and How

OCR:
- SST and SST-2
    - Stanford Sentiment Treebank
    - sentiment analysis (sentiment classification)
    - data: movie review phrases 
    - label: "positive", "negative", "neutral", "somewhat positive", "somewhat negative" (in SST-2: only "positive" and "negative")
    - What does the "2" mean?: SST-2 is a **binary** version of SST and Movie Review dataset (the neutral class was removed), that is, the data was classified only into positive and negative classes.
    - Binary classification experiments on full sentences (negative or somewhat negative vs somewhat positive or positive with neutral sentences discarded) refer to the dataset as **SST-2** or **SST binary**.

#### Natural Distribution Shift

- "anti-causal learning", "dataset bias", "the tank legend" or "the Clever Hans effect"
- (Ilyas) show that adversarial examples exist because of the presence of non-robust features
- ImageNet Sketch
    - They search only within the "black and white" color scheme.
- ObjectNet
    - There are 313 object classes **with 113 overlapping ImageNet**
    - objects are captured at unusual poses in cluttered, natural scenes, which can severely **degrade recognition performance**
- YouTube-BB
    - bounding boxes
    - 23 classes
    - The objects represent a subset of the **MS COCO label set** (COCO has 80 classes, ImageNet has 1000 classes)
    - objects in natural settings without editing or post-processing (phth: this degrades recognition performance like in ObjectNet)

## TODO

- linear projection instead of non-linear
    - vlt wegen?: "the activations of the highest layer of the transformer at the (EOS) token are treated as the feature representation of the text which is layer normalized and then **linearly projected** into the multi-modal embedding space"
- Tian2019 "contrastive objective vs. equivalent predictive objective"
- "Starting with the same bag-of-words encoding baseline"?
- WIT contruction
    - CLIP also uses search queries as part of the creation process of the pre-training dataset WIT. 
    - However, CLIP only uses full text sequences co-occuring with images as supervision and not just the queries, which are often only a single word or short n-gram. 
    - We also **restrict** this step in CLIP **to** text only querying for **sub-string matches** while most webly supervised work uses standard image search engines which have their own complex retrieval and filtering pipelines that often involve computer vision systems. 
- interpretation: text encoder as "configurable" linear classifier **during pre-training** (s. ganz unten in 3.1.2)
- what does "average performance" mean? maybe, average across all 27 datasets? (Fig. 6: few-shot vs Fig. 7: data efficiency)
    - anscheinend stimmt "average across all 27 datasets", s. caption of Fig. 10: evaluation suite 12 datasets vs 27 datasets
- EfficientNet: [watch](https://www.youtube.com/watch?v=3svIm5UC94I)
    - a study into how **systematic** upscaling of depth, width and resolution in a ResNet improves performance while saving compute
    - they propose a **constraint** (like in **linear programming**) for the three scaling factors 
        - one scaling factor for each: depth, width and resolution
    - achieve SOTA on ImageNet, while being more **computationally efficient** with much **less parameters**
- Figure 11: CLIP vs EfficientNet-L2
- formula Taori effective vs relative robustness
    - **effective**: rho = acc2(f) - beta(acc1(f))
        - main goal of a "robustness intervention" is to increase rho.
        - acc1: on ImageNet
        - acc2: on dataset with distribution shift
    - **relative**: tau = acc2(f') - acc2(f)
        - f: model without "robustness intervention"
        - f': derived model with "robustness intervention"
- limitation: 
    - selection of evaluation datasets is not carefully thought: haphazard collection of 27 datasets, capabilities of CLIP are co-adapted with these datasets
    - CLIP embeddings do not capture everything, and there are interesting demonstrations of its weakness. One such well-known case is a **typographic attack**. In this attack, a text on an image can lead to the wrong classification of the image. You’ll see other weaknesses in a section dedicated to unCLIP.
- section preliminaries zu "Vision-and-Language Pretraining": 
    - Für presentation die Erklärung in Johnson video "VirTex" zu diesem Thema benutzen!
- understand BERT: james briggs video
- VLP zhou
- Although CLIP can flexibly generate zero-shot classifiers for a wide variety of tasks and datasets, CLIP is still **limited to choosing from only those concepts in a given zero-shot classifier**. This is a significant restriction compared to a truly flexible approach like image captioning which could generate novel outputs

# Semantic Optical Flow

- [paper](https://www.cv-foundation.org/openaccess/content_cvpr_2016/papers/Sevilla-Lara_Optical_Flow_With_CVPR_2016_paper.pdf)
- [arxiv](https://arxiv.org/abs/1603.03911)
- [watch: Nvidia](https://www.youtube.com/watch?v=QwmBSTWgr_s)
- What do the colors (in the optical flow image) mean?
    - direction of the motion: color (according to the wheel)
    - magnitude of the motion: saturation
- traditional CV method: DiscreteFlow
    - very blurry (especially on motion boundaries and in untextured regions)
- new semantic segmentation method: Semantic Optical Flow / Semantic Flow
    - clear edges
