---
title: "Mathematical Methods of Signal and Image Processing - Chapter 2 - Local Operators"
read_time: false
excerpt: "For learning \"Mathematical methods of signal and image processing\"; content mostly from \"RWTH lecture Mathematical methods of signal and image processing\" by Benjamin Berkels."
toc: true
toc_label: "Contents"
toc_sticky: true
author_profile: false
categories:
  - Notes
tags:
  - msip
  - notes
  - lecture

---

# 2.1 Continuous Moving Average Filter, Circular Averaging

**Task**: Denoising

- image: $f: \mathbb{R}^d \to \mathbb{R}$
- image without noise: $f_0: \mathbb{R}^d \to \[0,1\]$
- noise: $n: \mathbb{R}^d \to \mathbb{R}$
- $M_rf(x) = \int^\text{avg}_{B_r(x)}f(y)dy$
  - where $B_r(x)$ is the ball around $x$ with radius $r$ and $\int_A^\text{avg}f(x)dx := \frac{1}{\text{Vol}(A)}\int_A f(x)dx$
  - $B_r(x)$ is like a **moving window**
  - you can use either a ball <span style="color:red">in 2-norm</span> (corresponding to filter $M_r$) or <span style="color:red">in $\infty$-norm (maximum norm)</span> (corresponding to filter $M_r^\infty$)
    - $M_r^\infty$ is easier to compute in the discrete case
- $M_rf(x) = \int_{\mathbb{R}^d} f(x+y)\left(\frac{1}{\| B_r(0) \|}\chi_{B_r(0)}(y)\right)dy$ ($M_r$ as a **linear filter**, $\frac{1}{\| B_r(0) \|}\chi_{B_r(0)}$ as **filter function**)
- slide demo: <span style="color:red">as the circle gets bigger the image gets more and more blurry (ie. you loose sharp structures, eg. edges), but at the same time the noise is reduced</span>
  - it will soon turn out that "loosing edges" is a property of linear filters, it is unavoidable
  - thus, denoising is not the best use case for linear filters

# 2.2 Cross-Correlation, Linear Filter

- $\left(\psi\star f\right)(x) = \int_{\Omega}\psi(y)f(x+y)dy$
- linear filter with kernel $\psi$: $M_\psi: f\mapsto(x \mapsto \left(\psi\star f\right)(x))$

# Convolution vs. Cross-Correlation

- For **real-valued functions**, of a continuous or discrete variable, convolution $( f \ast g )$ differs from cross-correlation $( f \star g )$ only in that either $f(x)$ or $g(x)$ is <span style="color:red">reflected about the y-axis</span> in convolution; thus <span style="color:red">the convolution is a cross-correlation of $g(−x)$ and $f(x)$</span>, or $f(−x)$ and $g(x)$.
- For **complex-valued functions**, the cross-correlation operator is the <span style="color:red">adjoint of</span> the convolution operator, ie. <span style="color:red">the convolution is a cross-correlation of $\overline{g(−x)}$ and $f(x)$</span>, or $\overline{f(−x)}$ and $g(x)$
  - **Beweis**:
    - "$ f(x) \ast g(x) = \overline{f(−x)} \star g(x)$" **(Eq. 1)** steht explizit [hier](https://en.wikipedia.org/wiki/Cross-correlation#Properties).
    - Für die Kommutation der convolution $( g \ast f )$ gilt nach **(Eq. 1)** $ g(x) \ast f(x) = \overline{g(−x)} \star f(x)$.
    - Da die LHS von **(Eq. 1)** und **(Eq. 2)** gleich sind **by commutativity**, müssen auch deren RHS gleich sein, dh. "$\overline{f(−x)} \star g(x) = \overline{g(−x)} \star f(x)$".

![conv_vs_corr](https://i.ibb.co/Q6sTW2Q/Comparison-convolution-correlation-svg.png)

- "The symmetry of $f$ is the reason $f\star g$ and $g\ast f$ (s. unten Mitte und unten links im Bild) are identical in this example."
  - dh. wäre $f$ nicht symmetrisch, wären $f\star g$ und $g\ast f$ <span style="color:red">unterschiedlich!</span>
  - **convolution**: $f\ast g$ und $g\ast f$ sind <span style="color:red">immer</span> gleich (wegen <span style="color:red">commutativity</span>)
  - **correlation**: $(f\star g)(x)$ und $(\overline{g}\star \overline{f})(-x)$ sind <span style="color:red">immer</span> gleich
    - im Reellen ist Kommutation also nur eine Spiegelung an der y-Achse

# 2.3 Properties of the cross-correlation

## 2.3 (i) Boundedness of the norm of the cross-correlation

- $f\in L^p$ and $g\in L^q$, then
  - (i) **existence/finiteness of the integral**: $(f\star g)\in L^r$, where $\frac{1}{r}+1 = \frac{1}{p}+\frac{1}{q}$
  - (ii) **Young's convolution inequality** for cross-correlation: $\\|f\star g\\|\_{L^r} = \\|f\\|\_{L^p}\\|g\\|\_{L^q}$
  - (iii) $(f\star g)(x) = (g\star f)(-x)$ (reflection about the y-axis)
    - <span style="color:red">but</span> for convolution: $(f\ast g)(x) = (g\ast f)(x)$
  - (iv) $\\|f\star g\\|\_{L^r} = \\|g\star f\\|\_{L^r}$

## 2.3 (ii) Inheritance of Differentiability, Derivatives of the Correlation

To understand $\psi \in C_c^k(\mathbb{R}^d)$:

see [section "compact"](#compact)

- $\psi\in C_c^k$ and $f\in L^p$, then
  - (i) $(f\star \psi)\in C^k$
    - in words:
      - "the resulting correlation <span style="color:red">**inherits differentiability**</span> of its arguments (it is sufficient if **one** of the arguments is differentiable for the result to be differentiable)"
      - "the result of the correlation is always **as smooth as the kernel**"
  - (ii) $\frac{\partial^\alpha}{\partial x^\alpha}(f\star \psi) = f\star \frac{\partial^\alpha}{\partial x^\alpha}\psi$
    - $\alpha$ is a vector of natural numbers including $0$ ($\partial x^0$ means "not derived in x-direction", $\partial y^1$ means "derived once in y-direction", etc.), eg. $\alpha=(2,1,0)$ means $\frac{\partial^3}{\partial x^2\partial y^1\partial z^0}$
    - in words: "you just need to compute the derivative of the kernel"
    - useful if: you need a differentiable approximation of your image, eg. for edge detection
  - (iii) $\frac{\partial^\alpha}{\partial x^\alpha}(\psi\star f) = (-1)^{\|\alpha\|}\frac{\partial^\alpha}{\partial x^\alpha}\psi\star f$
    - $\|\alpha\|$ is the **sum of components**, eg. with the example above $\|\alpha\|=2+1+0=3$
    - in words: again, "you just need to compute the derivative of the kernel"
    - note: $\psi$ is differentiable, but $f$ is not necessarily differentiable
    - we get (iii) because $\frac{\partial^\alpha}{\partial x^\alpha}(\psi\star f)(x) = \frac{\partial^\alpha}{\partial x^\alpha}(f\star \psi)(-x)$ and, by applying the chain rule to compute $\frac{\partial^\alpha}{\partial x^\alpha}(f\star \psi)(-x)$, we see that the <span style="color:red">outer derivative</span> of $(f\star \psi)(-x)$ is given by (ii), whereas the <span style="color:red">inner derivative</span>, $\frac{\partial}{\partial x}(-x)$, is $(-1)$ for <span style="color:red">each</span> derivative. Since we derive $\|\alpha\|$ times in total, we get $(-1)^{\|\alpha\|}$.

## 2.3 (iii) Approximating any Function with a Differentiable Function

- **conditions**:
  - $\psi\in L^1$, $\psi\geq 0$ and $\int_{\mathbb{R}^d}\psi(x)dx=1$
  - **$\psi_\epsilon$ a scaled version of $\psi$**: for $\epsilon > 0$, $\psi_\epsilon: \mathbb{R}^d \to \mathbb{R}, x \mapsto \frac{1}{\epsilon^d}\psi(\frac{x}{\epsilon})$
    - factor $\frac{1}{\epsilon^d}$ chosen s.t. integral over $\mathbb{R}^d$ stays $1$
    - gets narrower along $x$-axis and higher along $y$-axis as $\epsilon\to 0$
      - $\epsilon$ is the <span style="color:red">**width**</span> of $\psi_\epsilon$, ie. **low values of $\epsilon$** mean $\psi_\epsilon$ is **very concentrated**
  - $f\in L^\infty$
- (i) if $f$ is **continuous** in point $x$ then,
  - $\lim_{\epsilon\to 0}{(\psi_\epsilon\star f)(x)} = f(x)$, ie. $(\psi_\epsilon\star f)$ converges to $f$ in point $x\in \mathbb{R}^d$ (<span style="color:red">pointwise convergence</span>)
    - the narrower and more concentrated $\psi_\epsilon$ is the better $(\psi_\epsilon\star f)$ approximates $f$
- (ii) if $f$ is **uniformly continuous** then,
  - $(\psi_\epsilon\star f)$ converges to $f$ uniformly on each compact subset of $\mathbb{R}^d$ (<span style="color:red">uniform convergence</span>)
- tool that helps us approximating images $f$
  - mainly useful for proofs
  - the nice thing is $f$ can be <span style="color:red">any</span> function, we only need boundedness for $f$
    - then, the limit on the lhs of (i) gives us a way to approximate this $f$ with the differentiable function $(\psi_\epsilon\star f)$
  - it is often easier to prove s.th. for a differentiable function and then show that it also holds in the limit
    - so, this is a tool to show properties of the limit $f(x)$ on the rhs of (i)

# 2.4 The Difference Quotient Converges to the Derivative

- **conditions**:
  - $D\subset \mathbb{R}^d$ be <span style="color:red">compact and convex</span>
    - $D$ is, again, a large ball (because a ball is also convex)
    - at some point in the proof we have 2 points and we need everything in between the 2 points to be in the set that is <span style="color:red">why we need this convexity</span>
  - $f\in C^1(D)$
- then $$\forall i\in\{1,\ldots,d\}\forall\epsilon\in (0,\infty)\exists\delta\in (0,\infty)\forall x\in D\forall h\in (-\delta,\delta)\backslash \{0\} : \bigg\lvert \frac{f(x+he_i)-f(x)}{h} - \partial_if(x)\bigg\rvert <\epsilon,\quad\text{where}\quad x+he_i\in D$$
- $i$: coordinate direction of the derivative
- $x+he_i\in D$:
  - technicality: because we must not leave the domain where $f$ is defined
- in words: for each $\epsilon$ we find a $\delta$ so that our shifts $h$ are smaller than $\delta$ in absolute value. Then the difference between the difference quotient and the derivative is smaller than $\epsilon$
- **proof**:
  - (i) $D$ compact (aka "closed and bounded") $\Rightarrow$ $\partial_if$ uniformly continuous on $D$
  - (ii) $x\in D$, $h$ s.t. for $0<h<\delta$, $x+he_i\in D$
  - (iii) $D$ convex $\Rightarrow$ $g: \left[0,h\right] \to \mathbb{R},\,t\mapsto f(x+te_i)$ is well-defined 
    - **convex**: the affine combination $(1 − t)x + ty$ belongs to $D$ for all $x,y$ in $D$ and $t$ in the interval $\left[0,1\right]$
      - **problem**: (ii) sagt nur $x\in D$ und $x+he_i\in D$, aber was ist mit den <span style="color:red">Werten zwischen den beiden</span>, also die $x+te_i$ mit $t\in\left(0,h\right)$?
        - **solution**: <span style="color:red">convexity</span> legt fest, dass $x+te_i\in D$
      - dh. wäre $D$ **nicht** convex würde unter Umständen eine Stelle in $\left[0,h\right]$ nicht in $D$ liegen, sodass wir $f(x+te_i)$ an dieser Stelle nicht bestimmen könnten und deshalb $g$ an dieser Stelle dann nicht well-defined wäre.
  - (iv) **mean value theorem**: $\partial_if(x+\xi e_i)=g'(\xi)=\frac{g(h)-g(0)}{h}=\frac{f(x+he_i)-f(x)}{h}$

# 2.5 Correlation is Continuous, Edges are Always Blurred

<p style="border-width:3px; border-style:solid; border-color:#FF0000; padding: 1em;">
- <b>conditions</b>:<br>
&emsp;  - $\frac{1}{p}+\frac{1}{q}=1$ ($q,p$ are dual exponents)
&emsp;  - $f\in L^p$, $g\in L^q$ (neither of them needs to be continuous !)<br>
- then $$(f\star g)\in C(\mathbb{R}^d)$$
</p>

- in words: "the correlation is continuous" (although $f$ and $g$ do not have to be continuous!)
- in images: edges and sudden color changes are typical discontinuities in images. Now, <span style="color:green">(2.5)</span> tells us:
  - if you apply a linear filter, no matter how clever you design it, the result will be continuous! Thus, <span style="color:red">**edges are always blurred.**</span>
    - of course, you can have a <span style="color:red">high resolution and a small filter</span> and then you will have a very tiny blur. But this proposition shows fundamentally the limits regarding edge preservation of linear filters.
  - this is <span style="color:red">bad bec in images typically you want to have nice and sharp edges</span>. 
  - "Linear filters have their merits, but edge preservation is not one of them."
    - which is where <span style="color:red">non-linear filters</span> come in
- **proof**:
  - **exercise**: $\lVert T_hg-g\rVert_{L^q}\to 0$ for $h\to 0$
    - in words: "Shift a function by $h$. Then, compare the shifted version with the original. If the shift $h$ gets small then difference of the integrals gets small."

# 2.6 Positive Mollifier

- **conditions**:
  - $\psi$ as in <span style="color:green">(2.3 (iii))</span>, but additionally, $\psi\in C_c^\infty$
    - you can think of it as an "averaging kernel"
    - infinitely differentiable + compact support (so $\psi$ can have <span style="color:red">as many derivatives as we wish</span> and it is <span style="color:red">nicely localized</span>)
- then $\psi$ is a <span style="color:red">**positive mollifier**</span>
- this is just a <span style="color:red">convenient set of properties for kernels</span>, that is why these have a name

# 2.7 $C_c^\infty$ is dense in $L^p$

- (i) let $\Omega$ be open, $\psi$ be a mollifier and $1\leq p < \infty$ then $$\forall f\in L^p\,\forall\delta\,\exists\epsilon : \lVert \psi_\epsilon\star f-f\rVert_{L^p}<\delta$$
  - **Achtung**: wie $\epsilon$-$\delta$-Definition des $\lim$ Symbols, aber das $\epsilon$ ist der Index von $\psi_\epsilon$
    - phth: dh. "the more concentrated $\psi_\epsilon$ the better the approximation" (like in <span style="color:green">(2.3(iii.i))</span>)
      - phth: das muss stimmen, weil er im nächsten Punkt "very concentrated" sagt
  - in words: "if I convolve $f$ with a **very concentrated** kernel, then I'm very close to the original $f$"
    - <span style="color:red">this is like a more specific version of what we had in</span> <span style="color:green">(2.3(iii))</span>, but there we had a weaker filter kernel (not necessarily differentiable), but now in <span style="color:green">(2.7)</span> with this additional differentiability we get this nice property that <span style="color:red">we can get arbitrarily close to our $f$</span>.
      - in <span style="color:green">(2.3(iii))</span> war $(\psi_\epsilon\star f)$ nur <span style="color:red">einmal differentiable</span>, aber hier ist es <span style="color:red">infinitely differentiable</span>
      - dh. <span style="color:green">(2.7 (i))</span> <span style="color:red">ist ein Spezialfall von</span> <span style="color:green">(2.3 (iii))</span>
    - <span style="color:red">why do we want that?</span>:
      - $\psi_\epsilon$ is a mollifier, so it is **infinitely** often differentiable
      - thus, we can approximate $f$ with a function $\psi_\epsilon \star f$ that is **infinitely** often differentiable
- with <span style="color:green">(i)</span> we can show:

<p style="border-width:3px; border-style:solid; border-color:#FF0000; padding: 1em;">
- (ii) $C_c^\infty$ is dense in $L^p$<br>
</p>

- ie. for any $L^p$ function we can find a $C_c^\infty$ function that is arbitrarily close.<br>
  - and this allows to approximate any $f$ with nicely behaving functions which will be very helpful in the future
- **proof (ii)**:
  - **idea**: find **ANY** $C_c^\infty$ element that gets arbitrarily close to our $f$ (which is a $L^p$ function by assumption). Here, this is $\psi_\epsilon\star g$.
    - (a) show $\lVert \psi_\epsilon\star g-f\rVert_{L^p}<\delta$
    - (b) from <span style="color:green">(2.3(ii))</span> $\psi_\epsilon\star g\in C^\infty$
    - (c) show for $\epsilon>0$ small enough $\psi_\epsilon\star g\in C_c^\infty$
  - By (a) and (c) a $C_c^\infty$ function (here: $\psi_\epsilon\star g$) can get arbitrarily close to **ANY** $L^p$ function (here: $f$)
    - (like a sequence of **rational numbers** can get arbitrarily close to any **real number**)
- we will need this later: and this is actually kind of a preparation for some stuff we will need in the FT chapter, but it is fitting nicely here bec it uses the correlation
- **problem**:
  - time to look at the **discrete case** in order to actually compute sth (bec when you want to implement sth you surely would not compute analytically these integrals)

# 2.8 Discrete Filter, Discrete Correlation in 1D, Padding Methods

- (i) **discrete 1D-cross-correlation** of $F$ and $W$:
  - (i.a) <span style="color:red">for a filter with 3 elements</span>: $W=(w_{-1},w_0,w_1)$ and $F=(f_i)\_{i\in\mathbb{Z}}\in\mathbb{R}^{\mathbb{Z}}$, then $$\hat{f}_i = \sum_{j=-1}^{1}w_jf_{i+j}$$
    - the formula on the left means that we take the **weighted sum** of these $f$s with the $w$s
    - a simple discretization of the 1d version of the correlation
    - $(x, y)$ in the **continuous** correlation integral are now $(i, j)$ in the sum
  - (i.b) <span style="color:red">for a larger filter</span>: $W=(w_{-a},\ldots,w_a)\in \mathbb{R}^{2a+1}$, where $a\in\mathbb{N}$ is the <span style="color:red">**radius of the filter**</span>, the same holds
- (ii) **problem**: the filter needs $f_{i+j}$ for all $i\in\\{1,\ldots,m\\},j\in\\{-1,0,1\\}$ or $j\in\\{-a,\ldots,a\\}$, eg. for the pair $(i=1, j=-1)$ we have $i+j=0$, <span style="color:red">but there is no $f_{i+j}=f_0$</span>
  - anschaulich: "the filter will have to look outside the signal / image"
  - **solution**: different ways to define $f_i$ for $i\in\mathbb{Z}\backslash\\{1,\ldots,m\\}$
    - zero extension ($f_i=0$ außerhalb)
    - constant extension ($f_i=f_1$ links, $f_i=f_m$ rechts)
    - extension by mirroring at the boundary (aka reflection)
      - Berkels: "reflection works probably best, looks the most natural; the others have <span style="color:red">arbitrary jumps at boundaries</span>"
    - periodic extension
      - anschaulich: "dasselbe Signal / Bild links und rechts einfach drangesetzt" (für images meistens nicht helpful)

# 2.9 Discrete Correlation in 2D

- **discrete cross-correlation** $$(W\star F)_{i,j} = \sum_{i=-a}^{a}\sum_{j=-b}^{b}w_{k,l}f_{i+k,j+l}\quad\text{for}\quad(i,j)\in\mathcal{I}$$, where $f_{i+k,j+l}$ for $(i+k,j+l)\not\in\mathcal{I}$ is defined by extension (see <span style="color:green">(2.8(ii))</span>)
  - the center of the filter is the element of the filter with $(k=0,l=0)$
- **discrete convolution** $$(W\ast F)_{i,j} = \sum_{i=-a}^{a}\sum_{j=-b}^{b}w_{k,l}f_{i-k,j-l}\quad\text{for}\quad(i,j)\in\mathcal{I}$$, where $f_{i-k,j-l}$ for $(i-k,j-l)\not\in\mathcal{I}$ is defined by extension (see <span style="color:green">(2.8(ii))</span>)
  - the minus signs in the indices $(i-k,j-l)$ are the only thing that changes wrt the cross-correlation def. (like in the continuous case, where the difference was $y - x$ instead of $y + x$)
- Notation: see figure below
  - Berkels: I use <span style="color:red">square brackets</span> to indicate that this is <span style="color:red">NOT a matrix</span> - a <span style="color:red">matrix has round brackets</span>
    - (<span style="color:red">this is different from a matrix</span> bec the 1st index increases from left to right and the 2nd index from bottom to top and the center has index $(0,0)$. This is very different from the usual <span style="color:red">matrix index notation</span>, where the row number increases from top to bottom and the column number from left to right and $(0,0)$ is on the top left corner!)

<p align="center">
  <img src="https://i.ibb.co/9N53JXh/Screenshot-from-2024-02-23-23-29-31.png" alt="Screenshot-from-2024-02-23-23-29-31" border="0">
</p>

- **problem**:
  - sometimes it's helpful to think about these filters in matrix form

# 2.10 $(W\ast F)$ as a Matrix

- $(W\ast F)$ is linear in $F$. Thus, filtering with $W$ can be expressed as matrix.
- see notes
- Building this matrix $W$ is only useful for theoretical arguments. This is NOT how you would implement this, bec copying all these matrices like this would be a waste of resources. But still, it is good to have seen how you can do this.
  - (for implementation you would avoid building this matrix and just compute the sums directly)
- **problem**:
  - now, lets have a look at some filters you can construct in this way

# 2.11 Denoising Filters

- averaging filters have $\sum_{k,l}w_{k,l} = 1$
  - <span style="color:red">TODO</span>: not sure if the rank filters <span style="color:green">(2.11 (v))</span> fulfill this property

## 2.11 (i) Mean Value Filters

- correspond to the **moving average over squares $M_r^{\infty}$** that we had at the beginning of this chapter, see <span style="color:green">(2.1)</span>, **or** to some **more refined avgs** where eg. the center is weighted more.
  - in <span style="color:green">(2.1)</span> the filter was $\frac{1}{\| B_r(0) \|}\chi_{B_r(0)}$
- (i) **continuous case**: the <span style="color:red">moving average over squares $M_r^\infty$</span>, see <span style="color:green">(2.1)</span>
  - the <span style="color:red">ball becomes a square</span> when we use the $\infty$-norm
- (ii) **discrete case**: eg. $M_3, M_5$ or for arbitrary sizes $M_{a,b}=\frac{1}{ab}(1,\ldots,1)\in \mathbb{R}^a\otimes(1,\ldots,1)\in \mathbb{R}^b$
  - all ones with scaling factor
  - mean value filters fulfill $\sum_{k,l}w_{k,l}=1$, ie. <span style="color:red">elements must sum to one</span> bec we want Maß $1$
- tensor product notation
- <span style="color:red">the larger the filter the larger the averaging effect, ie. the more blurry the image gets</span>
- other mean value filters, eg.
  - (i) trying to average on a discrete circle instead of a discrete square
  - (ii) add a higher weight to the center pixel instead of equal weights everywhere

## 2.11 (ii) Gaussian Filter

- **idea**: $\psi\star f$ is as smooth as the $\psi$ <span style="color:green">(2.3 (ii))</span> $\Rightarrow$ "full smoothing" by choosing sth that is in $C^\infty$ $\Rightarrow g_\sigma\in C^\infty$
- (i) **continuous case**: $g_\sigma(x) = \frac{1}{(\sqrt{2\pi}\sigma)^d}e^{-(\frac{\lVert x\rVert}{2\sigma})^2}$
  - normalization factor (should integrate to $1$, depends on dim)
- (ii) **discrete case**: $(\widetilde{G}\_\sigma)\_{k,l} = \frac{1}{(\sqrt{2\pi}\sigma)^d}e^{-(\frac{k^2+l^2}{2\sigma})^2}$ (einfach das $\lVert x\rVert$ in (i) mit $k^2+l^2$ ersetzen)
  - **with normalization**: $(G\_\sigma)=\frac{1}{\sum\_{k,l}(\widetilde{G}\_\sigma)\_{k,l}}(\widetilde{G}\_\sigma)$ (einfach (ii) durch die Summe über alle möglichen (ii) dividieren)
  - <span style="color:red">assumes grid size 1</span>, otherwise $(k^2+l^2)$ has to be scaled corresponding to the grid size
  - biggest weight in the center like the Gaussian function in d-dim
  - smoothing/blurring (less sharp) is less pronounced than for the mean filter above, but also more isotropic bec filter is more rotationally symmetric

## 2.11 (iii) Binomial Filter

- an approximation of the Gaussian that can be computed rather easily (without having to evaluate exponential functions)
- $W=\frac{1}{2}(1,1)$ (the "averager")
  - <span style="color:red">**recursive definition of $B^n$**</span>: To get $B^n$ we correlate $W$ with $(0,B^{n-1},0)$. $B^0:=1$.
    - anschaulich:
      - When correlating $W$ with $(0,B^{n-1},0)$ we basically "average each pair" of neighboring values in $(0,B^{n-1},0)$.
      - E.g. when we average each pair in $(0,B^0,0) = (0,1,0)$ we get $(\frac{1}{2},\frac{1}{2}) = B^1$.
- the resulting $B_n$ are <span style="color:red">normalized rows of Pascal's triangle</span>
  - $B^1=\frac{1}{2}(1,1)$, <span style="color:red">$B^2=\frac{1}{4}(1,2,1)$ (used for the Sobel filter)</span>, $B^3=\frac{1}{8}(1,3,3,1)$, $B^4=\frac{1}{16}(1,4,6,4,1)$
  - that is why it is called <span style="color:red">Binomial filter</span>
- $B_{a,b} = B^{a-1}\otimes B^{b-1}$
  - (Achtung: genau gucken: eg. für $5\times 5$ Filter muss das tensor product $B^4\otimes B^4$ gebildet werden, nicht $B^5\otimes B^5$!)
  - **tensor product**: wie das normale matrix product für zwei $n\times 1$ Matrizen, wobei die <span style="color:red">zweite</span> (nicht die erste!) Matrix transponiert wird: $\underline{x}^{n\times 1} \otimes \underline{y}^{n\times 1} = \underline{x}^{n\times 1} \underline{y}^{1\times n}$
    - dass die zweite und nicht die erste Matrix transponiert wird, ist zB. in <span style="color:green">(2.13)</span> wichtig
- remember: larger filter means larger blurring (but also more denoising)
  - (you could estimate the filter size if you would have the standard deviation of the noise, but for real data you typically do not have that information, so you must try practically which size works best)
- <span style="color:red">for large $n$</span> the Binomial filters are a good <span style="color:red">approximation of</span> the <span style="color:red">**Gaussian filter**</span>
  - but Binomial filters are easier to compute
- one reason why you would use the Gaussian is <span style="color:red">when you need the derivative</span>
  - (because the Binomial approximates the Gaussian itself, but if you want to convolve with the derivative of the Gaussian - which we will do next - then you actually need the Gaussian filter.)

## 2.11 (iv) Duto Blur

- overlays an image $F$ with a smoothed version of itself $(G_\sigma\star F)$: $$\lambda F+(1-\lambda)(G_\sigma\star F),\quad\text{for}\quad\lambda\in\left[0,1\right]$$
  - ie. it is a convex combination of the filtered image with the original image
- strange effect: it looks somewhat sharp but also somewhat blurred because of this convex combination
- not sure whether it is used somewhere in practice. There are some lenses that have this kind of property. (lec9:) This is just for an artistic effect.

## 2.11 (v) Median Filter, Maximum and Minimum Filter

- a <span style="color:red">non-linear filter</span> (cannot preserve edges as we will see)
- **mean value filter** as solution of the minimization problem $$\label{eq:medianminimizationproblem}(M_{2a+1}\star F)_{i,j}=\text{argmin}_{f\in\mathbb{R}}\sum_{k=-a}^{a}\sum_{l=-a}^{a}\lvert f_{i+k,j+l}-f\rvert^2$$
- this is <span style="color:red">just a different interpretation</span> of what the mean filter does
  - if you solve the minimization problem by taking the <span style="color:red">necessary condition</span> "derivative of double sum=0" and then solve this equation wrt $f$ you get <span style="color:red">"$f =$ our orig. definition of the mean filter"</span> as the minimizer
- now think about <span style="color:red">outliers</span>:
  - for outliers the difference in ($\ref{eq:medianminimizationproblem}$) would be very large ( $f_{i+k,j+l}$ would be a nonsense value which would be far away from the average $f$ )
    - thus, bec of the square $\cdot^2$, these outliers would have a strong effect on the average. If you are familiar with these kinds of minimization problems then you know that using a $1$ instead of the square reduces this strong effect of the outliers. That is what we will do now.
- **problem**: the square makes the minimization problem <span style="color:red">susceptible to outliers in $F$</span>
- **solution**: replace square by a $1$
  - the solution of the resulting minimization problem is the <span style="color:red">**median**</span> (exercise)
  - double sum in ($\ref{eq:medianminimizationproblem}$) becomes a single sum (because if we reorder the matrix to a vector, the minimum does not change, so the order does not matter), thus the new minimization problem is simply $\min_{g\in\mathbb{R}}\sum_{i=1}^{n}\lvert g_i-g\rvert$, where the $g_i$ are all the $f_{i+k,j+l}$ in the filter window
- To summarize:
  - <span style="color:red">The mean minimizes the sum of squared deviations/errors, whereas the median minimizes the sum of absolute deviations/errors.</span>
- the median filter of an image $F$ is defined as the median over windows of size $(2a+1)\times(2a+1)$
- a non-linear filter
- <span style="color:red">very suitable to remove outlier pixels</span>, eg. salt-and-pepper noise (ie. when we randomly change pixels to either black or white)
  - whereas with the **mean value filter** (even with large mean value filters) the <span style="color:red">salt-and-pepper noise **cannot** be removed</span> and the image gets very blurry (see slides demo in lecture)
- of course, the 3x3 median filter <span style="color:red">will slightly move edges</span>, but in your window less than half of the values should be corrupted
- if instead of the median, the **maximum** or **minimum** is considered the filter is called <span style="color:red">maximum or minimum filter</span>
  - also non-linear
- these three filters are known as <span style="color:red">rank-filters</span>
  - we will revisit these under the name of <span style="color:red">"morphological filters"</span> later, where we will see why it would be a good idea to take the **minimum** or **maximum** and what kind of effect this has

## 2.11 (vi) Bilateral Filter, Selective Gaussian Filter (aka Nonlinear Gaussian Filter)

- see lec notes
- **idea**: prevent blurring by taking the <span style="color:red">gray value distance</span> into account with an <span style="color:red">**adaptive weighting function**</span>
  - reinterpret $(\psi\star f)$ as $(\psi\star f)(x)=\int_{\mathbb{R}^d}\psi(y-x)f(y)dy$ (by substitution $\phi(y)=y-x$ in the def. of the correlation)
    - the weight at position $y$ only depends on the distance $y-x$, <span style="color:red">but it is independent of $f(y)-f(x)$</span>
  - **idea**: take also into account the difference of the gray values $f(y)-f(x)$, so that you <span style="color:red">only average similar gray values</span>, $$(\psi\star f)(x)=\frac{1}{\mathcal{Z}}\int_{\mathbb{R}^d}\psi_1(y-x)\psi_2(f(y)-f(x))f(y)dy$$
    - thus, image intensity <span style="color:red">edges</span> are not averaged and <span style="color:red">the edges remain sharp</span>
    - $\mathcal{Z}:=\int_{\mathbb{R}^d}\psi_1(y-x)\psi_2(f(y)-f(x))dy$ is a normalization factor (again, bec we want Maß 1)
      - the computation of $\mathcal{Z}$ makes this filter <span style="color:red">more costly than linear filters</span> bec the normalization depends on the input image and has to be computed for every pixel
    - <span style="color:red">$\psi_1$</span> is the weight function encoding the <span style="color:red">spatial closeness</span>
    - <span style="color:red">$\psi_2$</span> is the weight function encoding the <span style="color:red">intensity closeness</span>
- if the weight functions are **Gaussians**, then the filter is called <span style="color:red">**Selective Gaussian Filter**</span> or <span style="color:red">**Nonlinear Gaussian Filter**</span>

<p align="center">
  <img src="https://i.ibb.co/YyPsnZx/Screenshot-from-2024-02-24-08-01-39.png" alt="Screenshot-from-2024-02-24-08-01-39" border="0">
</p>

- slide demo:
  - selective Gaussian removes the salt-and-pepper noise **AND ALSO** preserves all the edges sharply (regardless of the filter size used)
  - thus, this is a <span style="color:red">really practical filter for denoising</span>
- unlike for the aforementioned filters, here we can use <span style="color:red">"zero extension"</span>
  - we could not use zero extension for the aforementioned filters because the aforementioned filters did **not** take the gray value distance into account, so that zero extension would have caused ~~artificial edges~~ <span style="color:red">averaging of the boundary gray values with the $0$s outside the image</span> and thus, <span style="color:red">strong blurring</span> at the image boundaries!

# 2.12 Derivative Filters

- **problem**:
  - later we will see that we can use them for <span style="color:red">edge detection</span>
  - **continuous case**: later: more sophisticated approach: use the property of the correlation <span style="color:green">(2.3)</span> that when you convolve with a differentiable kernel you can convolve with the derivative of the kernel to get the derivative of the filtered image.
    - this approach is better because it does **not** have the <span style="color:red">**"oscillating signal problem"**</span> of the derivative filters (see below)
  - **discrete case**: But, we here first look at <span style="color:red">**finite differences**</span>
- (i) $D_{x_1}^{+}$: <span style="color:red">forward difference quotient</span> at $i,j$ in direction $x_1$
  - recall, how we defined the coordinate directions in a filter matrix $\underline{W}$ (1st coordinate direction goes from left to right and 2nd direction goes from bottom to top) 
    - therefore, we go from $-1$ to $1$ from left to right (ie. along the 1st coordinate direction), or in other words, when the index increases from $i$ to $i+1$
- (ii) $D_{x_1}^{-}$: <span style="color:red">backward difference quotient</span> at $i,j$ in direction $x_1$
- (iii) $D_{x_2}^{+}$, $D_{x_2}^{-}$: the quotients in direction $x_2$
- (iv) $D_{x_1}^{c}$, $D_{x_2}^{c}$: <span style="color:red">central difference quotient</span> at $i,j$ in direction $x_1$ and $x_2$
  - this is really just translating the finite differences for the derivatives into the filter notation. So, the exact factors come from applying <span style="color:red">**Taylor**</span> to the function so that you get the approximation order that you are looking for. This compuation of the factors should be on the last attendance sheet.
- (v) $D_{x_1}^{2}$, $D_{x_2}^{2}$: <span style="color:red">higher derivatives</span> at $i,j$ in direction $x_1$ and $x_2$
- (vi) derivative filters have <span style="color:red">$\sum_{k,l}w_{k,l} = 0$</span>, whereas averaging filters have $\sum_{k,l}w_{k,l} = 1$
- (vii) **oscillating signal problem**:
  - a problem typical for finite differences
  - <span style="color:red">central</span> diff quot $D_{x_1}^{c} = 0$, but the signal is not constant
    - ie. it is not desirable to have a derivative of $0$ for a signal that is absolutely NOT constant / a signal that is maximally oscillating.
    - why <span style="color:red">central</span> diff quot?: bec this is what we would usually do for the 1st order derivative bec the central diff is a better approximation than the forward or backward difference !
  - **solution**: make use of property <span style="color:green">(2.3)</span>
- **problem**:
  - despite these problems with the finite differences you can use them for edge detection:

# 2.13 Prewitt and Sobel Edge Detector

- **idea**:
  - here the idea of Prewitt and Sobel is to combine the <span style="color:red">derivative</span> in <span style="color:red">one direction</span> with an <span style="color:red">averaging</span> in the <span style="color:red">other direction</span>, so that you get a <span style="color:red">**smoothed approximation of the derivative**</span>.
  - you do this <span style="color:red">for both directions</span>: So, derivative in $x_1$ direction combined with smoothing in $x_2$ direction and vice versa to get an approximation of the gradient
  - then
    - the <span style="color:red">**magnitude of the gradient**</span> will be an estimate of where the edges are and
    - the <span style="color:red">**direction of the gradient**</span> will be an estimate of the direction of this edge.
  - summarizing:
    - ie the idea is not only use <span style="color:red">finite differences</span>, but combine it with <span style="color:red">smoothing</span> (to get rid of these negative effects of finite diff &rarr; see <span style="color:red">**oscillating signal problem**</span>)
- slide demo: 
  - if you look in $x_1$ direction you will NOT see this edge in $x_2$ direction and vice versa
  - but if you combine these absolute values you get a nice detection of where these edges are
  - and then you can also <span style="color:red">convert the angle into a color</span> and then show what the direction of your angle is (for this we will need to introduce the <span style="color:red">HSV space</span>)
- (i) **Prewitt filter**: <span style="color:red">mean value filter $M_3$ for **smoothing**</span>, $D_{x_1}^c$ for the derivative
  - $D_{x_1}^{Prewitt}=D_{x_1}^c\otimes M_3$ (merke: <span style="color:red">für $x_1$ direction erst $D_{x_1}^c$ dann $M_3$</span>, für $x_2$ andersrum)
  - use <span style="color:red">central difference</span> instead of forward difference (bec central diff has a <span style="color:red">better approximation order</span>)
- (ii) **Sobel filter**: <span style="color:red">$B^2$ for **smoothing**</span>, $D_{x_1}^c$ for the derivative
  - $D_{x_1}^{Sobel}=D_{x_1}\otimes B^2$ gleiche Merkregel wie für Prewitt
- remember: square brackets denote filters which have a different indexing than normal matrices
- (iii) $F$ be a discrete image, then $G_{x_1}=D_{x_1}^{Prewitt}\star F$ and $G_{x_2}=D_{x_2}^{Prewitt}\star F$
  - for visualization we use $T^{\text{norm}}(G_{x_1})$ and $T^{\text{norm}}(G_{x_2})$, where <span style="color:red">$T^{norm}$ maps the gradients to $\left[0,1\right]$</span> to get the gray values
  - slide demo: 
    - transition from white to black &rarr; <span style="color:red">maximal positive gradient</span> $= +1$, black edge in $G_{x_1}$ and $G_{x_2}$ visualization
    - transition from black to white &rarr; <span style="color:red">maximal negative gradient</span> $= -1$, white edge in $G_{x_1}$ and $G_{x_2}$ visualization
    - no transition / constant color &rarr; <span style="color:red">zero gradient</span> $= 0$, gray area in $G_{x_1}$ and $G_{x_2}$ visualization
- to <span style="color:red">**estimate edges**</span> we need to combine these two directions <span style="color:red">bec otherwise we would overlook certain directions of edges</span> if we would look in one direction only (as we have seen in a slide demo before).
- (iv) then $T^{\text{norm}}(\sqrt{G_{x_1}^2+G_{x_2}^2})$ is the <span style="color:red">**gradient magnitude**</span>
  - this is a simple <span style="color:red">edge detector</span> ("simple" compared to the Canny detector)
  - <span style="color:red">"square and root pixelwise"</span>: ie. the discrete image $\underline{F}$ is a matrix, both directional derivatives $\underline{G_{x_1}}$ and $\underline{G_{x_2}}$ give us matrices of the same size as $\underline{F}$, and for these matrices I <span style="color:red">square the entries</span> and <span style="color:red">sum them up</span> and <span style="color:red">take the square root of the individual entries</span>
  - again, $T^{\text{norm}}$ is applied on the result of the root, so that the grad magnitude is mapped to $\left[0,1\right]$ to get the gray values
  - the vector that contains $\underline{G_{x_1}}$ and $\underline{G_{x_2}}$ at every pixel gives us a <span style="color:red">gradient estimate at every pixel</span>. The gradient magnitude is then an <span style="color:red">estimate for edges</span>.
- (v) $\theta = \text{arctan2}(G_{x_1},G_{x_2})$ is the <span style="color:red">**gradient direction**</span> (direction of "steepest ascent")
  - **anschaulich**: the angle of this $(G_{x_1},G_{x_2})$ vector with the x-axis
  - <span style="color:red">**2-argument arctan**</span>: the normal $\text{arctan}$ <span style="color:red">only covers half of the unit circle</span> and the $\text{arctan2}$ fixes this problem
    - if you are in the right half plane $(a>0)$ you have the standard $\text{arctan}$ formula (the standard $\text{arctan}$ does not know angles outside $(-pi,pi\]$ - this is why the standard $\text{arctan}$'s value range is $(-pi,pi\]$)
  - $\text{arctan2}$ is often used in computer science
- (vi) all of this can be done with the **Sobel kernels** as well

# 2.14 HSV Space, Visualization of the Gradient Direction $\theta$

- **problem**: How to visualize $\theta$?
  - $V=\left[-\pi,\pi\right]$ is <span style="color:red">not really an interval</span>, but must rather be <span style="color:red">interpreted as a unit circle</span>
  - we <span style="color:red">need sth that takes into account that $-\pi$ and $\pi$ are the same</span> (otherwise we would have <span style="color:red">artificial jumps from black to white</span> when the angle changes from $-\pi$ to $\pi$ although <span style="color:red">these jumps would have no meaning</span> then. Therefore, we want to avoid such artificial discontinuities.)
- slide demo:
  - <span style="color:red">naive visualization</span>: $T^{norm}(\theta)$ has an artificial discontinuity (jump from black to white), even though there is a smooth transition of the angle from $-\pi$ to $\pi$.
    - thus, we <span style="color:red">need HSV</span>
- visualize the gradient direction $\theta$ by $H=\theta$
- visualize the gradient magnitude by
  - **option 1**: $S=V=1$
  - **option 2**: $S=G, V=1$
  - **option 3**: $S=G, V=G$
- <span style="color:red">**HSV cylinder**</span>: 3 axes like for RGB, but <span style="color:red">H axis goes in a circle</span>:
  - S: <span style="color:red">white to colorful</span>
  - V: <span style="color:red">dark to bright</span>
  - H: <span style="color:red">color: red -> green -> blue -> red (in a circle)</span>
    - this is the behaviour we want for our angle: <span style="color:red">$-\pi$ and $\pi$ are both red</span>
- to display we must <span style="color:red">convert HSV to RGB</span>
  - Berkels: "not a very nice mapping, all these <span style="color:red">transitions from one color to the next are linear</span> and we need to <span style="color:red">check in which of these transitions we are</span> to then do this explicit <span style="color:red">linear interpolation</span>"
- slide demo:
  - **option 1**: $(H,S,V) = (\theta, 1, 1)$ ie. on top of HSV cylinder, outer boundary (always bright + colorful)
    - **problem**: completely removes the magnitude and <span style="color:red">just shows direction of gradient</span> (&rarr; it's red where there is no gradient !)
    - **solution**: use the <span style="color:red">other 2 channels to visualize the **gradient magnitude**</span>
  - **option 2**: $(H,S,V) = (\theta, G, 1)$ ie when $\text{grad}=0$ use white, and when $\text{grad}>0$ then more colorful ($\text{grad}$ controls how close we are to the boundary of the cylinder/how far away from the center of the cylinder)
  - **option 3**: $(H,S,V) = (\theta, G, G)$ ie transition from black to colorful (instead of white to colorful as in 2.)

# 2.15 CNNs

- linear filters can be used as feature extractors, eg. in CNNs

# 2.16 Canny Edge Detector

- **problem**:
  - Problem with Prewitt and Sobel detector: there is no "knob" to configure <span style="color:red">how many edges you want</span> or to see <span style="color:red">how strong the edges have to be</span>. 
  - The Canny will give us a way to select how sensitive we want to be to the edges and also with <span style="color:red">some sane properties</span>, eg.
    - <span style="color:red">when you want fewer edges there will never be new edges</span> and
    - <span style="color:red">existing edges will either vanish or stay at their position</span> (ie. they will also not move to new positions).

<p align="center">
  <img src="https://i.ibb.co/mFCwCwt/Screenshot-from-2024-02-25-02-37-14.png" alt="Screenshot-from-2024-02-25-02-37-14" border="0">
</p>

- <span style="color:red">**2nd condition**</span>: "if we increase the **intensity of the smoothing $\sigma$** the edge stays where it is (aka it should not move)"
- <span style="color:red">**1st and 3rd condition**</span> specify what happens left and right of the edges

<p align="center">
  <img src="https://i.ibb.co/xhTBcbZ/Screenshot-from-2024-02-25-02-55-21.png" alt="Screenshot-from-2024-02-25-02-55-21" border="0">
</p>

- slide demo:
  - graph $f$ (black),$f'$ (green),$f''$ (red):
    - **black graph**: image
    - <span style="color:red">zero crossings</span> of **red graph** are where the edges are
  - look <span style="color:red">at left edge</span>:
    - the only thing we are allowed to do is to decrease the slope where we are locally at the edge, this means:
      - left of left edge: we are only allowed to increase values of $f$
      - right of left edge: we are only allowed to decrease values of $f$
  - the <span style="color:red">1st and 3rd condition</span> encode exactly this idea
    - $\partial^2x_u > 0$ detects whether we are left of the left edge
      - where $\partial_{\sigma}u > 0$ only allows to increase $f$
    - $\partial^2x_u < 0$ detects whether we are right of the left edge
      - where $\partial_{\sigma}u < 0$ only allows to decrease $f$
  - the situation <span style="color:red">at the right edge</span> is the opposite, but the 3 conditions apply there, too

- (i) **for $d=1$:** solutions of the heat equation $\partial_x^2u=\partial_\sigma u$ (a PDE), with $u(x,\sigma)$ fulfill these 3 properties
  - **for $d>1$:** using the <span style="color:red">Laplace operator</span> $\Delta_x$: $\Delta_xu = \sum_{i=1}^{d}\partial^2_{x_i}u$
- (ii) we require the initial value $u(x,0)=f(x)$ because <span style="color:red">for $\sigma=0$ we want to get back to the original image (no blur, but noisy)</span>
- (iii) **for $d=1$:** the unique solution is $u=(g_{\sqrt{2\sigma}}\ast f)(x)$, thus $g_\sigma$ is a suitable kernel for <span style="color:red">edge detection with a "knob" to configure the edge strength</span>
  - we just want that if we increase sigma then the edges decrease, but it does not matter how fast. In other words the scale of sigma does not matter
    - take $(\sigma^2)/2$ instead of $\sigma$
  - **for $d>1$:** <span style="color:red">**for all $d\in\mathbb{N}$**</span> the solution of the heat equation is $g_\sigma$ (exercise)
- (iv) the convolution can be expressed as a correlation $(g_{\sqrt{2\sigma}}\ast f)=(g_{\sqrt{2\sigma}}\star f)$
- (v) the <span style="color:red">**Canny Edge Detector**</span> computes
  - **size**: $\rho(x)=\sqrt{(\partial_{x_1}(g_{\sqrt{2\sigma}}\ast f))^2+(\partial_{x_2}(g_{\sqrt{2\sigma}}\ast f))^2}$
  - **direction of the edge** (direction of "steepest ascent"): $\theta(x)=\text{arctan2}((\partial_{x_1}(g_{\sqrt{2\sigma}}\ast f)), (\partial_{x_2}(g_{\sqrt{2\sigma}}\ast f)))$
    - the <span style="color:red">angle that the gradient has with the x-axis</span>
  - similar to the Sobel detector before, but here we use a Gaussian kernel instead of a Prewitt or Sobel kernel
  - as edges we consider all points where $\rho$ is strictly maximal locally in direction <span style="color:red">$(\sin(\theta),\cos(\theta))$ TODO</span>
- **implementation**: to implement this we will have to 
  - <span style="color:red">**discretize**</span>: ie instead of looking at all $\theta$s we <span style="color:red">only look at multiples of 45 degrees **by rounding $\theta$ to those multiples**</span> and then <span style="color:red">compare neighboring pixel values in direction $\theta$ and $\theta+180$</span>
    - figure below: **3 red boxes**: we compare the center value with the 2 neighboring pixels (at $\theta$ and $\theta+180$) and if the middle value is strictly bigger than the 2 outer values then you say there is an edge
  - <span style="color:red">**thresholding**</span>: disregard all edges for which $\rho<\text{threshold}$
    - $\text{threshold}$ is another parameter that you set; this will <span style="color:red">avoid interpreting small noise components as edges</span> (this noise will also lead to response in the gradient but the value may be quite small which is why we can just filter them out like this)

<p align="center">
  <img src="https://i.ibb.co/c3bfhjw/Screenshot-from-2024-02-25-04-40-24.png" alt="Screenshot-from-2024-02-25-04-40-24" border="0">
</p>

- slide demo:
  - <span style="color:red">the larger $\sigma$, the less edges</span> you see
  - <span style="color:red">edges stay where they are</span>, they do not move
  - <span style="color:red">no new edges appear</span>
  - small $\sigma$s give you even the tiny noise that you have in the intensities
  - but you can nicely <span style="color:red">control which kind of edge strength you want</span> to look at

# 2.17 Laplacian Sharpening

- **goal**: Sharpen an unsharp image
- **in 1D**: sharpening = emphasizing edges (edges = zeros of $f''$)
- <span style="color:red">**(i)**</span> $f-\tau f''$, where $\tau>0$
  - $\tau$: a measure for the <span style="color:red">degree of "edge amplification"</span>
  - **idea**:
    - in <span style="color:green">(2.16)</span>: at the zero crossing of the red line we are only allowed to make the slope of the edge (black line) <span style="color:red">**less steep**</span> (for increasing $\sigma$) (bec we do not want to move the edge to some other place)
    - **here**: we want to pronounce the edge &rarr; we have to make the slope of the edge <span style="color:red">**MORE steep**</span>. &rarr; subtracting the red line from the black line exactly achieves this effect
      - this works at all places in the domain, incl. the left as well as the right edge: for example:
        1. left of left edge: pos red values are subtracted from pos black curve
        2. right of left edge: neg red values are subtracted from pos black curve
        3. by 1. and 2. the slope of the black curve must increase
          - The same slope increase effect happens at the right edge.
- **for $d>1$**:
  - **problem**: edges occur in multiple directions (the gradient is orthogonal to the direction of the edge. We cannot simply look at positions where the Hessian is zero!)
  - **solution**: need a <span style="color:red">rotationally invariant</span> form of the 2nd derivative &rarr; <span style="color:red">**Laplace operator**</span>
  - <span style="color:red">**(ii)**</span> thus, $f-\tau\Delta_x f$
    - Berkels: facts about the <span style="color:red">**Hessian**</span>:
      - **its trace** is the **Laplace operator**
      - the sum of its eigenvalues is <span style="color:red">**invariant under rotation**</span> (but eigendirections can change!)
      - <span style="color:red">**key insight**</span>: all the <span style="color:red">mixed derivatives are not considered</span> by the Laplace operator. (exercise)
- **problem**: images are <span style="color:red">not smooth enough</span>
  - **solution**: like in <span style="color:green">(2.16)</span>, smoothen with $g_\sigma$
  - <span style="color:red">**(iii)**</span> thus, $(g_\sigma\star f)-\tau\Delta_x(g_\sigma\star f) = (g_\sigma-\tau\Delta_xg_\sigma)\star f$ (exercise)
- slide demo:
  - increase $\tau$ &rarr; sharper edges
  - <span style="color:red">**peppers image**</span>: **problem**: since Gaussian blur does not remove the stripe patterns <span style="color:red">we are also sharpening those stripe patterns</span>!

# C.2.1 Morphological Filters

# 2.18 Denoising of Objects

- <span style="color:red">**"objects"**</span> are different from images in that they have <span style="color:red">only $0$ or $1$ as values</span>, whereas images have a continuous value range.
- $f(x)=1$ if $\forall y: \chi_A(x+y)=1$ (universal quantifier) &rarr; "denoiser" of $\chi_A$
- $g(x)=1$ if $\exists y: f(x+y)=1$ (existential quantifier) &rarr; "grower" of $f$
- where
  - $A\subset\Omega$ is a <span style="color:red">**noisy object**</span> (ie. the "noise dots" also belong to $A$)
  - $\chi_A$ is the noisy image
  - $f$ is an image that takes the noisy image $\chi_A$ as input
  - $g$ is an image that takes the denoised image $f$ as input

# 2.19 Erosion and Dilation when $f(x)\in\\{0,1\\}$

- erosion = "denoiser" = $\ominus$
- dilation = "grower" = $\oplus$

# 2.20 Erosion and Dilation when $f(x)\in\\{0,1\\}$ - Alternative Formulation

# 2.21 Erosion and Dilation when $f(x)\in\left[0,1\right]$

- **generalization of** <span style="color:green">(2.19)</span>: when we plug in an image that has only values $0$ and $1$ by <span style="color:green">(2.20)</span> we know that the definitions for the characteristic functions coincide with this more general definition.
- (i) $f(x)\oplus B = \sup_{y\in B}f(x+y)$
- (ii) $f(x)\ominus B = \inf_{y\in B}f(x+y)$
- (iii) erosion and dilation coincide with the <span style="color:red">**minimum and maximum filter**</span>
  - **more precisely**: in <span style="color:green">(2.11 (v))</span> we saw: <span style="color:red">**median**</span> is just one operation that we could use. We could also use <span style="color:red">**minimum**</span> and <span style="color:red">**maximum**</span>. And we called this <span style="color:red">**rank-order filters**</span> (or <span style="color:red">**"rank filters"**</span>).
    - So <span style="color:red">**erosion**</span> and <span style="color:red">**dilation**</span> are in the same class of filters as the median, they are just a bit more specialized for the task here.
    - The <span style="color:red">only difference</span> is that
      - <span style="color:red">**in the discrete setting**</span> <span style="color:green">(2.11 (v))</span> we had a <span style="color:red">**rectangle as structuring element**</span>. Because we did the minimum or maximum or median <span style="color:red">**over a rectangle**</span> of pixels and since it is <span style="color:red">**finitely many values in this discrete rectangle**</span> we could also use maximum and minimum.
      - Here <span style="color:red">**in the continuous setting**</span> we must rely on $\sup$ and $\inf$ bec we do not know whether $\max$ and $\min$ will actually be attained.

# 2.22 Opening and Closing

- wenn $B$ und $-B$ gleich sind (zB. wenn $B$ ein Ball ist), dann macht das Minus keinen Unterschied, dh. dann ist $B = -B$
- (i) <span style="color:red">**opening**</span> = 1. erosion, 2. dilation
  - denoted by <span style="color:red">"open" circle</span> symbol $\circ$
  - **Memorize**: what we have used in the object denoising example <span style="color:green">(2.18)</span>
    - $f$ is applying erosion to $\chi_A$ and then to the result we apply dilation $g$ with $-B$
    - (notice, the minus sign before $B$, because in the definition of erosion and dilation there is a $+y$ in $f(x+y)=1$ and not $-y$)
- (ii) <span style="color:red">**closing**</span> = 1. dilation, 2. erosion
  - denoted by <span style="color:red">"closed" circle</span> symbol $\bullet$
  - **Memorize**: the closing <span style="color:red">"closes holes"</span>
    - also the hole would not reappear because if it is fully closed then the erosion cannot open it again

# 2.23 Segmentation with Background Equalization

- we had the isodata algorithm for the segmentation problem, but this algorithm gives a <span style="color:red">global threshold</span> and eg. <span style="color:red">uneven illumination</span> would cause local problems
- the **opening** is an **estimate of the background**:
  1. erosion removes this thin, bright writing
  2. dilation corrects the shrinkage caused by the erosion
- Here the structuring element $B$ must be wider than the width of the writing, otherwise it would not remove the writing
- in the end we get an **estimate of the background**: ie. the smudges stay, the general background stays, the illumination stays, but the writing is gone
- **idea**:
  - use this as <span style="color:red">**pre-processing**</span> step before isodata:
    - <span style="color:red">subtract</span> the estimated background from the image
  - this compensates
    - brightness variations and
    - small "smudges" (which can be seen as local brightness variations)
  - this is called <span style="color:red">**white-top-hat transform**</span>
  - **for white background**: <span style="color:red">**black-top-hat transform**</span>
  - connection: $f\bullet B - f = (-f) - (-f)\circ B$
    - from the duality of opening and closing <span style="color:green">(2.24)</span>
    - $\boxed{f - f\circ B} = f - (- ((-f)\bullet B)) \boxed{= f + (-f)\bullet B} \Rightarrow_{(f\to -f)} \boxed{(-f) - (-f)\circ B = -f + f\bullet B}$ which is the same result as above

<p align="center">
  <img src="https://i.ibb.co/Wc8gg6Y/Screenshot-from-2024-02-28-11-54-32.png" alt="Screenshot-from-2024-02-28-11-54-32" border="0">
</p>

# 2.24 Properties of the Morphological Operators

- $f,g: \mathbb{R}^d\to \left[0,1\right]$
- erosion, dilation:
  - **duality** $f\ominus B = - ((-f)\oplus B)$
  - **translational invariance** $T_hf\ominus B=$
    - if I translate my image by a vector $h$ and then apply the erosion this is the same as if I would first apply the erosion and then shift everything. &rarr; rather self-explaining (bec it does not matter where the structures are, the shrinkage and the growth is independent of the position of where we are in the image)
  - **monotonicity** $f\leq g \Rightarrow$
  - **distributivity**
    - here, the combination of "minimum and erosion" (or "maximum and dilation") is important - the other combinations are not allowed (&rarr; because the maximum is not compatible with the $\inf$, when you want to exchange that)
    - pointwise minimum $\land$ and maximum $\lor$ behave like $\cap$ and $\cup$: $f\land g = \chi_{A\cap B}$ and $f\lor g = \chi_{A\cup B}$
      - <span style="color:red">Gut zum Merken!</span> &rarr; vorstellen: <span style="color:red">2D Venn Diagramm</span> mit Mengen $A$ und $B$:
        - <span style="color:red">**Schnitt**</span> nimmt immer den <span style="color:red">**"tieferen" Wert**</span> von $A$ und $B$ &rarr; <span style="color:red">minimum</span>
        - <span style="color:red">**Union**</span> nimmt immer den <span style="color:red">**"höheren" Wert**</span> von $A$ und $B$ &rarr; <span style="color:red">maximum</span>

<p align="center">
  <img src="https://i.ibb.co/27xdK2Q/Screenshot-from-2024-02-28-13-13-09.png" alt="Screenshot-from-2024-02-28-13-13-09" border="0">
</p>

- opening, closing:
  - fulfill all <span style="color:red">except distributivity</span> and additionally:
  - **non-increasingness**
    - durch opening ("removing noise") kann $f$ nur kleiner werden oder gleich bleiben
  - **non-decreasingness**
    - durch closing ("closing holes") kann $f$ nur größer werden oder gleich bleiben
  - **idempotence**
    - ie. if you try to open sth multiple times you will not change anything &rarr; there is no reason to iterate opening or closing
- **problem**:
  - one thing mathematicians wonder when they see these properties is whether these properties already determine what these operators are or if there could be other operators that fulfill these properties

# 2.25 Summary of next two Propositions

- (i) **just a lemma**: <span style="color:red">**distributivity holds also for infinitely many arguments**</span>, where in the case of an infinite number of arguments you would take the infimum over <span style="color:red">all</span> functions instead of taking the minimum over $f$ and $g$ ( $=(f \land g)$ ) only, see <span style="color:green">(2.26)</span>
  - a lemma needed for the proof of (ii)
- (ii) **main proposition**: dilation and erosion are <span style="color:red">**the only**</span> translational invariant operators that fulfill the generalized distributivity in (i), see <span style="color:green">(2.27)</span>
  - **proof**: using (i)

# 2.26 Infinite Distributive Law

- $f_i\in F(\mathbb{R}^d,\left[0,1\right])$, (Warning: not binary images!)
- (i) generalized distributivity with the <span style="color:red">maximum</span> $(\lor_i f_i)\oplus B = \lor_i(f_i\oplus B)$
- (ii) generalized distributivity with the <span style="color:red">minimum</span> $(\land_i f_i)\ominus B = \land_i(f_i\ominus B)$
- where $\land_if_i := \inf_i f_i$ and $\lor_if_i := \sup_i f_i$ (<span style="color:red">instead of $\min$ and $\max$!</span>)

# 2.27 Dilation and Erosion are the only operators that fulfill <span style="color:green">(2.26)</span>

- **conditions**:
  - mapping $D$ maps **binary images** to **binary images**, ie. $D: F(\mathbb{R}^d, \\{0,1\\})\to F(\mathbb{R}^d, \\{0,1\\})$
  - mapping $D$ fulfills the <span style="color:red">translational invariance</span> property <span style="color:green">(2.24)</span>
  - mapping $D$ fulfills the <span style="color:red">generalized distributivity either (i) with the maximum or (ii) with the minimum</span> <span style="color:green">(2.26)</span>
  - some technical assumptions to exclude "pathological examples" (but they are necessary!)
- (i) then there is a nonempty $B$ s.t. $$D(f) = f\oplus B\quad \forall\, \text{binary images}\,f$$
  - This means that the <span style="color:red">dilation with $B$</span> is <span style="color:red">**characterized by**</span> these properties:
    - if you know $D$ is translationally invariant and fulfills the conditions above, then it is <span style="color:red">**necessarily**</span> a <span style="color:red">dilation with some set $B$</span>.
- (ii) then there is a nonempty $B$ s.t. $$D(f) = f\ominus B\quad \forall\, \text{binary images}\,f$$

# 2.28 Contrast Invariance of Erosion and Dilation

- **conditions**:
  - for all continuous, increasing intensity transformations $T: \mathbb{R}\to\mathbb{R}$ (&rarr; <span style="color:green">(Chapter 1)</span>)
  - $f: \mathbb{R}\to \left[0,1\right]$ (not binary!)
- then,
  - (i) $T(f)\oplus B = T(f\oplus B)$
  - (ii) $T(f)\ominus B = T(f\ominus B)$
- in words: "it does not matter whether we change the contrast first or whether we dilate first"
- **proof**:
  - $(T(f)\ominus B)(x) = \inf_{y\in B} T(f(x+y)) = T(\inf_{y\in B} f(x+y)) = T((f\ominus B)(x))$
  - interchangeability of infimum and this continuous, increasing functions $\inf_{y\in B} T(f(x+y)) = T(\inf_{y\in B} f(x+y))$ (shown in exercise)

# 2.29 Generalization of Erosion and Dilation

- replace the "flat" structuring element with a function $b(y)$, ie. $(f\oplus b)(x) := \sup_{y\in B} (f(x+y)+b(y))$
  - so far we were shifting this $B$ (probably a circle), but now this $b$ is somehow shaped and it can adjust your function graph to put different weights to pronounce more what is going on in the middle of the element.
  - for $b\equiv 0$ equivalent to the usual dilation
