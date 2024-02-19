---
title: "Mathematical Methods of Signal and Image Processing"
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

# Preliminaries

## supremum and infimum

- **infimum**: The **infimum** of a subset $S$ of a partially ordered set, $P$, assuming it exists, does not necessarily belong to $S$. If it does, it is a **minimum** or least element of $S$.
- **supremum**: Similarly, if the **supremum** of $S$ belongs to, $S$, it is a **maximum** or greatest element of $S$.
  - Theorem: The supremum of a set of **real numbers** always exists, in the worst case it is $\pm\infty$.
    - "The **completeness** of the real numbers implies (and is equivalent to) that any bounded nonempty subset $S$ of the real numbers has an infimum and a supremum. If $S$ is not bounded below, one often formally writes $\inf{S}=-\infty$ If $S$ is empty, one writes $\inf{S}=+\infty$", [Wikipedia](https://en.wikipedia.org/wiki/Infimum_and_supremum#Infima_and_suprema_of_real_numbers)
- **essential supremum** ("supremum almost everywhere"): While the exact definition is not immediately straightforward, intuitively the essential supremum of a function is the smallest value that is greater than or equal to the function values everywhere **while ignoring** what the function does at a **set of points of measure zero**.
  - For example, if one takes the function $f(x)$ that is equal to zero everywhere except at $x=0$ where $f(0)=1$, then the supremum of the function equals one. However, its **essential supremum** is zero because we are allowed to ignore what the function does at the single point where $f$ is peculiar.
- **essential infimum**: The essential infimum is defined in a similar way.

## supremum norm, $\infty$-norm, uniform norm

- to measure the "distance between two functions", cf. [uniform convergence theorem](https://www.youtube.com/watch?v=O2HKxNcom7g&list=PLkZG1WV2RkYJTALH2dvQiQvUGXyHawkhG&index=19)
- [Wikipedia](https://en.wikipedia.org/wiki/Uniform_norm):
  - the **uniform norm** is also called the **supremum norm**, the **Chebyshev norm**, the **infinity norm**, or, when the supremum is in fact the maximum, the **max norm**
  - the **uniform norm** (or **sup norm**) assigns to real- or complex-valued bounded functions $f$ defined on a set $S$ the non-negative number, $$\begin{equation*} \|f\|_{\infty }=\|f\|_{\infty ,S}=\sup_{s\in S}\{\,\vert f(s)\vert \,\} \end{equation*}$$ 
- in measure theory: $$\|f\|_{L^\infty(\Omega) }=\inf_{\mu(N)=0}\sup_{s\in\Omega\backslash N}\{\,\vert f(s)\vert \,\}$$

## pointwise convergence

- $\forall \tilde{x}\in I \,\forall \epsilon > 0 \,\exists N \,\forall n \geq N : \| f_n(\tilde{x}) - f(\tilde{x}) \| \lt \epsilon$
- does **not** imply uniform convergence
- does **not** conserve function properties:
  - continuity

## uniform convergence

- $\forall \epsilon > 0 \,\exists N \,\forall n \geq N \,\forall \tilde{x}\in I : \| f_n(\tilde{x}) - f(\tilde{x}) \| \lt \epsilon$
- converging function $f_n$ must be inside the <span style="color:red">**epsilon tube**</span> around $f$
- supremum norm $\lVert f_n - f\rVert_{\infty} \to 0$ for $n \to \infty$
- implies pointwise convergence
  - therefore, stronger than pointwise convergence
- conserves function properties:
  - continuity ($f_n$ continuous $\implies$ the limit function $f$ is also continuous)
  - boundedness ($f_n$ bounded $\implies$ the limit function $f$ is also bounded)

## open, closed

[Wikipedia](https://en.wikipedia.org/wiki/Closed_set):

"a **closed set** is a set whose complement is an open set."

"In a **topological space**, a closed set can be defined as a set which contains all its limit points."

"In a **vollständig/complete metric space**, a closed set is a set which is closed under the limit operation."

- Eine Teilmenge $U \subset X$ ist genau dann (&rarr; Bredies)
  - **offen**/**open**, falls sie nur aus **inneren Punkten** besteht, also es für jedes $x \in U$ ein $\epsilon > 0$ derart gibt, dass $B_\epsilon ( x ) \subset U$
  - **abgeschlossen**/**closed**, wenn sie nur aus **Berührpunkten** besteht, das heißt für jedes $x \in X$ für welches die Mengen $B_\epsilon ( x )$ für jedes $\epsilon > 0$ die Menge $U$ schneiden, auch $x \in U$ gilt

## closure

- **Abschluss** von $U$, notiert mit $\overline{U}$, die Menge aller Berührpunkte (&rarr; Bredies)

## dense

- **idea**: "a subset $A$ of a topological space $X$ is said to be dense in $X$ if every point of $X$ either belongs to $A$ or else is arbitrarily "close" to a member of $A$, [Wikipedia](https://en.wikipedia.org/wiki/Dense_set)
  - for instance, the **rational numbers** are a dense subset of the **real numbers** because every real number either is a rational number or has a rational number arbitrarily close to it"
- eine Teilmenge $U\subset X$ ist **dicht** in $X$, falls $\overline{U} = X$ (&rarr; Bredies)
- **approximation by polynomial functions** (Weierstrass): "any given **complex-valued continuous function** defined on a closed interval $[a,b]$ can be uniformly approximated as closely as desired by a **polynomial function**", [Wikipedia](https://en.wikipedia.org/wiki/Dense_set)

## separable

- $X$ wird als **separabel** bezeichnet, wenn es eine abzählbare dichte Teilmenge gibt (&rarr; Bredies)

## compact

<span style="color:red">**Merke**: "compact" = "closed and bounded"</span>

- **The idea** is that a compact space has no "punctures" or "missing endpoints", i.e., it includes all limiting values of points., [Wikipedia](https://en.wikipedia.org/wiki/Compact_space)
  - For example, the **open interval** $(0,1)$ would not be compact because it excludes the limiting values of $0$ and $1$, whereas the **closed interval** $[0,1]$ would be compact.
  - Similarly, the **space of rational numbers** $\mathbb {Q}$ is not compact, because it has infinitely many "punctures" corresponding to the irrational numbers, and the **space of real numbers** $\mathbb {R}$ is not compact either, because it excludes the two limiting values $+\infty$ and $-\infty$. However, the **extended real number line** would be compact, since it contains both infinities.
- **Formally**: In general, a set $K$ is called **compact**, if every sequence in $K$ has a converging subsequence with limit in $K$., lecture appendix
  - For $K \subset \mathbb{R}^d$, compactness of $K$ is equivalent to $K$ being closed and bounded. This equivalence is known as **Heine–Borel theorem**.

**Heine–Borel theorem**: For a subset $S$ of Euclidean space $\mathbb{R}^n$, the following two statements are equivalent:
- $S$ is **closed** and **bounded**
- $S$ is **compact**, that is, every open cover of $S$ has a finite subcover.

## topology

- **topology**: "die Menge $\mathcal{O}$ der offenen Mengen wird als die Topologie des topologischen Raumes $(X,\mathcal{O})$ bezeichnet", [Wikipedia](https://de.wikipedia.org/wiki/Topologie_(Mathematik)#Topologischer_Raum)
  - "Üblicherweise werden topologische Räume in den Lehrbüchern über die **offenen Mengen** definiert"
  - "Wird eine beliebige **Grundmenge** mit einer **Topologie** (einer topologischen Struktur) versehen, dann ist sie ein **topologischer Raum**, und ihre Elemente werden als **Punkte** aufgefasst. Die Topologie des Raumes bestimmt sich dann dadurch, dass bestimmte Teilmengen als **offen** ausgezeichnet werden."

## Function Spaces

### $C(U,Y)$

- Bredies 2.4:
  - **"pointwise continuous"** vs. **"continuous"** vs. **"uniformly continuous"**:
    - Ist $F$ in jedem Punkt $x \in U$ stetig, so nennt man $F$ einfach nur **stetig**, hängt darüber hinaus das $\delta$ nicht von $x$ ab, so ist $F$ **gleichmäßig stetig**
- Bredies 2.6:
  - $C(U,Y)$ is **not** $C(\overline{U},Y)$
    - in $C(U,Y)$ the function $F$ must be only **continuous**
    - in $C(\overline{U},Y)$ the function $F$ must be **bounded** and **uniformly continuous**
  - $C(U,Y)$ is **not** a normed space, but $C(\overline{U},Y)$ **together with** $\Vert F\Vert_{\infty}$ is
- Comparison: [$L^2$ vs. $C(U,Y)$](#l2-vs-cuy)

### $L^2$

- it is often convenient to think of $L^2(\mathbb{R}^n)$ as the **completion** of the **continuous functions** with respect to the $L^2$-norm, [mathworld.wolfram](https://mathworld.wolfram.com/L2-Space.html)
- examples for $L^2$ functions:
  - **Bounded functions**, defined on $\[ 0 , 1 \]$, are square-integrable. These functions are also in $L^p$, for any value of $p$., [wikipedia](https://en.wikipedia.org/wiki/Square-integrable_function#Examples)
- Comparison: [$L^2$ vs. $C(U,Y)$](#l2-vs-cuy)

### $L^2$ vs. $C(U,Y)$

- not all continuous functions are in $L^2$,
  - **counterexample**: the constant function $f=1$
- not all $L^2$ functions are continuous,
  - **counterexample**: the characteristic function of the finite set $A$, $\chi_{A}$

## Series

### Absolute Convergence

Wikipedia:

- "**absolutely convergent** series behave "nicely". For instance, **rearrangements** do not change the value of the sum. This is not true for **conditionally convergent** series"
- example: **alternating sum** $S=1-1+1-1+1-\dots$
  - "What is the value of $S$?"
  - "The answer is that because $S$ is **not absolutely convergent**, grouping or rearranging its terms changes the value of the sum"
  - "In fact, the series $1 - 1 + 1 - 1 + \dots$ **does not converge**, so $S$ does not have a value to find in the first place"
  - "A series that is absolutely convergent does not have this problem: grouping or rearranging its terms does **not** change the value of the sum"
- "If $G$ is complete with respect to the metric $d$, then **every absolutely convergent series is convergent**"
- **conditionally convergent**: "If a series is convergent but not absolutely convergent, it is called conditionally convergent. An **example** of a conditionally convergent series is the **alternating harmonic series**."
  - the **alternating harmonic series** converges to $\ln{2}$ or $\frac{3}{2}\ln{2}$ depending on the arrangement

# 1.1 Digital Images

- **spatial dimension**: $d$
- **d-dim cuboid**: $\Omega$, eg. $d=2$ and $\Omega=(a_1,b_1)\times(a_2,b_2)=(0,8)\times(0,6)$
- **value range**: $V$
- **image**: a mapping $f: \Omega \to V$, where $\Omega\subset \mathbb{R}^d$
  - **continuous gray scale image**: $f: (0,1)^2 \to \[0,1\]$
- **image size**: the vector $\underline{m}=(m_1,\dotsc,m_d)$ (aka "number of pixels"), e.g. $\underline{m}=(4,3)$
- **grid node/cell center**: $x^{\underline{j}}\in \Omega$ (aka "the center of each pixel", eg. $x^{(4,3)}$)
- **grid**: the $d$-array $\underline{X}=(x^{\underline{j}})_{\underline{j}\in I}$
  - $m$ and the cuboid $\Omega$ imply a grid
- **grid size**: the vector $\underline{h}$, where $h_1$ is the grid size along the x-axis, $h_2$ along the y-axis, etc.
  - **regular grid**: "all pixels have the same shape"
    - **cartesian grid**: $h_1=\dotsc=h_d$, "all grid sizes are the same"
- **cells**: $c_{\underline{j}}=\bigl\\{x\in\Omega\ \|\ \lvert x_i-x^{\underline{j}}_i\rvert <\frac{h_i}{2}\ \forall i \in \\{1,\dotsc,d\\}\bigr\\}$
  1. union of all closures of $c_\underline{j}$ equals closure of $\Omega$
  2. intersection of two cells is empty
  3. no cell is empty
- **cell-centered**: eg. used for finite differences (FDM)
- **mesh-centered**: eg. used for finite elements (FEM)
- **digital/discrete/pixel image**:
  - a $d$-array $F=(f_\underline{j})_{\underline{j}\in I}$
    - ie. a discrete image is **a matrix** and NOT a step function
  - **values** (eg. **gray value**): $f_\underline{j} \in V$
  - **examples**:
    - $d=2$, $\Omega = (0,8) \times (0,6)$ and $\underline{m}=(4,3)$
    - **grid:** $$\begin{equation}
                  \begin{aligned}
                    \underline{X} =  
                    \begin{pmatrix}
                      (1,5) & (3,5) & (5,5) & (7,5) \\
                      (1,3) & (3,3) & (5,3) & (7,3) \\
                      (1,1) & (3,1) & (5,1) & (7,1)
                    \end{pmatrix} 
                  \end{aligned}
                  \end{equation}$$
     - **discrete image (on this grid):** $$\begin{equation}
                                            \begin{aligned}
                                              F =  
                                              \begin{pmatrix}
                                                0 & 1 & 2 & 3 \\
                                                0 & 1 & 2 & 3 \\
                                                0 & 1 & 2 & 3
                                              \end{pmatrix} 
                                            \end{aligned}
                                            \end{equation}$$
- **actual image**: value range $V$ is **finite** and **quantized**
  - **values**: for grayscale image with $n$ bit precision $2^n$ possible values, eg. for 8 bit $V=\\{0,\dotsc,255\\}$
- **pixel** (for $d=2$) or **voxel** (for $d=3$): the tuple $(c_\underline{j},f_\underline{j})$
- **pixelated function**: a function $f: \Omega \to V$ that is constant on each cell, ie. $f\rvert_{c_\underline{j}}$ is constant for all $\underline{j}\in I$
  - **cell boundaries (of a pixelated function)**: $L=\Omega\setminus \bigcup_{\underline{j}\in I} c_\underline{j}$
    - **values**: we choose $f(x)=f(c_\underline{j})$ for each cell boundary point $x\in L$, where $\underline{j}$ is the **unique** cell above or to the right of each cell boundary point $x\in L$ (at "crossings" $\underline{j}$ is the cell above **and** to the right of $x\in L$, and the value on the outer border of $\Omega$ is not defined because it is not part of $\Omega$)
      - Note: this is just one possible choice
        - the cell boundaries are a **Lebesgue null set** $\Rightarrow$ values can be chosen freely in the sense of the Lebesgue measure
- **scanning** (**measurement of** $f$): $$(f,\psi)_{L^2}=\int_{\Omega} f(y)\psi(y)dy$$ (scalar product in $L^2$)
  - here:
    - $f\in L^2(\Omega)$ (Berkels: "an image where you can integrate the squared intensity, so it has to be finite")
    - $\psi$ is a **test function**: $\psi \in L^2(\mathbb{R}^d)$, $\psi \geq 0$ and $\int_{\mathbb{R}^d}\psi(x)dx=1$
      - **example test function**: **CCD sensor**: $$\begin{equation*}
                                       r_h(x) = \begin{cases}
                                       \frac{1}{h_d} &\lVert x\rVert_\infty < \frac{h}{2}\\
                                       0 &\text{else}
                                       \end{cases}
                                       \end{equation*}\ \text{and}\ (f,r_h(x-\cdot))_{L^2}=\int_\Omega f(y)r_h(x-y)dy=:r^h_x\left[f\right].$$
      - $h$: sensor width
      - $x^{\underline{j}}$: sensor position
      - <span style="color:red">intensities $f$ that fall in the sensor are averaged</span> 
      - with this CCD sensor one gets the discrete image $(r_{x^\underline{j}}^h\left[f\right])_{\underline{j}\in I}$ (which is a **matrix** and not a step function, see def. of **discrete image**, and $x^{\underline{j}}\in \Omega$ are **grid nodes**, see def. above)
- **coordinate systems**: in digital images rotated by $90^\circ$ clockwise (ie. 1st axis is vertical, 2nd axis is horizontal)

# 1.6 Point Operators / Intensity Transforms

- **intensity transformation**: $T: \mathbb{R} \to \mathbb{R}$
- **intensity transformed image**: $T\circ f$, where $f: \Omega \to \mathbb{R}$ is an image.

## clipping

- **clipping**: $$\begin{equation*} T_{\left[a,b\right]}^{\text{clip}}: \mathbb{R} \to \left[a,b\right],\ s \mapsto \begin{cases}
                                            a &s\leq a\\
                                            b &s\geq b\\
                                            c &\text{else}
                                            \end{cases}
                  \end{equation*}$$
- eg. in electron microscopy: sometimes just normalization is not enough (eg. outliers, very bright pixels) and you **must** apply **clipping** **first** to make certain structures visible and normalization **after** that

## normalization

- **normalization**: $$\begin{equation*} T_{\left[a,b\right]}^{\text{norm}}: \mathbb{R} \to \left[0,1\right],\ s \mapsto T_{\left[0,1\right]}^{\text{clip}}\left(\frac{s-a}{b-a}\right) \end{equation*}$$
  - sometimes normalization is not enough (eg. presence of outliers): in that case first clipping then normalization is required

## thresholding

- **thresholding**: $$\begin{equation*} T_{\theta}^{\text{threshold}}: \mathbb{R} \to \bigl\{0,1\bigr\},\ s \mapsto \begin{cases}
                                    0 &s\leq \theta\\
                                    1 &s > \theta
                                    \end{cases}
                  \end{equation*}$$
  - Heaviside function: $H := T_0^{\text{threshold}}$

## gamma correction

- **gamma correction**: $$\begin{equation*} T_{\gamma}: \left[0,1\right] \to \left[0,1\right],\ s \mapsto s^\gamma \end{equation*},\ \text{where}\ \gamma > 0$$
  - for $\gamma < 1$ strictly concave 
  - for $\gamma > 1$ strictly convex
  - **convex**:
    - if the line segment between any two distinct points on the graph of the function lies **above the graph** between the two points
    - if the epigraph is a convex set
      - **epigraph of a function**: the set of points on or above the graph of the function
    - convex function graph is shaped like a **cup**
      - note: a linear function is **both** convex and concave
  - for $\gamma < 1$: compress bright areas, expand dark areas, ie. more details in dark areas visible, but less details in the bright areas

## log transformation

- **log transformation**: $$\begin{equation*} T_{\log}: \left[0,\infty\right) \to \left[0,\infty\right),\ s \mapsto \log_2(1+s) \end{equation*},\ \text{where}\ \gamma > 0$$
  - **power spectrum**: absolute value of the Fourier transform
  - log transformation often used to visualize the power spectrum
  - choose base $2$, so that $\left[0,1\right]$ is mapped to $\left[0,1\right]$ **bijectively** (but you could choose other bases as well)
    - any continuous strictly monotonic function is **bijective** between its domain and range. (follows from the **intermediate value theorem**)
    - assume $b$ positive and unequal to 1 ($b=1$ and $b\leq 0$ are undefined in the usual log definition)
    - log is strictly increasing (for $b > 1$), or strictly decreasing (for $0 < b < 1$)
    - log is bijective for $b > 1$ and $0 < b < 1$
  - allows to catch very big values and bring them close to much smaller values
  - eg. useful **in electron microscopy**: some details in $\lvert FFT\left[f - \overline{f}\right] \rvert$ become only visible if **first** $T^{\log}$ and **then** $T^{\text{norm}}$ is applied instead of the other way around ($T^{\log}$ does not help much **after** $T^{\text{norm}}$ has been applied) (**gamma correction** could **not** do this unless you would use an extremely large gamma value that is exactly valid for the specific image, ie. gamma depends on the dataset, whereas the log transform does not have these problems)

## Intensity Transforms using Global Statistics / Histograms

- these global statistics of the intensity distributions in the image are computed once
- based on that we can construct some intensity transform
- <span style="color:red">the global statistics are represented by a **histogram**</span>

### 1.7 Discrete Histogram of $F$, Discrete CDF of $F$

- $F$ is a discrete image

### 1.8 Characteristic Function $\chi_{A}$, Level Set of $f$, Volume of a Bounded Set $A$

- $f$ is a pixelated image

### 1.9 Continuous Histogram, CDF of $f$, Discrete CDF of $f$

- $f$ is a pixelated image
- The continuous histogram $H_f$ is the distributional derivative of the CDF $G_f$.

### 1.10 Binning

- approximate the histogram of a continuous function discretely

### 1.11 Histogram Equalization

### 1.12 Histogram Matching

### 1.13 Segmentation by Thresholding / Isodata Algorithm

<span style="color:red">Note: Like 1.11 and 1.12 this method is also **histogram based**!</span>

- <span style="color:red">**Idea**:</span> **fixed point equation**: Find a threshold $\theta$ s.t. $\theta = \varphi(\theta)$
  - where $\varphi(\theta) := \frac{1}{2}(\int_{\\{f\leq\theta\\}}^\text{avg}f(x)dx + \int_{\\{f\gt\theta\\}}^\text{avg}f(x)dx)$
  - where $\int_A^\text{avg}f(x)dx := \frac{1}{\text{Vol}(A)}\int_A f(x)dx$
- <span style="color:red">**Method**:</span> **fixed point iteration**: $\theta^{n+1} = \varphi(\theta^n)$ with IV $\theta^1\in(0,1)$, typically $\theta^1 = \frac{1}{2}(\inf{f} + \sup{f})$ (mean of the value range)
  - **Implementation**: Isodata Algorithm

**What is a fixed point?**

- **fixed point**: Formally, $c$ is a fixed point of a function $f$ if $c$ belongs to both the domain and the codomain of $f$, and $f(c) = c$.
For example, if $f$ is defined on the real numbers by $f(x) = x^2 − 3x + 4$, then $2$ is a fixed point of $f$, because $f(2) = 2$. 
  - "a fixed point is an element that is mapped to itself by the function."

**Under which conditions does the fixed point iteration converge?**

- Proof of convergence of the sequence $\theta^n$:
  - **Monotone Convergence Theorem**: 
    1. If a sequence of **real numbers** is <span style="color:red">increasing and bounded above</span>, then its supremum is the limit.
    2. If a sequence of **real numbers** is <span style="color:red">decreasing and bounded below</span>, then its infimum is the limit.
    3. If $(a_{n})$ with $(n\in \mathbb{N})$ is a monotone sequence of **real numbers** (i.e., if $a_n \leq a_{n+1}$ for every $n \geq 1$ or $a_n \geq a_{n+1}$ for every $n \geq 1$), then this sequence has a finite limit **if and only if** the sequence is bounded.
  - $\varphi$ is <span style="color:red">increasing</span> (but not necessarily strictly increasing)
    - because it is a sum of two increasing functions (the two average gray value integrals)
    - **phth remember**: $\theta^n$ can be decreasing, but $\varphi$ is <span style="color:red">always</span> increasing!
  - $\varphi(\[0,1\])\subset\[0,1\] \Rightarrow \varphi$ is <span style="color:red">bounded</span>
    - because the two average gray value integrals are in $\[0,1\]$, so $\varphi$ must be in $\[0,1\]$, too
- **problem**: just because $\theta^n$ converges that does not mean its **limit is a fixed point**
- **problem**: moreover, we do not know if a fixed point even **exists**
- Proof of existence of a fixed point:
  - **fixed point theorems** assuming a **continuous** function $f$:
    - **Brouwer's fixed point theorem**: "states that for any **continuous** function $f$ mapping a nonempty compact convex set to itself, there is a point $x_0$ such that $f(x_0) = x_0$."
      - "The simplest forms of Brouwer's theorem are for continuous functions $f$ from a **closed interval** $I$ in the real numbers to itself or from a **closed disk** $D$ to itself."
      - "A more general form than the latter is for continuous functions from a **nonempty convex compact subset $K$ of Euclidean space** to itself."
  - **fixed point theorems** <span style="color:red">without</span> assuming a **continuous** function $f$:
    - "Every monotonic <span style="color:red">non-decreasing</span> $\varphi: \[0,1\] \to \[0,1\]$ has a fixed point.", a special case of the **Knaster-Tarski fixed point theorem** in order theory, note: in **order theory** "monotonic" means "non-decreasing", exercise
    - "If $\varphi$ is **<span style="color:red">not</span> continuous**, then the **limit of** $\theta^n$ is **not** necessarily **a fixed point**."
- thus, we **must** assume $\varphi$ is **continuous** in order to guarantee that the Isodata Algorithm will converge to a fixed point

**How to implement the Isodata Algorithm?**

- use the histogram $H_f$ to compute the average gray values $\int_{\\{f\leq\theta\\}}^\text{avg}f(x)dx$ and $\int_{\\{f\gt\theta\\}}^\text{avg}f(x)dx$
  - these integrals are approximated as discrete sums
  - the continuous histogram $H_f$ is approximated as the histogram of the discrete image $H_F$
  - **intuitively**: like discrete and continuous **center of mass** formula

# 2.0 Local Operators

## 2.1 Moving Average

**Task**: Denoising

- image: $f: \mathbb{R}^d \to \mathbb{R}$
- image without noise: $f_0: \mathbb{R}^d \to \[0,1\]$
- noise: $n: \mathbb{R}^d \to \mathbb{R}$
- $M_rf(x) = \int^\text{avg}_{B_r(x)}f(y)dy$
  - where $\int_A^\text{avg}f(x)dx := \frac{1}{\text{Vol}(A)}\int_A f(x)dx$
  - you can use either a ball in 2-norm or in $\infty$-norm
- $M_rf(x) = \int_{\mathbb{R}^d} f(x+y)\left(\frac{1}{\| B_r(0) \|}\chi_{B_r(0)}(y)\right)dy$ ($M_r$ as a **linear filter**, $\chi_{B_r(0)}$ as **filter function**)

## 2.2 Cross-Correlation, Linear Filter

- $\left(\psi\star f\right)(x) = \int_{\Omega}\psi(y)f(x+y)dy$
- linear filter with kernel $\psi$: $M_\psi: f\mapsto(x \mapsto \left(\psi\star f\right)(x))$

## Convolution vs. Cross-Correlation

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

## 2.3 Properties of the cross-correlation

### 2.3 (i) Boundedness of the norm of the cross-correlation

- $f\in L^p$ and $g\in L^q$, then
  - (i) **existence/finiteness of the integral**: $(f\star g)\in L^r$, where $\frac{1}{r}+1 = \frac{1}{p}+\frac{1}{q}$
  - (ii) **Young's convolution inequality** for cross-correlation: $\\|f\star g\\|\_{L^r} = \\|f\\|\_{L^p}\\|g\\|\_{L^q}$
  - (iii) $(f\star g)(x) = (g\star f)(-x)$ (reflection about the y-axis)
    - <span style="color:red">but</span> for convolution: $(f\ast g)(x) = (g\ast f)(x)$
  - (iv) $\\|f\star g\\|\_{L^r} = \\|g\star f\\|\_{L^r}$

### 2.3 (ii) Inheritance of Differentiability, Derivatives of the Correlation

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

### 2.3 (iii) Approximating any Function with a Differentiable Function

- **conditions**:
  - $\psi\in L^1$, $\psi\geq 0$ and $\int_{\mathbb{R}^d}\psi(x)dx=1$
  - **$\psi_\epsilon$ a scaled version of $\psi$**: for $\epsilon > 0$, $\psi_\epsilon: \mathbb{R}^d \to \mathbb{R}, x \mapsto \frac{1}{\epsilon^d}\psi(\frac{x}{\epsilon})$
    - factor $\frac{1}{\epsilon^d}$ chosen s.t. integral over $\mathbb{R}^d$ stays $1$
    - gets narrower along $x$-axis and higher along $y$-axis as $\epsilon\to 0$
  - $f\in L^\infty$
- (i) if $f$ is **continuous** in point $x$ then,
  - $\lim_{\epsilon\to 0}{(\psi_\epsilon\star f)(x)} = f(x)$, ie. $(\psi_\epsilon\star f)$ converges to $f$ in point $x\in \mathbb{R}^d$ (<span style="color:red">pointwise convergence</span>)
- (ii) if $f$ is **uniformly continuous** then,
  - $(\psi_\epsilon\star f)$ converges to $f$ uniformly on each compact subset of $\mathbb{R}^d$ (<span style="color:red">uniform convergence</span>)
- tool that helps us approximating images $f$
  - mainly useful for proofs
  - the nice thing is $f$ can be <span style="color:red">any</span> function, we only need boundedness for $f$
    - then, the limit on the lhs of (i) gives us a way to approximate this $f$ with the differentiable function $(\psi_\epsilon\star f)$
  - it is often easier to prove s.th. for a differentiable function and then show that it also holds in the limit
    - so, this is a tool to show properties of the limit $f(x)$ on the rhs of (i)

# 3.0 Frequency Domain

## 3.1 Fourier Transform

- "FT of $f$ in $\omega$": $\mathcal{F}f(\omega):=\hat{f}(\omega) = \frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}f(x)e^{-ix\cdot\omega}dx$
  - weighted sum of complex exponentials
- "the continuous FT": $\mathcal{F}: f \to \mathcal{F}f$
- **problem**:
  - $\mathcal{F}$ gives us a very different representation of the image in which we can eg. change some frequencies to pronounce s.th., but we <span style="color:red">need to go back</span> after that.
  - unless we can <span style="color:red">invert $\mathcal{F}$</span> it is useless for any kind of image processing &rarr; Is $\mathcal{F}$ invertible? If not, how can we change $\mathcal{F}$ to make it invertible?

## 3.2 Properties of the mapping $\mathcal{F}$

- $\mathcal{F}: L^1(\mathbb{R}^d,\mathbb{C}) \to C(\mathbb{R}^d,\mathbb{C})$
  - **problem**: $\mathcal{F}$ is a mapping between two completely different spaces
- $\mathcal{F}$ is linear and continuous.
  - **problem**: $\mathcal{F}$ is not necessarily integrable
- (A) $\mathcal{F}f\in C$:
  - $\lim_{n\to\infty}{\mathcal{F}f(\omega_n)}=\lim_{n\to\infty}{\mathcal{F}f(\omega)}$
    - dominated convergence theorem
      - (i) $\|g_n(x)\| \leq \|f(x)\|$, where (ii) $g_n(x):=f(x)e^{-ix\cdot \omega_n}\to\, (\text{integrand of}\, \mathcal{F}f)$ pointwisely and (iii) $\|f\|\in L^1$ integrable
        - **Achtung**: ohne die Betragsstriche um $f(x)$ würde $\|g_n(x)\| \leq \|f(x)\|$ nicht gelten
        - $z=re^{i\varphi}=r(\cos{(\varphi)}+i\sin{(\varphi)})$ (polar form of complex number)
- (B) $\mathcal{F}$ linear:
  - because integral $\mathcal{F}f$ linear
- (C) continuity of operator $\mathcal{F}$:
  - sufficient to show $\\|\mathcal{F}f\\|\_{C} \leq c\cdot\\|f\\|_{L^1}$
    - because continuity and boundedness are equivalent for linear operators (ex. 5.2)
      - and $\mathcal{F}$ is a linear operator acc. to (B)

## 3.3 FT of the characteristic function

- for $B>0$: $\mathcal{F}\chi_{\[B,B\]}(\omega) = \sqrt{\frac{2}{\pi}}B\,\text{sinc}\left(\frac{B\omega}{\pi}\right)$
  - where $\text{sinc}(x)=\left(\frac{\sin{(\pi x)}}{\pi x}\right)$, for $x=0$: $\text{sinc}(x)=1$
- thus, the FT of a function with compact support does not necessarily have a compact support
- **problem**: $\text{sinc}(x)$ is not integrable, ie. $\text{sinc}(x)\not\in L^1(\mathbb{R})$
- **problem**: $\mathcal{F}$ does not map $L^1$ to itself
  - this is bad news for the invertibility of $\mathcal{F}$

## 3.4 Properties of $\mathcal{F}$

- $f\in L^1$ and $\underline{A}\in \text{GL}(d)$ (set of invertible matrices, ie. $\det A \neq 0$), then
  - $\mathcal{F}(T_y f) = M_y(\mathcal{F}f)$
  - $\mathcal{F}(M_y f) = T_{-y}(\mathcal{F}f)$
  - $\mathcal{F}(D_{\underline{A}} f) = \frac{1}{\|\det \underline{A}\|}D_{\underline{A}^{-T}}(\mathcal{F}f)$
  - $\mathcal{F}(\overline{f}) = \overline{D_{-\mathbb{I}}(\mathcal{F}f)}$
- where $M_y: f \mapsto m_yf \, (M_y\,\text{ist einfach das Produkt}\, f(x)e^{ix\cdot y}), \, m_y: x \mapsto e^{ix\cdot y}$ and $D_y: f \mapsto (x \mapsto f(\underline{A}x))$

## 3.5 Hermitian, Skew-Hermitian

- **Hermitian** (if real: **even**): $\overline{f(\omega)} = f(-\omega)$
- **Skew-Hermitian** (if real: **odd**): $\overline{f(\omega)} = -f(-\omega)$ 
- we need this def. for 3.6

## 3.6 When is $\mathcal{F}$ even or odd?

- for $f\in L^1$ (eg. images)
  - $f$ real-valued $\Leftrightarrow$ $\mathcal{F}f$ hermitian
  - $f$ imaginary-valued $\Leftrightarrow$ $\mathcal{F}f$ skew-hermitian
- **Proof**: Based on (3.4) $\mathcal{F}(\overline{f}) = \overline{D_{-\mathbb{I}}(\mathcal{F}f)}$
- we need this for

## 3.7 Convolution Theorem for $\mathbb{C}$-valued Functions

- for $f,g\in L^1$ $$\mathcal{F}(f\ast g) = (2\pi)^{\frac{d}{2}}\mathcal{F}(f) \mathcal{F}(g)$$
- "the FT changes the <span style="color:red">**convolution**</span> to a **pointwise** multiplication"
- **simple proof**:
  - its just a little bit of computation with the integral, so no deep theory going on,
  - we just need **Fubini** to change the order of integrals and the **substitution rule** for multiple variables, both things we have used a couple of times.

## 3.8 "Integration by Parts" for $\mathcal{F}$

- this is like "Integration by Parts" for derivatives
- for $f,g\in L^1$ (eg. images) $$\int_{\mathbb{R}^d}(\mathcal{F}f)(x)g(x)dx = \int_{\mathbb{R}^d}f(x)(\mathcal{F}g)(x)dx$$
- This is very useful if you eg. know what the FT of $g$ is but you do not know what the FT of $f$ is. We will make use of that.
- **proof**: ex. (<span style="color:red">TODO</span>)

## 3.9 Generalize the Cross-Correlation to $\mathbb{C}$-valued Functions

- the convolution of $\mathbb{C}$-valued functions is exactly the same as the convolution for $\mathbb{R}$-valued functions, see [Wikipedia](https://en.wikipedia.org/wiki/Convolution#Domain_of_definition)
- <span style="color:red">but</span> for the cross-correlation the generalization is different!
- (i) for measurable $\psi,f$
  - $(\psi\star f)(x) = \int_{\Omega}\overline{\psi (y)}f(x+y)dy$
- (ii) Generalize 2.3:
  - $\overline{(\psi\star f)(x)} = (f\star \psi)(-x)$

## 3.10 "Correlation Theorem" for $\mathbb{C}$-valued Functions

- for $f,g\in L^1(\mathbb{R}^d, \mathbb{C})$ $$\mathcal{F}(f\ast g) = (2\pi)^{\frac{d}{2}}\overline{\mathcal{F}(f)} \mathcal{F}(g)$$
- "the FT changes the <span style="color:red">**correlation**</span> to a **pointwise** multiplication"
- "the 1st argument is conjugated"

## 3.11 Schwartz Space

- aka "space of rapidly decreasing functions on $\mathbb{R}^d$"
- $$S(\mathbb{R}^d,\mathbb{C}):=\\{f\in C^\infty : C_{\alpha,\beta}(f)<\infty\\}$$
- **Schwartz functions**: infinitely differentiable functions that rapidly decay
  - **rapidly decay**: for $x\to \infty$, $f\to 0$ **faster than any polynomial** goes to $\infty$
  - the same holds for **all** derivatives

## 3.25 $\mathcal{F}$ is bijective in $\mathcal{S}$

- if we restrict the domain of $\mathcal{F}$ from $L^1$ to $\mathcal{S}$ then we get a bijective FT

## 3.27 Extension of $\mathcal{F}$ from $\mathcal{S}$ to $L^2$

- There is a unique, bijective, linear isometry $$\mathcal{F}_2: L^2 \to L^2 \quad \text{with} \quad \mathcal{F}_2\vert_\mathcal{S} = \mathcal{F}.$$
- we wanted s.th. that is bijective when we transform it, $\mathcal{F}_2$ is what we were looking for

# TODO (zurückgestellt)

- Remark 1.3
