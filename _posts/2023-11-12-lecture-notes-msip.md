---
title: "Mathematical Methods of Signal and Image Processing - Chapter 1 - Point Operators"
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

## complete

- <span style="color:red">**completeness** is closely related to **Cauchy sequences**</span>
- A metric space $(X,d)$ is **complete** if any of the following equivalent conditions are satisfied:
  - Every Cauchy sequence of points in $X$ has a limit that is also in $X$.
  - Every Cauchy sequence in $X$ converges in $X$ (that is, to some point of $X$).
- **examples**:
  - The space $\mathbb{Q}$ of <span style="color:red">**rational numbers**</span>, with the standard metric given by the absolute value of the difference, is <span style="color:red">**not complete**</span>.
  - The <span style="color:red">**open interval** $(0,1)$</span>, again with the absolute difference metric, is <span style="color:red">**not complete**</span> either.
    - The sequence defined by $x_{n}={\frac{1}{n}}$ is Cauchy, but does not have a limit in the given space. However the <span style="color:red">**closed interval** $\[0,1\]$</span> is complete; for example the given sequence does have a limit in this interval, namely zero.
  - The space $\mathbb{R}$ of **real numbers** and the space $\mathbb{C}$ of **complex numbers** (with the metric given by the absolute difference) are complete, and so is **Euclidean space** $\mathbb{R}^n$, with the usual distance metric.
  - In contrast, **infinite-dimensional normed vector spaces** may or may not be complete; those that are complete are **Banach spaces**.

## Vector Spaces

- from less to more requirements:
  - **normed space**: the pair $(X,\lVert \cdot \rVert)$
  - **pre-Hilbert space**: a vector space with a scalar product, ie. the pair $(X,(\cdot,\cdot)_X)$
    - note: a scalar product induces a norm, thus, for each pre-Hilbert space there is an **associated normed space**
  - **Hilbert space**: a complete pre-Hilbert space

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

### Integrability

- <span style="color:red">**remember**</span>: we don't say "$f$ is integrable," but rather "$f$ is integrable over $D$"
  - eg. eg. for $f(x)=1/x$ the anti-derivative is $\ln{(\lvert x\rvert)}$, so $f(x)=1/x$ is "integrable over $D$" if the domain $D$ is an interval that doesn't contain $0$

### $L^2$

- (i) it is often convenient to think of $L^2(\mathbb{R}^n)$ as the **completion** of the **continuous functions** with respect to the $L^2$-norm, [mathworld.wolfram](https://mathworld.wolfram.com/L2-Space.html)
- (ii) examples for $L^2$ functions:
  - **Bounded functions**, defined on $\[ 0 , 1 \]$, are square-integrable. These functions are also in $L^p$, for any value of $p$., [wikipedia](https://en.wikipedia.org/wiki/Square-integrable_function#Examples)
- (iii) Comparison: [$L^2$ vs. $C(U,Y)$](#l2-vs-cuy)
- (iv) ${\displaystyle L^{2}}$ is the only **Hilbert space** among ${\displaystyle L^{p}}$ spaces
- <span style="color:red">Neither of the containments (v), (vi) holds in general!</span>
  - (v) $L^2\subset L^1$ **for bounded domains**, <span style="color:red">but not for unbounded domains!</span>, [stackexchange](https://math.stackexchange.com/a/18399)
    - (a) because what keeps a function from being integrable on a bounded set is being too large, and squares make large numbers larger
      - ie. if $f$ is defined on a bounded domain and in $L^2$, then we know the integral under $f^2$ is finite. But since $\lvert f\rvert<\lvert f\rvert^2$ ($f^2$ is an integrable mojorant of $f$) we also know that the integral under $f$ is finite. Thus, $f\in L^1$.
  - (vi) $L^1\subset L^2$ **for bounded functions**, <span style="color:red">but not for unbounded functions!</span>
    - (a) because what keeps a bounded function from being integrable is not going to zero fast enough, and squares make numbers go to zero faster
      - ie. if $f$ is bounded and in $L^1$, then we know it must go to zero fast enough to be in $L^1$. $f^2$ must go to zero even faster and there is nothing else that prevents $f^2$ from being integrable. Thus, $f$ must be in $L^2$.
      - if an unbounded function does **not** go to zero, then it is not integrable (intuitively clear!)

### $L^2$ vs. $C(U,Y)$

- not all continuous functions are in $L^2$,
  - **counterexample**: the constant function $f=1$
- not all $L^2$ functions are continuous,
  - **counterexample**: the characteristic function of the finite set $A$, $\chi_{A}$

### $C^\infty$, Smooth Functions

- **Achtung**: $f$ ist nur dann **smooth**, wenn es **infinitely differentiable** ist, nur **differentiable** reicht nicht!
- The function $f$ is said to be <span style="color:red">**infinitely differentiable**, **smooth**</span>, or of class $C^{\infty }$, if it has derivatives of <span style="color:red">**all**</span> orders

### $C^0$, $C^1$, etc

- The class $C^{0}$ consists of all **continuous functions**.
- The class $C^{1}$ consists of all **differentiable functions** whose **derivative is continuous**; such functions are called **continuously differentiable**.
  - Thus, a $C^{1}$ function is exactly a function whose derivative exists and is of class $C^{0}$

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

## Measure Theory

### Measurable Space

- $(\Omega,\mathcal{A})$ is a measurable space only if $\mathcal{A}$ is a $\sigma$-algebra
  - if $\mathcal{A}$ is not a $\sigma$-algebra then $(\Omega,\mathcal{A})$ is not a measurable space!
  - the $\sigma$-algebra properties ensure that the space is measurable

### Borel and Radon Measures

- $\mu$ is called a **Borel measure**, when the $\sigma$-algebra (=the domain of $\mu$) on which $\mu$ is defined is a **Borel $\sigma$-algebra**
- $\mu$ is called a **positive Radon measure**, when it is a Borel measure <span style="color:red">and</span> finite on all compact sets in the $\sigma$-algebra (=the domain of $\mu$) on which $\mu$ is defined
- <span style="color:red">**counting measure**</span>: Borel, but not Radon
- <span style="color:red">**Dirac measure**</span>: Radon (note: Radon implies Borel!)
- <span style="color:red">**($d$-dimensional) Lebesgue measure**</span>: Radon

### Measurable Function

- $(\Omega,\mathcal{A})$, $(\Sigma,\mathcal{B})$
- a function is called $\mathcal{A}$-$\mathcal{B}$ measurable or <span style="color:red">measurable</span>, if $f^{-1}(B)\in\mathcal{A}$ for all $B\in\mathcal{B}$

# C.1.1 Digital Images

- **spatial dimension**: $d$
- **d-dim cuboid**: $\Omega=(a_1,b_1)\times\ldots\times(a_d,b_d)\subset \mathbb{R}^d$
  - eg. $d=2$ and $\Omega=(a_1,b_1)\times(a_2,b_2)=(0,8)\times(0,6)$
- **value range**: $V$
  - **grayscale image**: $V=\mathbb{R}$ or $V=\left[0,1\right]$
  - **color image**: $V=\mathbb{R}^3$ or $V=\left[0,1\right]^3$ (depends on color model, eg. RGB, HSV)
- **image**: a mapping $f: \Omega \to V$, where $\Omega\subset \mathbb{R}^d$
  - **continuous gray scale image**: $f: (0,1)^2 \to \[0,1\]$ (with $0$ black, $1$ white)
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

# C.1.2 Point Operators aka Intensity Transforms

- **intensity transformation**: $T: \mathbb{R} \to \mathbb{R}$
- **intensity transformed image**: $T\circ f$, where $f: \Omega \to \mathbb{R}$ is an image.

## 1.6 (i) clipping

- **clipping**: $$\begin{equation*} T_{\left[a,b\right]}^{\text{clip}}: \mathbb{R} \to \left[a,b\right],\ s \mapsto \begin{cases}
                                            a &s\leq a\\
                                            b &s\geq b\\
                                            c &\text{else}
                                            \end{cases}
                  \end{equation*}$$
- eg. in electron microscopy: sometimes just normalization is not enough (eg. outliers, very bright pixels) and you **must** apply **clipping** **first** to make certain structures visible and normalization **after** that

## 1.6 (ii) normalization

- **normalization**: $$\begin{equation*} T_{\left[a,b\right]}^{\text{norm}}: \mathbb{R} \to \left[0,1\right],\ s \mapsto T_{\left[0,1\right]}^{\text{clip}}\left(\frac{s-a}{b-a}\right) \end{equation*}$$
  - sometimes normalization is not enough (eg. presence of outliers): in that case first clipping then normalization is required

## 1.6 (iii) thresholding

- **thresholding**: $$\begin{equation*} T_{\theta}^{\text{threshold}}: \mathbb{R} \to \bigl\{0,1\bigr\},\ s \mapsto \begin{cases}
                                    0 &s\leq \theta\\
                                    1 &s > \theta
                                    \end{cases}
                  \end{equation*}$$
  - Heaviside function: $H := T_0^{\text{threshold}}$

## 1.6 (iv) gamma correction

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

## 1.6 (v) log transformation

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

## Point Operators using Global Statistics / Histograms

- these global statistics of the intensity distributions in the image are computed once
- based on that we can construct some intensity transform
- <span style="color:red">the global statistics are represented by a **histogram**</span>

### 1.7 Histogram of $F$, CDF of $F$

- $F$ is a discrete image
- $H_F(s) := \sum_{\underline{j}\in\mathcal{I}}\delta_{s,f_{\underline{j}}}$
- $G_F(s) := \sum_{r=0}^{s}H_F(r)$

### 1.8 Characteristic Function $\chi_{A}$, Level Set of $f$, Volume of a Bounded Set $A$

- $f$ is a pixelated image
- (i) $\chi_A$
- (ii) $\\{f=s\\} := \\{x\in\Omega:f(x)=s\\}$ (s-level set; <span style="color:red">in 2d</span>: level line, isoline; <span style="color:red">in 3d</span>: level surface, isosurface),
  - and similarly, $\\{f\leq s\\}$, $\\{f\geq s\\}$ (s-sublevel set, s-superlevel set)
  - $\Rightarrow f^{-1}(s)=\\{f=s\\}, f^{-1}(\left(-\infty, s\right])=\\{f\leq s\\}$
- (iii) $\lvert A\rvert := \text{Vol}(A) := \int_A dx = \int_{\mathbb{R}^d}\chi_A(x)dx$

### 1.9 (i)-(iii) Histogram of $f$

- $f: \Omega \to \mathbb{R}$ is a pixelated image
- $H_f: \mathbb{R}\to \left[0,\infty\right), s\mapsto\text{Vol}(\\{f=s\\})$ <span style="color:red">does not work!</span> (counterexample: $f(x)=x$ on $\left[0,1\right]$)
- **idea**: extend from $\mathbb{R}$ to $\mathcal{B}(\mathbb{R})$ (<span style="color:red">subsets of $\mathbb{R}$</span>)
- (i) $H_f: \mathcal{B}(\mathbb{R})\to \left[0,\infty\right), A\mapsto\text{Vol}(\\{f\in A\\})$
- (ii) $H_f$ is a <span style="color:red">**measure**</span>
  - Generalization of functions (maps sets to values)
  - $H_f$ is the push-forward measure of the Lebesgue measure $\lambda$ under the mapping $f$
    - $\boxed{H_f(A)} := \text{Vol}(\\{f\in A\\}) = \text{Vol}(f^{-1}(A)) = \boxed{\lambda(f^{-1}(A))}$, where $\lambda(X)$ is the <span style="color:red">**Maß**</span> of set $X$
      - in diesem Sinne gibt $H_f$ das <span style="color:red">"Maß des Urbildes von $A$"</span> an (<span style="color:red">"Volume of the preimage of $A$"</span>)
      - brightsideofmathematics <span style="color:red">"image measure"</span>:
        - the key ingredients here are the **measurable function** $f$ and the **measure** $\mu$
        - the **measurable function** $f$ <span style="color:red">"pushes the measure $\mu$ forward"</span> to the right (here: from $\Omega$ to codomain $\mathbb{R}$), thus the name **"push-forward measure"**
        - in the end we get a new measure on the codomain
      - Berkels: "Take a volume in the unit square $\Omega$ and put it in image $f$ and get a volume in $\mathbb{R}$ as result. This $\mathbb{R}$-volume is the histogram."
    - in <span style="color:green">(A.8)</span> $\mu$ is used instead of $\lambda$ (but it is just a different notation, $\mu\equiv\lambda$):
      - $H_f: \mathcal{B}(\mathbb{R})\to \left[0,\infty\right), A\mapsto\mu(f^{-1}(A))$ (in the lecture: $A\mapsto\lambda(f^{-1}(A))$)
      - $H_f$ is denoted by $f(\mu)$ (in the lecture: $f(\lambda)$)

<p align="center">
  <img src="https://i.ibb.co/jW56mfz/Screenshot-from-2024-02-29-06-31-33.png" alt="Screenshot-from-2024-02-29-06-31-33" border="0"><br><br><br>
  <b>Recall:</b> <span style="color:red"><b>(d-dimensional) Lebesgue measure</b></span>:<br>
  <img src="https://i.ibb.co/80QJ2Vm/Screenshot-from-2024-02-29-06-51-54.png" alt="Screenshot-from-2024-02-29-06-51-54" border="0">
</p>

- (iii) because of (ii) and <span style="color:green">(A.9)</span>:
  - if $f$ is measurable and $g$ is $f(\lambda)$-integrable, then $\int_{\Omega}gdf(\lambda) = \boxed{\int_{\Omega}g(s)dH_f(s) = \int_{\mathbb{R}}(g\circ f)(x)dx}$
    - in words: we can integrate wrt measure $H_f$ without actually needing $H_f$, we just need to compose $g\circ f$

<p align="center">
  <img src="https://i.ibb.co/DDBNxRm/Screenshot-from-2024-02-29-07-13-07.png" alt="Screenshot-from-2024-02-29-07-13-07" border="0">
</p>

### 1.9 (iv)-(vi) CDF of $f$

- (iv) <span style="color:red">**CDF of $f$**</span>: $G_f(s) := \text{Vol}(\\{f\leq s\\})$
  - $G_f$ is **not** a measure!
  - no Lebesgue null sets (bec when we increase $s$ we increase the $s$-sublevel set continuously)
  - **properties of $G_f$**: (exercise)
    - $G_f: \mathbb{R}\to\left[0,\text{Vol}(\Omega)\right]$
    - $G_f$ is monotonically increasing
    - $G_f(s) = \text{Vol}(\Omega)\,\forall s > \text{ess sup}\,f$ 
- (v) the <span style="color:red">**change of $G_f$**</span> at $s$ is a <span style="color:red">**measure of $H_f$**</span> at $s$
  - **more precisely**: The continuous histogram $H_f$ is the distributional derivative of the CDF $G_f$. (exercise)
    - Recall: <span style="color:red">**distributions**</span> map **$C_c^\infty$ functions** to **real numbers**
      - every function $g\in L^1(\Omega)$ induces a distribution $f\mapsto\int_\Omega fgdx$
      - every positive Radon measure $\mu$ on $\mathcal{B}(\Omega)$ induces a distribution $f\mapsto\int_\Omega fd\mu$
        - **special case**: the distribution induced by the Dirac measure (a positive Radon measure!) is called **Dirac delta function** $\delta_x(f)=\int_{\Omega}fd\delta_x=f(x)$ (exercise: prove $\int_{\Omega}fd\delta_x=f(x)$ using the Lebesgue integral)
    - why <span style="color:red">**distributional derivative**</span>?: $G_f$ is a step function and thus, $G_f$ is <span style="color:red">**not differentiable**</span> in the <span style="color:red">classical sense</span>, but $G_f' = H_f$ holds in the <span style="color:red">distributional sense</span>, ie. $\langle D^{\lvert\alpha\rvert}G_f,\psi\rangle := \boxed{-\int_{\mathbb{R}}G_f\psi'ds = \int_{\mathbb{R}}\psi dH_f} =: \langle H_f,\psi\rangle$ (exercise)
      - $\int_{\mathbb{R}}\psi dH_f = \langle H_f,\psi\rangle$ gilt, weil das nach Definition für "distribution induced by a positive Radon measure" genau die Art ist wie ein **positive Radon measure** (hier: $H_f$) auf eine Testfunktion $\psi$ wirkt
- (vi) $G_f(s) = \prod_{i=1}^{d}h_i G_F(s)$ (continuous vs discrete CDF)

### 1.10 Binning

- **idea**: approximate the histogram of a continuous function discretely
- $B_k$: intervals (for $k=1,\ldots,m$)
- $r_k$: interval boundaries (for $k=0,\ldots,m$)
  - $r_0 = \inf f$ and $r_m = \sup f + 1$ (like <span style="color:red">minimum</span> and <span style="color:red">maximum</span>)
- <span style="color:red">$B_m$ has a different length</span> than the other $B_k$ bec $r_m = \sup_\Omega f + 1$, but all other $B_k$ have length $\frac{1}{m-1}(\sup f - \inf f)$
- $H_k = \text{Vol}(\\{f\in B_k\\}) = \\#(\\{\underline{j}\in \mathcal{I}:f_{\underline{j}}\in B_k\\})$ (<span style="color:red">Volume of the preimage of $B_k$</span> or for discrete images <span style="color:red">number of pixels with a value in $B_k$</span>)

<p align="center">
  <img src="https://i.ibb.co/c8Z5j0M/Screenshot-from-2024-03-01-06-20-09.png" alt="Screenshot-from-2024-03-01-06-20-09" border="0">
</p>

### 1.11 Histogram Equalization

- **Task**: <span style="color:red">improve contrast</span> (eg. due to bad conditions while shooting a photo or wrongly adjusted optical settings, the resulting image may exhibit a low contrast)
- (i) **desired property**: $G_{T\circ f}(s)=s\text{Vol}(\Omega)$ (the transformed image $T\circ f$ should have a linear CDF with slope $\text{Vol}(\Omega)$)
  - this property is <span style="color:red">equivalent to</span> "$H_{T\circ f}$ is a uniformly distributed histogram"
- (ii) **conditions**:
  - $G_f$ is <span style="color:red">strictly</span> increasing
- then the **continuous histogram equalization** is $$\boxed{T: \left[0,1\right]\to \left[0,1\right],\,s\mapsto\frac{G_f(s)}{\text{Vol}(\Omega)}}$$
- (iii) and the **discrete histogram equalization** is $$\boxed{T: \{0,\ldots,n\}\to \{0,\ldots,n\},\,s \mapsto \text{rd}(\frac{n}{\text{Vol}(\Omega)}G_f(s)) = \text{rd}(\frac{n}{\#\mathcal{I}}G_F(s))}$$
  - $G_F(s)\in \\{0, \ldots, \\#\mathcal{I}\\}$, $G_f(s)\in\left[0,\text{Vol}(\Omega)\right]$ <span style="color:green">(1.9 (iv))</span>
  - $n$ is the highest value in the value range $V := \\{0,\ldots,n\\}$
    - <span style="color:red">why do we need $n$?</span>:
      - bec for continuous images the value range was $V = \left[0,1\right]$, but for pixelated images and discrete images we have $V := \\{0,\ldots,n\\}$
      - when $f$ is a pixelated image (or discrete image) with range $V := \\{0,\ldots,n\\}$ then $T\circ f$ is also a pixelated image (or discrete image) with range $V := \\{0,\ldots,n\\}$
      - so we must multiply the fraction $\frac{G_f(s)}{\text{Vol}(\Omega)}\in \left[0,1\right]$ by $n$ and then **round** the result to the nearest integer in $V := \\{0,\ldots,n\\}$, so that we get a pixelated image (or discrete image) <span style="color:red">**with the same value range $V$**</span> as output
  - for either a <span style="color:red">pixelated</span> image $f$ or a <span style="color:red">discrete</span> image $F$
  - $\text{rd}$ means "ab $0.5$ wird aufgerundet"
- <span style="color:red">**(ii) fulfills property (i)**</span>
- <span style="color:red">**normalization**</span> vs <span style="color:red">**equalization**</span>:
  - **normalization** <span style="color:green">(1.6)</span> will put lowest value to $0$ (black) and highest to $1$ (white), i.e. histogram will have full width after normalization, but this just spreads things vertically, it does **not** redistribute values 
  - **equalization** flattens the distribution s.t. all the gray values are more or less taken equally
    - nonlinear change in intensities;
    - in the **continuous setting**: for **linear CDFs** you should get a **flat histogram** (uniformly distributed histogram)
    - in the **discrete case**: this does not work bec you have to send one gray value to another gray value, so you cannot split these peaks
      - eg. "if you have $100$ pixels with value $10$, you can change the value to $8$, but you cannot put one half to $8$ and the other half to $9$" &rarr; you can only <span style="color:red">approximate linear CDF</span> &rarr; <span style="color:red">step function</span> that <span style="color:red">looks like linear function</span> 
      - Bredies: Of course, in a discrete setting, an equalized histogram cannot be achieved, since equal gray values are mapped to equal gray values again
    - "makes good use of your gray value range" so that you <span style="color:red">see more details</span> in the equalized image, but at the cost that the equalized image <span style="color:red">does not look so natural</span>

### 1.12 Histogram Matching

- another application of the **same idea as histogram equalization**
- **Taks**: <span style="color:red">compensate sensor-dependent differences</span> (of two images of the same scene, but taken with different sensors)
  - But you <span style="color:red">cannot compensate e.g. contrast reversal</span> because if there is no strictly increasing relationship between these sensors, then there is no chance to compensate this 
- $T(s) = \frac{G_f(s)}{\text{Vol}(\Omega)}$ and $S(s) = \frac{G_g(s)}{\text{Vol}(\Omega)}$ (same as in <span style="color:green">(1.11)</span>)
- **idea**: transfer $H_f$ to $H_g$ by applying $S^{-1}\circ T$ on $f$
  - ie. first equalize as in <span style="color:green">(1.11)</span>, then invert the equalization with $S^{-1}$
- **conditions**:
  - $G_f$ and $G_g$ invertible (strictly increasing and continuous)
    - bec otherwise $S^{-1}$ is not defined
    - Berkels: bec to be able to apply the histogram equalization <span style="color:green">(1.11)</span> we need to <span style="color:red">assume</span> that the CDFs are invertible
- then, $$\boxed{G_{S^{-1}\circ T\circ f}(s) = G_g(s)}$$
- then, $H_{S^{-1}\circ T\circ f} = H_g$
  - in words: "the operation $S^{-1}\circ T\circ f$ makes a histogram $H_f$ look like <span style="color:red">any</span> other histogram $H_g$"
- Berkels: if - for whatever reason - you want your histogram to have a certain shape (which can be <span style="color:red">ANY shape</span> - as long as its CDF is strictly increasing!) you can do it with this kind of approach

### 1.13 Segmentation by Thresholding / Isodata Algorithm

<span style="color:red">Note: Like 1.11 and 1.12 this method is also **histogram based**!</span>

- **Task**:
  - <span style="color:red">for compression</span> (binary image needs less storage than grayscale)
  - <span style="color:red">as preprocessing step eg. for OCR</span>
- <span style="color:red">**Idea**:</span> **fixed point equation**: Find a threshold $\theta$ s.t. $\theta = \varphi(\theta)$
  - where $\boxed{\varphi(\theta) := \frac{1}{2}\left(\int_{\\{f\leq\theta\\}}^\text{avg}f(x)dx + \int_{\\{f\gt\theta\\}}^\text{avg}f(x)dx\right)}$
  - where $\int_A^\text{avg}f(x)dx := \frac{1}{\text{Vol}(A)}\int_A f(x)dx$ is "the average value of $f$ on the domain region $A$"
    - **warning**: this <span style="color:red">cannot</span> be the "center of mass" of $f$ on $A$ bec it must be a value in the <span style="color:red">codomain</span> of $f$, whereas the COM would be a value in the <span style="color:red">domain</span>!
- <span style="color:red">**Method**:</span> **fixed point iteration**: $\boxed{\theta^{n+1} = \varphi(\theta^n)}$ with IV $\theta^1\in(0,1)$, typically $\theta^1 = \frac{1}{2}(\inf{f} + \sup{f})$ (mean of the value range)
  - **Implementation**: Isodata Algorithm

**What is a fixed point?**

- **fixed point**: Formally, $c$ is a fixed point of a function $f$ if $c$ belongs to both the domain and the codomain of $f$, and $f(c) = c$.
  - For example, if $f$ is defined on the real numbers by $f(x) = x^2 − 3x + 4$, then $2$ is a fixed point of $f$, because $f(2) = 2$. 
  - <span style="color:red">"a fixed point is an element that is mapped to itself by the function."</span>

**Under which conditions does the fixed point iteration converge?**

- (i) Proof of convergence of the sequence $\theta^n$:
  - **Monotone Convergence Theorem**: 
    1. If a sequence of **real numbers** is <span style="color:red">increasing and bounded above</span>, then its supremum is the limit.
    2. If a sequence of **real numbers** is <span style="color:red">decreasing and bounded below</span>, then its infimum is the limit.
  - $\varphi$ is <span style="color:red">increasing</span> (but not necessarily strictly increasing) $\Rightarrow\theta^n$ is increasing (for $\theta^1\leq\theta^2$) or decreasing (for $\theta^1\geq\theta^2$)
    - **prooven in lec**: because $\varphi$ is a <span style="color:red">sum of two increasing functions</span> (the two **average gray value integrals** $\varphi_a$ and $\varphi_b$)
    - **phth remember**: $\theta^n$ can be decreasing, but $\varphi$ is <span style="color:red">always</span> increasing!
  - $\varphi(\[0,1\])\subset\[0,1\] \Rightarrow \varphi$ is <span style="color:red">bounded</span> $\Rightarrow \theta^n$ is <span style="color:red">bounded</span>
    - because the two average gray value integrals are in $\[0,1\]$, so $\varphi$ must be in $\[0,1\]$, too
- **problem**: just because $\theta^n$ converges that does not mean its **limit** $\theta^\ast$ (which must exist acc. to (i)!) is also a **fixed point**, ie. $\varphi(\theta^\ast) = \theta^\ast$
- **problem**: moreover, we do not know if a fixed point even **exists**
- (ii) Proof of existence of a fixed point:
  - (A) **Brouwer's fixed point theorem**: "states that for any **continuous** function $f$ mapping a nonempty compact convex set to itself, there is a point $x_0$ such that $f(x_0) = x_0$."
    - for $f: \left[a,b\right]\to \left[a,b\right]$: "The simplest forms of Brouwer's theorem are for continuous functions $f$ from a **closed interval** $I$ in the real numbers to itself (or from a **closed disk** $D$ to itself)."
    - for **Euclidean space**: "A more general form than the latter is for continuous functions from a **nonempty convex compact subset $K$ of Euclidean space** to itself."
  - (B) "Every monotonic <span style="color:red">non-decreasing</span> $\varphi: \[0,1\] \to \[0,1\]$ has a fixed point.", exercise
    - this is a special case of the **Knaster-Tarski fixed point theorem** in order theory, note: in **order theory** "monotonic" **always** means "non-decreasing"
    - "If $\varphi$ is **<span style="color:red">not</span> continuous**, then the **limit of** $\theta^n$ is **not** necessarily **a fixed point**."
      - This means that when $\varphi$ is **not** continuous $\theta^n$ will converge to *"something"*, but this *"something"* is **not** necessarily a fixed point.
- thus, we know that $\varphi$ will always converge to some <span style="color:red">**limit**</span>, but we **must** assume $\varphi$ is **continuous** in order to guarantee that this <span style="color:red">**limit**</span> to which the Isodata Algorithm will converge is also a <span style="color:red">fixed point</span>

<p align="center">
  <img src="https://i.ibb.co/ky36PRx/Screenshot-from-2024-03-01-10-44-57.png" alt="Screenshot-from-2024-03-01-10-44-57" border="0">
</p>

**How to implement the Isodata Algorithm?**

- use the histogram $H_f$ to compute the average gray values $\int_{\\{f\leq\theta\\}}^\text{avg}f(x)dx$ and $\int_{\\{f\gt\theta\\}}^\text{avg}f(x)dx$
  - the continuous histogram $H_f$ is approximated as the histogram of the discrete image $H_F$
  - **intuitively**: like discrete **center of mass** formula with $H_F(s)$ instead of $M(s)$
    - $\sum_{s=\text{floor}(\theta)+1}^{n}sH_F(s)/\sum_{s=\text{floor}(\theta)+1}^{n}H_F(s)$

