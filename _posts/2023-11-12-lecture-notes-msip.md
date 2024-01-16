---
title: "Mathematical methods of signal and image processing"
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
    - "The completeness of the real numbers implies (and is equivalent to) that any bounded nonempty subset $S$ of the real numbers has an infimum and a supremum. If $S$ is not bounded below, one often formally writes $\inf{S}=-\infty$ If $S$ is empty, one writes $\inf{S}=+\infty$", [Wikipedia](https://en.wikipedia.org/wiki/Infimum_and_supremum#Infima_and_suprema_of_real_numbers)
- **essential supremum** ("supremum almost everywhere"): While the exact definition is not immediately straightforward, intuitively the essential supremum of a function is the smallest value that is greater than or equal to the function values everywhere **while ignoring** what the function does at a **set of points of measure zero**.
  - For example, if one takes the function $f(x)$ that is equal to zero everywhere except at $x=0$ where $f(0)=1$, then the supremum of the function equals one. However, its **essential supremum** is zero because we are allowed to ignore what the function does at the single point where $f$ is peculiar.
- **essential infimum**: The essential infimum is defined in a similar way.

## topology

- **topology**: "die Menge $\mathcal{O}$ der offenen Mengen wird als die Topologie des topologischen Raumes $(X,\mathcal{O})$ bezeichnet", [Wikipedia](https://de.wikipedia.org/wiki/Topologie_(Mathematik)#Topologischer_Raum)
  - "Üblicherweise werden topologische Räume in den Lehrbüchern über die **offenen Mengen** definiert"
  - "Wird eine beliebige **Grundmenge** mit einer **Topologie** (einer topologischen Struktur) versehen, dann ist sie ein **topologischer Raum**, und ihre Elemente werden als **Punkte** aufgefasst. Die Topologie des Raumes bestimmt sich dann dadurch, dass bestimmte Teilmengen als **offen** ausgezeichnet werden."

# 1.1 Digital Images

- **spatial dimension**: $d$
- **d-dim cuboid**: $\Omega$, eg. $d=2$ and $\Omega=(a_1,b_1)\times(a_2,b_2)=(0,8)\times(0,6)$
- **value range**: $V$
- **image**: a mapping $f: \Omega \to V$, where $\Omega\subset \mathbb{R}^d$
  - **continuous gray scale image**: $f: (0,1)^2 \to \[0,1\]$
- **image size**: the vector $\underline{m}=(m_1,\dotsc,m_d)$ (aka "number of pixels"), e.g. $\underline{m}=(4,3)$
- **grid node**: $x^{\underline{j}}\in \Omega$ (aka "the center of each pixel", eg. $x^{(4,3)}$)
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
- **digital/discrete/pixel image**: a $d$-array $F=(f_\underline{j})_{\underline{j}\in I}$
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
  - **values**: we choose $f(x)=f(c_\underline{j})$ for each cell boundary point $x\in L$, where $\underline{j}$ is the **unique** cell above or to the right of each cell boundary point $x\in L$
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
      - intensities $f$ that fall in the sensor are averaged
      - with this CCD sensor one gets the discrete image $(r_{x^\underline{j}}^h\left[f\right])_{\underline{j}\in I}$ (a $d$-array)
- **coordinate systems**: in digital images rotated by $90^\circ$ clockwise (ie. 1st axis is vertical, 2nd axis is horizontal)

# 1.6 Point Operators / Intensity Transforms

- **intensity tranformation**: $T: \mathbb{R} \to \mathbb{R}$
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
  - eg. useful in electron microscopy: some details in $\lvert FFT\left[f - \overline{f}\right] \rvert$ become only visible if **first** $T^{\log}$ and **then** $T^{\text{norm}}$ is applied instead of the other way around ($T^{\log}$ does not help much after $T^{\text{norm}}$ has been applied) (**gamma correction** could **not** do this unless you would use an extremely large gamma value that is exactly valid for the specific image, ie. gamma depends on the dataset, whereas the log transform does not have these problems)

## Intensity Transforms using Global Statistics

- these global statistics of the intensity distributions in the image are computed once
- based on that we can construct some intensity transform
- the statistics are represented by a **histogram**

### 1.7 Discrete Histogram of $F$, Discrete CDF of $F$

- $F$ is a discrete image

### 1.8 Characteristic Function, Level Set of $f$, Volume of a Bounded Set $A$

- $f$ is a pixelated image

### 1.9 Continuous Histogram, CDF of $f$, Discrete CDF of $f$

- $f$ is a pixelated image
- The continuous histogram $H_f$ is the distributional derivative of the CDF $G_f$.

### 1.10 Binning

- approximate the histogram of a continuous function discretely

### 1.11 Histogram Equalization

### 1.12 Histogram Matching

### 1.13 Segmentation by Thresholding / Isodata Algorithm

- **Monotone Convergence Theorem**: 
  1. If a sequence of real numbers is increasing and bounded above, then its supremum is the limit.
  2. If a sequence of real numbers is decreasing and bounded below, then its infimum is the limit.
  3. If $(a_{n})$ with $(n\in \mathbb{N})$ is a monotone sequence of real numbers (i.e., if $a_n \leq a_{n+1}$ for every $n \geq 1$ or $a_n \geq a_{n+1}$ for every $n \geq 1$), then this sequence has a finite limit if and only if the sequence is bounded.

# 2.0 Local Operators

## 2.3 Properties of the cross-correlation

### 2.3 (i) Boundedness of the norm of the cross-correlation

### 2.3 (ii) Derivatives

To understand $\psi \in C_c^k(\mathbb{R}^d)$:

**compact**: "In general, a set $K$ is called **compact**, if every sequence in $K$ has a converging subsequence with limit in $K$. For $K \subset \mathbb{R}^d$, compactness of $K$ is equivalent to $K$ being closed and bounded. This equivalence is known as **Heine–Borel theorem**.", lecture appendix
- **Heine–Borel theorem**:
  - For a subset $S$ of Euclidean space $\mathbb{R}^n$, the following two statements are equivalent:
    - $S$ is **closed** and **bounded**
    - $S$ is **compact**, that is, every open cover of $S$ has a finite subcover.

# TODO (zurückgestellt)

- Remark 1.3
