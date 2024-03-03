---
title: "Mathematical Methods of Signal and Image Processing - Chapter 3 - The Frequency Domain"
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

# 3.0 Frequency Domain

## 3.1 Fourier Transform

- "FT of $f$ in $\omega$": $\mathcal{F}f(\omega):=\hat{f}(\omega) = \frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}f(x)e^{-ix\cdot\omega}dx$
  - weighted sum of complex exponentials
- "the continuous FT": $\mathcal{F}: f \to \mathcal{F}f$
- **problem**:
  - $\mathcal{F}$ gives us a very different representation of the image in which we can eg. change some frequencies to pronounce s.th., but we <span style="color:red">need to go back</span> after that.
  - unless we can <span style="color:red">invert $\mathcal{F}$</span> it is useless for any kind of image processing &rarr; Is $\mathcal{F}$ invertible? If not, how can we change $\mathcal{F}$ to make it invertible?

## 3.2 Properties of the mapping $\mathcal{F}$

- (i) $\mathcal{F}: L^1(\mathbb{R}^d,\mathbb{C}) \to C(\mathbb{R}^d,\mathbb{C})$
  - **problem**: $\mathcal{F}$ is a mapping between two completely different spaces
  - **problem**: $\mathcal{F}f$ is not necessarily integrable, ie. it can have infinite Maß
  - <span style="color:red">**needed for**</span>:
    - proof of <span style="color:green">(3.50)</span> Shannon-Whittaker
- (ii) $\mathcal{F}$ is linear and continuous.
- **proof**:
  - (A) $\mathcal{F}f\in C$:
    - $\lim_{n\to\infty}{\mathcal{F}f(\omega_n)}=\mathcal{F}f(\omega)$
      - dominated convergence theorem <span style="color:green">(B.11)</span> (see figure below)
        - <span style="color:green">(B.11 (i))</span> $\|g_n(x)\| \leq \|f(x)\|$, where <span style="color:green">(B.11 (ii))</span> $g_n(x):=f(x)e^{-ix\cdot \omega_n}\to_{n\to\infty}\, (\text{integrand of}\, \mathcal{F}f)$ pointwisely and <span style="color:green">(B.11 (iii))</span> $\|f\|\in L^1$ integrable
          - **Achtung**: ohne die Betragsstriche um $f(x)$ würde $\|g_n(x)\| \leq \|f(x)\|$ nicht gelten
          - $z=re^{i\varphi}=r(\cos{(\varphi)}+i\sin{(\varphi)})$ (polar form of complex number)
  - (B) $\mathcal{F}$ linear:
    - because integral $\mathcal{F}f$ linear
  - (C) continuity of operator $\mathcal{F}$:
    - sufficient to show $\\|\mathcal{F}f\\|\_{C} \leq c\cdot\\|f\\|_{L^1}$
      - because continuity and boundedness are equivalent for linear operators (ex. 5.2)
        - and $\mathcal{F}$ is a linear operator acc. to (B)

<p align="center">
  <img src="https://i.ibb.co/HtqC8sS/Screenshot-from-2024-03-02-04-04-56.png" alt="Screenshot-from-2024-03-02-04-04-56" border="0">
</p>

- **problem**:
  - so $\mathcal{F}f$ gives me a very different representation of my image and then, for instance, I can then do sth wtih that representation, I can change some frequencies to pronounce sth, but once I'm done making this manipulation with the frequencies, I need to go back.
  - So, unless I can invert this transform it is useless for any kind of image processing.
  - And the very first thing before we can even think about going back is, we need to know where we are, ie.
    - what is this kind of function?
    - do we have a chance of going back from $\mathcal{F}f$ to $f$ or is there sth that we need to change?

## 3.3 FT of the characteristic function

- (i) for $B>0$: $\mathcal{F}\chi_{\[-B,B\]}(\omega) = \sqrt{\frac{2}{\pi}}B\,\text{sinc}\left(\frac{B\omega}{\pi}\right)$
  - where $\text{sinc}(x)=\left(\frac{\sin{(\pi x)}}{\pi x}\right)$, for $x=0$: $\text{sinc}(x)=1$
- (ii) thus, the FT of a function with compact support does not necessarily have a compact support
- **problem**: $\text{sinc}(x)$ is not integrable, ie. $\text{sinc}(x)\not\in L^1(\mathbb{R})$
- **problem**: $\mathcal{F}$ does not map $L^1$ to itself
  - this is bad news for the invertibility of $\mathcal{F}$, but to use the FT in image processing we need the inverse FT &rarr; need a new space (<span style="color:red">Schwartz space</span>)
    - you can think of this space as infinitely differentiable functions that rapidly decay, so if you go to $\infty$ the values go to zero and the values go to zero faster than any polynomial goes to infty and the same is true for all derivatives
    - klarer: "it turns out that $F$ is <span style="color:red">bijective</span> in this Schwartz space"
      - ie. if we restrict the $L^1$ in <span style="color:green">(3.2)</span> to a smaller space (the Schwartz space) then we get a bijective transform
      - and then we will show that with this transform on the Schwartz space we get a FT on $L^2$, so for square integrable functions, and we will see that this is also bijective (so that we get sth that is bijective when we tranform from $L^2$ to $L^2$ and we have the inverse transform and we know how to compute it)

<p align="center">
  <img src="https://i.ibb.co/V9Q5t4h/Screenshot-from-2024-03-02-04-50-41.png" alt="Screenshot-from-2024-03-02-04-50-41" border="0">
</p>

- slide demo:
  - left: $f=\chi$: white circle on black background
  - right: $T^{norm}( \lvert Ff\rvert )=\text{sinc}$: white center blob + multiple white rings around center (with intensity dropping to the outside)

## 3.4 Properties of $\mathcal{F}$

- $f\in L^1$ and $\underline{A}\in \text{GL}(d)$ (set of invertible matrices, ie. $\det A \neq 0$), then
  - $\mathcal{F}(T_y f) = M_y(\mathcal{F}f)$
    - in words: "a shift in the spatial domain is a frequency change in the frequency domain"
  - $\mathcal{F}(M_y f) = T_{-y}(\mathcal{F}f)$
  - $\mathcal{F}(D_{\underline{A}} f) = \frac{1}{\|\det \underline{A}\|}D_{\underline{A}^{-T}}(\mathcal{F}f)$
    - in words: when we apply the FT on $f$ first: **it is still the same transform**, but we just have to do a different coordinate transformation with some extra scaling
  - $\mathcal{F}(\overline{f}) = \overline{D_{-\mathbb{I}}(\mathcal{F}f)}$
    - $D_{-\mathbb{I}}$ is a "function argument sign changer" operator: it just says that we have to multiply the argument with $-1$ (bec if I take $-\mathbb{I}$ this just means "change the sign" and then conjugate)
- where $M_y: f \mapsto m_yf$ ($M_y$ ist einfach das Produkt $f(x)e^{ix\cdot y}$), $m_y: x \mapsto e^{ix\cdot y}$ and $D_{\underline{A}}: f \mapsto (x \mapsto f(\underline{A}x))$

## 3.5 Hermitian, Skew-Hermitian

- **Hermitian** (if real: **even**): $\overline{f(\omega)} = f(-\omega)$
- **Skew-Hermitian** (if real: **odd**): $\overline{f(\omega)} = -f(-\omega)$ 
- we need this def. for <span style="color:green">(3.6)</span>
- <span style="color:red">**warning**</span>: there are
  - **complex-valued functions** $f: \mathbb{R} \to \mathbb{C}$
  - **complex functions** $f: \mathbb{C} \to \mathbb{C}$
  - the terminology hermitian and skew-hermitian is used for complex functions, for complex-valued functions **even symmetric** and **odd symmetric** are used

## 3.6 When is $\mathcal{F}$ even or odd?

- for $f\in L^1$ (eg. images), <span style="color:purple">read $L^2$</span>
  - (i) $f$ real-valued $\Leftrightarrow$ $\mathcal{F}f$ hermitian
  - (ii) $f$ imaginary-valued $\Leftrightarrow$ $\mathcal{F}f$ skew-hermitian
- **Proof**:
  - Based on
    - <span style="color:green">(3.4)</span> $\mathcal{F}(\overline{f}) = \overline{D_{-\mathbb{I}}(\mathcal{F}f)}$
    - $\mathcal{F}$ is linear <span style="color:green">(3.2)</span> and injective <span style="color:green">(3.23)</span>
- we need this for

## 3.7 Convolution Theorem for $\mathbb{C}$-valued Functions

- for $f,g\in L^1$ $$\mathcal{F}(f\ast g) = (2\pi)^{\frac{d}{2}}\mathcal{F}(f) \mathcal{F}(g)$$
- "the FT changes the <span style="color:red">**convolution**</span> to a **pointwise** multiplication"
- **simple proof**:
  - its just a little bit of computation with the integral, so no deep theory going on,
  - we just need **Fubini** to change the order of integrals and the **substitution rule** for multiple variables, both things we have used a couple of times.

<p align="center">
  <img src="https://i.ibb.co/6r1MhhC/Screenshot-from-2024-03-02-07-36-36.png" alt="Screenshot-from-2024-03-02-07-36-36" border="0">
</p>

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
  - "the 1st argument is conjugated"
- (ii) Generalize 2.3:
  - $\overline{(\psi\star f)(x)} = (f\star \psi)(-x)$

## 3.10 "Correlation Theorem" for $\mathbb{C}$-valued Functions

- for $f,g\in L^1(\mathbb{R}^d, \mathbb{C})$ $$\mathcal{F}(f\star g) = (2\pi)^{\frac{d}{2}}\overline{\mathcal{F}(f)} \mathcal{F}(g)$$
- "the FT changes the <span style="color:red">**correlation**</span> to a **pointwise** multiplication"
- "the 1st argument is conjugated"
- **problem**:
  - We need to continue to work on the inverse of the FT.
  - We have seen that the FT maps $L^1$ functions to continuous functions and that the transformed functions do not need to be integrable.
  - We could not even apply the FT twice bec after the FT we may loose this integrability.
  - This shows that we need a different space.
  - It will turn out that there is a space where the def. of the FT that we have is invertible.

## 3.11 Schwartz Space

- aka **space of rapidly decreasing <span style="color:red">smooth</span> functions on $\mathbb{R}^d$** ([smooth functions](#cinfty-smooth-functions)): $$S(\mathbb{R}^d,\mathbb{C}):=\{f\in C^\infty : C_{\alpha,\beta}(f) := \sup_{x\in \mathbb{R}^d}{\bigg\vert x^\alpha\frac{\partial^\beta}{\partial x^\beta} f(x)\bigg\vert} < \infty\}$$
- this definition is just formalizing what it means to be "rapidly decreasing"
- **Schwartz functions**: infinitely differentiable functions that rapidly decay
  - (i) infinitely differentiable
  - (ii) **rapidly decay**: for $x\to \infty$, $f\to 0$ **faster than any polynomial** goes to $\infty$
  - (iii) **all** derivatives decay rapidly as well

**idea**:

- first, think about the case $\alpha=2$ and $\beta=0$:

<p align="center">
  <img src="https://i.ibb.co/Qpxw8bJ/3-12alpha.png" alt="3-12alpha" border="0">
</p>

- **counterexample** to understand the role of $\beta$:
  - $f$ gets very tiny along the $x$-axis, but it still goes up and down rapidly, so the derivative would still be large, although the values are very small. Thus, this would NOT be a Schwartz function because all derivatives also have to fulfill this property

<p align="center">
  <img src="https://i.ibb.co/3vZK0S0/3-12.png" alt="3-12" border="0" />
</p>

- Is the $\text{sinc}$ function a Schwartz function?
  - No, it is not because the $\text{sinc}$ function cannot be integrated (see 3.14). We had that the $\text{sinc}$ function is **not** in $L^1$, whereas it will turn out that all Schwartz functions **can** be integrated (discussed later in 3.14) - which is necessary to apply the FT to them
  - but it is **not** obvious because you see the $\text{sinc}$ function going to zero nicely, **but** not fast enough (to be a Schwartz function) !

- **problem**: next, lets try to understand 3.11

## 3.12 $C_c^\infty$ Functions are always Schwartz functions

- (i) $C_c^\infty \subset S \subset C^\infty$
- why $C_c^\infty \subset S$ ?
  - because for the functions with compact support we know that they are $0$ at a certain point. Outside a certain ball that is large enough they are completely $0$ &rarr; and "$0$" is, of course, "rapidly decreasing to $0$" ("0" just cancels everything outside the ball) and thus, for all $f\in C_c^\infty$ 3.11(ii) is fulfilled
  - The <span style="color:red">**difference between $C_c^\infty$ and $S$**</span> is that in $S$ you do not have to be $0$ at the boundary (or $\to\infty$, respectively), but you have to get very quickly very close to $0$ (eg. $g_\sigma$ has this property).
    - **counterexample**: the Gaussian kernel <span style="color:red">$g_\sigma$ is a Schwartz function</span> **without** compact support, ie. $g_\sigma\not\in C_c^\infty$
      - **proof**:
        - all derivatives of $g_\sigma$ are a product of $g_\sigma$ and a polynomial
        - $g_\sigma$ goes to $0$ faster than polynomials go to $\infty$
        - products of polynomials are still polynomials
- the "$\subset$" in (i) is strict, ie. there are Schwartz functions which are not in $C_c^\infty$, eg. $g_\sigma$
- **problem**:
  - how does this help us with the FT?
  - now we have a new object, a Schwartz function, but we know nothing about Schwartz functions
    - what properties do Schwartz functions have?

## 3.13 Properties of Schwartz Functions

- $f\in S$, then
  - (i) $\frac{\partial^\gamma}{\partial x^\gamma}f \in S$ for all multi-indices $\gamma \in \mathbb{N}^d$ ("derivatives of Schwartz functions are Schwartz functions")
  - (ii) $pf\in S$ for all polynomials $p$ ("multiplication of a Schwartz function with a polynomial gives a Schwartz function")
  - (iii) $fg\in S$ for all $g\in S$ ("a product of Schwartz functions is a Schwartz function")
- **idea**:
  - the properties are not surprising: because we know that for all polynomials and derivatives $\bigg\vert x^\alpha\frac{\partial^\beta}{\partial x^\beta} f(x)\bigg\vert$ is bounded.
    - So, if we multiply $\bigg\vert x^\alpha\frac{\partial^\beta}{\partial x^\beta} f(x)\bigg\vert$ by another polynomial - as in (ii) - we get, again, a different polynomial, but it should be covered by the fact that we considered **ALL** possible polynomials in 3.11 (and we also considered **ALL** derivatives)
    - also, if you multiply $f$ with functions that go to $0$ - as in (iii) - then they go to $0$ even faster
- **proof**:
  - verify the conditions in definition 3.11:
    1. show that each object (i)-(iii) is in $C^\infty$
      - (i) $\frac{\partial^\gamma}{\partial x^\gamma}f \in C^\infty$ because $f \in S$ and $S\subset C^\infty$
      - (ii) polynomials are in $C^\infty$
      - (iii) $g$ is in $C^\infty$ because $g\in S$
    2. show that $C_{\alpha,\beta}$ is finite for each object (i)-(iii) ($C_{\alpha,\beta}< \infty$ for all $\alpha$ and $\beta$)
- **problem**:
  - **before**: we started with defining $\mathcal{F}$ on $L^1$ and noted that it maps from $L^1$ to the continuous functions, but the FTs are not necessarily in $L^1$ (we had this example of the characteristic function of an interval which was mapped to the cardinal sine function which is NOT integrable)
  - **now**: we want to show that if we restrict $\mathcal{F}$ to the Schwartz space ($\mathcal{F}: S \to S$) then we get a bijective function.
  - but to realize that the $\mathcal{F}$, as we defined it before, is actually defined on the Schwartz space we still need to know that the <span style="color:red">**Schwartz functions can be integrated**</span>.
    - So far we just said that they are a subset of $C^\infty$, but $C^\infty$ functions cannot be necessarily integrated (think about the constant function $1$ which is obviously as smooth as it gets, but the area under the function $1$ is $\infty$).
  - the first thing that we will do now is to show that <span style="color:red">**$S$ is a subset of $L^1$**</span> and actually we show a bit more:

## 3.14 Schwartz functions can be integrated wrt any Power

- (i) $1\leq p\leq\infty$, then $S\subset L^p$
  - in words: Schwartz functions can be integrated wrt any power (or in other words, this "rapidly decreasing to $0$" is enough to have finite area under the functions)
  - in particular, this means that we can apply $\mathcal{F}$ to the Schwartz space because we know we can apply it to $L^1$ functions.
- (ii) **a bound for the $L^p$-norm**: $p<\infty$ and $q\in \mathbb{N}$ with $2qp > d$, then there is a $C>0$ s.t. for all $f\in S$ $$\|f\|_{L^p}\leq C\bigg\|(1 + \|\cdot\|_2^{2q}) f\bigg\|_{L^\infty} < \infty$$
  - (ii) is just a technical tool if we need to say sth about the $L^p$ norm of a Schwartz function then we have a bound here (which is just the supremum of the Schwartz function $f$ multiplied by a certain polynomial, where the role of "$q$" is just that the "$q$" has to be big enough so that this polynomial is large enough to get this bound).
  - Berkels: this very specific structure of the bound in (ii) we will only need once (&rarr; in the proof of 3.18, see lec 15 (XI)), but the integrability (i) is what you should remember
- **problem**:
  - (A) Another tool that we will use has to do with derivatives: one of the things that we have to show in order to realize that the FT maps from the Schwartz functions to the Schwartz functions is to <span style="color:red">**show that the FT is differentiable**</span> <span style="color:green">(&rarr; **3.17**)</span>.
    - If I take the FT of the Schwartz function, for this to be a Schwartz function that it in particular needs to be infinitely often differentiable.
    - We need to figure out can we derive the FT of the Schwartz function and if so what are the derivatives.
  - (B) One tool that we need for (A) is <span style="color:red">**integration by parts**</span>, ie. if we do $\int fg'$ that this is the same as $-\int g'f + \text{boundary terms}$ <span style="color:green">(&rarr; **3.16**)</span>.
    - but this is more <span style="color:red">tricky</span> if you integrate <span style="color:red">over all of $\mathbb{R}^d$</span> because **usually** you would integrate over an integral or a **bounded domain** and then you have these terms on the boundary, but if there is <span style="color:red">no boundary</span> because you integrate over all of $\mathbb{R}^d$ you have to be extra cautious
    - and this is what the following lemma <span style="color:green">(&rarr; **3.15**)</span> prepares

## 3.15 Requirements for Integration by Parts over an Unbounded Domain

- **conditions**:
  - $f\in C^1$ s.t. $\partial_i f\in L^1$ for an $i\in \\{1,\ldots,d\\}$
    - we will need a minimum amount of assumptions on $f$, but you can think about this $f$ as a Schwartz function (that will be sufficient)
  - $\sup_{x\in \mathbb{R}^d}{\bigg\lvert f(x) \lVert x\rVert_{2}^{2d}\bigg\rvert}<\infty$,
    - ie $f$ must go to $0$ faster than $x^2$ (this is just a restriction for the behavior of $x\to\infty$)
- then $$\int_{\mathbb{R}^d}\partial_i f(x)dx = 0$$
- in words: the fact that $f$ goes to $0$ faster than $x^2$ leads to the fact that the <span style="color:red">derivatives cancel each other out along the space</span> (ie the mean value of the derivative is $0$ of this function).
- if you think about this, a Schwartz function fulfills ALL of these conditions without any extra effort

## 3.16 Integration by Parts in $\mathbb{R}^d$

- **Warning**: <span style="color:green">(3.8)</span> was <span style="color:red">for the operator $\mathcal{F}$</span>, but this is <span style="color:red">for the operator $\partial_i$</span>
- **conditions**:
  - choose $g,h\in C^1$ s.t. $f := gh$ fulfills the conditions of <span style="color:green">(3.15)</span> (ie differentiability and integrability of the derivative and this growth constraint)
    - dh. $g$ und $h$ müssen **nicht unbedingt** die conditions in <span style="color:green">(3.15)</span> erfüllen! Nur $f$ muss sie erfüllen.
  - $\partial_i gh, g\partial_i h\in L^1$
- (i) integration by parts in $\mathbb{R}^d$: $$\int_{\mathbb{R}^d} \partial_i g(x)h(x)dx = -\int_{\mathbb{R}^d} g(x)\partial_i h(x)dx$$
  - in words: we are allowed to move the derivative from one function to the other, we just get the flip in the sign. And if you can do this once you can also do this more often. If you would have more derivatives here we could iteratively put them over to the $h$ and for each derivative you get one "$-1$".
- (ii) or by iterating this for $\alpha\in \mathbb{N}^d$: $$\int_{\mathbb{R}^d} \frac{\partial_\alpha}{\partial x_\alpha} g(x)h(x)dx = (-1)^{\vert\alpha\vert}\int_{\mathbb{R}^d} g(x)\frac{\partial_\alpha}{\partial x_\alpha} h(x)dx$$
- Note: (i) only holds iff $0 = \int_{\mathbb{R}^d}\partial_i (gh)(x)dx$ which, in turn, only holds if $gh$ fulfills the conditions of <span style="color:red">3.15</span>

<p align="center">
  <img src="https://i.ibb.co/NnjY5kN/Screenshot-from-2024-03-03-11-49-49.png" alt="Screenshot-from-2024-03-03-11-49-49" border="0">
</p>

## 3.17 Differentiability of the $\mathcal{F}$-transformed Schwartz Function

- **conditions**:
  - $f\in S$
  - $p^\alpha (x) = x^\alpha$
- (i) $\mathcal{F}(f)\in C^\infty$
- (ii) $\mathcal{F}(\frac{\partial^\alpha}{\partial x^\alpha}f) = i^{\vert \alpha\vert}p^\alpha\mathcal{F}(f)$
- (iii) $\mathcal{F}(p^\alpha f) = i^{\vert \alpha\vert}\frac{\partial^\alpha}{\partial \omega^\alpha}\mathcal{F}(f)$
- in words: the FT converts **derivatives** into a **multiplication with polynomials** and vice versa (like the FT converted a translation into modulation and vice versa)
- **proof**:
  - (i) will be shown last
  - show (ii) using integration by parts 3.16 (which requires checking the conditions in 3.15, too)
  - show (iii), using B.12 "if the derivative is uniformly bounded by an integrable function, differentiation and integration may be interchanged"
  - with (iii) we know that we are in $C^\infty$ bec we know what the derivatives are, thus (i) holds
- **problem**: now we know $\mathcal{F}(f)\in C^\infty$, but we need to show that the mapping $\mathcal{F}$ maps to the Schwartz space

## 3.18 $\mathcal{F}f\in S$

- (i) $\mathcal{F}f\in S$
- (ii) if $(f_n)\_n\in S$ and $\lim_{n\to \infty}{C_{\alpha,\beta}(f_n)} = 0\, \forall \alpha,\beta\in \mathbb{N}_0^d$, then $$\lim_{n\to \infty}{C_{\alpha,\beta}(\mathcal{F}f_n)} = 0 \,\forall \alpha,\beta\in \mathbb{N}_0^d$$
  - in words: The 2nd part has to do with **continuity**: if we have a **sequence of Schwartz functions** and their coefficients converge to $0$ for all multiindices $\alpha$ and $\beta$, then the coefficients of the $\mathcal{F}$-transformed functions also converge to $0$.
    - This essentially means that we have **continuity** of $\mathcal{F}$ (not $\mathcal{F}f$, this we have already shown in 3.17(i)) on the Schwartz space (ie. $\mathcal{F}: S \to S$ is continuous), but this will be discussed in 3.19
- **proof**:
  - (i)
    - show $\mathcal{F}f\in C^\infty$
    - show $C_{\alpha,\beta}(\mathcal{F}f) < \infty$
    - using 3.17
  - (ii) 
    - we fix some $\alpha$, $\beta$ and we need to show that for this fixed $\alpha$, $\beta$ the limit of the coefficients of the Ftransformed functions $\lim_{n\to \infty}{C_{\alpha,\beta}(\mathcal{F}f_n)}$ is also $0$
    - using 3.14 (ii) the "technical" part about the upper bound
- **problem**: now lets come to the promised interpretation of 3.18 (ii), what does it mean and what does it have to do with continuity?

## 3.19 Alternative Definition of Convergence and Continuity

- so far we always used **norms** to think about convergence, eg. if $x_n$ converges to $x$ that meant that the norm of $x_n-x$ converges to $0$, but you can also define this in different ways
- (i) **convergence** of a sequence: $f_n\in S$ converges to $f\in S$, if $\lim_{n\to\infty}{C_{\alpha,\beta}(f_n-f)} = 0\,\forall\alpha,\beta$
- (ii) **continuity** in $0$: Let $f_n\in S$ converges to $0\in S$, ie. $\lim_{n\to\infty}{C_{\alpha,\beta}(f_n-0)} = 0\,\forall\alpha,\beta$, then $\lim_{n\to\infty}{C_{\alpha,\beta}(f_n)} = 0$ and thus, by <span style="color:green">**3.18 (ii)**</span>, $\lim_{n\to\infty}{C_{\alpha,\beta}(\mathcal{F}f_n)} = 0\,\forall\alpha,\beta$ and therefore, $$\lim_{n\to\infty}{C_{\alpha,\beta}(\mathcal{F}f_n-\mathcal{F}0)} = 0\,\forall\alpha,\beta$$
  - this means $\mathcal{F}f_n$ converges to $\mathcal{F}0$ (in the sense of our defined convergence (i)) which shows the <span style="color:red">**"continuity"**</span> of $\mathcal{F}$
- recall: def. of pointwise continuous in $x\in X$: $\lim{f(x_n)}=f(\lim{x_n})$, ie. "you can switch function evaluation and limit"
- **problem**:
  - This is more a side remark that if you want to look at $\mathcal{F}$ just in the Schwartz space and continuity there then this would be the way to go. But we are more importantly interested in the **bijectivity**.
  - We have shown the 1st part of our goal, ie. the form of the mapping $\mathcal{F}: S \to S$, but not its bijectivity.
  - Now it is time to look at the **inverse**.
  - And there is one final building block that we need before I can write down what the inverse is and show that it is the inverse.
  - This is a property of the Gaussian kernel.

## 3.20 Gaussian as an Eigenfunction of $\mathcal{F}$

- $g(x) := g_1(x) = \frac{1}{(2\pi)^\frac{d}{2}}e^{-\frac{1}{2}\lVert x\rVert^2}$
- $\mathcal{F}g = g$
  - "the FT of a Gaussian is the Gaussian itself"
  - "the Gaussian is an Eigenfunction of $\mathcal{F}$ with Eigenvalue $1$"
- proof: <span style="color:red">TODO</span>
  - "indirect" proof
  - we use that the Gaussian is separable
  - $g$ and $Fg$ solve the same linear homogeneous DE with the same IVs
- **problem**: finally, we can say what the inverse is (at least under some conditions, but it is sufficient for the Schwartz space)
  - I will formulate it a little more general than just on the Schwartz space, so that you also see at least sth about $L^1$

## 3.21 Fourier Inversion Theorem

- <span style="color:red">only applicable to continuous $L^1$-integrable functions that have an $L^1$-integrable FT (eg. Schwartz functions)</span>
- **conditions**:
  - (i) $f\in L^1$ with $\mathcal{F}f\in L^1$
    - must assume this to rule out $\chi_{\[-B,+B\]}\in L^1$ <span style="color:green">(3.3)</span>
    - because we know that $\mathcal{F}f\in L^1$ is not true for all $L^1$ functions, eg. we have seen $\chi_{\[-B,+B\]}\in L^1$ where we know that the cardinal sine is **not** in $L^1$.
    - $\mathcal{F}f\in L^1$ is definitely not true for all $L^1$ functions, but at least for some.
    - In particular, it is **true for all Schwartz functions**.
  - (ii) $f$ is continuous in $x$
    - which would be **true for all Schwartz functions**
- then $$f(x) = \frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}(\mathcal{F}f)(\omega)e^{ix\cdot\omega}d\omega$$
- Thus, we are looking at functions that are not only **continuous**, but also **integrable**. Because we must rule out eg. $\chi_{\[-1,+1\]}$ <span style="color:green">(3.3)</span> because its FT is the cardinal sine and we must not ignore this.
  - **outlook**: actually we will be able to repair this for $\chi_{\[-1,+1\]}$ <span style="color:green">(3.3)</span> because there **you can convert back**, <span style="color:red">but not with this formula</span>. It is somewhat different.
- **proof**:
  - by the DCT with the <span style="color:red">integrable</span> majorant $\lvert\mathcal{F}f\rvert$ (because from (i) we know $\mathcal{F}f\in L^1$) we know $\frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}(\mathcal{F}f)(\omega)e^{ix\cdot\omega}d\omega = \lim_{\epsilon\to 0}\frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}(\mathcal{F}f)(\omega)e^{-\frac{1}{2}\epsilon \lvert\omega\rvert^2+ix\cdot\omega}d\omega$
    - <span style="color:red">ie. without condition (i) this proof is not valid!</span>
    - conditions of DCT:
      1. integrand $f_n$ converges pointwisely to $f$
      2. integrable majorant $g$
  - Now, you may say that I make this easy looking formula much more complicated bec I put these extra terms in there. Why did I do this?
  - If you look closely the term $e^{-\frac{1}{2}\epsilon \lvert\omega\rvert^2}$ is **a Gaussian**.
  - And we have just learned that if we apply a FT to a Gaussian then the FT vanishes bec the Gaussian is a Eigenvalue <span style="color:green">(3.20)</span>.
  - And we know that we can move a FT from one factor to the other factor <span style="color:green">(3.8)</span>.
  - So, <span style="color:red">the idea</span> now is we take the FT from our $f$ where we **cannot** compute it and move it to our Gaussian where we **can** compute it.
  - In the end we have $\frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}(\mathcal{F}f)(\omega)e^{-\frac{1}{2}\epsilon \lvert\omega\rvert^2+ix\cdot\omega}d\omega = \int_{\mathbb{R}^d} f(\omega)\psi_\epsilon(x-\omega)d\omega = (f\ast\psi_\epsilon)(x)$
    - where $\psi(\omega) := \frac{1}{(2\pi)^{\frac{d}{2}}}e^{-\frac{1}{2}\lvert\omega\rvert^2}$ is the Gaussian and $\psi_\epsilon$ is its scaled version as in <span style="color:green">(2.3 (iii))</span>
    - to show the 1st equality
      - we use <span style="color:green">(3.8)</span> to move the $\mathcal{F}$ like so: $\frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}(\mathcal{F}f)(\omega)e^{-\frac{1}{2}\epsilon \lvert\omega\rvert^2+ix\cdot\omega}d\omega = \frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}f(\omega)\mathcal{F}(e^{-\frac{1}{2}\epsilon \lvert\omega\rvert^2+ix\cdot\omega})d\omega$
      - we rewrite $\frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}f(\omega)\mathcal{F}(e^{-\frac{1}{2}\epsilon \lvert\omega\rvert^2+ix\cdot\omega})d\omega = \int_{\mathbb{R}^d}f(\omega)\mathcal{F} (M_x D_{-\epsilon\mathbb{I}} \psi)d\omega$
      - we show that $\mathcal{F} (M_x D_{-\epsilon\mathbb{I}} \psi) = \psi_\epsilon(x-\omega)$ using <span style="color:green">(3.4)</span>
  - Then, using <span style="color:green">(B.10 (iii))</span>, we go the same limit $\lim_{\epsilon\to 0}$ back and the Gaussian will vanish again, ie. $\lim_{\epsilon\to 0}(f\ast\psi_\epsilon)(x) = f(x)$.
    - to fulfill the **2nd condition** of the pointwise convergence in B.10 (iii) the continuity of $f$ at position $x$ - that we assumed in (ii) - is necessary
    - <span style="color:red">ie. without condition (ii) this proof is not valid!</span>
    - $f$ does not fulfill the **3rd condition** of B.10 (iii), $f\in L^\infty$ (boundedness of $f$)
      - but in <span style="color:purple">**ex. 7.2**</span> we showed that $f\in L^1$ works, if $\psi\in S$
  - So, by sneaking in a small Gaussian and letting that absorb our FT and going back, we can get the inverse of $\mathcal{F}f$, ie. $f(x)$.

## 3.22 Fourier Inversion Theorem a.e. without Continuity of $f$

- without the continuity of $f$ the Inversion Theorem <span style="color:green">(3.21)</span> still holds **for almost all $x$**
  - as opposed to "for every $x$"
- **why useful**:
  - but this is just some more technical remark.
  - you need some measure theory to see that **convergence in the $L^1$ norm leads to pointwise a.e. convergence of a subsequence**.
  - But that is not so important.
  - I just wanted to point out that this **inversion theorem also holds in a more general sense of a.e.** with the only drawback that we loose the guarantee for a very specific $x$, but in terms of $L^1$ functions this is ok.
- **problem**:
  - now lets fill one little gap that I left when we were talking about these Hermitian and skew-Hermitian functions, where we said that if $f$ is real-valued this is equiv. to the $\mathcal{F}f$ being Hermitian. And **for the backward direction** <span style="color:red">we needed that the FT is injective</span>.
  - And with the inversion theorem we finally get this <span style="color:red">injectivity</span> ...

## 3.23 $\mathcal{F}: L^1 \to C$ is injective

- $\mathcal{F}: L^1 \to C$ is injective
- **proof**:
  - with Inversion Theorem a.e. without continuity of $f$ <span style="color:green">(3.22)</span>
- Thus, a byproduct of the inversion theorem is the injectivity.
- **problem**:
  - Based on this inversion theorem we can now show that <span style="color:red">the FT is a bijection on the Schwartz space</span> which then justifies essentially our introduction of the space
  - But, before we show that the FT is a bijection on the Schwartz space, we first show this simple formula here that essentially summarizes what this inverse formula says in a very compact way for the Schwartz space: It indicates that the <span style="color:red">FT is almost inverse to itself</span>. So, if we apply the FT twice then we get the function, but it is evaluated at the position $x$ multiplied by "$-1$". So, there is just this mirroring which has to do with the missing minus in the exponent of "$e^{ixw}$" in the inversion formula

## 3.24 $\mathcal{F}$ is almost inverse to itself

- if $f\in S$, then $(\mathcal{F}\mathcal{F}f)(x) = f(-x)$ for all $x$
- **problem**:
  - with this tool we can show the <span style="color:red">bijectivity of the FT on the Schwartz space</span>

## 3.25 $\mathcal{F}$ is bijective in $S$

- if we restrict the domain of $\mathcal{F}$ from $L^1$ to $S$ then we get a bijective FT
- (i) $\mathcal{F}$ is a bijection from $S$ to itself
- (ii) the inverse is $\mathcal{F}^{-1} = \mathcal{F}^3$
- (iii) $(\mathcal{F}^{-1}f)(x) = \frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}f(\omega)e^{ix\cdot\omega}d\omega$ for $f\in S$
  - the inversion formula <span style="color:green">(3.21)</span> describes what you need to apply to $\mathcal{F}f$ to go back to $f$
  - but this here is about how you apply the the inverse $\mathcal{F}^{-1}$ on any function
- (iv) $(f,g)\_{L^2} = (\mathcal{F}f,\mathcal{F}g)\_{L^2}$ for $f,g\in S$
  - where $(f,g)\_{L^2} := \int_{\mathbb{R}^d} f(x)\overline{g(x)}dx$
  - in words: "the FT does not change the $L^2$ scalar product"
    - the $L^2$ scalar product represents the "cosine of the angle" between two objects
    - thus, "the FT preserves lengths and angles"
  - thus, we also have $\lVert f\rVert_{L^2} := \sqrt{(f,f)\_{L^2}} = \sqrt{(\mathcal{F}f,\mathcal{F}f)\_{L^2}} =: \lVert \mathcal{F}f\rVert_{L^2}$
    - this is the "usual way" how you can create norms out of scalar products
- **proof**:
  - by <span style="color:green">(3.24)</span> we know $(\mathcal{F}^4f)(x)=(\mathcal{F}^2f)(-x)=f(x)$, thus $\mathcal{F}^4$ is the identity on $S$
  - (i) **bijective** = **injective** + **surjective**:
    - **injective**: if $\mathcal{F}f=\mathcal{F}g$ then $f = \mathcal{F}^3(\mathcal{F}f) = \mathcal{F}^3(\mathcal{F}g) = g$ ($f(a)=f(b)\Rightarrow a=b\,\forall a,b$)
    - **surjective**: if $f\in S$ then $f = \mathcal{F}(\mathcal{F}^3f)$ ("zu jedem Funktionswert gibt es eine Stelle")
  - (ii) $\mathcal{F}^4 = \text{id}_S$ implies $\mathcal{F}^{-1}=\mathcal{F}^3$
  - (iii) $(\mathcal{F}^{-1}f)(x)=(\mathcal{F}^3f)(x)=(\mathcal{F}^2(\mathcal{F}f))(x)=(\mathcal{F}f)(-x)$ (the last "$=$" follows from <span style="color:green">(3.24)</span>)
  - (iv) $(\mathcal{F}f,\mathcal{F}g)\_{L^2} := \int_{\mathbb{R}^d} \mathcal{F}f(x)\overline{\mathcal{F}g(x)}dx$ then use <span style="color:green">(3.8), (3.4)</span> and <span style="color:green">(3.24)</span> to arrive at $\int_{\mathbb{R}^d} f(x)\overline{g(x)}dx$
- **problem**:
  - So, this will give us a way to extend functions from a dense subset/dense subspace to a bigger space

## 3.26 Extension to a Bigger Space

- **problem**:
  - So, on the Schwartz space we have everything nicely under control (meaning <span style="color:green">(3.25 (i) - (iv))</span>),
  - but there is still the problem that objects we are usually working with, ie. <span style="color:red">images, are not Schwartz functions</span>!
    - **Why?**: <span style="color:red">Images are usually not continuous</span>, eg. you may have intensity jumps and if there is noise this would look like some kind of oscillation. And if images are not continuous they cannot be in $S$.
  - So, <span style="color:green">(3.25)</span> is way too strong to be used "as is" in practice, so we **need to extend** this to a bigger space.
  - So, <span style="color:red">the idea for extension</span> is:
    - recall, the statement we did in chapter 2, where we were looking at the correlation and convolution and what we showed there was that the continuous functions with compact support $C_c$ <span style="color:green">(B.8)</span> and $C_c^\infty$ <span style="color:green">(2.7)</span> are dense in $L^p$ for $1\leq p<\infty$ (ie. $\forall f \in L^p (\Omega)\, \forall \epsilon > 0\, \exists g \in C_c (\Omega) : \lVert f − g\rVert_{L^p(Ω)} < \epsilon$). In particular, they are dense in $L^2$.
      - Thus, for any $L^2$ function we can get arbitrarily close with a $C_c^\infty$ function and <span style="color:red">**$C_c^\infty$ functions are also Schwartz functions**</span> <span style="color:green">(3.12)</span>.
    - So, <span style="color:red">the idea is</span>, lets extend $\mathcal{F}$ from our Schwartz space to $L^2$ by using this denseness.
    - On the Schwartz space it should do exactly what we have shown in <span style="color:green">(3.25)</span>, but we want to extend it to the entire space $L^2$.
  - And such kind of extension is actually a general tool that one can use **without** having explicitly Schwartz space here and $L^2$ there - it is just working in normed vector spaces.
  - And this is what the next lemma is preparing.
  - So, this will give us a way to extend functions from a **dense** subspace $S$ (note: $S\subset L^2$ was shown in <span style="color:green">(3.14)</span>) to a bigger space $L^2$ (like the dense subspace $\mathbb{Q}$ can be extended to $\mathbb{R}$)
- **conditions**:
  - (i) $X$ a normed vector space, $Y$ a Banach space
    - **Banach Space**: a complete, normed vector space
      - in other words: sequences, where the difference between consecutive elements gets smaller and smaller (aka "**Cauchy sequence**"), have to converge
        - for <span style="color:red">**real**</span> sequences: Cauchy sequence $\Leftrightarrow$ convergent sequence ($\forall\epsilon>0\,\exists N\,\forall n\geq N: \lvert a_n-a\rvert < \epsilon$ **or** $\lim_{n\to \infty}a_n = a$)
      - sidenote: if you havent heard about **complete vector spaces** before think about the difference betweeen the rational numbers and the real numbers:
        - you can have a Cauchy sequence of rational numbers that does **not** converge to a rational number, eg. if you approximate $\pi$ with rational numbers (&rarr; Leibniz series) (this is a Cauchy sequence, you just add more and more digits), but in the rational numbers it does not converge because $\pi$ is not a rational number. Thus, the rational numbers are **not** complete (see "Preliminaries" &rarr; "complete").
      - essentially, we make the assumption that <span style="color:red">the set $Y$</span> does **not** have this problem, it <span style="color:red">is complete</span> (and <span style="color:red">$L^2$ is also complete</span>, so if you want to think more specifically and not in abstract terms, then <span style="color:red">think of $Y$ as $L^2$</span>)
      - it is just here that I am trying to present this in an abstract way, where we only assume what is essentially necessary and not more
  - (ii) norms $\lVert\cdot\rVert_X$ and $\lVert\cdot\rVert_Y$
  - (iii) $V\subset X$ is a <span style="color:red">**dense**</span> subspace of $X$
    - <span style="color:red">**Warning**</span>: $V$ must be <span style="color:red">dense</span> in $X$, ie. just <span style="color:red">being a subset of $X$ is not enough!</span> This is the main idea of this proposition.
    - this would be our Schwartz space $S$ = a dense subspace of $L^2$ (so, $V=S$ and $X=L^2$)
  - (iv) $F: V\to Y$ is a linear isometry (ie. (iv.i) $F$ is linear **and** (iv.ii) $\lVert F(x)\rVert_Y = \lVert x\rVert_X$ for all $x\in V$)
    - thinking about our specific setting: $F: S \to L^2$
      - check if the extension <span style="color:red">from "$F: S \to S$" to "$F: S \to L^2$"</span> is allowed:
        - We know that $F$ is mapping from the Schwartz space to the Schwartz space, but the Schwartz space is a subspace of $L^2$ so "$F: S \to L^2$" is fulfilled.
        - phth: "$F$ mapped in $S$ rein" && "$S$ ist in $L^2$" $\Rightarrow$ "$F$ mapped in $L^2$ rein"
      - check properties of mapping $F$:
        - (iv.i) **linear**: is clear
        - (iv.ii) **isometry**: automatically fulfilled bec we know that <span style="color:green">(3.25 (iv))</span> must hold for "$F: S \to S$". In other words, <span style="color:green">(3.25 (iv))</span> implies isometry: $$\begin{align*}
        \lVert f\rVert &= \sqrt{(f,f)}\\
        &= \sqrt{(Ff,Ff)}   \quad\text{( by 3.25 (iv) (with f=g) )}\\
        &= ||Ff||\end{align*}$$
          - and thus, the norm of $f$ is not changed by the FT "$F$".
      - important:
        - <span style="color:green">(3.25 (iv))</span> is stronger than the isometry statement in <span style="color:green">(3.26)</span> since <span style="color:green">(3.25 (iv))</span> preserves the scalar product, whereas in <span style="color:green">(3.26)</span> we do not even need to have a scalar product, we just need norms.
- **theorem**: then, there is a unique linear isometry $\hat{F}: X\to Y$ with $\hat{F}\vert_V = F$

## 3.27 Extension of $\mathcal{F}$ from $\mathcal{S}$ to $L^2$

- (i) There is a unique, <span style="color:red">**bijective**</span>, linear isometry $$\mathcal{F}_2: L^2 \to L^2 \quad \text{s.t.} \quad \mathcal{F}_2\vert_\mathcal{S} = \mathcal{F}.$$
  - ie. on the Schwartz space this isometry "$F_2$" is exactly the FT as we defined it
  - we wanted s.th. that is bijective when we transform it, $\mathcal{F}_2$ is what we were looking for
- (ii) **Plancherel formula**: $(f,g)\_{L^2} = (\mathcal{F}_2f,\mathcal{F}_2g)\_{L^2}$ for $f,g\in L^2$
  - where $(f,g)\_{L^2} := \int_{\mathbb{R}^d} f(x)\overline{g(x)}dx$
  - in words: "the FT on $L^2$ preserves the scalar product"
  - we have already seen this formula, but we just did not call it "Plancherel formula":
    - we already have shown this on the Schwartz space (see <span style="color:green">(3.25)</span>)
    - but it also holds for the **extended FT** on $L^2$
- **problem**:
  - this is only an existence statement for $\mathcal{F}_2$, but does not tell us <span style="color:red">**how to compute** $\mathcal{F}_2$</span> ! (this "computation of $\mathcal{F}_2$" will be discussed in <span style="color:green">(3.28)</span>)

## 3.28 Computation of $\mathcal{F}_2$ (for certain $f$)

- **problem**:
  - now we are ready to apply the FT to actual functions, but before we do this we would like to have a better understanding of what this $\mathcal{F}_2$ looks like.
  - <span style="color:green">(3.27)</span> is not very constructive: It just says that we have this $\mathcal{F}_2$ and that it has all the nice properties that we want to have, but if we would want **to compute it**, then we would have to first start with a sequence of Schwartz functions and apply the usual FT and that is, of course, not very feasible.
  - So, we would like to have a more clear expression for this. And one thing that we can show is the following.
- (i) for $f\in L^1\cap L^2$
  - $(\mathcal{F}\_2f)(\omega) = \frac{1}{(2\pi)^{\frac{d}{2}}} \int\_{\mathbb{R}^d}f(x)e^{-ix\cdot\omega}dx$ for a.e. $\omega$
  - in words: "the FT has the shape that we would expect, <span style="color:red">but it just holds for a.e. $\omega$</span>"
    - clarification: "shape that we would expect": the $L^1$ FT we had in the beginning
  - **problem**: this is only for functions that are both in $L^1$ and $L^2$ &rarr; eg. we cannot use this to transform the <span style="color:red">cardinal sine</span> bec it is **not** in $L^1$
    - but the characteristic function is both in $L^1$ and $L^2$ (bec the Lebesgue integral is defined via simple functions which in turn are defined via the characteristic functions), so there we can apply this
  - <span style="color:red">**needed for**</span>:
    - proof of <span style="color:green">(3.50)</span> Shannon-Whittaker
- (ii) for $f\in L^2$
  - $\lVert\mathcal{F}\_2f - \phi\_R\rVert\_{L^2}$ where $\phi\_R(\omega) := \frac{1}{(2\pi)^{\frac{d}{2}}} \int\_{B_R(0)}f(x)e^{-ix\cdot\omega}dx$
  - $\phi\_R(\omega)$ is just like the FT on $L^1$ with the only difference that we do not integrate over all of $\mathbb{R}^d$, but just over a ball
  - in words: what this says is that, if we integrate over larger and larger balls then we get exactly the extended FT "$\mathcal{F}\_2f$".
    - thus, "for $L^2$ functions we can still represent this transform $\mathcal{F}_2$ <span style="color:red">**as a limit**</span>"
  - $\phi\_R(\omega)$ is like $\lim_{R\to\infty}\int_{-R}^{R}f(x)dx$ in the figure below, ie. we have to go to $\infty$ <span style="color:red">"with the same speed in all directions"</span>

<p align="center">
  <img src="https://i.ibb.co/rvwZY1F/F2-approx-ball-to-infty.png" alt="F2-approx-ball-to-infty" border="0">
</p>

## 3.29 Computation of $\mathcal{F}_2^{-1}$, Properties of $\mathcal{F}_2$

- (i) <span style="color:green">3.28</span> holds for $\mathcal{F}_2^{-1}$ when we replace $e^{-ix\cdot\omega}$ with $e^{+ix\cdot\omega}$ in <span style="color:green">(3.28)</span>
- (ii) <span style="color:green">3.4, 3.6, "Conv Theorem" 3.7, "Corr Theorem" 3.10</span> still hold for $\mathcal{F}_2$
- the general integrals <span style="color:red">on all of $\mathbb{R}^d$</span> in the def. of the FT that we did originally they do not have to exist, but the structure is preserved as we have seen in <span style="color:green">3.28 (ii)</span>, we just <span style="color:red">have to think about these integrals as limits of integrals on balls</span>, not as the more general integral on all of $\mathbb{R}^d$.
- In the remainder I want to drop the difference between $\mathcal{F}$ and $\mathcal{F}_2$ in the notation and just write $\mathcal{F}$
  - But keep in mind that you actually have to use this limit representation. It is just to keep the notation simple.
- **problem**:
  - what can we actually practically get from the FT: It gives us a so called "**frequency domain representation**" of arbitrary $L^2$ functions

## 3.30 Frequency Domain Representation

<p style="border-width:3px; border-style:solid; border-color:#FF0000; padding: 1em;">
  for $f\in L^2$ we have $f = \mathcal{F}^{-1}\mathcal{F}f \Leftrightarrow \boxed{f(x) = \frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}(\mathcal{F}f)(\omega)e^{ix\cdot\omega}d\omega}$<br>
  &rarr; (im letzten Schritt wurde einfach nur das $\mathcal{F}^{-1}$ auf der rhs von $f = \mathcal{F}^{-1}\mathcal{F}f$ ausgeschrieben)
</p>

- this gives us now a different way of <span style="color:red">interpreting what $f$ is</span>:
  - $f$ is now expressed as a **superposition** of **complex exponential functions**.
  - Think of these **exponential functions** as $\cos(xw)+i\cdot\sin(xw)$. These have frequencies $\omega$.
  - <span style="color:red">$(\mathcal{F}f)(\omega)$ specifies the **weight** of $e^{ix\cdot\omega}$</span> (the **amplitudes** of cosine and sine)
    - so, in this sense <span style="color:red">$(\mathcal{F}f)(\omega)$ tells us how strongly is the **frequency** $\omega$ encoded by the corresponding cosine and sine functions</span> present in our signal $f$. Bec we are really reconstructing the entire signal $f$ just by these complex exponential functions $e^{ix\cdot\omega}$.
    - in the dual sense the <span style="color:red">$f(x)$</span> in $\boxed{\mathcal{F}f(\omega) = \frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}f(x)e^{-ix\cdot\omega}dx}$ <span style="color:green">(3.1)</span> <span style="color:red">tells us how strongly is the **position** $x$ encoded by the corresponding cosine and sine functions</span> present in our transformed signal $\mathcal{F}f$.
- And this also leads to some notation or some <span style="color:red">**terminology**</span>:
  - $\mathcal{F}f$ is the **frequency domain representation**
  - $f$ is the **space domain representation**
  - $f$ if $d=1$ is the **time domain representation**
- in this integral you see how <span style="color:red">this is a global transform</span> bec to reconstruct $f$ at a single position $x$ we need to know all of the frequencies everywhere.
  - So, to get one value $f(x)$ back, we <span style="color:red">need to look at the entire frequency domain representation</span>.
  - It is **not** sufficient to know just some of the frequencies <span style="color:red">to get $f$ at a single position</span>, but you need **all** of the frequencies (at least if you want an exact representation).
- **problem**:
  - this frequency domain representation gives us
    - a new <span style="color:red">way of thinking about what linear filters do</span>
    - and also a way to <span style="color:red">construct linear filters</span>

## 3.31 Linear Filters (Frequency Domain Representation)

- **Achtung**: Warum können wir hier das Conv Theorem auf $L^2$ functions anwenden?
  - in <span style="color:green">3.29</span> haben wir gesagt, dass das Conv Theorem für $\mathcal{F}_2$ gilt, d.h. für $f,g\in L^2$ (statt für $f,g\in L^1$ wie im ursprünglichen Conv Theorem <span style="color:green">3.7</span>), sodass wir hier das Conv Theorem auf $L^2$ functions anwenden können

<p style="border-width:3px; border-style:solid; border-color:#FF0000; padding: 1em;">
- image $f\in L^2$, convolution kernel $\psi\in L^2$, then $$\label{eq:fdomainreplinfilter}\mathcal{F}(\psi\ast f) = (2\pi)^{\frac{d}{2}}\mathcal{F}(\psi)\mathcal{F}(f)$$
</p>

- **problem**: <span style="color:red">the conv of two $L^2$ functions is bounded, but not necessarily in $L^2$</span>, but if you think about a kernel with compact support, it is all fine.
  - (Just a sidenote so that nobody complains that I am applying the conv theorem to sth that is not necessarily a $L^2$ or $L^1$ function. But in a certain sense this equation holds even in that case.)
- **idea**:
  - now we see <span style="color:red">how this convolution affects the frequencies in $f$</span>:
    - What is actually happening here is that we take the frequencies of $f$ and scale them with these weights $F(\psi)$
    - thus, the <span style="color:red">$F(\psi)$ shows how frequencies are either suppressed or pronounced</span>
    - $F(\psi)$ "scales" $F(f)$ at every point $\omega$ in the frequency domain
  - this is the <span style="color:red">**pointwise**</span> product of the FTs (see conv theorem <span style="color:green">3.7</span>)
    - "<span style="color:red">**pointwise**</span>": dh. an jedem Punkt $\omega$ (im Frequenzraum) wird die **Amplitude** $(\mathcal{F}f)(\omega)$ der komplexen Zahl $(\mathcal{F}f)(\omega)e^{ix\cdot\omega}$ im Integranden von $f(x) = \frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}(\mathcal{F}f)(\omega)e^{ix\cdot\omega}d\omega$ <span style="color:green">(3.30)</span>, ie. the **weight** of each complex exponential $e^{ix\cdot\omega}$ (as discussed in <span style="color:green">(3.30)</span>), mit $F(\psi)(\omega)$ multipliziert
      - bec ($\ref{eq:fdomainreplinfilter}$) says that $\mathcal{F}(\psi\ast f)(\omega) = (2\pi)^{\frac{d}{2}}(\mathcal{F}\psi)(\omega)(\mathcal{F}f)(\omega)$ holds <span style="color:red">separately</span> for each individual $\omega$
- because of this interpretation $\mathcal{F}\psi$ is called <span style="color:red">**transfer function**</span>
  - bec $\mathcal{F}\psi$ "transfers" (=changes) the frequencies of $f$ in a multiplicative fashion.
- and now that we have this interpretation we can think about filters that do specific things with certain frequencies (ie. manipulating certain frequencies). Eg. we can look at a filter where the transfer function $\mathcal{F}\psi$ is <span style="color:red">$0$ for high frequencies</span>:
  - a convolution kernel $\psi$ with $\mathcal{F}\psi(\omega)\approx 0$ for high $\omega$ is called <span style="color:red">**low-pass filter**</span>
    - name because "**low frequencies pass**" and high frequencies are removed
      - because from ($\ref{eq:fdomainreplinfilter}$) we see that, if $(\mathcal{F}\psi)(\omega) = 0$ for high $\omega$, then on the rhs of ($\ref{eq:fdomainreplinfilter}$) these $0$s (the <span style="color:red">"$(\mathcal{F}\psi)(\omega)$"s</span> where $(\mathcal{F}\psi)(\omega) = 0$) are multiplied with the corresponding frequencies (the <span style="color:red">"$(\mathcal{F}f)(\omega)$"s</span> where $(\mathcal{F}\psi)(\omega) = 0$) and essentially they (the product <span style="color:red">"$\mathcal{F}(\psi) * \mathcal{F}(f)$"</span> at point $\omega$) are then $0$, so they (the function values "$(\mathcal{F}(\psi \ast f))(\omega)$" at these high frequencies $\omega$) are suppressed and only those (function values at frequencies $\omega$) where this absolute value of $\omega$ is small will be there after applying the filter.
- slide demo: **low-pass filter**:
    - lower right:
      - this is <span style="color:red">the transfer function $\mathcal{F}\psi$</span>
      - the center of the square corresponds to $\omega=0$ and the further you go to the square boundaries the higher the frequencies $\omega$
      - we can see that the $\omega$ where the absolute value is small, so those close to $\omega=0$, have weight $1$ and everything else has weight $0$
    - top right:
      - so, in <span style="color:red">$(\psi \ast f)$</span> you can see that a lot of details here are gone in the transform, so this means the <span style="color:red">**low frequencies encode these larger smooth structures**</span>, whereas the <span style="color:red">**high frequencies encode noise and finer structures like edges**</span>
      - while these large structures are preserved there seems to be some **oscillations near to these sharp structures** - this has to do with the fact that $\psi$ (lower left) is, unlike a Gaussian, an "oscillating" filter
    - lower left:
      - here you can also see what <span style="color:red">the corresponding filter $\psi$</span> looks like that leads to such a low-pass filter
      - looks like a Gaussian, but there are some rings around the central white blob - it is **like these cardinal sine functions**, so that you have the strongest contribution in $0$, but there are still oscillations if you go further away from $0$. That is also why you can see some **"ringing artefacts"** in the filtered image (top right)
- slide demo: **high-pass filter (variant 1)**:
  - lower right:
    - <span style="color:red">transfer function</span>: frequencies near 0 are 0, but only higher frequencies are passing, but the extremely high frequencies are still dropped
  - top right:
    - in <span style="color:red">the filtered image</span> we see the edges in combination with some "ringing"
    - grey here means 0, so this has positive and neg values actually
    - and you see these edges by "+"es and "-"es
  - lower left:
    - this <span style="color:red">$\psi$</span> has a much higher weight at the outer circle than the $\psi$ of the low-pass filter
    - again, **like these cardinal sine functions**, <span style="color:red">but</span>
      - there is still some central white blob like for the low-pass filter, but there are prominent rings around it which are much more pronounced than for the $\psi$ of the low-pass filter (ie the "relative" weighting of the inner circle white blob to the outer rings is different) &rarr; see drawing below

<p align="center">
  <img src="https://i.ibb.co/vzTgXQp/lowpasshighpass.png" alt="lowpasshighpass" border="0">
</p>

- slide demo: **high-pass filter (variant 2)**:
  - discussed later in <span style="color:green">(3.32)</span>
  - <span style="color:red">transfer function</span>: frequencies near 0 are 0, but higher frequencies **AND** extremely high frequencies are passing (this transfer function is the **"opposite" of the low-pass filter transfer function**: black in the middle and white everywhere else)
- so, as we have seen these <span style="color:red">**low-pass filters reduce noise**</span> since noise consists mostly of high frequency components, but **edges are smeared out** which we could also see here - but the latter should not suprise us **because it is sill a linear filter** and we know from section 2 that edges cannot be preserved.
- **slide demo summary**:
  - **low-pass filter** removes the details and is somewhat denoising
  - **high-pass filter** extracts fine features like edges
  - in both cases there are some **"ringing" artefacts**, eg. in the high-passed image (variant 1) we do not only see the edges once, but the edges seem to have some "rings" around them <span style="color:red">because</span> of this frequency representation, where <span style="color:red">everything is global</span>, ie. changing one frequency changes the image globally
  - and a similar thing we did for the low-passed image, where we saw that the <span style="color:red">**resolution has an influence**</span> on what we perceive as high and low frequencies

<p style="border-width:3px; border-style:solid; border-color:#FF0000; padding: 1em;">
- image $f\in L^2$, correlation kernel $\psi\in L^2$, then $$\mathcal{F}(\psi\star f) = (2\pi)^{\frac{d}{2}}\overline{\mathcal{F}(\psi)}\mathcal{F}(f)$$
</p>

- ie. the same holds for the correlation: the FT of a correlation is also converted to a pointwise multiplication, the only difference being that the transfer function of the filter $\mathcal{F}\psi$ is conjugated.
- but we have the same interpretation that the frequencies of $f$ are changed by the transfer function $\mathcal{F}\psi$
- **problem**:
  - lets now take a look at specific filters given by the transfer function

## 3.32 Perfect Low-pass Filter, Decomposition of High and Low Frequencies

- (i) **Perfect Low-pass Filter**: a filter $\psi$ with $\mathcal{F}\psi = \frac{1}{(2\pi)^{\frac{d}{2}}}\chi_{B_r(0)}$
  - the scaling factor $\frac{1}{(2\pi)^{\frac{d}{2}}}$ is just for convenience, otherwise we would often have annoying $(2\pi)^{\frac{d}{2}}$ factors, eg. in ($\ref{eq:perfectlowpass}$)
  - name because (Achtung: correlation) $$\label{eq:perfectlowpass}\mathcal{F}(\psi\star f) = (2\pi)^{\frac{d}{2}}\overline{\mathcal{F}(\psi)}\mathcal{F}(f) = \chi_{B_r(0)}\mathcal{F}(f)$$
  - **in words**: "frequencies with abs values smaller or equal to $r$ are <span style="color:red">completely</span> preserved and all other freq are multiplied with $0$, ie. they are completely dropped"
  - $\psi$ is similar to what we described as <span style="color:red">"low-pass filter"</span>, but the specific one here we will call <span style="color:red">"perfect low-pass filter"</span> bec it is <span style="color:red">strictly $0$</span> outside the center, whereas a general "low-pass filter" only needs to be <span style="color:red">approximately $0$</span> outside the center.
    - So, low-pass filters, in principle, **can** change the lower frequencies, but this "perfect low-pass filter" does **not**.
- (ii) with this **perfect low-pass filter** $f$ can be decomposed into a high and low frequency component $$f_{low}=(\psi\star f),\,f_{high}=f-f_{low}$$
- (iii) **Perfect High-pass Filter**: the $f_{high}$ in (ii) corresponds to a perfectly high-pass filtered image
  - Achtung: $f_{high}$ ist **nicht** der "Perfect High-pass Filter", sondern das high-pass filtered Bild!
  - name because $$\mathcal{F}f_{high}=\mathcal{F}f-\mathcal{F}f_{low}=\mathcal{F}f-\chi_{B_r(0)}\mathcal{F}f=\chi_{\mathbb{R}^d\backslash B_r(0)}\mathcal{F}f$$
- slide demo: **high-pass filter (variant 2)**:
  - 3 astronaut images side-by-side:
    - **middle**: the low-pass filter is the same as the one I showed before
    - **right**: the high frequency component which now has to contain **all** the higher freqs
      - but it looks a little bit different from the **high-pass filtered image (variant 1)**, where we additionally dropped the higher freqs, so that it does not contain **all** the fine details, but only **some** fine details - so, in the <span style="color:red">perfect high-pass filtered image</span> the <span style="color:red">edges are sharper</span>
  - **peppers image**: here, again, you see that <span style="color:red">the resolution of the image has some influence</span> on what the high and the low freqs are. (note: I used the same relative radius of this low-pass filter)
- **problem**: one other app of the FT is sth I alluded to when we introduced the conv theorem

## 3.33 Deconvolution/Deblurring with the Convolution Theorem

<p style="border-width:3px; border-style:solid; border-color:#FF0000; padding: 1em;">
&#9679; Consider $f=\psi\ast f_0$, where $f$ is the blurred image and $f_0$ is the unblurred image. <span style="color:red">How to undo</span> this convolution with $\psi$?<br>
&#9679; if $\mathcal{F}\psi(\omega)\neq 0$ for all $\omega$, then $$\label{eq:deconvconvtheorem}\mathcal{F}^{-1}\left(\boxed{\frac{\mathcal{F}f}{(2\pi)^{\frac{d}{2}}\mathcal{F}\psi}}\right)=\mathcal{F}^{-1}\left(\frac{(2\pi)^{\frac{d}{2}}\mathcal{F}\psi\mathcal{F}f_0}{(2\pi)^{\frac{d}{2}}\mathcal{F}\psi}\right)=f_0$$<br>
&#9679; thus, if $f$ and $\psi$ are known <b>exactly</b> and $\mathcal{F}\psi(\omega)\neq 0$, then $f_0$ can be reconstructed <b>exactly</b>
</p>

- **3 sources of errors** (discussed below):
  - quantization
  - regularization
  - $\psi$ not known
- **problem**:
  - in practice, $f$ is **not** known **exactly**, since $f$ is **quantized** &rarr; may lead to <span style="color:red">reconstruction artifacts</span>
    - even 8-bit quantization has a noticable effect!
  - $\mathcal{F}\psi(\omega)\neq 0$ is not always the case. What do we do if this is not the case? Approximate the boxed expression in ($\ref{eq:deconvconvtheorem}$) by <span style="color:red">**regularization**</span>, ie. $\frac{a+bi}{c+di}\approx\frac{(ac+bd)+(bc-ad)i}{c^2+d^2+\epsilon}$ with the **regularization parameter** $\epsilon > 0$
    - this, however, means that <span style="color:red">filters with very small $\mathcal{F}\psi$</span> lead to high approximation errors because the boxed expression in ($\ref{eq:deconvconvtheorem}$) is poorly approximated then (because the approximation error introduced by the $\epsilon$ in the denominator changes the result of the division a lot when $(2\pi)^{\frac{d}{2}}\mathcal{F}\psi\approx\epsilon$)
- condition for the "undoable" filter: 
  - we do have the strong assumption: "the transfer function never vanishes, ie. it must not drop any freq"
- "undoable" filters: examples:
  - the simplest linear filter: <span style="color:red">**the constant filter**</span> (a one-by-one filter) that multiplies with a constant. Of course, you can undo a multiplication by a constant.
  - there are more filters that you can undo, but, <span style="color:red">**in general, you cannot undo**</span>. But for certain filters you can (eg. the <span style="color:red">**Gaussian blur**</span>).
  - <span style="color:red">**motion blur**</span> cannot be undone exactly, but reasonably well
- slide demo:
  - **slide 1**:
    - **left**: "exact $f$" (the Gaussian blur applied on the test image, where the result of the conv is stored with full double-precision (ie. 64 bits, whereas single-precision is 32 bits))
    - **right**: deconvolution with large $\epsilon = 10^{-1}$, if we <span style="color:red">decrease this $\epsilon$ the result is getting sharper</span> and here in this case this is almost a perfect reconstruction (for sufficiently small $\epsilon$, eg. $\epsilon = 10^{-9}$) (you can see on the right there are much more details visible after the deconv)
      - but here the key is that <span style="color:red">we know what the kernel is</span> and also that <span style="color:red">we have the exact values of $f$</span> &rarr; in practice, this would <span style="color:red">not</span> be <span style="color:red">very realistic</span> bec when you give an image usually it is "quantized". Your <span style="color:red">camera stores 8 bit per channel</span>.
  - **slide 2**:
    - **left**: quantized $f$ (stored with 8 bit precision)
      - but <span style="color:red">no visible difference on screen compared to</span> the "double-precision (exact) $f$" on <span style="color:red">slide 1</span> bec it is a 8 bit per channel projector (der wird 8 bit darstellen, egal ob wir 8-bit oder double-precision benutzen). Maybe you could see sth in HDR if you look closely.
    - **right**: this still works pretty well for reasonable $\epsilon$, but <span style="color:red">if $\epsilon$ gets too small (ca. $\epsilon < 10^{-6}$), then this breaks down</span> (the regularization here also has to compensate for the fact that we do not know this $f$ exactly)
  - **QnA**: <span style="color:red">"throwing away" vs. "redistributing"</span> information
    - if you have a high-pass filter that has really <span style="color:red">"thrown away"</span> your freqs, then they are gone and you cannot reconstruct sth that was completely dropped.
    - The Gaussian itself just <span style="color:red">redistributes</span> the information. So, it is looking blurry, but we know that at every position we have a certain combination of the values that we are looking for. That is why you can invert it: this is like when you apply a dense matrix to a vector, then all the values are summed up, but you can invert this matrix multiplication and it is similar here. But, of course, if the matrix would just drop half of the entries of the vector, then they are gone. Same with the low-pass filter.
- note: in practice, we do not know $\psi$, either, but here we assume that we know it (otherwise this problem would be underdetermined)
- **problem**:
  - we cannot apply $\mathcal{F}_2$ to a constant function &rarr; need to extend the FT
  - 2 remarks on how we can extend the FT

## 3.34 $\mathcal{F}$ of the Constant Function

- if we apply the FT on a constant function:
  - for $c\in\mathbb{R}^d\backslash 0$, $(\mathcal{F}c)(\omega)=\frac{c}{(2\pi)^{\frac{d}{2}}}\int_{\mathbb{R}^d}e^{-ix\cdot\omega}dx$
    - this integral does not exist (exercise)
      - a constant function on $\mathbb{R}^d$ is not in any $L^p$ space except for $L^\infty$
      - $\int_a^b e^{-ix\cdot\omega}dx$ diverges for $a\to -\infty$ and $b\to +\infty$
      - $\int_{-R}^{R} e^{-ix\cdot\omega}dx$ diverges for $R\to \infty$, so $\phi_R$ from <span style="color:green">(3.28)</span> diverges
- $\mathcal{F}c$ can be defined **implicitly** using the inversion theorem <span style="color:green">(3.21)</span>
  - $d$ arbitrary, but fixed, then $$\label{eq:FTconstfunction}\frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}(\mathcal{F}c)(\omega)e^{ix\cdot\omega}d\omega = c$$
- ($\ref{eq:FTconstfunction}$) says that, if we apply the inversion formula, then we must get back the constant function.
  - Thus, if we have a reasonable way of defining $\mathcal{F}c$, when we transform it back, we need to get back the constant function that we started with.
- **problem**:
  - consider the integrand on the lhs: it is actually <span style="color:red">very tough to find a function $\mathcal{F}c$ that would fulfill the property</span> that when you integrate it with this complex exponential $e^{ixw}$ that you just get back this constant $c$.
  - Actually, <span style="color:red">it is **not** sufficient to think of $\mathcal{F}c$ as a **function**</span>.
  - instead, $\mathcal{F}c$ must be thought of as a <span style="color:red">**measure**</span>.
- let $\mathcal{F}c(\omega)=(2\pi)^{\frac{d}{2}}c\delta_0$, where $\delta_0$ is the Dirac <span style="color:red">measure</span> at $0$, then ($\ref{eq:FTconstfunction}$) is fulfilled: $$\frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}(\mathcal{F}c)(\omega)e^{ix\cdot\omega}d\omega = \frac{1}{(2\pi)^{\frac{d}{2}}} \int_{\mathbb{R}^d}e^{ix\cdot\omega}d(\mathcal{F}c)(\omega) = \frac{1}{(2\pi)^{\frac{d}{2}}} e^{ix\cdot 0} \cdot (2\pi)^{\frac{d}{2}}c = c$$
- in this sense, the <span style="color:red">FT of the constant function</span> is the <span style="color:red">(scaled) Dirac measure</span>
  - makes sense bec, if you think about freqs, the constant function should have $0$ freq bec there is "nothing in there that changes".
  - notice:
    - **constant function**: maximally smooth, maximal support
    - **Dirac measure**: maximally concentrated
- **problem**:
  - now for the other direction: now we have sth that is transformed to a measure. But we can also extend the FT to measures.

## 3.35 $\mathcal{F}$ of the $\delta_x$ distribution, $\mathcal{F}$ of Tempered Distributions

- the FT can be extended to more general spaces than $L^2$, eg. to the <span style="color:red">space of tempered distributions</span>
  - Wikip. quotes:
    - "The <span style="color:red">delta function is a tempered distribution</span>, and therefore it has a well-defined Fourier transform."
    - "the space of tempered distributions. It is the continuous <span style="color:red">**dual space of the Schwartz space**</span>"
    - "<span style="color:red">all tempered distributions have a Fourier transform</span>, which is not true for an arbitrary distribution"
    - "The derivative of a tempered distribution is again a tempered distribution"
    - "The tempered distributions can also be characterized as *slowly growing*, meaning that each derivative of \[the tempered distribution\] $T$ grows at most as fast as some polynomial."
    - "Tempered distributions generalize the bounded (or slow-growing) locally integrable functions"
- we can define $\mathcal{F}\delta_x$ by its evaluation at $f\in S(\mathbb{R}^d,\mathbb{C})$ $$\boxed{\mathcal{F}\delta_x(f) := \delta_x(\mathcal{F}f)}$$
  - then, $\delta_x(\mathcal{F}f) = \int_{\mathbb{R}^d} (\mathcal{F}f)d\delta_x = (\mathcal{F}f)(x) = \frac{1}{(2\pi)^{\frac{d}{2}}}\int_{\mathbb{R}^d} f(y)e^{-iyx}dy$, where we used the definition of the **distribution induced by the measure $\delta_x$** <span style="color:green">(1.9 (v))</span> in the 1st and 2nd step and the definition of $\mathcal{F}$ in the last step
  - this means, since the distribution $\mathcal{F}\delta_x$ is described as the integral $\frac{1}{(2\pi)^{\frac{d}{2}}}\int_{\mathbb{R}^d} f(y)e^{-iyx}dy$, this distribution $\mathcal{F}\delta_x$ corresponds to the integral density $\frac{1}{(2\pi)^\frac{d}{2}}e^{-iyx}$. Such kinds of distributions are called <span style="color:red">"**regular distributions**" (= "it can be expressed as an integral with a density")</span>.
- **generalization of $\mathcal{F}_2$ to tempered distributions**:
  - in the same way we can define $$\boxed{\mathcal{F}g(f) := g(\mathcal{F}f)}$$ for any <span style="color:red">**tempered distribution $g$**</span>
  - we can view any <span style="color:red">function</span> $g\in L^2$ <span style="color:red">as a tempered distribution</span> by defining $g(f) := \int_{\mathbb{R}^d} gfdx$

<p align="center">
  <img src="https://i.ibb.co/hsDxCS2/Screenshot-from-2024-03-03-11-39-53.png" alt="Screenshot-from-2024-03-03-11-39-53" border="0">
</p>

- thus, for $g\in L^2$ this view coincides with our usual definition of $\mathcal{F}_2$ ($\mathcal{F}$ on $L^2$)
  - thus, this view is just a generalization of $\mathcal{F}_2$
- **problem**:
  - now there are still two things missing:
    1. this FT is <span style="color:red">not discrete</span>
    2. the FT has the (from an application perspective) <span style="color:red">unrealistic assumption</span> that we have <span style="color:red">functions on all of $\mathbb{R}^d$</span>, but real images do not live on all of $\mathbb{R}^d$, they live only on the image domain $\Omega$, ie. on some rectangle or on the unit square.
      - **Objection to point 2.**: But we said you can extend the image with $0$s on all of $\mathbb{R}^d$.
        - Berkels: Of course, you can extend it with $0$s, but then you have a FT that works on a much larger class of functions. So we developed this FT for $L^2$ functions from $\mathbb{R}^d$ to the complex numbers and, of course, those functions that have just support on the image domain would be $L^2$ functions so that you can apply the FT, but what you get back then is sth that has frequencies in all of $\mathbb{R}^d$. So, you start on a rectangle, but your FT lives on all of $\mathbb{R}^d$. So, you would greatly increase the size of your function. This is nothing that you want to do, if you do not have to!

# 3.2 Orthogonal Expansions

- **problem**:
  - So, the 1st step before we actually do the discretization is to replace this $\mathbb{R}^d$ in $f \in S(\mathbb{R}^d,\mathbb{C})$ with a square or a rectangle.
  - And this leads to Fourier series.
  - It will be consistent with the FT, but there it turns out that to represent functions in 1D / to represent functions on an interval you do not need **all** frequencies in $\mathbb{R}$, but you just need countably many.
  - This gives you a different or a series representation that is easier to handle.
  - QnA: Does conv theorem hold for the FS ?
    - Berkels: "That is a good question ..."
  - But before we can do the FS, we need to collect a little bit of background tools that will help us to understand what the FS is doing
  - I try to briefly sketch, where this will be heading:
    - instead of integral $\frac{1}{(2\pi)^{\frac{d}{2}}}\int_{\mathbb{R}^d} f(y)e^{-iyx}dy$ we will have an **infinite sum**, so it is a **series**
    - the **summands** will be certain **functions** $e^{-iyx}$ that are multiplied with certain **weights** $f(y)$
      - in this sense, these **complex exponentials** $e^{-iyx}$ have to be sth like a **basis**, but a **basis with infinitely many elements**.
    - to be able to understand this properly, we need to be able to handle these kind of basises, with countably many elements, and understand what it means to have this <span style="color:red">**infinite sum of functions**</span> and <span style="color:red">in which sense it converges</span> to sth or <span style="color:red">how it encodes some function</span>
    - and this is exactly what the next very brief section will do
  - but, again, we do this <span style="color:red">in an abstract setting</span>, so we can concentrate on the structures that we actually need

## 3.36 ONS

- let $X$ be a pre-Hilbert space
- (i) a sequence $(e_n)\_n$ is called **ONS**, if $(e_i,e_j)=\delta_{ij}$ for all $i,j$
  - <span style="color:red">think of eg. $\mathbb{R}^n$</span> with the canonical basis vectors and the Euclidean scalar product
  - but we will be going in the direction where <span style="color:red">these $e_n$ have to do with the complex exponentials $e^{-iyx}$</span> (but we will not consider them on all of $\mathbb{R}^d$ like we did above), I will have an example with the sine soon
- (ii) **k-th Fourier coefficient**: $x_i := (x,e_k)_X$
  - again, <span style="color:red">think of the $\mathbb{R}^n$ example</span>:
    - in $\mathbb{R}^n$ this $x$ would be a vector and if we take the canonical basis as ONS then these scalar products $(x,e_k)_X$ would just give us the <span style="color:red">**entries**</span> of the vector
    - but you can do the very same thing <span style="color:red">if this $x$ is a function</span> and it gives you sth equivalent to these coefficients that you know from $\mathbb{R}^n$
- **problem**:
  - **Example of an ONS**: to see that there are also rather simple ONSs in the infinite dimensional case, so actually $X$ is a function space, so we look at $L^2$, but this time not on all of $\mathbb{R}$, but
      - just <span style="color:red">on a fixed interval</span> $(0,\pi)$
      - and we look at the <span style="color:red">functions that are just $\sin(nx)$</span>

## 3.37 $f_n(x)=sin(nx)$ on $L^2((0,\pi))^{\mathbb{N}}$ are an ONS

- **conditions**:
  - $(f_n)_n\in L^2((0,\pi))^{\mathbb{N}}$
  - $f_n(x) = \sin{(nx)}$
    - this is essentially the <span style="color:red">imaginary component of $e^{-iyx}$</span> if we take $x=n$ (except there is a minus missing, but it does not matter)
      - ie. the $\sin{(nx)}$ are very closely related to the integrands that we have in our FT
- then $$(f_i,f_j)_{L^2} = \frac{\pi}{2}\delta_{ij}$$
  - in words: thus, after rescaling by $\frac{\pi}{2}$ these functions <span style="color:red">$f_n$ are an ONS</span>.
- **proof**:
  - to see this you have to compute the integral
- **problem**:
  - Is such an ONS <span style="color:red">complete</span>?
    - bec for sth to be a basis we need to be able to represent every element in the space with this and here so far we only know that they are orthogonal to each other, but we do not know <span style="color:red">if they span the whole space</span>
      - and, once we know that they can span the whole space, we will get this Fourier series
  - <span style="color:green">next lec</span>: although we now know that these $\sin(nx)$ are orthogonal to each other, we do NOT know if they are sufficient to represent EVERYTHING as a sum or a series of these $\sin(nx)$. (we dont know if $\sin(nx)$ is complete in $L^2$)
    - In the following we want to study what does it mean for such an ONS to be sth like a <span style="color:red">basis of $L^2$</span>
    - And the 1st tool on the way to understand this is the following proposition.

## 3.38 Best approximation by Fourier Coefficients $x_k$

- **conditions**:
  - let $X$ be a pre-Hilbert space
  - $(e_n)\_{n=1}^K$ an ONS
    - $K\in\mathbb{N}$ is finite
- (i) $\lVert x - \sum_{k=1}^{K} x_k e_k\rVert^2 = \lVert x\rVert^2 - \sum_{k=1}^{K} \lvert x_k\rvert^2$
  - ie. $\sum_{k=1}^{K}x_ke_k$ approximates $x$
  - this has some <span style="color:red">important consequences</span>: the $\sum_{k=1}^{K} \lvert x_k\rvert^2$ must be smaller than $\lVert x\rVert^2$ bec the lhs norm is always positive (but we will come to that later <span style="color:green">(3.40)</span>)
- (ii) $\lVert x - \sum_{k=1}^{K} c_k e_k\rVert^2 = \lVert x\rVert^2 - \sum_{k=1}^{K} \lvert x_k\rvert^2 + \sum_{k=1}^{K}\lVert c_k + x_k\rVert^2$
  - $c_k\in\mathbb{K}$ are any other coefficients than the Fourier coefficients
  - we can immediately see that the rhs is smallest if we choose $c_k = x_k$, ie the "error" (the difference of the linear combination and the element $x$) is minimal when we use exactly the Fcoeffs $x_k$.
    - in this sense, <span style="color:red">the Fcoeffs give us the "best approximation" of any element $x\in X$</span>

## 3.39 Interpretation of 3.38

- <span style="color:green">(3.38(ii))</span> quantifies how well an $x\in X$ is approximated by a finite linear combination of an ONS.
- the error is minimal for $c_k = x_k$
- **problem**:
  - what changes if we have <span style="color:red">infinitely</span> many elements (or rather what we can say <span style="color:red">if the ONS has infinitely many elements</span>)

## 3.40 Bessel's Inequality

- let $X$ be a pre-Hilbert space
- (i.1) $\sum_{k=1}^{K}\lvert x_k\rvert^2 \leq \lVert x\rVert^2$
  - $K\in\mathbb{N}$ is <span style="color:red">finite</span>
  - <span style="color:green">(i.1)</span> is one interesting way to describe <span style="color:red">whether we can exactly represent any element as a linear combination of the ONS</span>
    - we <span style="color:red">just have to check if the sum of the squared Fcoeffs is the same as the norm</span>. This is an easier to check condition than <span style="color:green">(i.2)</span>.
  - **proof**:
    - not surprising, bec of <span style="color:green">(3.38(i))</span>
- The more interesting part that we do **not** know yet is:
- (i.2) equality in (i.1) holds iff $\lVert x - \sum_{k=1}^{K} x_k e_k\rVert^2 = 0$
  - <span style="color:green">(i.2)</span> means that equality in <span style="color:green">(i.1)</span> holds <span style="color:red">iff $x$ is exactly represented by this linear combination</span>.
  - **proof**:
    - this also follows from <span style="color:green">(3.38 (i))</span> bec we have an equality, so the only way that the rhs can be $0$ is when the lhs is $0$.
      - so, for this part we do not have to prove anything except for looking at <span style="color:green">(3.38(i))</span>
- (ii.1) $\sum_{k=1}^{\infty}\lvert x_k\rvert^2 \leq \lVert x\rVert^2$
  - Thus, this inequality in <span style="color:green">(i.1)</span> is <span style="color:red">still true</span>. So, even if you have infinitely many elements here, the <span style="color:red">sum of the squared Fcoeffs can never exceed the norm there (on the rhs)</span> bec this shows that <span style="color:red">this series here (on the lhs) converges (bec it has a finite value)</span>.
- here we have to be a little more careful with the formulation:
- (ii.2) equality in (ii.1) holds iff $\lim_{K\to\infty}\lVert x - \sum_{k=1}^{K} x_k e_k\rVert^2 = 0$
  - in words: the linear combination converges to $x$
  - since there are infinitely many entries in the ONS we cannot just look at the infinite sum, but we can look at the convergence of functions.
  - This is what is important for a <span style="color:red">basis</span>. Bec this way you can at least <span style="color:red">approximate any $x$ as a finite linear combination of the ONS</span>. This will be the necessary extension for when we call an ONS <span style="color:red">complete</span> (later).

## 3.41 Interpretation of 3.40

- <span style="color:green">(3.40)</span> means that
  - if $\sum_{k=1}^{\infty}\lvert x_k\rvert^2 = \lVert x\rVert^2$ then the sequence $(\sum_{k=1}^{K}x_ke_k)_K$ converges to $x$
- that means we can
  - <span style="color:red">represent $x$</span> as an <span style="color:red">**infinite linear combination**</span>
  - or at least <span style="color:red">approximate $x$</span> arbitrarily well with a <span style="color:red">**finite linear combination**</span>.
- **problem**:
  - In this sense we are motivated to define what an <span style="color:red">**ONB**</span> is:
    - if our ONS is s.t. <span style="color:green">(3.40 (i.2))</span> (**or** <span style="color:green">(3.40 (ii.2))</span>, if the ONS has **infinitely** many elements) holds <span style="color:red">FOR ALL $x$</span>, then we call it a <span style="color:red">**basis**</span>:

## 3.42 CONS aka ONB

- **condition**:
  - let $X$ be a pre-Hilbert space
  - $N\subset\mathbb{N}$ finite or infinite
    - $N$ is a <span style="color:red">set</span> and not a number! It contains <span style="color:red">all possible index values</span>, ie. it is an **index set**. For a finite basis it looks like $N=\\{1,\ldots,M\\}$, where $M$ is the total number of basis elements.
  - $(e_n)\_n\in X^N$ an ONS
- the ONS is called <span style="color:red">**complete**</span> or <span style="color:red">**CONS**</span> of $X$ or <span style="color:red">**ONB**</span>, if $$\sum_{k\in N}\lvert x_k\rvert^2 = \lVert x\rVert^2\quad\forall\,x$$
- **problem**:
  - To shorten the notation:

## 3.43 Implication of 3.40 and 3.42

- <span style="color:green">(3.40) and (3.42)</span> together imply:

<p style="border-width:3px; border-style:solid; border-color:#FF0000; padding: 1em;">
- if $(e_n)_n$ is a CONS, then <span style="color:red">$\sum_{k=1}^{K}x_ke_k$ converges to $x$</span> for $K\to\infty$
</p>

- you should really <span style="color:red">think of this as a limit</span> of these sums and **not** as actually summing up infinitely many elements.
- **problem**:
  - The following corollary illustrates this from a different angle:

## 3.44 Necessary and Sufficient Condition for an ONS to be a CONS

<p style="border-width:3px; border-style:solid; border-color:#FF0000; padding: 1em;">
- <b>condition</b>:<br>
&emsp;  - same as in <span style="color:green">(3.42)</span><br>
- the ONS is called <span style="color:red"><b>complete</b></span> or <span style="color:red"><b>CONS</b></span> of $X$ or <span style="color:red"><b>ONB</b></span>, if $$\forall_{x}\forall_{\epsilon}\exists_{K\in\mathbb{N}}\exists_{i_1,\ldots,i_K\in N}\exists_{c_{i_1},\ldots,c_{i_K}\in\mathbb{K}}\bigg\lVert x - \sum_{k=1}^{K}c_{i_k}e_{i_k}\bigg\rVert<\epsilon$$
</p>

- <span style="color:red">$K$</span> is the <span style="color:red">number of basis elements $e_k$ we need to approximate $x$</span> and this number does not have to be equal to the <span style="color:red">total number of basis elements in the CONS $\lvert N\rvert$</span> (note: $N$ is a <span style="color:red">set</span>, $\lvert N\rvert$ denotes the number of elements in the set $N$)
  - thus, the <span style="color:red">$i_1,\ldots,i_K\in N$</span> select <span style="color:red">some</span> of the basis elements $e_k$, but not necessarily <span style="color:red">all</span> of them
- **memorize**: the ONS is **complete**, iff $$\overline{\text{span}\{e_k:k\in N\}} = X$$
  - Why the <span style="color:red">**closure**</span>?
    - the $\text{span}$ are the finite linear combinations of the $e_k$
    - so the span itself is only **finite** linear combinations, but with a finite linear combination we only get arbitrarily close, <span style="color:red">but we cannot get it exactly</span>. That is why we still need to put the <span style="color:red">**closure**</span> to have <span style="color:red">limits of the CONSes</span>.
    - and in this sense, this is a basis. So, this now nicely justifies the term "basis".
- this means that <span style="color:red">for any element $x$</span> in the space $X$, <span style="color:red">any accuracy $\epsilon$</span> is sufficient, if I take a <span style="color:red">**finite**</span> linear combination of the orthonormal elements.
  - in other words, the 1st capital $K$ we can leave out. It is just that we need <span style="color:red">**finitely**</span> many to approximate this $x$ arbitrarily well.
- **proof**:
  - with <span style="color:green">(3.42), (3.40)</span> and <span style="color:green">(3.38 (ii))</span>

## 3.45 Parseval's Identity

<p style="border-width:3px; border-style:solid; border-color:#FF0000; padding: 1em;">
- <b>conditions</b>:<br>
&emsp;  - let $X$ be a pre-Hilbert space<br>
&emsp;  - <span style="color:red">$N=\mathbb{N}$ infinite</span><br>
&emsp;&emsp;  - here I only explicitly consider the case that the ONS has infinitely many elements bec it is obvious that this holds for finitely many elements<br>
&emsp;  - $(e_n)_n\in X^\mathbb{N}$ <span style="color:red">a CONS</span><br>
- then $$(x,y)_X = \sum_{k=1}^{\infty}(x,e_k)_X\overline{(y,e_k)_X}\quad\forall x,y$$
</p>
- ie. $(x,y)_X$ is a "series of the product of the Fcoeffs"
- you can think of this as a <span style="color:red">generalization</span> that <span style="color:red">we can represent the norm in the CONS as the sum of the squared Fcoeffs</span>, see <span style="color:green">(3.42)</span>.
- ie. <span style="color:red">we can represent the scalar product as the infinite sum of the product of the Fcoeffs</span>
- **or**: you can think of this as the $C^\infty$ scalar product of the Fcoeffs of $X$ with the Fcoeffs of $Y$

# 3.3 Fourier Series

## 3.46

## 3.47

- $e_k(x)=e^{ikx}$ are an ONB of $L^2(\left[-\pi,\pi\right],\mathbb{C})$ wrt scalar product $(\cdot,\cdot)_{\left[-\pi,\pi\right)}$

## 3.48

- (ii) $e^B_{k}(x)=e^{ik\frac{\pi}{B}x}$

## 3.50 Sampling Theorem, Shannon-Whittaker

- **conditions**:
  - $f\in L^2$
  - $\hat{f}(\omega)=\mathcal{F}f(\omega)=0$ outside $\left[-B,B\right]$
    - So, we look at a function where the FT lives on a circle and everything outside is $0$ which means that there are no high frequencies and $B$ is the bound. This is eg. the case after you have applied a <span style="color:red">**low-pass filter**</span>, then you know that your signal must have bounded frequency.
- (i) $f$ is continuous
- (ii) $f$ is uniquely determined by the values $f(\frac{k\pi}{B})$
- (iii) formula with $f=\text{sinc}$
- **proof**:
  - <span style="color:red">for (i)</span>:
    - we know
      - $f\in L^2(\mathbb{R})$
      - $\hat{f}(\omega)=\mathcal{F}f(\omega)=0$ outside $\left[-B,B\right]$
      - $L^2(\left[-B,B\right],\mathbb{C})\subset L^1(\left[-B,B\right],\mathbb{C})$
        - because the only thing that can go wrong in $L^2$ is that you may have a singularity, but the square $\cdot^2$ makes it just go quicker to $\infty$, thus, <span style="color:red">if it (the integration wrt the absolute value) works with a square then it has to work with a $1$, too</span>
    - $\Rightarrow \hat{f}\in L^2(\mathbb{R},\mathbb{C})\cap L^1(\mathbb{R},\mathbb{C})$
      - $\Rightarrow$ <span style="color:green">(3.29 (i)), (3.28 (i))</span> **conditions** fulfilled, <span style="color:red">so we can write $f=\mathcal{F}^{-1}\hat{f}$</span>
        - $\Rightarrow$ <span style="color:green">(3.2 (i))</span> **conditions** fulfilled, so $f=\mathcal{F}^{-1}\hat{f}$ is continuous
  - <span style="color:red">for (ii)</span>:
    - **because of (i)** we can write $$\boxed{f\left(\frac{k\pi}{B}\right)=\sqrt{\frac{2}{\pi}}B(\hat{f},e^{B}_{-k})_{\left[-B,B\right]}}\,\text{where}\, e^B_{k}(x)\,\text{defined in (3.48).}$$
      - in words: all Fcoeffs $(\hat{f},e^{B}_{-k})$ are solely determined by $f(\frac{k\pi}{B})$
        - $\Rightarrow$ <span style="color:green">(3.48)</span> **conditions** fulfilled, so that $\hat{f}\in L^2(\left[-\pi,\pi\right],\mathbb{C})$ (Achtung: <span style="color:red">$\hat{f}$</span> und nicht $f$) can be uniquely expressed as a linear combination of the $L^2(\left[-\pi,\pi\right],\mathbb{C})$ functions $e^{B}_{-k}$ (see <span style="color:green">(3.47)</span>)
          - $\mathcal{F}_2$ bijective $\Rightarrow$ $f$ is <span style="color:red">also</span> uniquely determined by the values $f(\frac{k\pi}{B})$
  - <span style="color:red">for (iii)</span>:
    - **because of (ii)** we can write $\hat{f}=\sum\_{k\in\mathbb{Z}}(\hat{f},e^{B}\_{k})\_{\left[-B,B\right]}e^{B}\_{k}$
      - extend to all of $\mathbb{R}$ by $\hat{f}=\chi_{\left[-B,B\right]}\sum\_{k\in\mathbb{Z}}(\hat{f},e^{B}\_{k})\_{\left[-B,B\right]}e^{B}\_{k}$
        - $\Rightarrow$ by $\mathcal{F}^{-1}$ on both sides, $f=\text{"formula with sinc in (3.50)"}$
- **summarizing**:
  - so we have shown the Sampling Theorem using
    - many of the things we learned about the <span style="color:red">**FT**</span> in the past **(for (i))**
    - in combination with the new knowledge about the <span style="color:red">**FS**</span> **(for (ii) and (iii))**
      - the FS was necessary
        - to get this specific representation
        - to see that these countably many values are enough to fully characterize our $f$

# 3.4 DFT

## 3.62 Multiplication of Large Numbers

- express numbers by their decimal digits: $a=\sum_{i=0}^{m}a_i 10^i$ and $b=\sum_{j=0}^{n}a_j 10^j$
- then $$ab = \sum_{k=0}^{m+n}10^k\sum_{l=0}^{m+n}a_lb_{k-l}$$
- **merke**: both sums go from $0$ to $m+n$
- can be computed efficiently with <span style="color:red">**FFT**</span>
- **proof**:
  - bec of **discrete convolution theorem**

## 3.63 Exploiting Symmetries

**in FFTW**: `r2c` (for the real input to complex-Hermitian output DFT) and `c2r` (for the complex-Hermitian input to real output DFT)

<p align="center">
  <img src="https://i.ibb.co/5Wc5v1z/Screenshot-from-2024-02-27-02-03-11.png" alt="Screenshot-from-2024-02-27-02-03-11" border="0">
</p>

# 3.5 Trigonometric Interpolation

## 3.70 (Complex) Trigonometric Polynomials of Degree $m − 1$

- $\text{span}{\\{e_j\\}}$

## 3.71 Why are they called "(Complex) Trigonometric Polynomials"?

- **Connection to Taylor Series**:
  - <span style="color:red">**Under VERY strong assumptions**</span> the <span style="color:red">coeffs of the FS and the Taylor series</span> are the same.
  - But, <span style="color:red">**in general**</span>, FS and the Taylor series are <span style="color:red">**fundamentally different**</span> (the concepts behind them are different)!
  - <span style="color:red">**Taylor series**</span>
    - is <span style="color:red">**LOCAL**</span>, ie if I do the **Taylor expansion** of this $f$ **at any point on the unit circle**, I only need to know
      - what $f$ is <span style="color:red">at that point</span> and
      - all of its derivatives at that point and
      - I do **not** need to know what $f$ is doing outside of this point (and actually I can change $f$ outside of this point without changing what the Taylor series is)
    - but to be able to compute the Taylor series <span style="color:red">**we need $f$ to be infinitely differentiable**</span>.
    - So, <span style="color:red">**at one point we need to locally know this $f$ fully**</span>.
  - <span style="color:red">**Fourier series**</span>
    - is <span style="color:red">**GLOBAL**</span>
      - ie we must know all the values of this $f$ <span style="color:red">on the entire interval $\left[-\pi,\pi\right]$ (in 2d: on the entire circle)</span>,
      - but you do **not** need any differentiability.
      - The only thing <span style="color:red">**you need is square integrability**</span> ($L^2$-integrability). 

## 3.72 Basic Properties of the Space $T_m$

## 3.73 A property of the Complex Exponentials

## 3.74 Showing the Orthogonality of the DFT using 3.73

## 3.75 Trigonometric Interpolation Problem
