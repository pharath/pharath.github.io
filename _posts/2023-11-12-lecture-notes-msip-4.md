---
title: "Mathematical Methods of Signal and Image Processing - Chapter 4 - The Wavelet Transform"
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

# 4 The Wavelet Transform

- The FT is a <span style="color:red">**global operator**</span>. You don't always want this full globality. So, it is natural to ask for <span style="color:red">**more local transforms**</span>, ie. where one coefficient only depends on a small part of your function, not on its entirety. So that you could by manipulating the coefficients of your transform <span style="color:red">**make local changes**</span> to your function versus being forced to do global changes. Think about the <span style="color:green">**low-pass filter**</span>: this removed the high frequencies/the details, but it also introduced some "<span style="color:red">**ringing**</span>" because of these global changes. And the <span style="color:red">**wavelet transform**</span> is a <span style="color:red">**more localized transform**</span>.
- So we want something that is more localized than the FT.

# 4.1 The Continuous Wavelet Transform

## 4.1 The Continuous Wavelet Transform

- **conditions**:
  - $f,\psi\in L^2$
    - $f$: the image
    - $\psi$: encodes <span style="color:red">how</span> you want to transform
  - $a>0$ (<span style="color:red">scale parameter</span>)
  - $b\in\mathbb{R}$ (<span style="color:red">spatial parameter</span>)
- then, the <span style="color:red">**wavelet transform of $f$**</span> is $$L_{\psi}f(a,b) = \int_\mathbb{R} f(x)\boxed{\frac{1}{\sqrt{a}}\psi\left(\frac{x-b}{a}\right)}dx$$
- like a <span style="color:red">**convolution**</span>, see <span style="color:green">(4.2 (ii) (B))</span>
- we are integrating our image $f$ and we are <span style="color:red">weighting this by $\frac{1}{\sqrt{a}}\psi$</span>. And what we do with our $\psi$ is: we <span style="color:purple">shift $\psi$ to a certain **position** $b$</span>. So, we have our $\psi$ and we shift it with $b$ over the signal wherever we want to have it and we <span style="color:red">**scale** $\psi$ with $a$ and $\sqrt{a}$</span>. So this $\frac{1}{\sqrt{a}}$ and $a$ is exactly the change of variables that **"zooms in"** or **"out"** to grab some information from your signal.
  - Eg. if you would think that $\psi$ is just the <span style="color:green">**characteristic function of $\left[0,1\right]$**</span>, then it would give you a <span style="color:green">**mean value on $\left[0,1\right]$**</span>, if $a=1$ and $b=0$,
    - and if you now <span style="color:green">change $b$</span> you would take different mean values because you are shifting this $\psi$
    - and with <span style="color:green">changing this $a$</span> you would be able to do mean values or some accumulation over smaller or larger intervals depending on how you choose this $a$.
    - So this somehow samples ... (*interrupted by question*)
  - **Q&A**: in case $\psi$ does <span style="color:red">**not**</span> have <span style="color:red">**compact support**</span> then it is <span style="color:red">**still global**</span>, not local! So, finding good $\psi$s is a very important question. For now we just know (vaguely) that we have some $\psi$. We will <span style="color:red">need to find out what suitable $\psi$s are</span> and what they should fulfill.
  - But, in general, you can have this image and you go to a certain position of your signal with $b$ and you choose a certain <span style="color:red">**zoom factor** $a$</span> and then you somehow <span style="color:red">**aggregate values in there**</span>.

## 4.2 FT vs WT, Representations of $L_{\psi}f$

- (i) $L_{\psi}$ maps <span style="color:purple">$f:\mathbb{R}\to\mathbb{C}$</span> to <span style="color:purple">$L_{\psi}f:(0,\infty)\times\mathbb{R}\to\mathbb{R}$</span>
  - in words: the result of the WT is a <span style="color:red">2D function</span> (2-dimensional domain for $L_{\psi}f$)
    - recall: the result of the FT is the **1D function**, ie. $\mathcal{F}$ maps <span style="color:purple">$f:\mathbb{R}\to\mathbb{C}$</span> to <span style="color:purple">$\mathcal{F}f: \mathbb{R}\to\mathbb{C}$</span> (domain and codomain are the same for $f$ and $\mathcal{F}f$)
  - <span style="color:red">2D</span>: because you can evaluate it for different $a$s and $b$s, not only for different frequencies.
  - We increase from a <span style="color:red">1D domain</span> to a <span style="color:red">2D domain</span> which already implies that this representation <span style="color:green">(i)</span> somehow has to be <span style="color:red">**overcomplete**</span>. So, this has to have **redundant information** because it cannot be that we need a <span style="color:red">2D function</span> to describe a <span style="color:red">1D function</span>. The <span style="color:red">2D function</span> has much **more power to store information**.
- (ii) 2 representations of $L_{\psi}f$ (equivalent to <span style="color:green">(4.1)</span>):
  - (A) $L_{\psi}f$ as <span style="color:red">**scalar product**</span>: $L_{\psi}f(a,b)=\frac{1}{\sqrt{a}}(f,T_{-b}D_{\frac{1}{a}}\psi)_{L^2}$
  - (B) $L_{\psi}f$ as <span style="color:red">**convolution**</span>: $L_{\psi}f(a,b)=\frac{1}{\sqrt{a}}(f\ast D_{-\frac{1}{a}}\psi)(b)$
    - just a side remark: <span style="color:green">(B)</span> is related to <span style="color:green">(2.16)</span>: $u(x,\sigma)=(g_{\sqrt{2\sigma}}\ast f)(x)$, where we also filtered with kernels of different sizes, so there is some relation
  - depending on the circumstances one representation will be more useful than the other
- (iii) Where does $L_{\psi}f$ map? Just a variant of $L^2$ on $\left[0,\infty\right)\times\mathbb{R}$: $$L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2}):=\left\{F:[0,\infty)\times\mathbb{R}\to\mathbb{R}\ \mu-\text{measurable}:\int_\mathbb{R}\int_0^{\infty}\lvert F(a,b)\rvert^2\frac{dadb}{a^2} < \infty\right\}$$
  - **notation**: this <span style="color:red">$\frac{dadb}{a^2}$</span> in $L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})$ is <span style="color:red">not a set</span> into which we map (like eg. in the $L^2(\mathbb{R}^d,\mathbb{C})$ notation, where we map from $\mathbb{R}^d$ to $\mathbb{C}$), but it tells us that we need to change the integration measure $dadb$ by rescaling it with $\frac{1}{a^2}$. (just Berkels' notation)
  - everything <span style="color:red">except the $a^2$ scaling</span> would be "normal Lebesgue", in particular, $\int_\mathbb{R}\int_0^{\infty}\lvert F(a,b)\rvert^2 dadb < \infty$ would be "normal Lebesgue", ie. the square integral is finite
    - we <span style="color:red">weight this integral $\int_\mathbb{R}\int_0^{\infty}\lvert F(a,b)\rvert^2 dadb$ by $a^2$</span>: that means **for large $a$** this integral will be decreased because we multiply by sth that is small, but **for small $a$** this integral will be strongly increased. So, this has a <span style="color:red">**singularity at $a=0$**</span> which puts certain requirements on this transform $F$, so that <span style="color:red">**$\lvert F(a,b)\rvert^2$ has to vanish at $a=0$**</span> (otherwise $\int_\mathbb{R}\int_0^{\infty}\lvert F(a,b)\rvert^2 dadb$ would not be finite) and even a bit more.
- (iv) equipped with the same <span style="color:red">**equivalence relation**</span> used for the Lebesgue spaces <span style="color:green">(B.3)</span> and <span style="color:red">**scalar product**</span> $$(F,G)_{L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})}:=\int_\mathbb{R}\int_0^{\infty}\lvert F(a,b)G(a,b)\rvert^2\frac{dadb}{a^2}$$
  - the scalar product looks <span style="color:red">like the normal scalar product for $L^2$</span>, but we add this <span style="color:red">scaling $a^2$</span> (which we also required for the new $L^2$ space that we just defined in <span style="color:green">(iii)</span>)
- **problem**: so far it is unclear why this $a^2$ is sitting there, but in the next proposition we will see why we have it and how it relates to the wavelet transform

## 4.3 $L_{\psi}$ maps into $L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})$, $L_{\psi}$ is Linear, Plancherel Formula

- **conditions**:
  - $\psi\in L^2$ s.t. $0<c_\psi:=2\pi\int_{0}^{\infty}\frac{\lvert \hat{\psi}(\omega)\rvert^2}{\omega}d\omega<\infty$ (**warning**: these are 3 conditions!)
    - where $\hat{\psi}$ is the FT of $\psi$
- then,
  - (i) $L_\psi:L^2\to L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})$
    - this means that the functions that we get from this transform are not in the normal $L^2$ space, but in this "$L^2$ with the different scaling $a^2$". That is why we need this new space.
  - (ii) $L_\psi$ is a linear mapping
  - (iii) $(L_{\psi} f,L_{\psi} g)_{L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})}=c\_{\psi}(f,g)\_{L^2}$ for all $f,g\in L^2$
    - like **Plancherel formula** which said that $(\mathcal{F}f,\mathcal{F}g)\_{L^2} = (f,g)\_{L^2}$. This is essentially also true here. <span style="color:red">**Except that**</span> we do not have the normal scalar product on the lhs, but this <span style="color:red">**new scalar product**</span> and there is this extra <span style="color:red">**scaling factor $c_\psi$**</span>.
    - <span style="color:green">(iii)</span> says even more than that $L_{\psi}$ is <span style="color:red">**continuous**</span> (discussed more precisely in <span style="color:green">(4.5 (ii))</span>). Because <span style="color:green">(iii)</span> is a very strong property. It says that $L_{\psi}$ is <span style="color:red">**almost**</span> like an <span style="color:red">**isometry**</span>, except that it <span style="color:red">**changes lengths by a constant factor $c_\psi$**</span>.
- **proof**:
  - prove <span style="color:green">(iii)</span> using
    - Plancherel formula <span style="color:green">(3.27)</span>
    - properties of $\mathcal{F}$ for translation, modulation, etc. <span style="color:green">(3.4)</span>
    - Fubini
    - $f$ real-valued $\Rightarrow$ $\mathcal{F}f$ hermitian <span style="color:green">(3.6)</span>
  - <span style="color:green">(iii)</span> now immediately implies <span style="color:green">(i)</span>
    - because <span style="color:green">(i)</span> claims that we are mapping to this special space and for this special space we need that $\int_\mathbb{R}\int_0^{\infty}\lvert F(a,b)\rvert^2\frac{dadb}{a^2}$ is finite, but if I plug in $f$ or $g$ in <span style="color:green">(iii)</span> then we see that the integral we are looking for $(L_{\psi} f,L_{\psi} f)_{L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})}$ is the same as $c\_{\psi}(f,f)\_{L^2}$, ie. the norm squared of $f$, and the norm squared of $f$ is, of course, finite since we are starting with $f\in L^2$.
  - $L_{\psi}$ is linear <span style="color:green">(ii)</span> is clear because the integral in the def. of $L_{\psi}$ is linear in $f$ (see def. of $L_{\psi}$)
- **sidenote**: if you look closely in <span style="color:green">(4.3)</span>, <span style="color:red">we did not need that $c_{\psi} > 0$</span>, we only needed $c_{\psi} < \infty$. But we will need $c_{\psi} > 0$ to get the inverse.

## 4.4 Admissibility Condition, Wavelet

- (i) the condition $0<c_\psi:=2\pi\int_{0}^{\infty}\frac{\lvert \hat{\psi}(\omega)\rvert^2}{\omega}d\omega<\infty$ is called <span style="color:red">**admissibility condition**</span> and $c_\psi$ is the <span style="color:red">**admissibility constant**</span>
  - name "<span style="color:red">**admissibility**</span>" because if this condition is <span style="color:red">**not**</span> fulfilled, ie $c\_{\psi} = \infty$, then the norm in <span style="color:green">(4.3)</span> $c\_{\psi}(f,g)\_{L^2}$ explodes.
  - If $c\_{\psi} = 0$ then the norm $c\_{\psi}(f,g)\_{L^2}$ just degenerates: so that $f$ is mapped to norm $0$, although it was not $0$.
- (ii) any $\psi\in L^2$ that fulfills the admissibility condition is called <span style="color:red">**wavelet**</span>
- **outlook**:
  - $c_\psi<\infty$ gives us the **continuity** of the WT &rarr; <span style="color:green">(4.5 (ii))</span>
  - $c_\psi>0$ gives us the **invertibility** of the WT &rarr; <span style="color:green">(4.6)</span>

## 4.5 $L_{\psi}$ Almost Isometry, $L_{\psi}$ is Continuous, $\hat{\psi}(0) = 0\Leftrightarrow$ Mean Value of $\psi$ (not $\hat{\psi}$!) is $0$

- (i) the scalar product in <span style="color:green">(4.2 (iv))</span> induces the <span style="color:red">**norm**</span> $$\lVert L_{\psi}f\rVert_{L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})}=\sqrt{(L_{\psi}f,L_{\psi}f)_{L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})}}\stackrel{(4.3)}{=}\sqrt{c_{\psi}(f,f)_{L^2}}=\sqrt{c_{\psi}}\lVert f\rVert_{L^2}\quad\forall f\in L^2$$
- (ii) from <span style="color:green">(i)</span> we see that <span style="color:green">(4.3)</span> shows that <span style="color:red">**the WT is continuous**</span> <span style="color:red">if $c_{\psi}<\infty$</span>
  - because <span style="color:green">(i)</span> shows the **boundedness** of the WT and since the WT is **linear** <span style="color:green">(4.3 (ii))</span> we can conclude from <span style="color:green">(ex05 problem 2)</span> that the WT is **continuous**
- (iii) since $\displaystyle\int_0^1\frac{1}{\omega}d\omega=\lim_{a\searrow 0}\int_a^1\frac{1}{\omega}d\omega=\ldots=\infty$ (*since the integral in the admissibility condition has the same structure as this integral here, ie. it includes the integral from $0$ to $1$, if the $\lvert \hat{\psi}(\omega)\rvert^2$ does not go below a certain value then this integral in the admissibility condition will be $\infty$. This means $\lvert \hat{\psi}(\omega)\rvert$ <span style="color:red">**must**</span> go to $0$, otherwise the integral in the admissibility condition cannot be finite!*) it is <span style="color:red">**necessary**</span> that <span style="color:red">$\lvert \hat{\psi}(\omega)\rvert^2$ decays to $0$ quickly enough for $\omega \to 0$</span> <span style="color:purple">**(ie. $\lvert \hat{\psi}(\omega)\rvert^2 \to 0$ for $\omega \to 0$)**</span> in a suitable sense <span style="color:red">to get $c_\psi < \infty$</span> (*ie. the finiteness of the integral in the admissibility condition at $0$ can only come from the fact that $\lvert \hat{\psi}(\omega)\rvert^2$ also vanishes there*).
  - In case $\hat{\psi}$ is **continuous** (at least in $0$), <span style="color:purple">**$\lvert \hat{\psi}(\omega)\rvert^2 \to 0$ for $\omega \to 0$**</span> implies <span style="color:red">$\hat{\psi}(0) = 0$ is necessary (but not sufficient)</span> to get $c_\psi < \infty$.
    - note: since $\psi\in L^2$ and the FT maps from $L^2$ to $L^2$ it is not guaranteed that $\hat{\psi}$ is continuous.
    - phth: In case $\hat{\psi}$ is **not** continuous in $0$, $\lvert \hat{\psi}(\omega)\rvert^2 \to 0$ for $\omega \to 0$ does **not** imply $\hat{\psi}(0) = 0$ (eg. think about the function $f$ with $f=1$ for $x=0$ and $f=0$ for $x\neq 0$, then $f$ is **not** continuous in $0$ and $\lvert f(x)\rvert^2\to 0$ for $x\to 0$, <span style="color:red">**but**</span> $f(0)\neq 0$)
  - What does $\hat{\psi}(0) = 0$ mean?
    - we have discussed this: the $0$ frequency is the only frequency that you can easily interpret: <span style="color:red">**the $0$ frequency encodes somehow the mean**</span>: so <span style="color:red">if the $0$ frequency is $0$ then the mean of your image is $0$</span> (here: mean of this $\psi$ is $0$).
  - In other words, it is necessary (but not sufficient) that the <span style="color:red">mean value of $\psi$ (not $\hat{\psi}$!) is $0$</span> for $\psi$ to be a wavelet.
- (iv) The condition <span style="color:red">$c_\psi > 0$ is necessary</span> to guarantee the <span style="color:red">**invertibility of the transform**</span> on its image <span style="color:green">(4.6)</span>:

## 4.6 Wavelet Inversion Theorem

- **conditions**:
  - $\psi\in L^2$
  - $\psi$ is <span style="color:red">a wavelet</span>
    - ie. $\psi$ fulfills the <span style="color:red">admissibility condition</span>
- then, $$\text{for all}\ f\in L^2:\ f(x) = \frac{1}{c_\psi}\int_{\mathbb{R}}\int_0^\infty L_{\psi}f(a,b)\frac{1}{\sqrt{a}}\psi\left(\frac{x-b}{a}\right)\frac{dadb}{a^2}\quad\text{for a.e.}\ x\in\mathbb{R}$$
- in words: we can transform back by integrating over $da$ and $db$ (instead of $dx$ as in the WT <span style="color:green">(4.1)</span>) with the proper scaling $a^2$ and the same integration weight $\frac{1}{\sqrt{a}}\psi\left(\frac{x-b}{a}\right)$ as in the WT <span style="color:green">(4.1)</span>
- **proof**:
  - <span style="color:red">**Hilbertian adjoint of $L_{\psi}$**</span> $L_{\psi}^\ast:L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})\to L^2(\mathbb{R})$
    - a mapping that goes exactly in the other direction than $L_{\psi}$, ie. it is mapping **from** the <span style="color:red">weighted $L^2$</span> space **to** <span style="color:red">$L^2$</span>
    - **idea**:
      - in finite dimensions the expression $(L_{\psi}f,F)\_{L^2(\left[0,\infty\right)\times\mathbb{R},\frac{dadb}{a^2})}$ would be a scalar product on some $\mathbb{R}^n$ and the linear mapping $L_{\psi}$ would be a matrix, ie. the expression would be like the scalar product $(\underline{A}x,y)\_{\mathbb{R}^n}$
      - recall (a standard linear algebra trick): what can you do to <span style="color:red">move the matrix $\underline{A}$ in $(\underline{A}x,y)\_{\mathbb{R}^n}$ to the other argument $y$</span>?
        - for **real** $\underline{A}$: $(\underline{A}x,y)\_{\mathbb{R}^n}=(x,\underline{A}^{\top} y)\_{\mathbb{R}^n}$
        - for **complex** $\underline{A}$: $(\underline{A}x,y)\_{\mathbb{R}^n}=(x,\overline{\underline{A}^{\top}} y)\_{\mathbb{R}^n}$
          - recall: **Hermitian matrices** $\underline{A}=\overline{\underline{A}^{\top}}$ can be understood as the complex extension of real **symmetric matrices** $\underline{A}=\underline{A}^{\top}$., [Hermitian matrix](https://en.wikipedia.org/wiki/Hermitian_matrix)
        - there are <span style="color:red">2 kinds of adjoints</span>, [Hilbert space adjoint vs Banach space adjoint](https://math.stackexchange.com/questions/2110405/hilbert-space-adjoint-vs-banach-space-adjoint)
          - **"Hilbertian" adjoint** (usually called **conjugate transpose**)
          - "Dual" or **"Banachian" adjoint**
      - the Hilbertian adjoint just <span style="color:red">generalizes the concept</span> of transposing or adjoining a matrix to move it from one scalar product argument to the other <span style="color:red">to linear mappings on Hilbert spaces</span> (so, just think of it as a **generalized transpose**)
- **problem**:
  - so far we have only shown that it is <span style="color:red">**necessary**</span> that the <span style="color:red">mean of $\psi$ is $0$</span> (to ensure $\hat{\psi}(0)=0$), but this is <span style="color:red">**not sufficient**</span> for $\psi$ to be a wavelet!

## 4.7 Example Wavelets: 1st and 2nd Derivatives of the Gaussian, Haar Wavelet

### 4.7 (i) 1st Derivative of the Gaussian

- (i) <span style="color:green">**1st derivative of the Gaussian**</span>: Let $\psi(x):=xe^{-\frac{x^2}{2}}=-\frac{d}{dx}g(x)$, where $g(x):=e^{-\frac{x^2}{2}}$
  - **idea**: to show that this is a wavelet we need to compute $c_\psi$ and this involves computing the FT of $\psi$ (now you can see why I chose the Gaussian first because there we know what the FT is)
- then, $$\hat{\psi}(\omega)=\mathcal{F}\psi(\omega)=\mathcal{F}\left(-\frac{d}{dx}g\right)(\omega)\stackrel{(3.17)}{=}-i\omega\mathcal{F}g(\omega)\stackrel{(3.20)}{=}-i\omega g(\omega)=-i\omega e^{-\frac{\omega^2}{2}}=-i\psi(\omega)$$ $$\Rightarrow c_\psi=2\pi\int_0^\infty\frac{\lvert \hat{\psi}(\omega)\rvert^2}{\omega}d\omega=2\pi\int_0^\infty\frac{\lvert -i\omega e^{-\frac{\omega^2}{2}}\rvert^2}{\omega}d\omega=2\pi\int_0^\infty\frac{\omega^2 e^{-\frac{2\omega^2}{2}}}{\omega}d\omega=2\pi\int_0^\infty\omega e^{-\omega^2}d\omega=\pi\int_0^\infty 2\omega e^{-\omega^2}d\omega=\pi\left(-e^{-\omega^2}\right)\bigg\vert_0^\infty=\pi$$ $$\Rightarrow \psi\ \text{is a wavelet.}$$
- **important**: unlike the mean of $\psi$, the mean of $g$ is <span style="color:red">**not**</span> $0$, so $g$ cannot be a wavelet itself!
- **intuition**:
  - $g$ is always above the x-axis, ie. the mean (average value of the function) of $g$ must be positive and not equal to $0$!
  - $\psi$ is in part below the x-axis. 
    - $\psi$ is a product of the function $x$ and a Gaussian. This product must be negative for $x<0$ because $x$ is negative for $x<0$. Ie. the graph of $\psi$ must be in the 3rd quadrant. Since $x=0$ for $x=0$ the graph must go through the origin. For $x>0$ the graph must be in the 1st quadrant because $x$ and the Gaussian are both positive there.

<p align="center">
  <a href="https://imgbb.com/"><img src="https://i.ibb.co/C21fLts/Screenshot-from-2024-03-23-12-10-04.png" alt="Screenshot-from-2024-03-23-12-10-04" border="0"></a>
</p>

- With the **convolution representation** of the wavelet transform, we get $$L_{\psi}f(a,b)=\frac{1}{\sqrt{a}}(f\ast D_{-\frac{1}{a}}\psi)(b)=-\frac{1}{\sqrt{a}}\left(f\ast D_{-\frac{1}{a}}g^\prime\right)(b)=\frac{a}{\sqrt{a}}\left(f\ast (D_{-\frac{1}{a}}g)^\prime\right)(b)=\sqrt{a}\left(f\ast (D_{-\frac{1}{a}}g)\right)^\prime(b)$$
  - Similarly to the **Canny Edge Detector** <span style="color:green">(2.16)</span>, the input $f$ is convolved with a scaled Gaussian for different scales (ie. different kernel sizes).
    - in <span style="color:green">(2.16)</span> we noticed that for different scales we get different edge strengths and we had some nice properties and it is somewhat similar here
  - Unlike the **Canny Edge Detector** <span style="color:green">(2.16)</span>, a derivative is taken after the convolution.

### 4.7 (ii) Mexican Hat

- (ii) <span style="color:green">**2nd derivative of the Gaussian**</span>, aka <span style="color:red">**Mexican hat**</span>: Let $\psi(x):=\left(1-x^2\right)e^{-\frac{x^2}{2}}=-\frac{d^2}{dx^2}g(x)$
  - one can show (exercise, similar to <span style="color:green">(i)</span>) that $\hat{\psi}(\omega)=\omega^2 e^{-\frac{\omega^2}{2}}$ and $c_\psi=\pi$. Thus, the <span style="color:red">**Mexican hat is also a wavelet**</span>.

<p align="center">
  <a href="https://ibb.co/cC6s7jw"><img src="https://i.ibb.co/X37GvqD/Screenshot-from-2024-03-23-12-56-31.png" alt="Screenshot-from-2024-03-23-12-56-31" border="0"></a>
</p>

- this gives us some hint at what wavelets can look like and what kind of information they possibly extract from our functions:
  - these wavelets are <span style="color:red">**not**</span> **local mean values** because the wavelets themselves have mean $0$, but you can think of <span style="color:green">(i)</span>, for instance, as something like a <span style="color:red">**smoothed local derivative**</span>. Because, at $x=0$, $\psi$ weights the things to the right of the point $x=0$ positively and the things to the left of the point $x=0$ negatively. Thus, $\psi$ is something like a derivative (cf. derivative filters <span style="color:green">(2.12)</span>).
  - however, <span style="color:green">(ii)</span> is more like a <span style="color:red">**2nd derivative**</span> because you have a positive value in the center and negative values in the surrounding, but all very smoothed out.

### 4.7 (iii) Haar Wavelet

- (iii) <span style="color:red">**Haar wavelet**</span>: $$\begin{equation*} \psi(x):=
                                    \begin{cases}
                                    1 & 0 \leq x < \frac{1}{2}\\
                                    -1 & \frac{1}{2} \leq x < 1\\
                                    0 & \text{else}
                                    \end{cases}
                                \end{equation*}$$
  - here it is easier to see what $\psi$ computes: $\psi$ computes the mean over $\left[0,\frac{1}{2}\right]$ minus the mean over $\left[\frac{1}{2},1\right]$, so it is also sth like a derivative of your input
  - to show that the <span style="color:red">**Haar wavelet is a wavelet**</span> we can
    - represent $\psi$ as $\psi=\chi_{\left[0,\frac{1}{2}\right)}-\chi_{\left[\frac{1}{2},1\right)}$ and then we can use the properties of the FT to show that $$\hat{\psi}(\omega)=\frac{1}{\sqrt{2\pi}}ie^{-i\frac{\omega}{2}}\sin\left(\frac{\omega}{\psi}\right)\text{sinc}\left(\frac{\omega}{4\pi}\right)$$
      - this should not surprise us because we know that the FT of $\chi_{\left[-B,B\right]}$ is a cardinal sine and we know that shifts are turned into modulations, and therefore, $\chi_{\left[0,\frac{1}{2}\right)}$ would have to be shifted to $\chi_{\left[-\frac{1}{4},\frac{1}{4}\right)}$ (ie. the support of $\chi$ is shifted to the origin s.t. $\chi$ has the shape $\chi_{\left[-B,B\right]}$) and $\chi_{\left[\frac{1}{2},1\right)}$ as well, and that is encoded in this modulation $e^{-i\frac{\omega}{2}}$ (cf. <span style="color:green">(3.4)</span>)
      - this is not completely obvious and you really need to compute this to see that this formula is correct
    - based on that we can show $c_\psi=\ln{2}$, thus $\psi$ is a wavelet
      - **interesting**: you do not need to compute this manually because $c_\psi=\ln{2}$ also follows from <span style="color:green">(4.16)</span> + <span style="color:green">(4.17 (i))</span>
        - **outlook**: we will show how to generate wavelets with certain properties and with this generation we can also create the Haar wavelet and that is why it will inherit the general properties
  - Unlike the <span style="color:red">**derivatives of the Gaussian**</span> we considered before (ie. <span style="color:green">(i)</span> and <span style="color:green">(ii)</span>) that are very <span style="color:red">**smooth**</span> and have <span style="color:red">**non-compact support**</span>, the <span style="color:red">**Haar wavelet**</span> is <span style="color:red">**discontinuous**</span> and has <span style="color:red">**compact support**</span>.
    - **interesting**: it is actually quite difficult to find wavelets that are both smooth and have compact support (will be discussed later)

<p align="center">
  <a href="https://imgbb.com/"><img src="https://i.ibb.co/Sc60Ybn/Screenshot-from-2024-03-23-14-37-59.png" alt="Screenshot-from-2024-03-23-14-37-59" border="0"></a>
</p>

# 4.2 The Discrete Wavelet Transform

- **note**: unlike what the name suggests this discrete WT is <span style="color:red">**not fully discrete**</span> like the DFT. (However, there are *some* relations to the FS, so it is more like the FS.)
- **problem**: One thing that is very apparent is that this **WT so far is redundant** because we take a 1D function and convert it to a 2D function. Therefore, this 2D function has to have some redundant information because **we cannot need a 2D function to describe a 1D function**.
- **more precisely**: The (continuous) wavelet transform of a function $f \in L^2 (\mathbb{R})$ is apparently a <span style="color:red">**redundant representation of $f$**</span> ($f$ is univariate (1D function), $L_\psi f$ is bivariate (2D function)).
  - The question arises <span style="color:red">**whether it is sufficient to know $L_\psi f$ on a subset of $\left(0,\infty\right] \times \mathbb{R}$**</span> (ie. perhaps we do not need to compute it for <span style="color:red">**all**</span> the <span style="color:red">**scalings**</span> and all the <span style="color:red">**shifts**</span>, but just for <span style="color:red">**some**</span> of them).
    - It will turn out that this is indeed the case for certain <span style="color:red">**discrete subsets**</span>, but showing this requires several preparations.
    - But it is very important to venture into this because <span style="color:red">**in order to practically use the WT**</span> we need to figure out for which scalings and for which translations we actually **need** these computations.
- **problem**: The first tool on this way is the MRA:

## 4.8 MRA, MSA, Generator

- For $j \in \mathbb{Z}$, let $V_j$ be (vii) a <span style="color:red">closed subspace of $L^2(\mathbb{R})$</span>.
- Then, $(V_j)_{j\in \mathbb{Z}}$ is called <span style="color:red">**multiresolution analysis (MRA)**</span> or <span style="color:red">**multiscale approximation (MSA)**</span>, if
it fulfills the following conditions:
  - (i) <span style="color:red">**translational invariance**</span> $\forall j, k \in \mathbb{Z} : f \in V_j \Leftrightarrow T_{2^j k} f \in V_j$
    - when we fix a certain $j$, we must be able to shift $f$ in $V_j$ while $f$ stays in $V_j$. But we are not allowed to shift arbitrarily, but we have to make finite steps: $k$ is the number of steps that we can take. So we can take arbitrary many steps (because $k$ is in $Z$), but the step size is $2^j$, ie. the bigger the $j$ the larger the step. Which hints that these $V_j$'s get smaller and smaller (&rarr; <span style="color:green">(ii)</span>) because for $j=0$ we must be able to shift by step size $2^0=1$, for $j=1$ we only need to be able to shift by step size $2$, etc.
  - (ii) <span style="color:red">**inclusion**</span> $\forall j \in \mathbb{Z} : V_{j+1} \subset V_j$
    - ie. the space is getting smaller when I increase $j$
      - "if $j_1 > j_2$ the space $V_{j_1}$ is included in $V_{j_2}$"
      - with increasing $j$ the spaces can only get smaller
  - (iii) <span style="color:red">**scaling**</span> $\forall j \in \mathbb{Z} : f \in V_j \Leftrightarrow D_{\frac{1}{2}} f \in V_{j+1}$
    - this $D_{\frac{1}{2}}$ just means that everything is spread out
      - eg. lets say your $f$ is a characteristic function of the interval $\left[0,1\right]$, then when you apply $D_{\frac{1}{2}}$ it will be the characteristic function of $\left[0,2\right]$ (which also relates to the fact that if $j$ gets bigger then these shifts $T_{2^jk}$ get bigger)
  - **problem**: we need some more properties to make this meaningful because so far with <span style="color:green">(i-iii)</span> we could just say that every $V_j$ is all of $L^2$, then everything here is trivially fulfilled. So, we need to prevent that with the following properties:
  - (iv) <span style="color:red">**trivial intersection**</span> $\bigcap_{j\in \mathbb{Z}} V_j = \\{0\\}$
    - ie. the intersection of all the $V_j$'s is just $0$ which formalizes that these spaces get smaller and smaller and at some point there is nothing in there any more.
  - **problem**: but <span style="color:green">(iv)</span> would still be fulfilled if we just say all $V_j$'s are just $0$, then everything is trivially fulfilled. We do not want this, either. Therefore, we require <span style="color:green">(v)</span>:
  - (v) <span style="color:red">**completeness**</span> (actually, "denseness in $L^2$") $\overline{\bigcup_{j\in \mathbb{Z}} V_j} = L^2(\mathbb{R})$
    - phth: aka "$\bigcup_{j\in \mathbb{Z}} V_j$ is dense in $L^2$"
      - siehe "Prerequisites" &rarr; "dense" &rarr; def. <span style="color:green">(i)</span>
    - Berkels: in this sense the $V_j$'s must capture everything that is in $L^2$.
    - phth: complete in the sense that $L^2$ is complete (**Riesz-Fischer theorem**) and we must be able to approximate any $L^2$ function arbitrarily well with a sequence of functions in $\bigcup_{j\in \mathbb{Z}} V_j$, where the limit of this sequence need not necessarily be in $\bigcup_{j\in \mathbb{Z}} V_j$ (like we can approximate any element of $\mathbb{R}$ with a sequence in $\mathbb{Q}$)
  - (vi) <span style="color:red">**orthonormal basis**</span> $\exists\phi \in V_0 : \\{T_k \phi : k \in \mathbb{Z}\\}$ is a CONS of $V_0$
    - $T_k \phi$ means "shift $\phi$ by integers $k$"
    - the "shift $\phi$ by integers" links to the translational invariance <span style="color:green">(i)</span> for $j=0$. There \[in <span style="color:green">(i)</span>\] we \[require that all $f$ in $V_0$\] also need to be able to shift by integers. Thus, if $\phi$ is in $V_0$, then all the $T_k\phi$ must \[necessarily!\] be in $V_0$ by property <span style="color:green">(i)</span>. But \[in <span style="color:green">(vi)</span>\] what we want is that this \[set\] is a CONS!
      - which gives us a very nice <span style="color:red">**way to construct a basis**</span> for these kind of spaces
- The function $\phi$ from the orthonormal basis property is called <span style="color:red">**generator**</span> or <span style="color:red">**scaling function**</span> of the MRA.
  - name "**generator**" comes from <span style="color:green">(vi)</span> because $\phi$ generates an ONS of $V_0$
  - name "**scaling function**" comes from the fact that $\phi$ fulfills the scaling equation <span style="color:green">(4.11)</span>, ie. $\phi$ can be expressed by a scaled version of itself
- **problem**: this sounds like a very abstract set of properties where it is not so clear yet why this is good or perhaps not even clear how we could even choose this $V_j$. So let us first start with a simple <span style="color:green">**example**</span> of such an MRA:

## 4.9 Example MRA: "Space of Functions that are Constant on Dyadic Intervals"

- for $j\in\mathbb{Z}$ let $$V_j:=\{f\in L^2 : f\vert_\left[k2^j,(k+1)2^j\right)\ \text{is constant for all}\ k\in\mathbb{Z}\}$$
  - ie. $V_j$ is the space of square integrable functions that are constant on the <span style="color:red">**dyadic intervals $\left[k2^j , (k + 1)2^j\right)$**</span> ("dyadic" because the interval limits are powers of $2$, related: dyadic logarithm)

<p align="center">
  <a href="https://ibb.co/9N5Kbpr"><img src="https://i.ibb.co/t4rjqHJ/Screenshot-from-2024-03-25-23-19-09.png" alt="Screenshot-from-2024-03-25-23-19-09" border="0"></a>
</p>

- then, <span style="color:red">$(V_j)_{j\in\mathbb{Z}}$ is a MRA</span>.

### Proof (vii)

- <span style="color:green">(vii)</span> The $V_j$ are obviously closed subspaces of $L^2(\mathbb{R})$.
  - Berkels: If you have a sequence of functions $f_n\in V_j$ <span style="color:red">for a fixed $j$</span> that are piecewise constant on the interval $\left[k2^j , (k + 1)2^j\right)$, then their limit $\lim_{n\to\infty}f_n = f$ will also be piecewise constant on the same interval $\left[k2^j , (k + 1)2^j\right)$, ie. $f\in V_j$. There is nothing that could happen to make it \[$\lim_{n\to\infty}f_n$\] not piecewise constant.
    - "**Folgen-Abgeschlossenheit**" (Bredies S.17): "(...) für jede Folge $( x_n )$ in $U$ mit $x_n \to x$ der Grenzwert $x$ auch in $U$ ist"
    - recall: "closed" = "abgeschlossen bzgl bestimmter Operationen" (aka "closed under the limit operation")
      - zB. closed bzgl Multiplikation heißt, Multiplikation führt nicht aus der Menge raus, dh. das Produkt bleibt in der Menge, in der die Faktoren waren
      - "In a **complete metric space**, a closed set is a set which is <span style="color:red">**closed under the limit operation**</span>.", [Wikipedia](https://en.wikipedia.org/wiki/Closed_set)
        - $L^2$ is complete (**Riesz-Fischer Theorem**)

### Proof (i) - (iii)

- <span style="color:green">(i)</span> Translational invariance, <span style="color:green">(ii)</span> inclusion and <span style="color:green">(iii)</span> scaling are obviously fulfilled from the corresponding properties of the dyadic intervals.
  - <span style="color:green">(i) - (iii)</span> are **intuitively** clear:
    - <span style="color:green">(i)</span> we need to be able to shift by multiples of $2^j$: if say $j=0$, then we need to shift the function by $2^0=1$, or by $2^1=2$, etc. and this just shifts the constant values from one interval to another interval, but they will stay piecewise constant on these intervals, you just shift the values to different intervals
    - <span style="color:green">(ii)</span> if you scale by $\frac{1}{2}$ you are supposed to go from $V_j$ to $V_{j+1}$. Lets look at the green function here: if we scale that by $\frac{1}{2}$ that means we double everthing, ie. this value will be mapped to that (...), so you just switched from being constant on intervals of length one to being constant on intervals of length two
    - <span style="color:green">(iii)</span> if you are piecewise constant on intevals of length $2^{j+1}$ of course this is also piecewise constant on intervals of length $2^j$ (because the latter is just a weaker property)

### Proof (iv)

- <span style="color:green">(iv)</span> A function that is in the intersection of all $V_j$ has to be constant
  - on $\left[0,1\right)$ **and** on $[−1, 0)$ (for $j=0$).
  - on $\left[0,2\right)$ **and** on $[−2, 0)$ (for $j=1$).
  - ... etc.
  - on $\left[0,\infty\right)$ **and** on $(−\infty, 0)$ (for $j\to \infty$).
  - The only $L^2$-function ($L^2$ means the function must be integrable on $\left[0,\infty\right)$ and $(−\infty, 0)$!) that fulfills this is the zero function.

### Proof (v)

- <span style="color:green">(v)</span> The completeness follows from the fact $C_c(\mathbb{R})$ is dense in $L^2(\mathbb{R})$ <span style="color:green">(B.8)</span>, **continuous functions on compact sets** (ie. $C_c(\mathbb{R})$) are **uniformly continuous** and that **uniformly continuous functions** can be approximated in the supremum norm with **functions constant on dyadic intervals**.
  - phth: thus, functions constant on dyadic intervals are dense in $L^2$
  - Berkels: You need to stack these arguments, ie. first go to compact support, then go to uniform continuity and then you know you just need to make these intervals small enough to have the necessary approximation.
  - phth:
    - This prooves that "$\bigcup_{j\in \mathbb{Z}} V_j$ is dense in $L^2$" (denseness).
    - But, $\bigcup_{j\in \mathbb{Z}} V_j$ is NOT complete in the sense that "all Cauchy sequences in $\bigcup_{j\in \mathbb{Z}} V_j$ converge in $\bigcup_{j\in \mathbb{Z}} V_j$" (completeness).
    - $L^2$ is complete (**Riesz-Fischer Theorem**), but this does not necessarily mean that $\bigcup_{j\in \mathbb{Z}} V_j$ is complete (because eg. $\mathbb{Q}$ is dense in $\mathbb{R}$, but $\mathbb{Q}$ is not complete just because $\mathbb{R}$ is complete!).
    - But we can approximate any $L^2$ function arbitrarily well with a function in $\bigcup_{j\in \mathbb{Z}} V_j$.
  - recall:
    - "completeness" = "Cauchy sequences in this set converge to a point in this set"
    - "$\mathbb{Q}$ is dense in $\mathbb{R}$"
    - "$L^p$ is complete" (cf. section "Prerequisites")
    - "supremum norm" = "distance between two functions"
    - ["uniform continuity"](https://en.wikipedia.org/wiki/Uniform_continuity) def.

### Proof (vi)

- <span style="color:green">(vi)</span> Finally, $\phi = \chi_{\left[0,1\right)}$ is a generator \[because\]
  - <span style="color:green">(vi.i)</span> The ONS property \[of $\\{T_k \phi : k \in \mathbb{Z}\\}$\] is obviously fulfilled and
    - Behauptung: betrachte $V_0 = \\{f \in L^2 : f \vert_{\left[k,k+1\right)}\ \text{is constant for all}\ k \in \mathbb{Z}\\}$, dann ist $\phi = \chi_{ \left[0,1\right) } \in V_0$ ein Generator, weil:
    - Berkels:
      - So, \[for $\phi$ to be a generator\] we need that these shifted versions of $T_k\phi$ are a CONS.
      - the orthonormal property is obvious because if you shift $\left[0,1\right]$ by an integer, then it will not intersect with the interval $\left[0,1\right]$ any more. So, for different integer shifts $k$ you just have this function \[$\chi_{ \left[0,1\right) }$\] on $\left[k,k+1\right]$ and those \[$\chi_{ \left[0,1\right) }$ and $\chi_{ \left[k,k+1\right) }$\] do not intersect.
        - dh. die $L^2$-norm der Produktfunktion $\chi_{ \left[m,m+1\right) } \cdot \chi_{ \left[n,n+1\right) }$ ist $0$ für $m \neq n$ und $1$ für $m=n$, was die ONS Eigenschaft der shifted versions of $T_k\phi$ beweist (see ONS def.). 
  - <span style="color:green">(vi.ii)</span> the completeness \[of $\\{T_k \phi : k \in \mathbb{Z}\\}$\] is shown based on the fact that <span style="color:green">(A)</span> <span style="color:purple">the squared $L^2$-norm of an element of $V_0$</span> has to be <span style="color:green">(B)</span> <span style="color:purple">the sum of the squared values of the element on each of the intervals $\left[k,k+1\right)$</span>.
    - Berkels:
      - <span style="color:green">(B)</span> is equivalent to the Fcoeffs in <span style="color:green">(3.42)</span>
      - so, if I am supposed to give you the $L^2$-norm <span style="color:green">(rhs of (3.42))</span> of the green function in the picture above it is always the squared area under the graph and, since the length is $1$, it is just the squares of the heights. So, this is a series. Based on this, you directly see that the completeness def. <span style="color:green">(3.42)</span> is fulfilled, where we needed to show that <span style="color:purple">the norm of function squared</span> <span style="color:green">(rhs of (3.42))</span> is the same as <span style="color:purple">the sum of the squared Fcoeffs</span> <span style="color:green">(lhs of (3.42))</span>. Here, the Fcoeffs <span style="color:green">(lhs of (3.42))</span> are exactly the interval heights (because we just multiply a function that is constant on the interval $\left[k,k+1\right)$ which gives us then the value in the interval $\left[k,k+1\right)$).
    - phth: "<span style="color:green">(A)</span> = <span style="color:green">(B)</span>" $\Leftrightarrow$ shifted versions of $T_k\phi$ are a CONS (cf. <span style="color:green">(3.42)</span>)
    - phth: consider an element $f \in V_0$, then
      - <span style="color:green">(A)</span> is $\lVert f\rVert^2_{L^2}$
      - <span style="color:green">(B)</span> is $\sum_{k\in Z}(f, T_k\chi_{\left[0,1\right)})^2$, where $(f, T_k\chi_{\left[0,1\right)})$ is the k-th Fcoeff

## 4.10 Orthogonal Projection $P_{V_j}$

- **conditions**:
  - Let $(V_j)\_{j\in\mathbb{Z}}$ be a MRA.
  - Let $P\_{V_j}$ denote the <span style="color:red">**orthogonal projection from $L^2$ to $V_j$**</span>.
    - Note that the orthogonal projection exists since $L^2$ is a Hilbert space and $V_j$ a nonempty, closed subspace of $L^2$. ([**Hilbert projection theorem**](https://en.wikipedia.org/wiki/Hilbert_projection_theorem#Statement))
  - Let $f \in L^2$.
- Then, we have $\lim_{j\to\infty} P\_{V_j} f = f$ and $\lim_{j\to -\infty} P\_{V_j} f = 0$.
  - This means that we can use the nested spaces $V_j$ to approximate $f$, <span style="color:red">the smaller $j$, the better the approximation</span>.
    - So, if we have a very small $j$ ($j\to -\infty$, highly negative), then we get very close to $f$ and if it is very large ($j\to\infty$), then it is very coarse. So, we can decide how well we want to approximate $f$ by choosing this $V_j$.
    - we can approximate $f$ with the spaces $V_j$ arbitrarily well if $j$ is "negative enough"
- **proof**:
  - the reason we are looking at this is to get familiar with the consequences of all of the properties in <span style="color:green">(4.8)</span>. So far, it is not clear why we need these properties. Here this gets clear.
  - $j\to\infty$: <span style="color:green">(4.14)</span>
  - $j\to -\infty$: show that $\lVert f − P\_{V_j} f\rVert_{L^2} = \inf_{u\in V_j} \lVert f − u\rVert_{L^2}$ for all $j \in \mathbb{Z}$ goes to $0$ for $j\to -\infty$
    - use **completeness** <span style="color:green">(4.8 (v))</span> and **inclusion** <span style="color:green">(4.8 (ii))</span> properties

## 4.11 Scaling Equation

- **conditions**:
  - Let $(V_j)\_{j\in\mathbb{Z}}$ be a MRA with generator $\phi$.
    - again, this is just some object that fulfills all of the properties in <span style="color:green">(4.8)</span>
    - and we have one example <span style="color:green">(4.9)</span> that fulfills all of the properties in <span style="color:green">(4.8)</span>, so we know that there are such MRAs, thus it makes sense to look at them
  - Let $\phi_{j,k} (x) := 2^{-\frac{j}{2}}\phi(2^{-j}x - k)$ for all $j, k \in \mathbb{Z}$ and $x \in \mathbb{R}$.
    - **problem**: why do we look at $\phi_{j,k}(x)$? &rarr; Because $\phi_{j,k}(x)$ is an ONB of $V_j$ &rarr; <span style="color:green">(i)</span>

### 4.11 (i) $\\{\phi_{j,k} : k \in \mathbb{Z}\\}$ is an ONB/CONS of $V_j$

- (i) The scaling property <span style="color:green">(4.8 (iii))</span> and the orthonormal basis property <span style="color:green">(4.8 (vi))</span> together imply that, <span style="color:red">for all $j \in \mathbb{Z}$, $\\{\phi_{j,k} : k \in \mathbb{Z}\\}$ is an ONB/CONS of $V_j$</span> (**exercise**).
  - **problem**: why is this the case?
    - so, in <span style="color:green">(4.8)</span> we have assumed that if we start with $\phi$ then these $T_k\phi$s are a CONS
    - but if we look at the new notation then this $T_k\phi$ is nothing but $\phi_{0,k}$
      - $j=0$ eliminates this scaling $2^{-\frac{j}{2}}$ and $2^{-j}$ in $\phi_{j,k}(x)$
      - and we just have the translation $(x-k)$ in $\phi_{j,k}(x)$
    - the $\phi_{0,k}$s are an ONS of $V_0$
    - but then we have this scaling property <span style="color:green">(4.8 (iii))</span>, ie. that $f$ is in $V_j$ **iff** the scaled version $D_{\frac{1}{2}}f$ is in $V_{j+1}$ and this we can use to convert the ONS from $V_0$ to the other $V$s
    - the scaling $2^{-\frac{j}{2}}$ is exactly done in such a way that the **normality property** does not vanish because if you just scale by $D_{\frac{1}{2}}f$ you would scale the $L^2$-norm (ie. the scaling would change the $L^2$-norm), but the $2^{-\frac{j}{2}}$ corrects for this (**exercise**: when you just compute it you will see how this fits together)
    - so, from one ONB of $V_0$ we can go to ONBs for all of our $V_j$s
    - **note**: if we have such an MRA then the <span style="color:red">absolute value of $j$ has no meaning by itself</span> because if we have $(V_j)\_{j\in\mathbb{Z}}$ then we can just shift $j$ by any number and it still goes from $-\infty$ to $\infty$ just that $j=0$ then means perhaps $j=10$, so there is no inherent value. The only thing that puts a specific value to the $j$ is that for $V_0$ we have the generator $\phi$. And, since we have the generator at $0$, it is natural that we just go one level finer and end up with $V_{-1}$. In principle, we could just invert the ordering of the $j$s, it would not change anything, except for that going up with $j$ (better/**finer** approximation) and going down with $j$ (worse/**coarser** approximation) would mean the opposite, ie. whether you call "going finer" "increasing $j$" or "decreasing $j$" does not matter. So, the construction <span style="color:green">(4.8)</span> links different scales, but it is completely arbitrary whether you want to associate fine scales with a large $j$ or with a negative $j$. It is just one decision. You can just replace $j$ with $-j$ everywhere, if you want, and everything stays consistent.
  - **problem**: in this way we can connect amongst other things $V_0$ and $V_{-1}$ &rarr; <span style="color:green">(ii)</span>

### 4.11 (ii) The Generator $\phi$ is not only in $V_0$, but also in $V_{-1}$

- (ii) Due to the inclusion property <span style="color:green">(4.8 (ii))</span>, we have <span style="color:red">$\phi \in V_0 \subset V_{-1}$, ie. the generator is not only in $V_0$, but also in $V_{-1}$</span>.

### 4.11 (iii) We can express the Generator $\phi$ in the CONS $\\{\phi_{-1,k} : k \in \mathbb{Z}\\}$

- (iii) Since $\\{\phi_{-1,k} : k \in \mathbb{Z}\\}$ is a CONS of $V_{-1}$, <span style="color:red">**we can express the generator $\phi$ in this CONS**</span> $$\label{eq:scaling-equation}\phi = \sum_{k\in\mathbb{Z}} h_k \phi_{-1,k} = \sum_{k\in\mathbb{Z}} h_k \sqrt{2}\phi(2 \cdot - k),$$ where the dot "$\cdot$" is just a placeholder (not a variable!) and <span style="color:red">$h_k = (\phi, \phi_{-1,k} )_{L^2}$</span> for $k \in \mathbb{Z}$ are <span style="color:red">**the Fcoeffs**</span>, cf. <span style="color:green">(3.43)</span> (note that this does <span style="color:red">**not**</span> mean that the series is converging pointwise a.e.).
  - This equation is called <span style="color:red">**scaling equation**</span> and the reason why $\phi$ is also called <span style="color:red">**scaling function**</span> (cf. <span style="color:green">(4.8)</span>).
  - **note**: This also means that we can express $\phi$ with scaled versions of itself. This puts some interesting constraints on this $\phi$.
  - phth: this does <span style="color:red">**not**</span> imply the <span style="color:purple">pointwise convergence</span> $\phi\color{red}{(x)} = \sum_{k\in\mathbb{Z}} h_k\sqrt{2}\phi(2\color{red}{x} - k)$ for all $x$, but only <span style="color:purple">convergence of the $L^2$-norm of the series</span> <span style="color:green">(rhs)</span> to the $L^2$-norm of $\phi$ <span style="color:green">(lhs)</span> (see <span style="color:green">(3.48)</span>)
  - <span style="color:red">**watch**</span>: lecture 27, beginning of <span style="color:green">(4.12)</span>: long discussion about <span style="color:green">(3.43)</span> and <span style="color:green">(3.48)</span>, <span style="color:purple">convergence of the series in the $L^2$-norm</span> vs. <span style="color:purple">pointwise convergence</span>
    - **counterexample**: in the figure below: consider the following sequence of functions:
      - (black) $1$ on $\left[0,1\right]$, otherwise $0$
      - (red) $1$ on $\left[0,\frac{1}{2}\right]$, otherwise $0$
      - (blue) $1$ on $\left[\frac{1}{2},1\right]$, otherwise $0$
      - (green) $1$ on $\left[0,\frac{1}{4}\right]$, otherwise $0$
      - (...) $1$ on $\left[\frac{1}{4},\frac{1}{2}\right]$, otherwise $0$
      - (...) $1$ on $\left[\frac{1}{2},\frac{3}{4}\right]$, otherwise $0$
      - (...) $1$ on $\left[\frac{3}{4},1\right]$, otherwise $0$
      - (...) $1$ on $\left[0,\frac{1}{8}\right]$, otherwise $0$
      - etc.
      - thus, wherever you are in this sequence, you will always pick up a $1$
      - thus, <span style="color:red">**at no point**</span> this sequence will <span style="color:purple">converge pointwise to $0$</span>, <span style="color:red">**but**</span> the <span style="color:purple">$L^2$-norm converges to $0$</span> because the intervals get smaller and smaller
      - this shows that the $L^2$-convergence does not give you pointwise control, not even on a.e. point
        - for most of the part this is not so important and if you go to discrete you do not have these kind of problems, but I stress this here, so that you do not fall for this equality ($\ref{eq:scaling-equation}$) showing more than it is - and that is the reason why I did not put an $(x)$ in ($\ref{eq:scaling-equation}$), like $\phi\color{red}{(x)} = \sum_{k\in\mathbb{Z}} h_k\sqrt{2}\phi(2\color{red}{x} - k)$, because **it would be wrong**! (actually, it was wrong in the older version of the lecture notes) It is very easy to overlook. That is why I am stressing it.
        - similarly, the FS simply does not converge pointwisely, no matter how you interpret this
    - **note**: we will see later that for "nice generators" and "nice wavelets" only a finite amount of these $h_k$s will be non-zero and then all of these \[convergence related\] problems are gone because if this sum here in ($\ref{eq:scaling-equation}$) is **finite** then you do not need to consider convergence because you can compute the finite sum and then you are done - it is just when you have a series ("infinite sum") then you need to be careful! The wavelets that you use <span style="color:red">**in practice**</span> will have only a finite number of $h_k$s that are not zero.

<p align="center">
  <a href="https://ibb.co/Kxb8b3j"><img src="https://i.ibb.co/RTYJYVp/Screenshot-from-2024-03-27-11-53-22.png" alt="Screenshot-from-2024-03-27-11-53-22" border="0"></a>
</p>

- **problem**: with <span style="color:green">(iii)</span> we see a direct connection to the **continuous WT**:

### 4.11 (iv) $(f, \phi_{j,k} )\_{L^2} = L_{\phi} f (2^j , 2^j k)$

- (iv) For $f \in L^2 (\mathbb{R})$, we have $$(f, \phi_{j,k} )_{L^2} = \int_{\mathbb{R}}f(x)2^{-\frac{j}{2}} \phi(2^{-j} x - k) dx = \boxed{\int_{\mathbb{R}}f(x) \sqrt{2^j} \phi \left(\frac{x - 2^j k}{2^j}\right) dx} = L_{\phi} f (2^j , 2^j k).$$ 
  - **problem**: Why did we rewrite $(f, \phi_{j,k} )_{L^2}$ to the boxed formula?
    - This should look familiar from <span style="color:green">(4.1)</span>.
    - The boxed formula is exactly the continuous WT for certain $a$ and $b$.
  - Thus, <span style="color:red">$(f, \phi_{j,k} )\_{L^2}$ is the evaluation of the **continuous WT $L_{\phi} f$** for $a = 2^j$ and $b = 2^j k$</span>.
    - in particular, we can get all of these necessary coefficients $(f, \phi_{j,k} )\_{L^2}$ <span style="color:red">**just by evaluating the WT at certain discrete parameters**</span>
    - **problem**: we first have to understand what these $V_j$s are in order to conclude more from this, but we can already see that there is a strong link to the WT by this
- **problem**: to further work with this I will prepare a helping lemma that tells us sth about these $h_k$

## 4.12 $h_k$s orthogonal to themselves when shifted

- **condition**:
  - Let $h_k$ be the coefficients of the scaling equation of an MRA.
- then $$\sum_{k\in\mathbb{Z}} h_k h_{k+2m} = \delta_{0,m}\ \text{for all}\ m \in \mathbb{Z}$$
- that means, in a certain sense, the scaling equation coeffs are orthogonal to themselves if you shift them by $2m$
- for $m=0$ that means their squared sum is $1$

## 4.13 Prerequisites: Orthogonal Complement, Direct Sum

- <span style="color:red">**outer/direct sum**</span>: [direct sum](https://en.wikipedia.org/wiki/Direct_sum)
- <span style="color:red">**orthogonal complement**</span>:
  - "the set $W^{\perp}$ of all vectors in $V$ that are orthogonal to every vector in $W$"
    - phth: thus, <span style="color:red">$W^{\perp}$ always includes the vector $0$</span> (important for the proof of <span style="color:green">(4.14)</span>)
  - Für einen Unterraum $U \in X$ ist der Unterraum aller Vektoren $$U_\perp = \{ y \in X\ \vert\ x \perp y\ \text{for all}\ x \in U \}$$ das <span style="color:red">**orthogonale Komplement von $U$**</span>., (Bredies)
  - **properties**: [Wikipedia](https://en.wikipedia.org/wiki/Orthogonal_complement#Properties_2)
    - <span style="color:red">$X\cap X^{\perp}=\\{0\\}$</span>

## 4.13 Approximation Space, Detail Space, Wavelet Space

- **conditions**:
  - Let $(V_j)_{j\in\mathbb{Z}}$ be a MRA.
  - Let $W_j$ be such that $V_{j−1} = V_j \oplus W_j$ and $V_j \perp W_j$, ie. $W_j$ is the <span style="color:red">**orthogonal complement**</span> of $V_j$ in $V_{j-1}$.
    - we know that $V_j$ is contained in $V_{j-1}$ and everything that is missing in $V_j$ is concentrated in $W_j$. In a way that we only pick up the orthogonal complements that are missing. That is a nice separation.
- Then, $V_j$ is called <span style="color:red">**approximation space**</span> to the scale $j$
  - because we know that the projection of $f$ to $V_j$ approximates $f$ in a sense that if $j$ goes to $-\infty$ we get $f$ back
- and $W_j$ is called <span style="color:red">**detail space**</span> or <span style="color:red">**wavelet space**</span> to the scale $j$.
  - Because you can think of it, when going from $V_j$ to the larger $V_{j-1}$, then we are adding some stuff and the stuff that we add are just "details" \[from the "detail space"\].
  - If the approximation gets better \[as $j$ gets smaller\], apparently, we have to add things \[functions\] that were missing, ie. that were not representable on these coarser spaces \[$V_j$ with larger $j$\], and that motivates the name "detail space".
- **problem**: so, all of the $W_j$s are enough to represent the signals/all $L^2$ functions. Now, we can iterate this:
 
## 4.14 Splitting $V_j$ into Details, Splitting a $L^2$-Function in all its Details in all Scales $j$

### 4.14 (i) $V_j$ Decomposition into Details

- The definition of $W_j$ implies by iteration that $$\label{eq:Vj-decomposition}V_j \stackrel{(4.13)}{=} V_{j+1} \oplus W_{j+1} \stackrel{(4.13)}{=} V_{j+2} \oplus W_{j+2} \oplus W_{j+1} = \ldots \stackrel{(4.13)}{=} V_{j+M} \oplus \color{blue}{\bigoplus_{m=j+1}^{j+M} W_m}$$ for all $M \geq 1.$
  - Berkels: "<span style="color:red">**outer sum**</span>" ($\color{blue}{\bigoplus_{m=j+1}^{j+M} W_m}$) also means that the intersection of all of these $W_j$s is $\\{0\\}$.
    - **explanation A**: \[because by def. of $W_j$, $W_j$ contains all vectors that are orthogonal to the vectors in $V_j$, thus, since the vector $0$ is orthogonal to all vectors in all $V_j$s, all $W_j$ must include the vector $0$\]
    - **explanation B**: from Lemma below: $V=U\oplus W$ <span style="color:red">**iff**</span> the intersection of $U$ and $W$ contains <span style="color:red">**only**</span> the vector $0$ (see the "<span style="color:red">**iff**</span>" in the Lemma)! In particular, in our case, the intersection of <span style="color:purple">$V_{j+M}$ and all $W_m$s for $j+1\leq m\leq j+M$</span> must be the set containing <span style="color:red">**only**</span> the vector $0$. But this also means that the intersection of <span style="color:purple">all $W_m$s for $j+1\leq m\leq j+M$</span> must be the set containing <span style="color:red">**at least**</span> the vector $0$ (think about a Venn diagram for 3 intersecting sets: intersection area gets larger, when you take away one intersecting set). Later, we will see $V_j = \bigoplus_{m\geq j+1} W_m$ and with this we can say that the intersection of <span style="color:purple">all $W_m$s for $m\geq j+1$</span> must be the set containing <span style="color:red">**only**</span> the vector $0$. This must hold for all $j$. So, it is safe to say "the intersection of all of these $W_j$s is $\\{0\\}$", as Berkels said.

<p align="center">
  <a href="https://ibb.co/ZSsMqM2"><img src="https://i.ibb.co/cw5r4rY/Screenshot-from-2024-03-29-15-11-36.png" alt="Screenshot-from-2024-03-29-15-11-36" border="0"></a><br>
  <a href="https://www.math.umd.edu/~millson/teaching/MATH405spring2020/MATH405lectures/Lecture13directsums.pdf" target="_blank">source</a><br>
</p>

- This immediately implies $V_j \supset \bigoplus_{m\geq j+1} W_m$.
  - because from ($\ref{eq:Vj-decomposition}$) we see that $V_j$ contains this outer sum (in $\color{blue}{blue}$) for all $M$, and in particular, also the infinite summation $M\to\infty$ of all of these outer sums (in $\color{blue}{blue}$)
- **problem**: The question is: Can there be something missing in $\bigoplus_{m\geq j+1} W_m$ or does it have to be <span style="color:red">**all of**</span> entire $V_j$. The answer is, it is all, but we have to show this:
  - Assume there is $v \in V_j \setminus \bigoplus_{m\geq j+1} W_m$. Since $$\label{eq:VjplusM-decomposition}V_j \setminus \left(\bigoplus_{m\geq j+1} W_m\right) \stackrel{(\ref{eq:Vj-decomposition})}{=} \left(V_{j+M} \oplus \left(\color{purple}{\bigoplus_{m=j+1}^{j+M} W_m}\right)\right) \setminus \left(\color{purple}{\bigoplus_{m\geq j+1} W_m}\right) \stackrel{(\text{explanation 1})}{=} V_{j+M} \setminus \left(\bigoplus_{m \geq j+1+M} W_m\right),$$ we have, $v \in V_{j+M}$ for all $M \geq 1$.
    - **explanation 1**: **intuitiv**: das letzte Gleichzeichen gilt, weil Mengen ($\color{purple}{purple}$ in ($\ref{eq:VjplusM-decomposition}$)) zu $V_{j+M}$ addiert "$\oplus$" und subtrahiert "$\setminus$" werden und sich dabei gegenseitig "canceln" ($\color{red}{red}$ in ($\ref{eq:VjplusM-decomposition-explanation}$)) $$\label{eq:VjplusM-decomposition-explanation}V_{j+M}\oplus\{\color{red}{(j+1)+\ldots+(j+M)}\}\setminus\{\color{red}{(j+1)+\ldots+(j+M)}+(j+M+1)+\ldots+(\infty)\}=V_{j+M}\setminus\{(j+M+1)+\ldots+(\infty)\}$$
  - due to the **inclusion property**, this implies $v \in V_M$ for all $M \in \mathbb{Z}$
  - due to the **trivial intersection property**, this implies $v = 0$. This is a contradiction to $v\not\in \bigoplus_{m\geq j+1} W_m \color{blue}{\supset \\{0\\}}$.
    - $\color{blue}{blue}$ part: all of these $W_m$s contain $0$ (recall: **orthogonal complement properties**: intersection of $V_j$ and $W_j$ is $\\{0\\}$)
- (i) Thus, we have shown $$V_j = \bigoplus_{m\geq j+1} W_m.$$
  - thus, "<span style="color:red">the $V_j$ can be split into all details contained for these larger $V_j$ spaces</span>"

### 4.14 (ii) $L^2$ Decomposition into Details

- (ii) Combined with the completeness, we have $$\color{red}{L^2(\mathbb{R})} \stackrel{(\text{completeness prop. of}\ V_j)}{=} \overline{\bigcup_{j\in\mathbb{Z}} V_j} \stackrel{(\text{i})}{=} \overline{\bigcup_{j\in\mathbb{Z}}\bigoplus_{m\geq j+1} W_m} \color{red}{= \overline{\bigoplus_{m\in\mathbb{Z}} W_m}}$$.
  - thus, "<span style="color:red">we can split any $L^2$ function in sums of elements of this $W_m$</span>"
    - in other words: this is like "splitting the function in all its details in all the scales"
- These equations justify the name "**multiscale analysis**": the spaces $V_j$ allow a systematic approximation of functions on different scales.

### 4.14 (iii) Proof of 4.10 (continued)

- (iii) Since $V_j \oplus W_j$ is an <span style="color:red">**orthogonal decomposition of $V_{j−1}$**</span>, we have $$P_{V_{j-1}} \stackrel{(\text{orth. decomposition})}{=} P_{V_j} + P_{W_j} \Rightarrow P_{W_j} = P_{V_{j-1}} - P_{V_j}.$$ Due to $$\label{eq:L2-decomposition}L^2(\mathbb{R}) \stackrel{(\text{ii})}{=} \overline{\color{blue}{\bigoplus_{m\in\mathbb{Z}} W_m}} = \overline{\bigoplus_{m\geq j+1} W_m \oplus \bigoplus_{m\leq j} W_m} \stackrel{(\text{i})}{=} \overline{\color{red}{V_j} \oplus \color{purple}{\bigoplus_{m\leq j} W_m}},$$ $f \in L^2(\mathbb{R})$ can be expressed by $$\label{eq:f-projection-to-V-and-W}f \stackrel{(\ref{eq:L2-decomposition})}{=} \color{blue}{\sum_{m\in\mathbb{Z}} P_{W_m} f} \stackrel{(\ref{eq:L2-decomposition})}{=} \color{red}{P_{V_j} f} + \color{purple}{\sum_{m\leq j} P_{W_m} f}.$$ This implies that <span style="color:red">$P_{V_j} f \to 0$ for $j \to \infty$ holds</span> (because <span style="color:magenta">**explanation 2**</span>), which we claimed earlier (proof of <span style="color:green">(4.10)</span>).
  - <span style="color:magenta">**explanation 2**</span>: "sandwich": the last equality in ($\ref{eq:f-projection-to-V-and-W}$) must hold for all $j$, thus, since ($\color{blue}{blue}$) and ($\color{purple}{purple}$) are equal for $j\to\infty$, this equality can only hold for $j\to\infty$ if $P_{V_j}f$ vanishes
  - **explanation 3**:
    - ($\color{blue}{blue}$ part of ($\ref{eq:f-projection-to-V-and-W}$)) is the <span style="color:purple">orthogonal projection</span> of $f$ to the **orthogonal decomposition of $L^2$** in the form of $\overline{\color{blue}{\bigoplus_{m\in\mathbb{Z}} W_m}}$
    - ($\color{red}{red}+\color{purple}{purple}$ part of ($\ref{eq:f-projection-to-V-and-W}$)) is the <span style="color:purple">orthogonal projection</span> of $f$ to the **orthogonal decomposition of $L^2$** in the form of $\overline{\color{red}{V_j} \oplus \color{purple}{\bigoplus_{m\leq j} W_m}}$
    - related: "Decomposition of a vector space into direct sums is **not unique**.", [Wikipedia](https://en.wikipedia.org/wiki/Projection_(linear_algebra)#Spectrum)
- **problem**: Let's look at our only MRA example again and see what we can get about these projections there:

## 4.15 Example: Piecewise Constant MRA

### 4.15 (i) Projection of $f$ to the Approximation Space $V_j$

- Let $(V_j )j\in\mathbb{Z}$ the piecewise constant MRA from <span style="color:green">(4.9)</span> and let $f \in L^2 (\mathbb{R})$.
- Since the elements of $V_j$ are constant on the intervals $\left[k2^j , (k + 1)2^j \right)$ and $\lVert f − P_{V_j} f\rVert_{L^2} = \inf_{u\in V_j} \lVert f − u\rVert_{L^2}$, we get for $x \in \left[k2^j , (k + 1)2^j \right)$ that (**intuitively**: boils down to approximating a function on an inteval with a single value) $$(P_{V_j} f )(x) = \color{blue}{2^{−j}}\color{red}{\int^{(k+1)2^j}_{k2^j} f(y)dy} = 2^{−j}\int_{\mathbb{R}} \chi_{\left[k2^j ,(k+1)2^j \right)}(y)f(y)dy$$ $$\label{eq:PVj1}(\ldots) = 2^{−\frac{j}{2}}\int_{\mathbb{R}}2^{−\frac{j}{2}}\chi_{\left[0,1\right)}(2^{−j}y − k)f(y)dy = 2^{−\frac{j}{2}}\int_{\mathbb{R}}\phi_{j,k}(y)f(y)dy = (f, \phi_{j,k} )_{L^2} \phi_{j,k}(x)$$.
  - ($\color{red}{red}$) is the mean of $f$ on the dyadic interval $\left[k2^j , (k + 1)2^j \right)$
  - ($\color{blue}{blue}$) is a normalization factor

<p align="center">
  <a href="https://ibb.co/mhZcSN5"><img src="https://i.ibb.co/3S6CkRm/Screenshot-from-2024-03-29-18-42-23.png" alt="Screenshot-from-2024-03-29-18-42-23" border="0"></a>
</p>

- thus, we can compute this projection $P_{V_j}f$ in terms of the generator, ie. in terms of the CONS of $V_j$ given by $\phi_{j,k}$:
- (i) Noting that for any $x \in \mathbb{R}$, we have $\phi_{j,k} (x) = 0$ for all $k \in \mathbb{Z}$ but one, we get $$\label{eq:PVj2}P_{V_j} f = \sum_{k\in \mathbb{Z}} (f, \phi_{j,k} )_{L^2} \phi_{j,k}$$.
  - $P_{V_j} f$ is the "Projection of $f$ to our approximation space $V_j$"
  - **note**: ($\ref{eq:PVj1}$) is for $x$ and ($\ref{eq:PVj2}$) is for <span style="color:red">**all**</span> $x$. So, why is ($\ref{eq:PVj2}$) valid? When going from ($\ref{eq:PVj1}$) to ($\ref{eq:PVj2}$), we have used that for any $x \in \mathbb{R}$, we have <span style="color:red">$\phi_{j,k} (x) = 0$ for all $k \in \mathbb{Z}$ but one</span> 
  - for simplicity, consider a fixed $j'$ first
  - Because \[ for a fixed $j'$ \] these dyadic intervals do not overlap and this $x'$ is only in just one of these dyadic intervals \[ say $x' \in \left[k'2^j,(k'+1)2^j\right)$ \]. Thus, \[for any $x'$,\] the series (the sum over $k\in \mathbb{Z}$) is just a sum with <span style="color:red">one summand \[which is non-zero.</span> This summand is the one with the factor $\phi_{j,k'}(x')$ (because $x' \in \left[k'2^j,(k'+1)2^j\right)$ as assumed above, thus, $\phi_{j,k'}(x') = 2^{-\frac{j}{2}}\chi_{ \left[k'2^j,(k'+1)2^j\right) }(x') = 2^{-\frac{j}{2}} \neq 0$ will always hold).\]
  - from next lec:
    - Berkels: ($\ref{eq:PVj2}$) is a finite sum with only one non-zero coeff. \[phth: must be wrong: probably Berkels means the summands of the series because the coeffs $(f, \phi_{j,k} )\_{L^2}$ themselves cannot be $0$ because the scalar product is an integral over $\mathbb{R}$, thus, the scalar product with $\phi_{j,k}$ "samples" $f$ on $\left[k2^j,(k+1)2^j\right)$ for each $k$ which would be $0$ only if both $f$ and $\phi_{j,k}$ are $0$ on this interval\]
    - ~~This non-zero coeff is $(f, \phi_{j,k'})\_{L^2}$ (ie. the scalar product of $f$ with the scaled $\chi_{ \left[k'2^j,(k'+1)2^j\right) })$.~~
    - ~~The scalar product of $f$ with all other $\chi_{ \left[m2^j,(m+1)2^j\right) }$ with $m \neq k'$ is always zero bec $x'$ is not in $\left[m2^j,(m+1)2^j\right)$, so $\phi_{j,m}(x') = 0$.~~
  - of course, this also follows **from the ONB property of these $\phi_{j,k}$**, but here we actually computed this how it naturally arises from how we chose the generator
- **problem**: now, lets compute the $h_k$ for our specific p.w. constant MRA:

### 4.15 (ii) Compute $h_k$

- Moreover, we have for this MRA that $$ h_k = (\phi, \phi_{−1,k} )_{L^2} = \int_0^1 2^{\frac{1}{2}} \phi(2x − k) dx = \int_0^1 \sqrt{2}\chi_{\left[\frac{k}{2},\frac{k+1}{2}\right)} (x) dx = \begin{cases}
                                       \frac{\sqrt{2}}{2}  & k \in \{0, 1\}\\
                                       0 &\text{else}
                                       \end{cases} = 2^{−\frac{1}{2}} (\delta_{k,0} + \delta_{k,1} ).$$

### 4.15 (iii) Compute Scaling Equation

- Thus, in this case, the <span style="color:red">**scaling equation**</span> simplifies to $$\phi = \sum_{k\in\mathbb{Z}} h_k \sqrt{2}\phi(2 \cdot -k) = \phi(2\cdot) + \phi(2 \cdot -1). $$
  - ie. <span style="color:red">$\phi$ can be represented by shifted, scaled copies of itself</span> (which is exactly what the scaling equation said, but here we now specifically see which two translations and scalings we need
  - thus, the series becomes a **finite sum**!

### 4.15 (iv) Compute $\phi_{j,k}$

- Composing this with $x \mapsto 2^{-j} x - k$, this shows $$\phi(2^{-j} \cdot -k) = \phi(2^{-(j-1)} \cdot -2k) + \phi(2^{-(j-1)} \cdot -(2k + 1)).$$ Multiplying both sides with $2^{-\frac{j}{2}}$ shows $$\label{eq:phi-jk}\phi_{j,k} = \frac{1}{\sqrt{2}} (\phi_{j-1,2k} + \phi_{j-1,2k+1} ).$$
  - ie. each of these $\phi_{j,k}$ we can represent as linear combination of two of the $\phi_{j-1,\ldots}$, so from one finer level
- **problem**: we have already checked $P_{V_j}$ above (the projection to approximation spaces), now, we check $P_{W_j}$ (the projection to detail spaces)

### 4.15 (v) Compute Projection to $W_j$

- This allows us to compute <span style="color:red">**the projection to $W_{j+1}$**</span>. We get $$P_{W_{j+1}} f \stackrel{(\text{EA})}{=} P_{V_j} f − P_{V_{j+1}} f \stackrel{(\text{EB})}{=} \sum_{k\in\mathbb{Z}} (f, \phi_{j,k} )_{L^2} \phi_{j,k} − \sum_{k\in\mathbb{Z}} (f, \phi_{j+1,k} )_{L^2} \phi_{j+1,k}$$
  - **EA**: for this, lets recall our def. of these $W_j$s: so, $V_{j-1} = V_j \oplus W_j$ <span style="color:green">(4.13)</span> implies that the projection to $V_{j-1}$ is a projection to $V_j$ plus a projection to $W_j$ and if I just solve for the projection to $W_j$ then I have to subtract $V_j$ on both sides, so we get this result $P_{V_j}f - P_{V_{j+1}}f$ here
  - **EB**: we computed both $P_{V_j}$ and $P_{V_{j+1}}$ last time (see beginning of <span style="color:green">(4.15)</span>: $P_{V_j}f = \ldots$)
- Splitting the first sum in **even** and **odd** indices and using ($\ref{eq:phi-jk}$) for the second sum, leads to

<p align="center">
  <a href="https://ibb.co/z5bR1wh"><img src="https://i.ibb.co/qn9szwM/Screenshot-from-2024-03-29-19-47-41.png" alt="Screenshot-from-2024-03-29-19-47-41" border="0"></a>
</p>

### 4.15 (vi) Wavelet Corresponding to the Generator

- what we introduce now is actually the <span style="color:red">**wavelet that is corresponding to the generator**</span>.
  - recall:
    - if you think about <span style="color:green">(4.1)</span> where we had the continuous WT, we started with this wavelet $\psi$ that we used to multiply our function $f$ with for the WT and integrate
    - but here we started with a generator that is called $\phi$ (not $\psi$) and is not called "wavelet" but "generator"
    - and now comes the connection back to the wavelet that we had before. This is what this $\psi$ will be.
- Introducing $$\psi(x) := \phi(2x) - \phi(2x - 1)\ \text{and}\ \psi_{j,k} (x) := 2^{-\frac{j}{2}} (2^{-j}x - k)$$ we get $\psi_{j+1,k} = \frac{1}{\sqrt{2}} (\phi_{j,2k} - \phi_{j,2k+1} )$ and the representation of $P_{W_{j+1}} f$ simplifies to $$\label{eq:PWf}P_{W_{j+1}} f = \sum_{k\in\mathbb{Z}} (f, \psi_{j+1,k} )_{L^2} \psi_{j+1,k}$$ .
  - define scaled and translated versions of this $\psi$, exactly like we did with $\phi$
  - same scaling as for $\phi_{j,k}$
- This $\psi$ we already know, you just have to recognize it:
- The function $\psi$ is the <span style="color:red">**Haar wavelet**</span> we have seen before in <span style="color:green">(4.7 (iii))</span>.
  - Because $\phi(2x)$ is the characteristic function on the interval $\left[0,\frac{1}{2}\right)$ and, accordingly, $\phi(2x-1)$ is the characteristic function on the interval $\left[\frac{1}{2},1\right)$. This means the function $\psi$ is $1$ on the interval $\left[0,\frac{1}{2}\right)$ and $-1$ on the interval $\left[\frac{1}{2},1\right)$ which is exactly what was the Haar wavelet!

### 4.15 (vii) $\\{\psi_{j,k} : k \in \mathbb{Z} \\}$ is an ONB of $W_j$

- Moreover, we have $$\label{eq:orthonormal}(\psi_{j,k} , \psi_{j,m})_{L^2} = \frac{1}{2}((\phi_{j−1,2k} − \phi_{j−1,2k+1}) , (\phi_{j−1,2m} − \phi_{j−1,2m+1}))_{ L^2 } = \frac{1}{2} (\delta_{k,m} + 0 + 0 + \delta_{k,m} ) = \delta_{k,m}$$ .
  - this means that the $\psi_{j,k}$ that we get are also an ONS
- Combined with the representation of $P_{W_{j+1}}$ we derived above ($\ref{eq:PWf}$) and that $P_{W_{j+1}}g=g$ (for $g\in W_{j+1}$ for all $j$) is the identity on $W_{j+1}$ , this means that <span style="color:red">$\\{\psi_{j,k} : k \in \mathbb{Z} \\}$ is an orthonormal basis of $W_j$</span> .
  - $P_{W_{j+1}}g=g$ gilt für alle $f\in W_{j+1}$ und heißt, dass ($\ref{eq:PWf}$) für alle $f\in W_{j+1}$ zeigt, dass $f$ durch die Operation $P_{W_j}$ nicht verändert wird. ($\ref{eq:PWf}$) kann also auf alle $f\in W_{j+1}$ angewendet werden und kann also alle $f\in W_{j+1}$ als Linearkombination darstellen. $\Rightarrow$ completeness of ONS $\\{\psi_{j,k}\\}$.
  - (i) "orthonormality of ONS" follows from ($\ref{eq:orthonormal}$)
  - (ii) "each element $f$ of $W_{j+1}$ can be represented by $P_{W_{j+1}}$ as a projection in this space" follows from the $P_{W_{j+1}}g=g$ argument
  - (iii) "With $P_{W_{j+1}}$ we can represent any element $f \in W_{j+1}$ as linear combination of the $\psi_{j+1,k}$s" follows from ($\ref{eq:PWf}$) and (ii) ("completeness of ONS")
  - phth:
    - (ii), (iii) müssen für alle $j$ gelten: 
      - zB. mit index shift $j-1$ wird aus (iii):
      - "With $P_{W_{j}}$ we can represent any element $f \in W_{j}$ as linear combination of the $\psi_{j,k}$s"
    - bestätigt er auch später: "the $P_{W_{j+1}}g=g$ argument holds for all $j$, it is not only for $j+1$, but I can insert $j-1$ for $j$. It is somewhat confusing because of all of these $j$ and $j+1$ and we are jumping back and forth between these, but $j$ is arbitrary."
- The existence of a wavelet that generates an orthonormal basis for $W_j$ not only holds in the special case here, but more generally.
  - so, this general construction that we can start with the generator $\phi$, then use the scaling equation and all these combinations to construct the $\psi$ and then get CONSes for all our $W_j$s, is not linked just to the Haar wavelet, but this is a very general connection.

## 4.16 $\psi$ is a Wavelet, $\\{\psi_{j,k}\\}$ is an ONB/CONS of $W_j$ and $L^2$

- **conditions**:
  - let $(V_j )_{j\in\mathbb{Z}}$ be a MRA with generator $\phi$
  - let $h_k$ be the coefficients of the scaling equation.
  - let $\psi \in V_{−1}$ be defined by $\psi := \sum_{k\in\mathbb{Z}} (−1)^k h_{1−k} \phi_{−1,k}$ . 
    - this $\psi$ may look surprising, but if we insert the p.w.c. MRA we only have $h_0$ and $h_1$ not equal to $0$ and actually, both times it is just $\sqrt{2}/2$ (see <span style="color:green">(4.15 (ii))</span>) and then we see that we get exactly the combination of characteristic function of the interval $\left[0,1/2\right]$ which just comes from the $k=0$ minus the characteristic function of $\left[1/2,1\right]$ which comes from $k=1$, where we also get this minus here from the $(-1)^k$. So, for the one case that we computed this $\psi$ here is exactly the $\psi$ that we had above, but here it is now told in general.
- Then,
  - (i) <span style="color:red">$\\{\psi_{j,k} : k \in Z\\}$ is an orthonormal basis of $W_j$.</span>
    - which we showed for our specific generator $\phi$, but it holds for all generators
  - (ii) <span style="color:red">$\\{\psi_{j,k} : j, k \in Z\\}$ is an orthonormal basis of $L^2 (\mathbb{R})$.</span>
    - We already saw last time that $L^2$ is the closure of the sum of all the $W_j$s. Thus, if we have <span style="color:green">(i)</span> then <span style="color:green">(ii)</span> follows from what we already know.
  - (iii) <span style="color:red">$\psi$ is a wavelet with $c_\psi = \ln {2}$.</span>
    - again, with the same scaling as above
    - which is now the promised connection in its generality. So, for a function to be a wavelet we needed this corresponding constant $c_\psi$ to be bigger than $0$ and smaller than $\infty$ (this was for the continuous WT). In this case we can even exactly tell what this constant is for the $\psi$ and it is always the same if this $\psi$ is constructed from an MRA and its generator. So, in essence, this means that <span style="color:red">**"for any MRA we get a corresponding wavelet"**</span>.
- **proof**:
  - we will use similar techniques that we used in the specific case above where we did the computation for a very specific MRA (the p.w.c. MRA).
  - Lets see if there are any unforeseen things happening on the way

## 4.17 Example: Wavelet Pairs $\phi,\psi$

### 4.17 (i) Piecewise Constant MRA, Haar Wavelet

- Let $(V_j )_{j\in\mathbb{Z}}$ the **piecewise constant MRA** from <span style="color:green">(4.9)</span>.
- Recalling, $h_k = 2^{-\frac{1}{2}} (\delta_{k,0} +\delta_{k,1} )$, we get $$\psi = \sum_{k\in\mathbb{Z}} (−1)^k h_{1−k} \phi_{−1,k} = (\ldots) = -\chi_{\left[\frac{1}{2},1\right)}+\chi_{\left[0,\frac{1}{2}\right)}$$ which is again the <span style="color:red">**Haar wavelet**</span>

### 4.17 (ii) Daubechies Wavelets

- Constructing <span style="color:red">wavelets that both have compact support and have some smoothness</span> is highly non-trivial.
- The **discovery** of compact, regular and orthogonal wavelets by **Ingrid Daubechies in 1988** is one of the reasons why wavelets became very popular in the 90s-era.
- The <span style="color:red">**Daubechies wavelets**</span> are a family of <span style="color:red">**orthogonal**</span> wavelets that are characterized by a <span style="color:red">**maximal number $k$ of vanishing moments**</span>, ie. $$\int_{\mathbb{R}}x^l \psi(x) dx = 0\ \text{for}\ l \in \{0, \ldots , k - 1\},$$ for a given support.
- Perhaps the **most famous** wavelet pair $\phi, \psi$ is the <span style="color:green">**Daubechies wavelet with two vanishing moments**</span>, denoted by `db2`.
- Interestingly, the <span style="color:green">**Daubechies wavelet with one vanishing moment**</span>, `db1`, is <span style="color:red">**equivalent to the Haar wavelet**</span>.
- The <span style="color:purple">**values of the coefficients $h_k$**</span> are available as <span style="color:purple">**tabulated numerical values**</span>. <span style="color:purple">**For `db2`**</span>, the values are also available <span style="color:purple">**analytically**</span> (see notes)

<p align="center">
  <a href="https://ibb.co/BLctnVm"><img src="https://i.ibb.co/hsLyYRk/Screenshot-from-2024-03-30-15-41-55.png" alt="Screenshot-from-2024-03-30-15-41-55" border="0"></a>
</p>

### 4.17 (iii) Symlets, Daubechies' Least-Asymmetric Wavelets

- A **modification of the Daubechies wavelets** are the <span style="color:red">**symlets**</span>, which are sometimes also called <span style="color:red">**Daubechies' least-asymmetric wavelets**</span>.
- As the name suggests, these wavelets are <span style="color:red">more symmetric than the Daubechies wavelets</span> and <span style="color:red">also have vanishing moments</span>.
- For instance, the <span style="color:green">**symlet with four vanishing moments**</span> is denoted by `sym4`.
- Note that `db2` and `sym2` <span style="color:red">**are equivalent**</span>.

# 4.3 Fast Wavelet Transform

- (i) Due to <span style="color:green">(4.16)</span>, we know that
  - <span style="color:red">$\psi$ constructed from an MRA is a wavelet</span> and
  - <span style="color:red">$\\{\psi_{j,k} : j, k \in \mathbb{Z}\\}$ is an orthonormal basis of $L^2 (\mathbb{R})$</span>.
  - In particular, we have $$f = \sum_{j,k\in\mathbb{Z}} (f, \psi_{j,k} )_{L^2} \psi_{j,k}.$$
- (ii) Recalling $(f, \psi_{j,k} )\_{L^2} = L_\psi f (2^j , 2^j k)$, <span style="color:green">(i)</span> shows that <span style="color:red">it is sufficient to know $L_\psi f$ on the discrete set $\\{(2^j , 2^j k) : j, k \in \mathbb{Z}\\} \subset \left[0, \infty\right) \times \mathbb{R}$ to encode $f$</span>.
- (iii) In the sense of <span style="color:green">(ii)</span>, <span style="color:red">the coefficients $(f, \psi_{j,k} )\_{L^2}$ are **the discrete wavelet transform of $f$**</span>.
- (iv) The key to a <span style="color:red">**fast wavelet transform**</span> is to use that those <span style="color:red">coefficients can be computed recursively</span>, which is a consequence of the scaling equation and the definition of $\psi$:

## 4.18 FWT Recursion Formulas: Computing the Coefficients $(f, \psi_{j,k} )\_{L^2}$ Recursively

- basically, the same as <span style="color:green">(4.19)</span>, but <span style="color:green">(4.19)</span> is the short notation of <span style="color:green">(4.18)</span>
- Herleitung der <span style="color:red">**FWT Recursion Formulas**</span> <span style="color:green">(4.19)</span>:

<p align="center">
  <a href="https://ibb.co/PY0m18N"><img src="https://i.ibb.co/VN5jM8t/Screenshot-from-2024-03-30-16-28-44.png" alt="Screenshot-from-2024-03-30-16-28-44" border="0"></a>
</p>

## 4.19 FWT: Compute $c^{j+1}$ and $d^{j+1}$ from $c^j$

- <span style="color:red">**FWT Recursion Formulas**</span>:

<p align="center">
  <a href="https://ibb.co/NrfccmB"><img src="https://i.ibb.co/G9GzztS/Screenshot-from-2024-03-30-16-21-50.png" alt="Screenshot-from-2024-03-30-16-21-50" border="0"></a>
</p>

- The FWT uses these <span style="color:green">**FWT Recursion Formulas**</span> to
  - <span style="color:red">**compute**</span> from $P_{V_j} f$ (given in form of the coefficient vector $c^j$ ) the coarser projection $P_{V_{j+1}} f$ (as the coefficient vector $c^{j+1}$ ) and the wavelet component $P_{W_{j+1}} f$ (as the coefficient vector $d_{j+1}$).
- Note that the sums are finite in case the scaling equation coefficient sequence only has finitely many nonzero entries.
- If only few are nonzero (e. g. for the Haar wavelet only $h_0$ and $h_1$ are nonzero), only few computations have to be done when going from $j$ to $j + 1$.

## 4.20 Invert the FWT: Compute $c^j$ from $c^{j+1}$ and $d^{j+1}$

- invert the (fast) wavelet transform, ie. we need to be able to
  - <span style="color:red">**compute**</span> $P_{V_j} f$ <span style="color:red">**from**</span> <span style="color:purple">the coarser projection $P_{V_{j+1}} f$</span> <span style="color:red">**and**</span> the <span style="color:purple">details $P_{W_{j+1}} f$</span>

## 4.21 Compact Notation of the FWT and IFWT (Algorithm)

## 4.22 $H$ and $G$ can be expressed as Convolutions

## 4.23 Coefficients $c_j = ((f, \phi_{j,k} )\_{L_2} )k∈Z$ for one $j \in \mathbb{Z}$, ie. the representation of our signal $f$ on an initial scale $j$

- it is sufficient, if we can <span style="color:red">**sample**</span> our signal <span style="color:red">**$f$ at equidistant points**</span> as long as the distance between neighboring sample points is sufficiently small.
- the FWT for decomposition depth $M$ can be computed with $\mathcal{O}(nN )$, which is even faster than the FFT, if $n$ is small. Similarly, one can show that the IFWT is also $\mathcal{O}(nN )$.

## 4.24 2D Discrete Wavelet Transform

- There are numerous ways to construct a two-dimensional wavelet transform.
- The easiest way is to extend the one-dimensional wavelet transform by using the tensor product.

## 4.25 JPEG, JPEG2000

- <span style="color:red">**Wavelets**</span> have <span style="color:red">**countless of applications**</span>, both in theoretical and in practical work. Here, we just confine to briefly mention **one** practical application.
- Like the DCT, the <span style="color:red">**FWT (Fast Wavelet Transform)**</span> can be used <span style="color:red">**for image compression**</span> by only storing the most important wavelet coefficients.
- In <span style="color:red">**JPEG2000**</span>, the block-wise DCT is replaced by a wavelet transform, which results in significantly improved compression rates, ie. **better image quality** using the **same amount of data** or **same image quality** using **less data**.
- While the <span style="color:red">**classical JPEG**</span> format is still <span style="color:red">**much more popular**</span> in digital photography than JPEG2000, digital cinema uses JPEG2000 as basis for video compression.
