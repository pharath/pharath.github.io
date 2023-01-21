---
title: "Algorithms Notes"
read_time: false
excerpt: "For learning Algorithms."
header:
  teaser: /assets/images/C_logo.png
  overlay_image: /assets/images/C_logo.png
  overlay_filter: 0.5 
toc: true
toc_label: "Contents"
toc_sticky: true
categories:
  - Notes
tags:
  - algorithms
  - notes

---

## Decision Problems

### N-th Prime

[source](https://qr.ae/prYAPn)

NP technically refers to decision problems, so let's give a formal definition of the NTH-PRIME decision problem:

NTH-PRIME( n, X ) is true iff the nth prime is equal to X.

Is the **Sieve of Eratosthenes** a polynomial-time algorithm for this decision problem? No, because the input is of size $\log_2 n+\log_2 X \approx 2\log_2 n+\log_2 \log n$. But, the sieve takes time $\mathcal{O}(n\log{n}\log{\log{n}})$, which is **exponential in the input size**.

There are **more efficient methods**, see Most efficient algorithm for nth prime, deterministic and probabilistic?
The Meissel-Lehmer method runs in time $\mathcal{O}(n^{2/3})$. But note that this is still exponential in input size log2n. A paper by Lagarias and Odlyzko claims $\mathcal{O}(n^{1/2})$: Computing π(x): An analytic method.

Is this formulation of the problem even in NP? Well, a naive "certificate" that X is the nth prime requires a list of the previous n−1 primes. That takes space $\mathcal{O}(n\log n)$, which is exponential in the input size as well, at least in the "hard" cases. Meissel-Lehmer lets you do better, but not enough to make this polynomial, I think. Relaxing our problem to require the nth prime $\leq X$ instead does not improve matters. The appropriate complexity category for this problem is actually `#P` (Sharp-P) since the answer cannot be verified in polynomial time.

**Nobody can definitively say that NTH-PRIME is not in P**, of course, but all the best known solutions today require time exponential in the length of the input (but sub-linear in the value of the input.)

For practical purposes, you can use a precomputed table that gets you into the right range, then sieve on the range: see The Nth Prime Algorithm
But this only works up to the maximum size of the precomputed table.
