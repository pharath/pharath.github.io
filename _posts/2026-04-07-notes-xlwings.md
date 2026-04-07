---
title: "Xlwings Notes"
read_time: false
excerpt_separator: "<!--more-->"
toc: true
toc_sticky: true
categories:
  - Notes
tags:
  - xlwings
  - notes
---

## Why is `new_df = new_df.sort_index(kind='mergesort')` necessary?

Short answer: **it’s there to make `keep="first"` / `keep="last"` deterministic and time-correct.**

---

## 🔹 What problem it solves

When you do:

```python
new_df = pd.concat([self.df, other.df], axis=0)
```

the resulting order is:

* all rows from `self.df`
* then all rows from `other.df`

👉 This is **not guaranteed to be time-ordered**

---

## 🔥 Why that matters

Your duplicate handling depends on order:

```python
new_df.index.duplicated(keep="last")
```

👉 “last” means:

> last **in the current row order**, not last in time

---

## ❌ Without sorting (buggy behavior)

Imagine:

```python
self.df:
2024-01-02 → A

other.df:
2024-01-01 → B
2024-01-02 → C
```

After concat:

```
2024-01-02 → A
2024-01-01 → B
2024-01-02 → C
```

Now:

```python
keep="last"
```

keeps **C**, but:

* ordering is inconsistent
* time series is not sorted
* logic becomes fragile

---

## ✅ With sorting

```python
new_df = new_df.sort_index(kind="mergesort")
```

Result:

```
2024-01-01 → B
2024-01-02 → A
2024-01-02 → C
```

Now:

* duplicates are **grouped**
* “last” means **latest occurrence within same timestamp group**
* behavior is consistent and predictable

---

## 🔹 Why `mergesort` specifically?

```python
sort_index(kind="mergesort")
```

Because:

* ✅ **stable sort**
* preserves original order **within equal indices**

👉 This is critical:

If two rows share the same timestamp:

* their relative order (self vs other) is preserved
* so `keep="last"` reliably prefers `other`

---

## ❗ Without stable sort (e.g. quicksort)

Pandas might reorder equal timestamps unpredictably:

👉 then:

* sometimes `self` wins
* sometimes `other` wins
  💥 exactly the issue you described earlier

---

## 🔑 Key insight

This line ensures:

> “For equal timestamps, preserve concat order → so `other` overwrites `self`”

---

## 🧠 Summary

```python
new_df = new_df.sort_index(kind="mergesort")
```

is necessary because it:

1. Sorts time series chronologically
2. Groups duplicate timestamps together
3. Preserves order between duplicates (**critical**)
4. Makes `keep="last"` deterministic

---

## 🚀 Alternative (if you want to avoid sorting dependency)

If your intent is strictly:

> “other overrides self”

You could skip sorting entirely and do:

```python
new_df = self.df.copy()
new_df.update(other.df)
```

But:

* ❌ doesn’t handle non-overlapping rows the same way
* ❌ not equivalent to concat
