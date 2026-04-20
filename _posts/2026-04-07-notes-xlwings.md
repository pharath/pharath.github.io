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

## xlwings Functions

- `examples_write_to_sheet()`
- `examples_read_from_sheet()`
- `example_get_user_input_text()`
- `example_get_user_input_range_as_dataframe()`
- `example_get_user_input_range_as_array()`
- `load_timeseries(kind: str, curve_name: xw.Range, start_date: Optional[datetime], end_date: Optional[datetime], date_format: Optional[str], force_reload: Optional[bool])`
- `load_timeseries_excel(curve_name: xw.Range, start_date: Optional[datetime], end_date: Optional[datetime], force_reload: Optional[bool])`
- `load_timeseries_volue(curve_name: xw.Range, start_date: Optional[datetime], end_date: Optional[datetime], force_reload: Optional[bool])`
- `load_timeseries_datacenter(curve_name: xw.Range, start_date: Optional[datetime], end_date: Optional[datetime], force_reload: Optional[bool])`
- `load_timeseries_csv(curve_name: xw.Range, start_date: Optional[datetime], end_date: Optional[datetime], force_reload: Optional[bool])`
- `load_timeseries_enmacc(curve_name: xw.Range, start_date: Optional[datetime], end_date: Optional[datetime], force_reload: Optional[bool])`
- `load_requests_enmacc()`
- `load_concluded_requests_enmacc(concluded_from: datetime, concluded_to: datetime)`
- `write_timeseries_to_sheet(rng: xw.Range, n: Optional[int], start_date: Optional[datetime], end_date: Optional[datetime])`
- `describe_data(rng: xw.Range)`
- `list_cached_sources()`
- `clear_cache(return_value_in_cell: Optional[str]) -> str`
- `add_price_timeseries(ts1_id: xw.Range, ts2_id: xw.Range, floor: Optional[bool])`
- `subtract_price_timeseries(ts1_id: xw.Range, ts2_id: xw.Range, floor: Optional[bool])`
- `multiply_price_timeseries(ts1_id: xw.Range, ts2_id: xw.Range)`
- `concat_price_timeseries(ts1_id: xw.Range, ts2_id: xw.Range, duplicate_strategy: str)`
- `CORREL2(df)`

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
