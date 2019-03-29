# Algorithms in Standard Template Library

This directory contains sample usage of algorithms available in `<algorithm>`.

[Online resource](http://www.cplusplus.com/reference/algorithm/)

# Check all elements
They take iterators and a lambda function to evaluate each element.

- `all_of` (c++11)
- `any_of` (c++11)
- `none_of` (c++11)
- `for_each`
  processes each element by a given lambda function.
- `count`
  counts the number of elements equal to a given value
- `count_if`
  counts the number of elements giving True based on a given lambda function

# Find an element
- `find`
  finds a value
- `find_if`
  finds an element based on a given lambda function
- `find_if_not` (c++11)
- `find_first_of`
  finds an element based on a given set
- `adjacent_find`
  finds an element whose neighbor is equal to it.

# Fund a subsequence
- `search`
  finds the first occurrence
- `find_end`
  finds the last occurrence
- `search_n`
  finds the first n-times repeated elements

# Comparison of two containers
- `equal`
- `mismatch`
  *returns an iterator representing the first mismatch*
- `is_permutation` (c++11)

# Modification
- `copy`
- `copy_n` (c++11)
- `copy_if` (c++11)
- `copy_backward`
- `move` (c++11)
- `move_backward` (c++11)
- `swap`
- `swap_ranges`
- `iter_swap`

- `transform`

- `replace`
- `replace_if`
- `replace_copy`
- `replace_copy_if`
- `fill`
- `fill_n`

- `generate`
- `generate_n`

- `random_shuffle`
- `shuffle` (c++11)

- `remove`
- `remove_if`
- `remove_copy`
- `remove_copy_if`
- `unique`
- `unique_copy`

- `reverse`
- `reverse_copy`

- `rotate`
- `rotate_copy`


- `is_partitioned` (c++11)
- `partition`
- `stable_partition`
- `partition_copy` (c++11)
- `partition_point` (c++11)

- `sort`
- `stable_sort`
- `partial_sort`
- `partial_sort_copy`
- `is_sorted` (c++11)
- `is_sorted_until` (c++11)
- `nth_element`

- `lower_bound`
- `upper_bound`
- `equal_range`
- `binary_search`

# Sets
- `merge`
  *copies the merged sequence to another container*
- `inplace_merge`
  *merges two sequences adjacent to each other*
- `includes`
  A ⊃ B
- `set_union`
  A ∪ B
- `set_intersection`
  A ∩ B
- `set_difference`
  A - (A ∩ B)
- `set_symmetric_difference`
  (A ∪ B) - (A ∩ B)

# Heap
A sequence of numbers is organized as a heap.
- `make_heap`
  organizes a container so its elements are disposed in a heap.
- `pop_heap`
  pop the maximum value from the heap and place it to the end of the container.
- `push_heap`
  push the element at the end to the heap.
- `sort_heap`
  performs the heap sort (?)
- `is_heap`
  True if the container is organized as a heap.
- `is_heap_until`
  returns an iterator pointing to the element which breaks the rule of heap.

