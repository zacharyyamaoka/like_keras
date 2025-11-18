# #!/usr/bin/env python3

# """
# Tests for common_tasks.py queue functionality.
# """

# # PYTHON
# import pytest
# import time
# from collections import deque

# # BAM
# from bam_diagonistics.diagonstic_helper import make_queue, TimedQueue

# # seting value of 1 to be about the time it takes to append a value..

# # Time scale for faster testing - set to 0.001 for very fast tests
# TIME_SCALE = 0.001


# def test_deque_max_length():
#     """Test that deque respects maxlen constraint."""
#     # Create a deque with maxlen=3
#     queue = make_queue(3, "count")
#     assert isinstance(queue, deque)
#     assert queue.maxlen == 3

#     # Add 5 items, should only keep last 3
#     for i in range(5):
#         queue.append(i)

#     # Should only have last 3 items
#     assert len(queue) == 3
#     assert list(queue) == [2, 3, 4]


# def test_timed_queue_max_age():
#     """Test that TimedQueue respects max_age_seconds constraint."""
#     # Create a timed queue with scaled max age
#     max_age = 3 * TIME_SCALE
#     queue = make_queue(max_age, "timed")
#     assert isinstance(queue, TimedQueue)

#     # Add items quickly
#     queue.append(1)
#     queue.append(2)
#     queue.append(3)

#     # All items should still be present (added quickly)
#     assert len(queue) == 3
#     assert list(queue) == [1, 2, 3]

#     # Wait for items to expire
#     time.sleep(4 * TIME_SCALE)  # Wait for items to expire

#     # Queue should be empty after expiration
#     assert len(queue) == 0
#     assert list(queue) == []


# def test_timed_queue_partial_expiration():
#     """Test that TimedQueue correctly handles partial expiration."""
#     # Create a timed queue with scaled max age
#     max_age = 3 * TIME_SCALE
#     queue = make_queue(max_age, "timed")

#     # Add first item
#     queue.append(1)
    
#     # Wait a bit, then add second item
#     time.sleep(2 * TIME_SCALE)  # Wait between items
#     queue.append(2)
    
#     # Wait for first item to expire but second to remain
#     time.sleep(2 * TIME_SCALE)  # Wait for first item to expire

#     # Only second item should remain
#     assert len(queue) == 1
#     assert list(queue) == [2]


# def test_timed_queue_indexing():
#     """Test that TimedQueue indexing works correctly."""
#     queue = make_queue(10 * TIME_SCALE, "timed")  # Use scaled timeout for this test

#     # Add items
#     queue.append(10)
#     queue.append(20)
#     queue.append(30)

#     # Test indexing
#     assert queue[0] == 10
#     assert queue[1] == 20
#     assert queue[2] == 30
#     assert queue[-1] == 30

#     # Test slicing
#     assert list(queue[0:2]) == [10, 20] # type: ignore


# def test_timed_queue_iteration():
#     """Test that TimedQueue iteration works correctly."""
#     queue = make_queue(10 * TIME_SCALE, "timed")  # Use scaled timeout for this test

#     # Add items
#     queue.append(100)
#     queue.append(200)
#     queue.append(300)

#     # Test iteration
#     items = list(queue)
#     assert items == [100, 200, 300]


# def test_make_queue_interface():
#     """Test that make_queue creates correct queue types."""
#     # Test count-based queue
#     count_queue = make_queue(5, "count")
#     assert isinstance(count_queue, deque)
#     assert count_queue.maxlen == 5

#     # Test timed queue
#     timed_queue = make_queue(1.0, "timed")
#     assert isinstance(timed_queue, TimedQueue)
#     assert timed_queue.max_age == 1.0


# def test_queue_edge_cases():
#     """Test edge cases for both queue types."""
#     # Test empty deque
#     count_queue = make_queue(3, "count")
#     assert len(count_queue) == 0
#     assert list(count_queue) == []

#     # Test empty timed queue
#     timed_queue = make_queue(10 * TIME_SCALE, "timed")  # Use scaled timeout for this test
#     assert len(timed_queue) == 0
#     assert list(timed_queue) == []

#     # Test single item in deque
#     count_queue.append(42)
#     assert len(count_queue) == 1
#     assert list(count_queue) == [42]

#     # Test single item in timed queue
#     timed_queue.append(42)
#     assert len(timed_queue) == 1
#     assert list(timed_queue) == [42]


# if __name__ == "__main__":
#     pytest.main([__file__])  