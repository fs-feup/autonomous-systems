#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

template <typename T>
class CircularBuffer {
public:
  explicit CircularBuffer(size_t capacity = 0)
      : capacity_(capacity), head_(0), size_(0), buffer_(capacity) {}

  void set_capacity(size_t capacity) {
    capacity_ = capacity;
    buffer_.resize(capacity);
    clear();
  }

  void clear() {
    head_ = 0;
    size_ = 0;
  }

  inline size_t size() const { return size_; }
  inline size_t capacity() const { return capacity_; }
  inline bool empty() const { return size_ == 0; }
  inline bool full() const { return size_ == capacity_; }

  /** Add item into circular buffer */
  void push(const T& item) {
    if (capacity_ == 0) return;

    buffer_[head_] = item;
    head_ = (head_ + 1) % capacity_;

    if (size_ < capacity_) size_++;
  }

  /** Access the most recent item */
  const T& latest() const {
    if (size_ == 0) throw std::out_of_range("CircularBuffer::latest - buffer empty");

    size_t index = (head_ + capacity_ - 1) % capacity_;
    return buffer_[index];
  }

  /** Access i-th element from the end (0 = latest, 1 = previous, ...) */
  const T& from_end(size_t i) const {
    if (i >= size_) throw std::out_of_range("CircularBuffer::from_end - index too large");

    size_t index = (head_ + capacity_ - 1 - i) % capacity_;
    return buffer_[index];
  }

  /** Iterate raw access */
  const T& operator[](size_t index) const {
    if (index >= size_) throw std::out_of_range("CircularBuffer::operator[] - index too large");

    size_t real_index = (head_ + capacity_ - size_ + index) % capacity_;
    return buffer_[real_index];
  }

private:
  size_t capacity_;
  size_t head_;
  size_t size_;
  std::vector<T> buffer_;
};
