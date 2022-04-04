#pragma once

#include <vector>

template <typename T>
class CircularBuffer {
 public:
  CircularBuffer(){};
  CircularBuffer(size_t size) { data_.reserve(size); }
  void reserve(const size_t new_capacity) {
    // TODO: If making smaller, would lose newer points...
    data_.reserve(new_capacity);
    if (size() > 0) {
      head_ = head_ % size();
      tail_ = tail_ % size();
    }
  }
  bool empty() const { return head_ == tail_ && !full(); }
  bool full() const { return data_.size() == data_.capacity(); }
  size_t size() const { return data_.size(); }
  size_t capacity() const { return data_.capacity(); }

  void push_back(const T& val) {
    if (full()) {
      data_[tail_] = val;
      if (++tail_ == 0) {
        tail_ = 0;
      }
      if (++head_ == 0) {
        head_ = 0;
      }
    } else {
      data_.push_back(val);
      head_++;
    }
  }

  T& operator[](const size_t index) { return data_[GetShiftedIndex(index)]; }
  const T& operator[](const size_t index) const {
    return data_[GetShiftedIndex(index)];
  }
  void clear() {
    data_.clear();
    head_ = 0;
    tail_ = 0;
  }
  bool operator==(const CircularBuffer<T>& other) const {
    return &other == this;
  }

  template <typename CB>
  class Iterator {
   public:
    using difference_type = int64_t;
    using value_type = std::remove_const<T>;
    using iterator_category = std::random_access_iterator_tag;
    using pointer = const T*;
    using reference = T&;

    Iterator(CB& data, const size_t i) : data_(&data), i_(i) {}
    Iterator(const Iterator<CB>& other) : data_(other.data_), i_(other.i_) {}

    Iterator<CB>& operator=(const Iterator<CB>& other) {
      data_ = other.data_;
      i_ = other.i_;
      return *this;
    }
    Iterator<CB>& operator+=(const difference_type i) {
      i_ += i;
      return *this;
    }
    Iterator<CB>& operator-=(const difference_type i) {
      i_ -= i;
      return *this;
    }
    const T& operator*() const { return (*data_)[i_]; }
    const T* operator->() const { return &(*data_)[i_]; }
    const T& operator[](const difference_type i) const {
      return (*data_)[i_ + i];
    }

    Iterator<CB>& operator++() {
      ++i_;
      return *this;
    }
    Iterator<CB> operator++(int) {
      auto out = *this;
      ++*this;
      return out;
    }
    Iterator<CB>& operator--() {
      --i_;
      return *this;
    }
    Iterator<CB> operator--(int) {
      auto out = *this;
      --*this;
      return out;
    }
    Iterator<CB> operator+(const Iterator<CB>& rhs) const {
      return Iterator<CB>(*data_, i_ + rhs.i_);
    }
    Iterator<CB> operator-(const Iterator<CB>& rhs) const {
      return Iterator<CB>(*data_, i_ - rhs.i_);
    }
    Iterator<CB> operator+(const difference_type& rhs) const {
      return Iterator<CB>(*data_, i_ + rhs);
    }
    Iterator<CB> operator-(const difference_type& rhs) const {
      return Iterator<CB>(*data_, i_ - rhs);
    }

    bool operator==(const Iterator<CB>& other) const { return i_ == other.i_; }
    bool operator!=(const Iterator<CB>& other) const {
      return !(*this == other);
    }
    bool operator<(const Iterator<CB>& other) const { return i_ < other.i_; }
    bool operator>(const Iterator<CB>& other) const { return i_ > other.i_; }
    bool operator<=(const Iterator<CB>& other) const { return i_ <= other.i_; }
    bool operator>=(const Iterator<CB>& other) const { return i_ >= other.i_; }

    operator difference_type() const { return i_; }

   private:
    difference_type i_;
    CB* data_;
  };  // Iterator

  const Iterator<const CircularBuffer<T>> begin() const {
    return Iterator(*this, 0);
  }
  const Iterator<const CircularBuffer<T>> end() const {
    return Iterator(*this, size());
  }

 private:
  size_t GetShiftedIndex(const size_t index) const {
    size_t real_idx = tail_ + index;
    if (real_idx >= data_.size()) {
      real_idx -= data_.size();
    }
    return real_idx;
  }
  std::vector<T> data_;
  size_t head_{0};
  size_t tail_{0};
};  // class CircularBuffer
