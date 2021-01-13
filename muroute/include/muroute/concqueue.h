#ifndef CONCQUEUE_H
#define CONCQUEUE_H

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <stack>

// thread-safe queue
template <typename T>
class ConcQueue {
 public:
  ConcQueue() = default;
  virtual ~ConcQueue() = default;

  template <typename... Args>
  void enqueue(Args &&... args) {
    addData_protected([&] { queue_.emplace(std::forward<Args>(args)...); });
  }

  T dequeue(void) noexcept {
    std::unique_lock<std::mutex> lock{mutex_};
    while (queue_.empty()) {
      condNewData_.wait(lock);
    }
    auto elem = std::move(queue_.front());
    queue_.pop();
    return elem;
  }

  T dequeue_n_drain(void) noexcept {
    std::unique_lock<std::mutex> lock{mutex_};
    while (queue_.empty()) {
      condNewData_.wait(lock);
    }

    auto elem = std::move(queue_.back());  // get latest

    while (!queue_.empty()) queue_.pop();

    return elem;
  }

  size_t size(void) const {
    std::lock_guard<std::mutex> lock{mutex_};
    return queue_.size();
  }

  bool empty(void) const {
    std::lock_guard<std::mutex> lock{mutex_};
    return queue_.empty();
  }

  const T &front(void) const {
    std::unique_lock<std::mutex> lock{mutex_};
    return queue_.front();
  }

 private:
  template <class F>
  void addData_protected(F &&fct) {
    std::unique_lock<std::mutex> lock{mutex_};
    fct();
    lock.unlock();
    condNewData_.notify_one();
  }

  std::queue<T> queue_;
  mutable std::mutex mutex_;
  std::condition_variable condNewData_;
};

// thread-safe bounded circular buffer
template <class T>
class ConcRingBuffer {
 public:
  explicit ConcRingBuffer()
      : full_(false),
        buffer_(std::unique_ptr<T[]>(new T[default_capacity])),
        capacity_(default_capacity) {}
  explicit ConcRingBuffer(size_t size)
      : full_(false),
        buffer_(std::unique_ptr<T[]>(new T[size])),
        capacity_(size) {}

  void push(T item) {
    std::unique_lock<std::mutex> lock(mutex_);
    buffer_[head_] = item;
    if (full_) tail_ = (tail_ + 1) % capacity_;
    head_ = (head_ + 1) % capacity_;
    full_ = head_ == tail_;
    lock.unlock();
    condNewData_.notify_one();
  }

  T pop(void) noexcept {
    std::unique_lock<std::mutex> lock(mutex_);
    while (empty()) condNewData_.wait(lock);
    auto elem = std::move(buffer_[tail_]);
    full_ = false;
    tail_ = (tail_ + 1) % capacity_;
    return elem;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = tail_;
    full_ = false;
  }

  bool empty() const { return (!full_ && (head_ == tail_)); }

  bool full() const { return full_; }

  size_t capacity() const { return capacity_; }

  size_t size() const {
    size_t size = capacity_;
    if (!full_)
      size = head_ >= tail_ ? head_ - tail_ : capacity_ + head_ - tail_;
    return size;
  }

 private:
  bool full_;
  mutable std::mutex mutex_;
  std::condition_variable condNewData_;
  std::unique_ptr<T[]> buffer_;
  size_t head_ = 0;
  size_t tail_ = 0;
  const size_t capacity_;
  static constexpr size_t default_capacity = 30;
};

// thread-safe stack
template <typename T>
class ConcStack {
 public:
  ConcStack() = default;
  virtual ~ConcStack() = default;

  template <typename... Args>
  void push(Args &&... args) {
    addData_protected([&] { stack_.emplace(std::forward<Args>(args)...); });
  }

  T pop(void) noexcept {
    std::unique_lock<std::mutex> lock{mutex_};
    while (stack_.empty()) condNewData_.wait(lock);
    auto elem = std::move(stack_.top());
    stack_.pop();
    return elem;
  }

  size_t size(void) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stack_.size();
  }

  bool empty(void) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stack_.empty();
  }

 private:
  template <class F>
  void addData_protected(F &&fct) {
    std::unique_lock<std::mutex> lock{mutex_};
    fct();
    lock.unlock();
    condNewData_.notify_one();
  }

  std::stack<T> stack_;
  mutable std::mutex mutex_;
  std::condition_variable condNewData_;
};

#endif /* CONCQUEUE_H */
