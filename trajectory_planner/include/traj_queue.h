/*
 * Thread-Safe Motion Queue
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef TRAJ_QUEUE_H
#define TRAJ_QUEUE_H

#include "traj_planner.h"
#include <mutex>
#include <condition_variable>
#include <vector>

namespace traj_planner {

// ============================================================================
// Lock-Free Ring Buffer for Real-Time Use
// ============================================================================

template<typename T, size_t Capacity>
class RingBuffer {
public:
    RingBuffer() : head_(0), tail_(0) {}
    
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next_head = (head + 1) % Capacity;
        
        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false;  // Full
        }
        
        buffer_[head] = item;
        head_.store(next_head, std::memory_order_release);
        return true;
    }
    
    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // Empty
        }
        
        item = buffer_[tail];
        tail_.store((tail + 1) % Capacity, std::memory_order_release);
        return true;
    }
    
    bool peek(T& item) const {
        size_t tail = tail_.load(std::memory_order_relaxed);
        
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        
        item = buffer_[tail];
        return true;
    }
    
    size_t size() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        
        if (head >= tail) {
            return head - tail;
        }
        return Capacity - tail + head;
    }
    
    bool empty() const {
        return head_.load(std::memory_order_acquire) == 
               tail_.load(std::memory_order_acquire);
    }
    
    bool full() const {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next_head = (head + 1) % Capacity;
        return next_head == tail_.load(std::memory_order_acquire);
    }
    
    void clear() {
        tail_.store(head_.load(std::memory_order_acquire),
                    std::memory_order_release);
    }
    
    static constexpr size_t capacity() { return Capacity - 1; }
    
    // Direct access for planning (not thread-safe!)
    T* at(size_t index) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        size_t actual_index = (tail + index) % Capacity;
        return &buffer_[actual_index];
    }
    
    const T* at(size_t index) const {
        size_t tail = tail_.load(std::memory_order_relaxed);
        size_t actual_index = (tail + index) % Capacity;
        return &buffer_[actual_index];
    }

private:
    std::array<T, Capacity> buffer_;
    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;
};

// ============================================================================
// Mutex-Protected Queue for General Use
// ============================================================================

template<typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t max_capacity = 256)
        : max_capacity_(max_capacity) {
        buffer_.reserve(max_capacity);
    }
    
    bool push(const T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.size() >= max_capacity_) {
            return false;
        }
        buffer_.push_back(item);
        cond_.notify_one();
        return true;
    }
    
    bool pop(T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            return false;
        }
        item = buffer_.front();
        buffer_.erase(buffer_.begin());
        return true;
    }
    
    bool pop_wait(T& item, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cond_.wait_for(lock, timeout, [this] { return !buffer_.empty(); })) {
            return false;
        }
        item = buffer_.front();
        buffer_.erase(buffer_.begin());
        return true;
    }
    
    bool peek(T& item) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            return false;
        }
        item = buffer_.front();
        return true;
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }
    
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.empty();
    }
    
    bool full() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size() >= max_capacity_;
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
    }
    
    size_t capacity() const { return max_capacity_; }
    
    // Lock for external access
    std::mutex& getMutex() { return mutex_; }
    
    // Direct access (caller must hold mutex!)
    T* at(size_t index) {
        if (index < buffer_.size()) {
            return &buffer_[index];
        }
        return nullptr;
    }
    
    const T* at(size_t index) const {
        if (index < buffer_.size()) {
            return &buffer_[index];
        }
        return nullptr;
    }

private:
    std::vector<T> buffer_;
    size_t max_capacity_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
};

// ============================================================================
// Specialized Motion Queue
// ============================================================================

using MotionRingBuffer = RingBuffer<MotionSegment, 128>;
using MotionThreadQueue = ThreadSafeQueue<MotionSegment>;

} // namespace traj_planner

#endif // TRAJ_QUEUE_H
