#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace flexiv_rt {

constexpr size_t kCacheLineSize = 64;

/// SPSC (single-producer / single-consumer) lock-free ring buffer.
/// Producer (Python thread) calls try_write(); consumer (RT thread) calls try_read().
/// head and tail live on separate cache lines to avoid false sharing between
/// the writer core and the reader core.
template <typename T, size_t N>
struct RealTimeBuffer {
    static_assert(N > 0, "Buffer capacity must be > 0");
    static_assert((N & (N - 1)) == 0, "N must be a power of two for fast modulo");

    alignas(kCacheLineSize) std::array<T, N> data;
    alignas(kCacheLineSize) std::atomic<uint64_t> head{0};  // written by producer
    alignas(kCacheLineSize) std::atomic<uint64_t> tail{0};  // written by consumer

    /// Try to enqueue one item.  Returns false if the buffer is full.
    bool try_write(const T& item) noexcept {
        const uint64_t h = head.load(std::memory_order_relaxed);
        const uint64_t t = tail.load(std::memory_order_acquire);
        if (h - t >= N) return false;           // full
        data[h & (N - 1)] = item;
        head.store(h + 1, std::memory_order_release);
        return true;
    }

    /// Try to dequeue one item.  Returns false if the buffer is empty.
    bool try_read(T& item) noexcept {
        const uint64_t t = tail.load(std::memory_order_relaxed);
        const uint64_t h = head.load(std::memory_order_acquire);
        if (h <= t) return false;               // empty
        item = data[t & (N - 1)];
        tail.store(t + 1, std::memory_order_release);
        return true;
    }

    /// Check whether the buffer is empty (approximate — data-race-free but
    /// the answer may be stale by the time the caller acts on it).
    bool empty() const noexcept {
        return head.load(std::memory_order_acquire)
            <= tail.load(std::memory_order_acquire);
    }

    /// Discard all pending items.  Only safe to call when no concurrent
    /// try_write / try_read is in progress (e.g. during init or after e-stop).
    void clear() noexcept {
        head.store(0, std::memory_order_relaxed);
        tail.store(0, std::memory_order_relaxed);
    }
};

} // namespace flexiv_rt
