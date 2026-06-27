// ring.hpp - Single-producer / single-consumer ring buffer.
//
// Lock-free for the SPSC case: the producer only ever writes `head_`, the
// consumer only ever writes `tail_`. Reads of the other side's index may be
// slightly stale, which is always conservative (producer sees less free space,
// consumer sees fewer bytes) so it can never overflow or underflow. This makes
// it safe to push from an ISR while popping from the main loop (or vice-versa)
// without disabling interrupts, as long as 32-bit aligned loads/stores are
// atomic (true on RV32).
//
// Indices are free-running (never masked); size = head_ - tail_ stays correct
// across the uint32_t wrap as long as Capacity <= 2^31. Power-of-two capacities
// use a cheap mask for indexing; other capacities use modulo.
#pragma once

#include <stdint.h>
#include <stddef.h>

template <uint32_t Capacity, typename T = uint8_t>
class Ring
{
    static_assert(Capacity > 0, "Capacity must not be zero");
    static_assert(Capacity <= 0x80000000u, "Capacity must be <= 2^31");

public:
    void clear() { head_ = 0; tail_ = 0; }

    uint32_t size() const { return head_ - tail_; }       // items available to read
    uint32_t free() const { return Capacity - size(); }   // items available to write
    bool     empty() const { return head_ == tail_; }
    bool     full() const { return size() == Capacity; }

    // Producer side: copy up to `len` items in, returns the number accepted.
    __attribute__((always_inline)) uint32_t push(const T* src, uint32_t len)
    {
        uint32_t n = free();
        if (len < n) n = len;
        return copyIn(src, n);
    }

    bool push(const T& item)
    {
        return push(&item, 1) == 1;
    }

    // Producer side, atomic at the API level: either the whole block fits and is
    // copied, or nothing is written. Use this for framed records that must not
    // be torn in the stream.
    __attribute__((always_inline)) bool pushAll(const T* src, uint32_t len)
    {
        if (free() < len) return false;
        copyIn(src, len);
        return true;
    }

    // Zero-copy producer side: write straight into the backing buffer, no temp + no
    // second copy. freeContig() = contiguous writable run from head_ to the wrap point
    // (or to "full", whichever is smaller). Fill up to that many at writePtr(), then
    // writeCommit(n). Single-producer only (head_ owner).
    uint32_t freeContig() const
    {
        uint32_t f = free();
        uint32_t toWrap = Capacity - index(head_);
        return (f < toWrap) ? f : toWrap;
    }
    T*   writePtr() { return &buf_[index(head_)]; }
    void writeCommit(uint32_t n) { head_ += n; }

    // Zero-copy consumer side: read straight from the backing buffer, no temp +
    // no second copy. readContig() = contiguous readable run from tail_ to the
    // wrap point (or to "empty", whichever is smaller). Consume up to that many
    // at readPtr(), then readCommit(n). Single-consumer only (tail_ owner).
    uint32_t readContig() const
    {
        uint32_t s = size();
        uint32_t toWrap = Capacity - index(tail_);
        return (s < toWrap) ? s : toWrap;
    }
    const T* readPtr() const { return &buf_[index(tail_)]; }
    T*       readPtr() { return &buf_[index(tail_)]; }
    void     readCommit(uint32_t n) { tail_ += n; }

    // Consumer side: copy up to `len` items out, returns the number read.
    uint32_t pop(T* dst, uint32_t len)
    {
        uint32_t n = size();
        if (len < n) n = len;
        uint32_t t = tail_;            // cache the volatile once (see push)
        for (uint32_t i = 0; i < n; i++)
        {
            dst[i] = buf_[index(t + i)];
        }
        tail_ = t + n;
        return n;
    }

    bool pop(T* item)
    {
        return pop(item, 1) == 1;
    }

private:
    __attribute__((always_inline)) uint32_t copyIn(const T* src, uint32_t n)
    {
        uint32_t h = head_;            // cache the volatile once: it only advances after the
                                       // loop, so the consumer still sees data appear atomically
                                       // at head_=h+n - but we save n volatile reloads.
        for (uint32_t i = 0; i < n; i++)
        {
            buf_[index(h + i)] = src[i];
        }
        head_ = h + n;
        return n;
    }

    static uint32_t index(uint32_t pos)
    {
        if constexpr ((Capacity & (Capacity - 1)) == 0)
        {
            return pos & (Capacity - 1);
        }
        else
        {
            return pos % Capacity;
        }
    }

    volatile uint32_t head_ = 0;   // written by producer only
    volatile uint32_t tail_ = 0;   // written by consumer only
    T                 buf_[Capacity];
}; // class Ring
