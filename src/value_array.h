#pragma once

#include <stdint.h>

/// @brief Helper class for ValueArray
/// @details This allows you to reference ValueArray<T,N> without knowing N up-front.
/// @tparam T scalar type of value
template<typename T>
class ValueArrayBase
{
public:
    ValueArrayBase(int capacity)
    { }

    T operator[](size_t i) noexcept {
        return get(i);
    }

    virtual size_t size() const = 0;
    virtual T get(size_t i) = 0;
};

/// @brief An array of values
/// @tparam T scalar type of value
/// @tparam N size of the history (rolling buffer)
template<typename T, size_t N>
class ValueArray : public ValueArrayBase<T>
{
public:
    ValueArray()
        : ValueArrayBase<T>(N)
    { }

    /// @brief Record a sample. Oldest sample will be removed when full.
    /// @param sample value to record.
    void add(T sample) {
        samples[i++] = sample;

        if (n < N)
            n++;

        if (i >= N) {
            i = 0; // wrap
        }

        _is_dirty = true;
    }

    /// @brief Check if the collection has changed.
    /// @return True if the value has changed since the last call to is_dirty()
    bool is_dirty() {
        bool r = _is_dirty;
        _is_dirty = false;
        return r;
    }

    /// @brief The number of samples recorded, up to a maximum of N
    size_t size() const override {
        return n;
    }

    size_t capacity() const {
        return N;
    }

    /// @brief Get a sample at j
    /// @param j Index of the sample
    /// @return Sample
    T get(size_t j) override {
        if (n == N) {
            // Buffer is full, use internal index to offset the array
            j += i;
        }
        // Wrap
        while (j >= N) {
            j -= N;
        }

        return samples[j];
    }

protected:
    size_t i = 0;
    size_t n = 0;
    bool _is_dirty = true;
    std::array<T, N> samples;
};
