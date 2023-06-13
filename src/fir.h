#pragma once

#include <stddef.h>

/// @brief FIR filter class
/// @tparam N_TAPS Number of filter taps to allocate
template <size_t N_TAPS>
class FirFilter {
public:
    /// @brief Create a FIR filter
    /// @param taps List of tap coefficients. Must have exactly N_TAPS coefficients as defined in the template.
    FirFilter(const float taps[N_TAPS]) : taps(taps), samples(), last_index(0)
    {
        for (int i = 0; i < N_TAPS; i++) {
            samples[i] = 0;
        }
    }

    /// @brief Add a sample to the filter
    /// @param sample Value to filter
    void add(float sample) {
        samples[last_index++] = sample;
        if (last_index == N_TAPS) {
            last_index = 0;
            ready = true;
        }
    }

    /// @brief Get the current filtered value
    /// @return Filtered value
    float get() const {
        float acc = 0;
        int index = last_index;
        for (int i = 0; i < N_TAPS; ++i) {
            index = index != 0 ? index-1 : N_TAPS-1;
            acc += samples[index] * taps[i];
        };
        return acc;
    }

    /// @brief Whether the filter is ready to return a sample
    bool isReady() const {
        return ready;
    }

private:
    const float*                taps;
    std::array<float, N_TAPS>   samples;
    unsigned int                last_index;
    bool                        ready;
};

#define DEFINE_FIR_FILTER(taps) FirFilter<(sizeof(taps)/sizeof(taps[0]))>
#define MAKE_FIR_FILTER(taps) (FirFilter<(sizeof(taps)/sizeof(taps[0]))> { taps })
