// components/features/FFTHelperESPDSP.hpp
#pragma once

#include "esp_heap_caps.h"

namespace features {

enum class WindowType {
    HAMMING,      // Default for speech (0.54 - 0.46*cos)
    HANN,         // Smooth, low sidelobes
    BLACKMAN,     // Very low sidelobes
    FLAT_TOP,     // Best for amplitude accuracy
    RECTANGULAR   // No window (box)
};

class FFTHelperESPDSP {
public:
    /**
     * @brief Construct FFT helper with specified size
     * @param size FFT size (must be power of 2: 16, 32, 64, 128, 256, 512, 1024, 2048, 4096)
     */
    explicit FFTHelperESPDSP(int size);
    
    /**
     * @brief Destructor - frees allocated resources
     */
    ~FFTHelperESPDSP();
    
    /**
     *
     * @brief Compute power spectrum from real input signal
     * @param input Input signal (time domain)
     * @param output Power spectrum (size/2 + 1 bins)
     * @param input_size Number of samples in input (will be zero-padded if < size)
     */
    void computePowerSpectrum(const float* input, float* output, int input_size);
    
    /**
     * @brief Compute magnitude spectrum from real input signal
     * @param input Input signal (time domain)
     * @param output Magnitude spectrum (size/2 + 1 bins)
     * @param input_size Number of samples in input
     */
    void computeMagnitudeSpectrum(const float* input, float* output, int input_size);
    
    /**
     * @brief Set window function type
     * @param type Window type (Hamming, Hann, Blackman, etc.)
     */
    void setWindowType(WindowType type);
    
    /**
     * @brief Check if FFT is properly initialized
     */
    bool isInitialized() const;
    
    /**
     * @brief Get FFT size
     */
    int getSize() const;
    
    /**
     * @brief Convert frequency (Hz) to FFT bin index
     * @param hz Frequency in Hz
     * @param sample_rate Sample rate in Hz
     * @return Bin index
     */
    int hzToBin(float hz, float sample_rate) const;
    
    /**
     * @brief Convert FFT bin index to frequency (Hz)
     * @param bin Bin index
     * @param sample_rate Sample rate in Hz
     * @return Frequency in Hz
     */
    float binToHz(int bin, float sample_rate) const;
    
private:
    int size_;                  // FFT size
    float* fft_buffer_;         // Complex FFT buffer (size * 2)
    float* window_;             // Window function
    bool initialized_;          // Initialization status
    
    void cleanup();             // Clean up allocated resources
    
    // Prevent copying
    FFTHelperESPDSP(const FFTHelperESPDSP&) = delete;
    FFTHelperESPDSP& operator=(const FFTHelperESPDSP&) = delete;
};

} // namespace features