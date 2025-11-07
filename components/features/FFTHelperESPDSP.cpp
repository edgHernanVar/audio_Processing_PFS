// components/features/FFTHelperESPDSP.cpp
#include "./FFTHelperESPDSP.hpp"
#include "esp_log.h"
#include "esp_dsp.h"
#include <cmath>
#include <cstring>

namespace features {

static const char* TAG = "FFTHelperESPDSP";

FFTHelperESPDSP::FFTHelperESPDSP(int size) 
    : size_(size)
    , fft_buffer_(nullptr)
    , window_(nullptr)
    , initialized_(false) 
{
    // Validate FFT size (must be power of 2)
    if (size <= 0 || (size & (size - 1)) != 0) {
        ESP_LOGE(TAG, "FFT size must be power of 2, got %d", size);
        return;
    }
    
    // ESP-DSP supports FFT sizes from 16 to 4096
    if (size < 16 || size > 4096) {
        ESP_LOGE(TAG, "FFT size must be between 16 and 4096");
        return;
    }
    
    // Allocate FFT buffer (complex data: real and imaginary interleaved)
    // Size = N * 2 for complex data (real + imaginary)
    fft_buffer_ = (float*)heap_caps_aligned_alloc(16, size * 2 * sizeof(float), 
                                                   MALLOC_CAP_DEFAULT | MALLOC_CAP_32BIT);
    
    // Allocate window buffer
    window_ = (float*)heap_caps_aligned_alloc(16, size * sizeof(float), 
                                               MALLOC_CAP_DEFAULT | MALLOC_CAP_32BIT);
    
    if (!fft_buffer_ || !window_) {
        ESP_LOGE(TAG, "Failed to allocate FFT buffers");
        cleanup();
        return;
    }
    
    // Initialize ESP-DSP FFT tables
    esp_err_t ret = dsps_fft2r_init_fc32(nullptr, size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FFT initialization failed: %s", esp_err_to_name(ret));
        cleanup();
        return;
    }
    
    // Generate Hamming window
    // Hamming: w(n) = 0.54 - 0.46 * cos(2πn/(N-1))
    dsps_wind_hann_f32(window_, size);
    
    // Manually adjust to Hamming (Hann is close but not exact)
    for (int i = 0; i < size; ++i) {
        window_[i] = 0.54f - 0.46f * std::cos(2.0f * M_PI * i / (size - 1));
    }
    
    initialized_ = true;
    
    ESP_LOGI(TAG, "ESP-DSP FFT initialized: size=%d, buffers=%p/%p", 
             size, fft_buffer_, window_);
}

FFTHelperESPDSP::~FFTHelperESPDSP() {
    cleanup();
}

void FFTHelperESPDSP::cleanup() {
    if (fft_buffer_) {
        heap_caps_free(fft_buffer_);
        fft_buffer_ = nullptr;
    }
    
    if (window_) {
        heap_caps_free(window_);
        window_ = nullptr;
    }
    
    if (initialized_) {
        dsps_fft2r_deinit_fc32();
        initialized_ = false;
    }
}

void FFTHelperESPDSP::computePowerSpectrum(const float* input, float* output, int input_size) {
    if (!initialized_) {
        ESP_LOGE(TAG, "FFT not initialized");
        return;
    }
    
    // Step 1: Apply window and prepare complex input
    // Format: [real0, imag0, real1, imag1, ..., realN-1, imagN-1]
    for (int i = 0; i < input_size && i < size_; ++i) {
        fft_buffer_[i * 2 + 0] = input[i] * window_[i];  // Real part
        fft_buffer_[i * 2 + 1] = 0.0f;                   // Imaginary part (always 0 for real input)
    }
    
    // Zero-pad if input is shorter than FFT size
    for (int i = input_size; i < size_; ++i) {
        fft_buffer_[i * 2 + 0] = 0.0f;
        fft_buffer_[i * 2 + 1] = 0.0f;
    }
    
    // Step 2: Perform FFT
    // dsps_fft2r_fc32 performs in-place complex FFT
    dsps_fft2r_fc32(fft_buffer_, size_);
    
    // Step 3: Bit-reverse reordering (required by ESP-DSP)
    dsps_bit_rev_fc32(fft_buffer_, size_);
    
    // Step 4: Convert from complex to real (CCS format)
    // This reorganizes the data to be more convenient
    dsps_cplx2reC_fc32(fft_buffer_, size_);
    
    // Step 5: Compute power spectrum
    // Power = Real² + Imag²
    // For real input, we only need first half + Nyquist (size/2 + 1 bins)
    int half_size = size_ / 2 + 1;
    
    for (int i = 0; i < half_size; ++i) {
        float real = fft_buffer_[i * 2 + 0];
        float imag = fft_buffer_[i * 2 + 1];
        
        // Compute power and normalize by FFT size
        output[i] = (real * real + imag * imag) / size_;
    }
    
    // Note: DC (bin 0) and Nyquist (bin N/2) have no imaginary component
    // but the formula above handles them correctly
}

void FFTHelperESPDSP::computeMagnitudeSpectrum(const float* input, float* output, int input_size) {
    if (!initialized_) {
        ESP_LOGE(TAG, "FFT not initialized");
        return;
    }
    
    // Apply window and prepare input
    for (int i = 0; i < input_size && i < size_; ++i) {
        fft_buffer_[i * 2 + 0] = input[i] * window_[i];
        fft_buffer_[i * 2 + 1] = 0.0f;
    }
    
    for (int i = input_size; i < size_; ++i) {
        fft_buffer_[i * 2 + 0] = 0.0f;
        fft_buffer_[i * 2 + 1] = 0.0f;
    }
    
    // Perform FFT
    dsps_fft2r_fc32(fft_buffer_, size_);
    dsps_bit_rev_fc32(fft_buffer_, size_);
    dsps_cplx2reC_fc32(fft_buffer_, size_);
    
    // Compute magnitude spectrum (square root of power)
    int half_size = size_ / 2 + 1;
    for (int i = 0; i < half_size; ++i) {
        float real = fft_buffer_[i * 2 + 0];
        float imag = fft_buffer_[i * 2 + 1];
        output[i] = std::sqrt(real * real + imag * imag) / size_;
    }
}

void FFTHelperESPDSP::setWindowType(WindowType type) {
    if (!initialized_ || !window_) {
        return;
    }
    
    switch (type) {
        case WindowType::HAMMING:
            // Hamming: 0.54 - 0.46 * cos(2πn/(N-1))
            for (int i = 0; i < size_; ++i) {
                window_[i] = 0.54f - 0.46f * std::cos(2.0f * M_PI * i / (size_ - 1));
            }
            ESP_LOGI(TAG, "Window type: Hamming");
            break;
            
        case WindowType::HANN:
            // Hann: 0.5 * (1 - cos(2πn/(N-1)))
            dsps_wind_hann_f32(window_, size_);
            ESP_LOGI(TAG, "Window type: Hann");
            break;
            
        case WindowType::BLACKMAN:
            // Blackman: 0.42 - 0.5*cos(2πn/(N-1)) + 0.08*cos(4πn/(N-1))
            dsps_wind_blackman_f32(window_, size_);
            ESP_LOGI(TAG, "Window type: Blackman");
            break;
            
        case WindowType::FLAT_TOP:
            // Flat-top window (good for amplitude accuracy)
            dsps_wind_flat_top_f32(window_, size_);
            ESP_LOGI(TAG, "Window type: Flat-top");
            break;
            
        case WindowType::RECTANGULAR:
            // Rectangular (no window)
            for (int i = 0; i < size_; ++i) {
                window_[i] = 1.0f;
            }
            ESP_LOGI(TAG, "Window type: Rectangular");
            break;
    }
}

bool FFTHelperESPDSP::isInitialized() const {
    return initialized_;
}

int FFTHelperESPDSP::getSize() const {
    return size_;
}

// Utility: Get frequency bin corresponding to a Hz value
int FFTHelperESPDSP::hzToBin(float hz, float sample_rate) const {
    return static_cast<int>((hz * size_) / sample_rate);
}

// Utility: Get Hz value for a frequency bin
float FFTHelperESPDSP::binToHz(int bin, float sample_rate) const {
    return (static_cast<float>(bin) * sample_rate) / size_;
}

} // namespace features