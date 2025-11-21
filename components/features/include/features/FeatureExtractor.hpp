// components/features/include/features/FeatureExtractor.hpp
#pragma once

#include <vector>
#include <cstdint>

namespace features {

enum class FeatureStatus {
    OK,
    ERROR_NOT_INITIALIZED,
    ERROR_INVALID_CONFIG,
    ERROR_INSUFFICIENT_DATA,
    ERROR_COMPUTATION
};

struct FeatureConfig {
    uint32_t sample_rate;        // Hz (e.g., 16000)
    int num_mfcc;                // Number of MFCC coefficients (e.g., 13, 40)
    int num_mel_bins;            // Number of mel filterbanks (e.g., 40, 80)
    int frame_length_ms;         // Frame/window length in ms (e.g., 25-30)
    int frame_stride_ms;         // Hop size in ms (e.g., 10)
    int fft_length;              // FFT size (e.g., 512, 1024)
    float lower_frequency_hz;    // Lower frequency limit (e.g., 20)
    float upper_frequency_hz;    // Upper frequency limit (e.g., 8000)
    
    // Feature options
    bool use_log_mel;            // If true, return log-mel instead of MFCC
    bool include_delta;          // Include Δ MFCC (first derivative)
    bool include_delta_delta;    // Include ΔΔ MFCC (second derivative)
    bool normalize_features;     // Apply per-feature normalization
    bool use_energy;             // Include frame energy as first coefficient
    
    // Pre-emphasis (usually done in DSP, but can be done here)
    float pre_emphasis_coeff;    // 0.0 = disabled, 0.97 = typical
};

// Default configuration for speech recognition
inline FeatureConfig defaultSpeechFeatureConfig() {
    return FeatureConfig {
        .sample_rate = 16000,
        .num_mfcc = 13,              // Classic speech: 13 MFCCs
        .num_mel_bins = 40,
        .frame_length_ms = 25,
        .frame_stride_ms = 10,
        .fft_length = 512,
        .lower_frequency_hz = 20.0f,
        .upper_frequency_hz = 8000.0f,
        .use_log_mel = false,
        .include_delta = true,       // Δ improves accuracy
        .include_delta_delta = true, // ΔΔ improves accuracy
        .normalize_features = true,
        .use_energy = false,
        .pre_emphasis_coeff = 0.0f   // Already done in DSP
    };
}

// Configuration for environmental sound classification
inline FeatureConfig defaultEnvironmentFeatureConfig() {
    return FeatureConfig {
        .sample_rate = 16000,
        .num_mfcc = 40,              // More coefficients for complex sounds
        .num_mel_bins = 80,
        .frame_length_ms = 30,
        .frame_stride_ms = 10,
        .fft_length = 1024,
        .lower_frequency_hz = 00.0f,
        .upper_frequency_hz = 8000.0f,
        .use_log_mel = false,
        .include_delta = false,      // Often not needed for environment
        .include_delta_delta = false,
        .normalize_features = true,
        .use_energy = false,
        .pre_emphasis_coeff = 0.0f
    };
}

struct FeatureVector {
    std::vector<float> data;     // Row-major: [frame0_coeff0, frame0_coeff1, ..., frame1_coeff0, ...]
    int num_frames;              // Number of time frames
    int num_coefficients;        // Coefficients per frame
    uint64_t timestamp_us;       // Timestamp of feature extraction
    
    // Statistics (useful for debugging)
    float energy_mean;
    float energy_std;
    bool contains_speech;        // Based on energy/VAD
    
    FeatureVector() 
        : num_frames(0)
        , num_coefficients(0)
        , timestamp_us(0)
        , energy_mean(0.0f)
        , energy_std(0.0f)
        , contains_speech(false) 
    {}
    
    // Get feature at specific frame and coefficient
    float at(int frame, int coeff) const {
        return data[frame * num_coefficients + coeff];
    }
    
    // Get pointer to specific frame
    const float* framePtr(int frame) const {
        return &data[frame * num_coefficients];
    }
    
    // Total number of features
    size_t size() const {
        return data.size();
    }
    
    // Expected size
    size_t expectedSize() const {
        return num_frames * num_coefficients;
    }
};

class FeatureExtractor {
public:
    virtual ~FeatureExtractor() = default;
    
    // Initialize with configuration
    virtual FeatureStatus init(const FeatureConfig& config) = 0;
    
    // Compute features from PCM audio
    virtual FeatureStatus compute(const std::vector<int16_t>& pcm, 
                                   FeatureVector& out) = 0;
    
    // Compute features from float audio (normalized [-1, 1])
    virtual FeatureStatus compute(const std::vector<float>& audio, 
                                   FeatureVector& out) = 0;
    
    // Get expected number of frames for given audio length
    virtual int getExpectedFrames(size_t num_samples) const = 0;
    
    // Get current configuration
    virtual const FeatureConfig& getConfig() const = 0;
    
    // Check if initialized
    virtual bool isInitialized() const = 0;
    
    // Reset internal state (if any caching)
    virtual void reset() = 0;
};

    FeatureExtractor* createMFCCExtractor();
    
} // namespace features