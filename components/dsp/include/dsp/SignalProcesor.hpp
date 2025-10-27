// components/dsp/include/dsp/SignalProcessor.hpp
#pragma once

#include <vector>
#include <cstdint>

namespace dsp {

enum class ProcessStatus {
    OK,
    ERROR_INVALID_CONFIG,
    ERROR_EMPTY_BUFFER,
    ERROR_SAMPLE_RATE
};

struct DSPConfig {
    uint32_t sample_rate;        // Hz (e.g., 16000)
    bool enable_dc_removal;      // Remove DC offset
    float hpf_cutoff_hz;         // High-pass filter (0 = disabled)
    float hpf_q;                 // Q factor for HPF
    float lpf_cutoff_hz;         // Low-pass filter (0 = disabled)
    float lpf_q;                 // Q factor for LPF
    float pre_emphasis_alpha;    // Pre-emphasis coefficient (0.0 = disabled, 0.97 = typical)
    float target_gain_db;        // Target gain in dB (0 = no gain adjustment)
    bool auto_normalize;         // Normalize to [-1.0, 1.0] range
};

// Default configuration for speech processing
inline DSPConfig defaultSpeechConfig() {
    return DSPConfig {
        .sample_rate = 16000,
        .enable_dc_removal = true,
        .hpf_cutoff_hz = 40.0f,
        .hpf_q = 0.707f,
        .lpf_cutoff_hz = 7500.0f,
        .lpf_q = 0.707f,
        .pre_emphasis_alpha = 0.97f,
        .target_gain_db = 0.0f,
        .auto_normalize = false
    };
}

class SignalProcessor {
public:
    virtual ~SignalProcessor() = default;
    
    // Initialize with configuration
    virtual ProcessStatus init(const DSPConfig& config) = 0;
    
    // Process audio buffer in-place (int16_t PCM format)
    virtual ProcessStatus process(std::vector<int16_t>& pcm) = 0;
    
    // Process raw pointer (useful for interfacing with C APIs)
    virtual ProcessStatus process(int16_t* pcm, size_t length) = 0;
    
    // Process 32-bit samples (from I2S) and convert to 16-bit
    virtual ProcessStatus processI2S(const std::vector<int32_t>& i2s_data, 
                                      std::vector<int16_t>& pcm) = 0;
    
    // Convert ADC samples to PCM format
    virtual ProcessStatus adcToPCM(const std::vector<uint16_t>& adc_raw,
                                    std::vector<int16_t>& pcm,
                                    uint16_t adc_bits = 12) = 0;
    
    // Reset filter states (call when starting new audio stream)
    virtual void reset() = 0;
    
    // Dynamic configuration updates
    virtual void enableDCRemoval(bool enable) = 0;
    virtual void setHPF(float cutoff_hz, float q = 0.707f) = 0;
    virtual void setLPF(float cutoff_hz, float q = 0.707f) = 0;
    virtual void setPreEmphasis(float alpha) = 0;
    virtual void setGain(float gain_db) = 0;
    virtual void enableNormalization(bool enable) = 0;
    
    // Query configuration
    virtual const DSPConfig& getConfig() const = 0;
    
    // Utility: Calculate RMS energy (useful for VAD)
    virtual float calculateRMS(const std::vector<int16_t>& pcm) const = 0;
    
    // Utility: Detect clipping
    virtual bool hasClipping(const std::vector<int16_t>& pcm, 
                            float threshold = 0.95f) const = 0;
};

} // namespace dsp