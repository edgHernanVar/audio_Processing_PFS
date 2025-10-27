// components/dsp/SignalProcessorESP.cpp
#include "./include/dsp/SignalProcesor.hpp"
#include "esp_log.h"
#include "esp_dsp.h"  // Optional: ESP-DSP library for optimized operations
#include <cmath>
#include <algorithm>
#include <cstring>

namespace dsp {

static const char* TAG = "SignalProcessorESP";

// Biquad filter structure for IIR filtering
struct BiquadState {
    float b0, b1, b2;  // Numerator coefficients
    float a1, a2;      // Denominator coefficients (a0 normalized to 1)
    float x1, x2;      // Input history
    float y1, y2;      // Output history
    
    BiquadState() : b0(1), b1(0), b2(0), a1(0), a2(0), 
                    x1(0), x2(0), y1(0), y2(0) {}
    
    void reset() {
        x1 = x2 = y1 = y2 = 0.0f;
    }
    
    // Process single sample
    float process(float input) {
        float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        
        // Update history
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        
        return output;
    }
};

class SignalProcessorESP : public SignalProcessor {
private:
    DSPConfig config_;
    bool initialized_;
    
    // Filter states
    BiquadState hpf_;
    BiquadState lpf_;
    
    // DC removal
    float dc_estimate_;
    const float dc_alpha_ = 0.995f;  // Smoothing factor for DC tracking
    
    // Pre-emphasis
    float pre_emphasis_prev_;
    
    // Normalize peak tracking
    float peak_estimate_;
    const float peak_decay_ = 0.9999f;
    
public:
    SignalProcessorESP() 
        : initialized_(false)
        , dc_estimate_(0.0f)
        , pre_emphasis_prev_(0.0f)
        , peak_estimate_(1.0f)
    {
        std::memset(&config_, 0, sizeof(config_));
    }
    
    ProcessStatus init(const DSPConfig& config) override {
        if (config.sample_rate == 0) {
            
            //ESP_LOGE(TAG, "Invalid sample rate");
            return ProcessStatus::ERROR_SAMPLE_RATE;
        }
        
        config_ = config;
        
        // Design HPF if enabled
        if (config_.hpf_cutoff_hz > 0) {
            designHighpass(config_.hpf_cutoff_hz, config_.hpf_q, config_.sample_rate, hpf_);
            //ESP_LOGI(TAG, "HPF enabled: %.1f Hz, Q=%.3f", config_.hpf_cutoff_hz, config_.hpf_q);
        }
        
        // Design LPF if enabled
        if (config_.lpf_cutoff_hz > 0) {
            designLowpass(config_.lpf_cutoff_hz, config_.lpf_q, config_.sample_rate, lpf_);
            //ESP_LOGI(TAG, "LPF enabled: %.1f Hz, Q=%.3f", config_.lpf_cutoff_hz, config_.lpf_q);
        }
        
        /*ESP_LOGI(TAG, "DSP initialized: %lu Hz, DC:%d, PreEmp:%.2f", 
                 config_.sample_rate, config_.enable_dc_removal, config_.pre_emphasis_alpha);
        */
        initialized_ = true;
        return ProcessStatus::OK;
    }
    
    ProcessStatus process(std::vector<int16_t>& pcm) override {
        return process(pcm.data(), pcm.size());
    }
    
    ProcessStatus process(int16_t* pcm, size_t length) override {
        if (!initialized_) {
            return ProcessStatus::ERROR_INVALID_CONFIG;
        }
        
        if (length == 0) {
            return ProcessStatus::ERROR_EMPTY_BUFFER;
        }
        
        // Process pipeline: DC removal -> HPF -> LPF -> Pre-emphasis -> Gain -> Normalize
        
        for (size_t i = 0; i < length; ++i) {
            float sample = static_cast<float>(pcm[i]) / 32768.0f;  // Convert to [-1, 1]
            
            // 1. DC Removal (high-pass at ~0.1 Hz)
            if (config_.enable_dc_removal) {
                dc_estimate_ = dc_alpha_ * dc_estimate_ + (1.0f - dc_alpha_) * sample;
                sample -= dc_estimate_;
            }
            
            // 2. High-pass filter
            if (config_.hpf_cutoff_hz > 0) {
                sample = hpf_.process(sample);
            }
            
            // 3. Low-pass filter
            if (config_.lpf_cutoff_hz > 0) {
                sample = lpf_.process(sample);
            }
            
            // 4. Pre-emphasis filter: y[n] = x[n] - alpha * x[n-1]
            if (config_.pre_emphasis_alpha > 0.0f) {
                float emphasized = sample - config_.pre_emphasis_alpha * pre_emphasis_prev_;
                pre_emphasis_prev_ = sample;
                sample = emphasized;
            }
            
            // 5. Apply gain
            if (config_.target_gain_db != 0.0f) {
                float gain = std::pow(10.0f, config_.target_gain_db / 20.0f);
                sample *= gain;
            }
            
            // 6. Auto-normalize (optional peak tracking)
            if (config_.auto_normalize) {
                float abs_sample = std::abs(sample);
                if (abs_sample > peak_estimate_) {
                    peak_estimate_ = abs_sample;
                } else {
                    peak_estimate_ *= peak_decay_;
                }
                
                if (peak_estimate_ > 0.01f) {  // Avoid division by zero
                    sample /= peak_estimate_;
                }
            }
            
            // Clip to [-1, 1] and convert back to int16
            sample = std::max(-1.0f, std::min(1.0f, sample));
            pcm[i] = static_cast<int16_t>(sample * 32767.0f);
        }
        
        return ProcessStatus::OK;
    }
    
    ProcessStatus processI2S(const std::vector<int32_t>& i2s_data, 
                             std::vector<int16_t>& pcm) override {
        pcm.resize(i2s_data.size());
        
        // Convert I2S 32-bit to 16-bit PCM (take upper 16 bits)
        for (size_t i = 0; i < i2s_data.size(); ++i) {
            pcm[i] = static_cast<int16_t>(i2s_data[i] >> 16);
        }
        
        return process(pcm);
    }
    
    ProcessStatus adcToPCM(const std::vector<uint16_t>& adc_raw,
                          std::vector<int16_t>& pcm,
                          uint16_t adc_bits) override {
        pcm.resize(adc_raw.size());
        
        // Convert unsigned ADC to signed PCM centered at 0
        uint16_t adc_max = (1 << adc_bits) - 1;
        uint16_t adc_mid = adc_max / 2;
        
        for (size_t i = 0; i < adc_raw.size(); ++i) {
            int32_t centered = static_cast<int32_t>(adc_raw[i]) - adc_mid;
            // Scale to 16-bit range
            pcm[i] = static_cast<int16_t>((centered * 32767) / adc_mid);
        }
        
        return process(pcm);
    }
    
    void reset() override {
        hpf_.reset();
        lpf_.reset();
        dc_estimate_ = 0.0f;
        pre_emphasis_prev_ = 0.0f;
        peak_estimate_ = 1.0f;
        //ESP_LOGI(TAG, "DSP state reset");
    }
    
    void enableDCRemoval(bool enable) override {
        config_.enable_dc_removal = enable;
        if (!enable) {
            dc_estimate_ = 0.0f;
        }
    }
    
    void setHPF(float cutoff_hz, float q) override {
        if (cutoff_hz > 0 && initialized_) {
            config_.hpf_cutoff_hz = cutoff_hz;
            config_.hpf_q = q;
            designHighpass(cutoff_hz, q, config_.sample_rate, hpf_);
        } else {
            config_.hpf_cutoff_hz = 0;
            hpf_.reset();
        }
    }
    
    void setLPF(float cutoff_hz, float q) override {
        if (cutoff_hz > 0 && initialized_) {
            config_.lpf_cutoff_hz = cutoff_hz;
            config_.lpf_q = q;
            designLowpass(cutoff_hz, q, config_.sample_rate, lpf_);
        } else {
            config_.lpf_cutoff_hz = 0;
            lpf_.reset();
        }
    }
    
    void setPreEmphasis(float alpha) override {
        config_.pre_emphasis_alpha = std::max(0.0f, std::min(1.0f, alpha));
    }
    
    void setGain(float gain_db) override {
        config_.target_gain_db = gain_db;
    }
    
    void enableNormalization(bool enable) override {
        config_.auto_normalize = enable;
        if (enable) {
            peak_estimate_ = 1.0f;
        }
    }
    
    const DSPConfig& getConfig() const override {
        return config_;
    }
    
    float calculateRMS(const std::vector<int16_t>& pcm) const override {
        if (pcm.empty()) return 0.0f;
        
        float sum_squares = 0.0f;
        for (int16_t sample : pcm) {
            float normalized = sample / 32768.0f;
            sum_squares += normalized * normalized;
        }
        
        return std::sqrt(sum_squares / pcm.size());
    }
    
    bool hasClipping(const std::vector<int16_t>& pcm, float threshold) const override {
        int16_t clip_level = static_cast<int16_t>(32767 * threshold);
        
        for (int16_t sample : pcm) {
            if (std::abs(sample) >= clip_level) {
                return true;
            }
        }
        return false;
    }
    
private:
    // Design biquad coefficients for 2nd-order Butterworth highpass
    void designHighpass(float fc, float Q, uint32_t fs, BiquadState& bq) {
        float w0 = 2.0f * M_PI * fc / fs;
        float cos_w0 = std::cos(w0);
        float sin_w0 = std::sin(w0);
        float alpha = sin_w0 / (2.0f * Q);
        
        float a0 = 1.0f + alpha;
        bq.b0 = ((1.0f + cos_w0) / 2.0f) / a0;
        bq.b1 = (-(1.0f + cos_w0)) / a0;
        bq.b2 = ((1.0f + cos_w0) / 2.0f) / a0;
        bq.a1 = (-2.0f * cos_w0) / a0;
        bq.a2 = (1.0f - alpha) / a0;
        
        bq.reset();
    }
    
    // Design biquad coefficients for 2nd-order Butterworth lowpass
    void designLowpass(float fc, float Q, uint32_t fs, BiquadState& bq) {
        float w0 = 2.0f * M_PI * fc / fs;
        float cos_w0 = std::cos(w0);
        float sin_w0 = std::sin(w0);
        float alpha = sin_w0 / (2.0f * Q);
        
        float a0 = 1.0f + alpha;
        bq.b0 = ((1.0f - cos_w0) / 2.0f) / a0;
        bq.b1 = (1.0f - cos_w0) / a0;
        bq.b2 = ((1.0f - cos_w0) / 2.0f) / a0;
        bq.a1 = (-2.0f * cos_w0) / a0;
        bq.a2 = (1.0f - alpha) / a0;
        
        bq.reset();
    }
};

// Factory function
SignalProcessor* createSignalProcessor() {
    return new SignalProcessorESP();
}

} // namespace dsp