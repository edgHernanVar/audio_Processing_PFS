// components/features/MFCCExtractorESP.cpp
#include "./include/features/FeatureExtractor.hpp"
#include "FFTHelperESPDSP.hpp"  // Use optimized ESP-DSP FFT
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <memory>
#include "esp_mac.h"
#include "esp_system.h"

namespace features {

static const char* TAG = "MFCCExtractor";

class MFCCExtractorESP : public FeatureExtractor {
private:
    FeatureConfig config_;
    bool initialized_;
    
    // Computed parameters
    int frame_length_samples_;
    int frame_stride_samples_;
    int num_frames_;
    
    // Mel filterbank
    float** mel_filterbank_;
    int num_mel_bins_;
    int fft_bins_;
    
    // DCT matrix for MFCC
    std::vector<std::vector<float>> dct_matrix_;
    
    // Working buffers
    std::unique_ptr<FFTHelperESPDSP> fft_helper_;
    std::vector<float> power_spectrum_;
    std::vector<float> mel_energies_;
    std::vector<float> frame_buffer_;
    
    // Normalization statistics (computed from data)
    std::vector<float> feature_mean_;
    std::vector<float> feature_std_;
    bool have_stats_;
    
public:
    MFCCExtractorESP() 
        : initialized_(false)
        , frame_length_samples_(0)
        , frame_stride_samples_(0)
        , num_frames_(0)
        , mel_filterbank_(nullptr)  // <-- Moved up
        , num_mel_bins_(0)
        , fft_bins_(0)
        , have_stats_(false)
        
    {
        std::memset(&config_, 0, sizeof(config_));
    }
    ~MFCCExtractorESP() override {
        // Cleanup
        if (mel_filterbank_) {
            for (int i = 0; i < num_mel_bins_; ++i) {
                heap_caps_free(mel_filterbank_[i]);
            }
            heap_caps_free(mel_filterbank_);
        }
    }
    
    FeatureStatus init(const FeatureConfig& config) override {
        if (config.sample_rate == 0 || config.num_mfcc == 0) {
            ESP_LOGE(TAG, "Invalid configuration");
            return FeatureStatus::ERROR_INVALID_CONFIG;
        }
        
        config_ = config;
        
        // Calculate frame parameters
        frame_length_samples_ = (config_.sample_rate * config_.frame_length_ms) / 1000;
        frame_stride_samples_ = (config_.sample_rate * config_.frame_stride_ms) / 1000;
        
        ESP_LOGI(TAG, "Frame: %d samples (%.1f ms), Stride: %d samples (%.1f ms)",
                 frame_length_samples_, config_.frame_length_ms,
                 frame_stride_samples_, config_.frame_stride_ms);
        
        // Initialize FFT
        fft_helper_ = std::make_unique<FFTHelperESPDSP>(config_.fft_length);
        fft_bins_ = config_.fft_length / 2 + 1;
        power_spectrum_.resize(fft_bins_);
        
        // Build mel filterbank
        if(!buildMelFilterbank()){
            ESP_LOGE(TAG, "Failed to allocate mel filterbank - out of memory");
            return FeatureStatus::ERROR_INVALID_CONFIG;
        }
        
        mel_energies_.resize(config_.num_mel_bins);
        
        // Build DCT matrix if using MFCC (not log-mel)
        if (!config_.use_log_mel) {
            buildDCTMatrix();
        }
        
        // Allocate frame buffer
        frame_buffer_.resize(frame_length_samples_);
        
        // Initialize normalization stats
        int total_coeffs = config_.num_mfcc;
        if (config_.include_delta) total_coeffs += config_.num_mfcc;
        if (config_.include_delta_delta) total_coeffs += config_.num_mfcc;
        
        feature_mean_.resize(total_coeffs, 0.0f);
        feature_std_.resize(total_coeffs, 1.0f);
        
        initialized_ = true;
        ESP_LOGI(TAG, "MFCC initialized: %d coeffs, %d mel bins, %d FFT",
                 config_.num_mfcc, config_.num_mel_bins, config_.fft_length);
        
        return FeatureStatus::OK;
    }
    
    FeatureStatus compute(const std::vector<int16_t>& pcm, FeatureVector& out) override {
        if (!initialized_) {
            return FeatureStatus::ERROR_NOT_INITIALIZED;
        }

        if(pcm.size() < static_cast<size_t>(frame_length_samples_)){
            ESP_LOGE(TAG, "Insufficient audio data: %d samples (need >= %d)",
                     pcm.size(), frame_length_samples_);
            return FeatureStatus::ERROR_INSUFFICIENT_DATA;
        }

       
        
        // Convert int16 to float
        std::vector<float> audio(pcm.size());
        for (size_t i = 0; i < pcm.size(); ++i) {
            audio[i] = pcm[i] / 32768.0f;
        }
        
        return compute(audio, out);
    }
    
    FeatureStatus compute(const std::vector<float>& audio, FeatureVector& out) override {
        if (!initialized_) {
            return FeatureStatus::ERROR_NOT_INITIALIZED;
        }
        
        if (audio.size() < static_cast<size_t>(frame_length_samples_)) {
            ESP_LOGE(TAG, "Insufficient audio data: %d samples (need >= %d)",
                     audio.size(), frame_length_samples_);
            return FeatureStatus::ERROR_INSUFFICIENT_DATA;
        }
        
        // Calculate number of frames
        num_frames_ = 1 + (audio.size() - frame_length_samples_) / frame_stride_samples_;
        
        // Allocate output
        int coeffs_per_frame = config_.use_log_mel ? config_.num_mel_bins : config_.num_mfcc;
        out.data.resize(num_frames_ * coeffs_per_frame);
        out.num_frames = num_frames_;
        out.num_coefficients = coeffs_per_frame;
        out.timestamp_us = esp_timer_get_time();
        
        // Temporary storage for MFCC before deltas
        std::vector<float> mfcc_base;
        if (config_.include_delta || config_.include_delta_delta) {
            mfcc_base.resize(num_frames_ * coeffs_per_frame);
        }
        
        // Extract features for each frame
        for (int frame = 0; frame < num_frames_; ++frame) {
            int start_idx = frame * frame_stride_samples_;
            
            // Extract frame
            for (int i = 0; i < frame_length_samples_; ++i) {
                frame_buffer_[i] = audio[start_idx + i];
            }
            
            // Apply pre-emphasis if configured
            if (config_.pre_emphasis_coeff > 0.0f) {
                applyPreEmphasis(frame_buffer_.data(), frame_length_samples_);
            }
            
            // Compute power spectrum via FFT
            fft_helper_->computePowerSpectrum(frame_buffer_.data(), 
                                             power_spectrum_.data(),
                                             frame_length_samples_);
            
            // Apply mel filterbank
            applyMelFilterbank(power_spectrum_.data(), mel_energies_.data());
            
            // Compute log mel energies
            for (int i = 0; i < config_.num_mel_bins; ++i) {
                mel_energies_[i] = std::log(mel_energies_[i] + 1e-10f);  // Add epsilon
            }
            
            // Store features
            float* frame_out = &out.data[frame * coeffs_per_frame];
            
            if (config_.use_log_mel) {
                // Use log-mel directly
                std::memcpy(frame_out, mel_energies_.data(), 
                           config_.num_mel_bins * sizeof(float));
            } else {
                // Apply DCT to get MFCC
                applyDCT(mel_energies_.data(), frame_out);
                
                // Store for delta computation if needed
                if (config_.include_delta || config_.include_delta_delta) {
                    std::memcpy(&mfcc_base[frame * coeffs_per_frame], 
                               frame_out, 
                               coeffs_per_frame * sizeof(float));
                }
            }
        }
        
        // Compute deltas if requested
        if (config_.include_delta || config_.include_delta_delta) {
            // Need to expand output to include deltas
            int total_coeffs = coeffs_per_frame;
            if (config_.include_delta) total_coeffs += coeffs_per_frame;
            if (config_.include_delta_delta) total_coeffs += coeffs_per_frame;
            
            std::vector<float> full_features(num_frames_ * total_coeffs);
            
            // Copy base MFCC
            for (int frame = 0; frame < num_frames_; ++frame) {
                std::memcpy(&full_features[frame * total_coeffs],
                           &mfcc_base[frame * coeffs_per_frame],
                           coeffs_per_frame * sizeof(float));
            }
            
            // Compute delta
            if (config_.include_delta) {
                computeDelta(mfcc_base, full_features, coeffs_per_frame, coeffs_per_frame);
            }
            
            // Compute delta-delta
            if (config_.include_delta_delta) {
                int delta_offset = config_.include_delta ? coeffs_per_frame * 2 : coeffs_per_frame;
                std::vector<float> delta_features(num_frames_ * coeffs_per_frame);
                
                // Extract delta features
                for (int frame = 0; frame < num_frames_; ++frame) {
                    std::memcpy(&delta_features[frame * coeffs_per_frame],
                               &full_features[frame * total_coeffs + coeffs_per_frame],
                               coeffs_per_frame * sizeof(float));
                }
                
                computeDelta(delta_features, full_features, coeffs_per_frame, delta_offset);
            }
            
            out.data = std::move(full_features);
            out.num_coefficients = total_coeffs;
        }
        
        // Apply normalization if configured
        if (config_.normalize_features && have_stats_) {
            normalizeFeatures(out);
        }
        
        // Compute statistics
        computeStatistics(out);
        
        ESP_LOGD(TAG, "Extracted %d frames x %d coeffs = %d features",
                 out.num_frames, out.num_coefficients, out.size());
        
        return FeatureStatus::OK;
    }
    
    int getExpectedFrames(size_t num_samples) const override {
        if (num_samples < static_cast<size_t>(frame_length_samples_)) {
            return 0;
        }
        return 1 + (num_samples - frame_length_samples_) / frame_stride_samples_;
    }
    
    const FeatureConfig& getConfig() const override {
        return config_;
    }
    
    bool isInitialized() const override {
        return initialized_;
    }
    
    void reset() override {
        // Clear any cached state if needed
        have_stats_ = false;
    }
    
private:
    // Convert frequency to mel scale
    float hzToMel(float hz) {
        return 2595.0f * std::log10(1.0f + hz / 700.0f);
    }
    
    // Convert mel scale to frequency
    float melToHz(float mel) {
        return 700.0f * (std::pow(10.0f, mel / 2595.0f) - 1.0f);
    }
    
    // Build mel filterbank
    bool buildMelFilterbank() {
        float low_mel = hzToMel(config_.lower_frequency_hz);
        float high_mel = hzToMel(config_.upper_frequency_hz);
        
        // Mel points evenly spaced
        std::vector<float> mel_points(config_.num_mel_bins + 2);
        for (int i = 0; i < config_.num_mel_bins + 2; ++i) {
            mel_points[i] = low_mel + (high_mel - low_mel) * i / (config_.num_mel_bins + 1);
        }
        
        // Convert back to Hz
        std::vector<float> hz_points(config_.num_mel_bins + 2);
        for (int i = 0; i < config_.num_mel_bins + 2; ++i) {
            hz_points[i] = melToHz(mel_points[i]);
        }
        
        // Convert to FFT bin numbers
        std::vector<int> bin_points(config_.num_mel_bins + 2);
        int fft_bins = config_.fft_length / 2 + 1;
        for (int i = 0; i < config_.num_mel_bins + 2; ++i) {
            bin_points[i] = static_cast<int>(
                (config_.fft_length + 1) * hz_points[i] / config_.sample_rate
            );
        }
        
        
        //calculate memory needed
        num_mel_bins_ = config_.num_mel_bins;
        size_t bytes_per_bin = fft_bins * sizeof(float);
        size_t total_bytes = bytes_per_bin * num_mel_bins_;

        //allocate mel filterbank
        mel_filterbank_ = (float**)heap_caps_malloc(num_mel_bins_ * sizeof(float*), MALLOC_CAP_SPIRAM);

        if(!mel_filterbank_){
            ESP_LOGE(TAG, "Failed to allocate mel filterbank pointers");
           
            return false;
        }
        for (int i = 0; i < num_mel_bins_; ++i) {
            mel_filterbank_[i] = (float*)heap_caps_malloc(
                bytes_per_bin,
                MALLOC_CAP_SPIRAM
            );
        
            if (!mel_filterbank_[i]) {
                ESP_LOGE(TAG, "Failed to allocate mel bin %d", i);
                // Clean up previous allocations
                for (int j = 0; j < i; ++j) {
                    heap_caps_free(mel_filterbank_[j]);
                }
                heap_caps_free(mel_filterbank_);
                mel_filterbank_ = nullptr;
                return false;
            }
        
            // Initialize to zero
            memset(mel_filterbank_[i], 0, bytes_per_bin);
        }

        
        for (int i = 0; i < num_mel_bins_; ++i) {
            int left = bin_points[i];
            int center = bin_points[i + 1];
            int right = bin_points[i + 2];
            
            // Bounds check
            if (left < 0) left = 0;
            if (center >= fft_bins_) center = fft_bins_ - 1;
            if (right >= fft_bins_) right = fft_bins_;
            
            // Rising slope
            for (int j = left; j < center && j < fft_bins_; ++j) {
                mel_filterbank_[i][j] = static_cast<float>(j - left) / (center - left);
            }
            
            // Falling slope
            for (int j = center; j < right && j < fft_bins_; ++j) {
                mel_filterbank_[i][j] = static_cast<float>(right - j) / (right - center);
            }
        }
        //Check memory befire allocation
        size_t needed_bytes = config_.num_mel_bins * fft_bins * sizeof(float);
        ESP_LOGI(TAG, "Allocating %.2f KB for mel filterbank",
                 needed_bytes / 1024.0f);   

        
        
        // Build triangular filters
        
        
        ESP_LOGI(TAG, "Built mel filterbank: %d bins from %.0f to %.0f Hz",
                 config_.num_mel_bins, config_.lower_frequency_hz, config_.upper_frequency_hz);
        return true;
    }
    
    // Build DCT matrix
    void buildDCTMatrix() {
        dct_matrix_.resize(config_.num_mfcc);
        
        float norm = std::sqrt(2.0f / config_.num_mel_bins);
        
        for (int i = 0; i < config_.num_mfcc; ++i) {
            dct_matrix_[i].resize(config_.num_mel_bins);
            for (int j = 0; j < config_.num_mel_bins; ++j) {
                dct_matrix_[i][j] = norm * std::cos(M_PI * i * (j + 0.5f) / config_.num_mel_bins);
            }
        }
    }
    
    // Apply pre-emphasis filter
    void applyPreEmphasis(float* signal, int length) {
        for (int i = length - 1; i > 0; --i) {
            signal[i] -= config_.pre_emphasis_coeff * signal[i - 1];
        }
    }
    
    // Apply mel filterbank to power spectrum
    void applyMelFilterbank(const float* power_spectrum, float* mel_energies) {
        for (int i = 0; i < num_mel_bins_; ++i) {
            mel_energies[i] = 0.0f;
            for (size_t j = 0; j < fft_bins_; ++j) {
                mel_energies[i] += power_spectrum[j] * mel_filterbank_[i][j];
            }
        }
    }   
    
    // Apply DCT to get MFCC
    void applyDCT(const float* mel_energies, float* mfcc) {
        for (int i = 0; i < config_.num_mfcc; ++i) {
            mfcc[i] = 0.0f;
            for (int j = 0; j < config_.num_mel_bins; ++j) {
                mfcc[i] += dct_matrix_[i][j] * mel_energies[j];
            }
        }
    }
    
    // Compute delta features
    void computeDelta(const std::vector<float>& features, 
                     std::vector<float>& output,
                     int coeffs_per_frame,
                     int output_offset) {
        const int N = 2;  // Context window (standard for speech)
        
        for (int frame = 0; frame < num_frames_; ++frame) {
            for (int coeff = 0; coeff < coeffs_per_frame; ++coeff) {
                float delta = 0.0f;
                float norm = 0.0f;
                
                for (int n = -N; n <= N; ++n) {
                    int neighbor_frame = frame + n;
                    
                    // Handle boundaries by replication
                    if (neighbor_frame < 0) neighbor_frame = 0;
                    if (neighbor_frame >= num_frames_) neighbor_frame = num_frames_ - 1;
                    
                    delta += n * features[neighbor_frame * coeffs_per_frame + coeff];
                    norm += n * n;
                }
                
                output[frame * output.size() / num_frames_ + output_offset + coeff] = 
                    norm > 0 ? delta / norm : 0.0f;
            }
        }
    }
    
    // Normalize features
    void normalizeFeatures(FeatureVector& features) {
        for (int frame = 0; frame < features.num_frames; ++frame) {
            for (int coeff = 0; coeff < features.num_coefficients; ++coeff) {
                int idx = frame * features.num_coefficients + coeff;
                features.data[idx] = (features.data[idx] - feature_mean_[coeff]) / 
                                     (feature_std_[coeff] + 1e-8f);
            }
        }
    }
    
    // Compute statistics for debugging
    void computeStatistics(FeatureVector& features) {
        // Compute energy statistics
        float energy_sum = 0.0f;
        float energy_sq_sum = 0.0f;
        
        for (int frame = 0; frame < features.num_frames; ++frame) {
            float frame_energy = 0.0f;
            for (int coeff = 0; coeff < features.num_coefficients; ++coeff) {
                float val = features.at(frame, coeff);
                frame_energy += val * val;
            }
            energy_sum += frame_energy;
            energy_sq_sum += frame_energy * frame_energy;
        }
        
        features.energy_mean = energy_sum / features.num_frames;
        features.energy_std = std::sqrt(energy_sq_sum / features.num_frames - 
                                       features.energy_mean * features.energy_mean);
        
        // Simple VAD: check if energy is above threshold
        features.contains_speech = features.energy_mean > 0.01f;  // Adjust threshold
    }
};

// Factory function
FeatureExtractor* createMFCCExtractor() {
    return new MFCCExtractorESP();
}

} // namespace features