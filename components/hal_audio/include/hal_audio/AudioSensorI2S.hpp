#pragma once

#include <cstdint>
#include <cstddef>

namespace hal_audio{

    enum class AudioStatus{

        OK,
        ERROR_INIT,
        ERROR_START,
        ERROR_READ,
        ERROR_STOP,
        NOT_INITIALIZED

    };

    struct AudioConfig {
    uint32_t sample_rate;      // e.g., 16000 Hz
    uint8_t bits_per_sample;   // e.g., 16 or 32
    uint8_t channels;          // 1 = mono, 2 = stereo
    size_t dma_buffer_count;   // Number of DMA buffers
    size_t dma_buffer_len;     // Length of each DMA buffer (in samples)
    };

    class AudioSensor {
        public:
        virtual ~AudioSensor() = default;

        // Initialize the audio sensor
        virtual AudioStatus init(const AudioConfig& config) = 0;
        
        // Start capturing audio
        virtual AudioStatus start() = 0;
        
        // Read audio samples (blocking or timeout-based)
        virtual AudioStatus read(int32_t* buffer, size_t samples, size_t* bytes_read, uint32_t timeout_ms = 1000) = 0;
        
        // Stop capturing audio
        virtual AudioStatus stop() = 0;
        
        // Deinitialize the sensor
        virtual AudioStatus deinit() = 0;
        
        // Get current configuration
        virtual const AudioConfig& getConfig() const = 0;
        
        // Check if sensor is initialized
        virtual bool isInitialized() const = 0;
    };
}