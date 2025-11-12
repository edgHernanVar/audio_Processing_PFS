#include "hal_audio/AudioSensorI2S.hpp"
#include "dsp/SignalProcesor.hpp"
#include "features/FeatureExtractor.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <vector>
#include <cmath>
#include <stdio.h>

extern "C" 
{   
    void app_main(void);
}

//using namespace hal_audio;

static const char* TAG = "MAIN";

//Structure for passing audio data between tasks
struct AudioFrame{
    std::vector<int16_t> samples;
    uint64_t timestamp;
    float rms_energy;
};

struct PipelineConfig{
    uint32_t sample_rate = 16000;
    size_t capture_frames_ms = 32; //32ms frames for low latency
    size_t inference_window_ms = 1000; //1 second analysis window   
};

struct FeatureFrame{
    features::FeatureVector features;
    uint64_t timestamp_us;
};

//global queue for inter-task communication
static QueueHandle_t audio_queue = nullptr;
static QueueHandle_t feature_queue = nullptr;
//AudioSensor* createI2SAudioSensor();

// Function to print memory information
void print_memory_info() {
    ESP_LOGI(TAG, "=== Memory Info ===");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap: %lu bytes", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "Free internal: %lu bytes", 
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Free PSRAM: %lu bytes", 
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Largest free block: %lu bytes", 
             heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
}
void audio_capture_task(void* pvParameters)
{
    hal_audio::AudioSensor* sensor = static_cast<hal_audio::AudioSensor*>(pvParameters);
    dsp::SignalProcessor* dsp_processor = dsp::createSignalProcessor();

    if(!sensor)
    {
        ESP_LOGE(TAG, "Audio sensor is null");
        vTaskDelete(NULL);
        return;
    }

    if(!dsp_processor)
    {
        ESP_LOGE(TAG, "Failed to create DSP processor");
        vTaskDelete(NULL);
        return;
    }

    //configure DSP for Glass breaker application
    dsp::DSPConfig dsp_config {
        .sample_rate = 16000,//match I2S sample rate
        .enable_dc_removal = true,
        //remove low freq rumble(doors, steps)

        .hpf_cutoff_hz = 200.0f, // Higher - glass breaking starts ~200 Hz
        .hpf_q = 0.707f,

        //keep high freq (glass breaking is 2-15 kHz)
        .lpf_cutoff_hz = 8000.0f, //Nyquist limit at 16kHz
        .lpf_q = 0.707f,

        //disable pre-emphasis here, MFCC will handle it
        .pre_emphasis_alpha = 0.02f,//let the MFCC handle pre-emphasis

        //moderate gain (don't over amplify noise)
        .target_gain_db = 3.0f, //Boost quiet sounds
        .auto_normalize = false
    };
    

    if(dsp_processor->init(dsp_config) != dsp::ProcessStatus::OK)
    {
        ESP_LOGE(TAG, "Failed to initialize DSP processor");
        delete dsp_processor;
        vTaskDelete(NULL);
        return;
    }

    const size_t frame_size = 512; //32ms at 16kHz

    std::vector<int32_t> i2s_buffer;
    i2s_buffer.reserve(frame_size * 2);//reserve extra space
    i2s_buffer.resize(frame_size );//set initial size

    //pcm buffer after DSP processing
    std::vector<int16_t> pcm_buffer;
    pcm_buffer.reserve(frame_size * 2);//reserve extra space

    ESP_LOGI(TAG, "Audio capture task started");
    ESP_LOGI(TAG, "I2S buffer: %d samples, PCM buffer capacity: %d", 
             i2s_buffer.size(), pcm_buffer.capacity());

    // ✅ Detection variables
    uint32_t frame_count = 0;
    float baseline_rms = 0.005f;      // Normal room noise level
    float rms_history[10] = {0};      // Keep last 10 frames
    int history_index = 0;
    
    // Thresholds for glass breaking
    const float ONSET_THRESHOLD = 0.08f;      // Sudden increase NOTE: lowered from 0.2f because too sensitive
    const float PEAK_THRESHOLD = 0.15f;       // Peak during event
    const float RATIO_THRESHOLD = 15.0f;      // RMS increase  // increased from 10.0f for better sensitivity
    

    uint32_t detection_cooldown = 0;
   

   
    
    while(true)
    {
       
        // Read I2S data
        size_t bytes_read = 0;
        

        hal_audio::AudioStatus status = sensor->read(
            i2s_buffer.data(),
            frame_size,
            &bytes_read,
            1000 // 1 second timeout
        );
        

        if(status != hal_audio::AudioStatus::OK || bytes_read == 0){

            ESP_LOGW(TAG, "No audio data read, status: %d", static_cast<int>(status));
            vTaskDelay(pdMS_TO_TICKS(100)); // Retry reading

        }else if(status == hal_audio::AudioStatus::OK && bytes_read > 0){

            ESP_LOGD(TAG, "Read %d bytes from I2S AND STATUS OK", bytes_read);

            //process I2S data through DSP
            size_t samples_read = bytes_read / sizeof(int32_t);
            //resize buffer to actual samples read
            i2s_buffer.resize(samples_read);
            //process I2S data through DSP
            dsp::ProcessStatus dsp_status = dsp_processor->processI2S(i2s_buffer,pcm_buffer);

            if(dsp_status != dsp::ProcessStatus::OK)
            {
                ESP_LOGE(TAG, "DSP processing error");
               
            }else{
                float rms = dsp_processor->calculateRMS(pcm_buffer);

                if(rms < baseline_rms * 2.0f)
                {
                    //Update baseline using moving average
                    baseline_rms = 0.9f * baseline_rms + 0.1f * rms;
                }

                rms_history[history_index] = rms;
                history_index = (history_index + 1) % 10;//using modulo for circular buffer
                
                
                if(detection_cooldown > 0)
                {
                    detection_cooldown--;
                }else{
                    float rms_ratio = rms / (baseline_rms + 1e-3f);//lets first try with 3 digits

                    //Check for sudden onset
                    bool sudden_onset = (rms> ONSET_THRESHOLD) &&
                                        (rms_ratio > RATIO_THRESHOLD);
                    
                    //Check for peak energy
                    bool high_peak = (rms > PEAK_THRESHOLD);

                    //calculate peak amplitude
                    int16_t peak_amplitude = 0;
                    for(auto sample : pcm_buffer)
                    {
                        if(std::abs(sample) > peak_amplitude)
                        {
                            peak_amplitude = std::abs(sample);
                        }
                    }

                    if( sudden_onset || high_peak)
                    {
                        ESP_LOGW(TAG, "🔔 GLASS BREAKING DETECTED!");
                        ESP_LOGI(TAG, "  RMS: %.4f (baseline: %.4f, ratio: %.1fx)", 
                                    rms, baseline_rms, rms_ratio);
                        ESP_LOGI(TAG, "  Peak amplitude: %d", peak_amplitude);
                        
                        // TODO: Trigger full 1-second capture for MFCC analysis
                        
                        AudioFrame frame{
                        .samples = pcm_buffer,
                        .timestamp = (uint64_t)esp_timer_get_time(),
                        .rms_energy = rms
                        };

                        if(xQueueSend(audio_queue, &frame, 0) != pdTRUE)
                        {
                            ESP_LOGW(TAG, "Audio queue full, dropping frame");
                            //Restore pcm_buffer size for next read
                            //pcm_buffer = std::move(frame.samples);
                        }
                        
                        // TODO: Send to ML classifier
                        
                        // Cooldown to avoid multiple detections
                        detection_cooldown = 50;  // ~800ms cooldown
                    }

                    
                }

                //for mfcc extraction task
                
                

                //check for clipping
                if(dsp_processor->hasClipping(pcm_buffer))
                {
                    ESP_LOGW(TAG, "Clipping detected in audio frame");
                }

                //Periodic status log
                if (++frame_count % 50 == 0) {
                    ESP_LOGI(TAG, "Frame %lu: RMS %.4f, Baseline %.4f, Peak/Base %.1fx", 
                            frame_count, rms, baseline_rms, rms / (baseline_rms + 0.001f));
                }
                
            }

            
            
            i2s_buffer.resize(frame_size); //reset buffer size for next read

            // Optional: Add small delay if needed
            vTaskDelay(pdMS_TO_TICKS(10));

        }

    }

    

    delete dsp_processor;
    
    vTaskDelete(NULL);
}

//task 2 medium priority
void feature_extraction_task(void* Pvparameters)
{

    features::FeatureExtractor* mfcc = features::createMFCCExtractor();

    //config MFCC
    features::FeatureConfig mfcc_config = features::defaultEnvironmentFeatureConfig();
    mfcc_config.lower_frequency_hz = 200.0f; //match DSP high-pass
    mfcc_config.include_delta = true;
    mfcc_config.include_delta_delta = true;

    if(mfcc->init(mfcc_config) != features::FeatureStatus::OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MFCC extractor");
        vTaskDelete(NULL);
        delete mfcc;
        return;
    }

    const size_t window_size = 16000; //1 second at 16kHz
    std::vector<int16_t> audio_window;
    audio_window.reserve(window_size + 512); //extra space for overlap

    ESP_LOGI(TAG, "Feature extraction task started");
    AudioFrame frame;
    uint32_t inference_count = 0;
    while(true)
    {
        if(xQueueReceive(audio_queue, &frame, portMAX_DELAY) == pdTRUE)
        {
            const float VAD_THRESHOLD = 0.015f; //adjust as needed
            
            if(frame.rms_energy < VAD_THRESHOLD)
            {
                if(!audio_window.empty())
                {
                    ESP_LOGD(TAG, "Silence detected, clearing audio window");
                    audio_window.clear();
                }
            }else{
                audio_window.insert(
                    audio_window.end(),
                    frame.samples.begin(),
                    frame.samples.end()
                );
                if(audio_window.size() >= window_size)
                {
                    audio_window.erase(
                        audio_window.begin(),
                        audio_window.begin() + (audio_window.size() - window_size)
                    );

                    ESP_LOGI(TAG, "Performing MFCC extraction on %d samples", audio_window.size());
                    features::FeatureVector feature_vector;
                    features::FeatureStatus feat_status = mfcc->compute(audio_window, feature_vector);

                    if(feat_status != features::FeatureStatus::OK)
                    {
                        ESP_LOGE(TAG, "MFCC extraction error: %d", static_cast<int>(feat_status));
                    }else{

                        ESP_LOGI(TAG, "Extracted %d frames x %d coeffs = %d features",
                                 feature_vector.num_frames,
                                 feature_vector.num_coefficients,
                                 feature_vector.size());

                        //send to next stage (classifier)
                        FeatureFrame feat_frame{
                            .features = std::move(feature_vector),
                            .timestamp_us = (uint64_t)esp_timer_get_time()
                        };

                        if(xQueueSend(feature_queue, &feat_frame, 0) != pdTRUE)
                        {
                            ESP_LOGW(TAG, "Feature queue full, dropping frame");
                        }else{
                            inference_count++;
                        }

                        inference_count++;
                        ESP_LOGI(TAG, "Completed inference %d", inference_count);   
                    }
                    
                    audio_window.clear();
                }   
            }
        }
    }

    delete mfcc;
    vTaskDelete(NULL);
}

void ml_inference_task(void* pvParameters) {
    ESP_LOGI(TAG, "ML inference task started");
    
    // TODO: Initialize TensorFlow Lite Micro model
    // ml::Classifier* classifier = createClassifier();
    // classifier->init("model.tflite");
    
    FeatureFrame frame;
    
    while (true) {
        if (xQueueReceive(feature_queue, &frame, portMAX_DELAY) == pdTRUE) {
            
            ESP_LOGI(TAG, "Running inference on %d features", frame.features.size());
            
            // TODO: Run ML inference
            // auto result = classifier->classify(frame.features.data.data(),
            //                                    frame.features.size());
            
            // For now, just log
            ESP_LOGI(TAG, "Features shape: [%d, %d]",
                     frame.features.num_frames,
                     frame.features.num_coefficients);
            
            // Check quality
            if (frame.features.contains_speech) {
                ESP_LOGI(TAG, "Speech detected! Energy: mean=%.4f std=%.4f",
                         frame.features.energy_mean,
                         frame.features.energy_std);
            }
            
            // Simulate inference time
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // TODO: Send results to event filter and publisher
        }
    }
    
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Audio catching");

    print_memory_info();

    hal_audio::AudioSensor* sensor = hal_audio::createI2SAudioSensor();

    if(!sensor)
    {
        ESP_LOGE(TAG, "Failed to create I2S audio sensor");
        return;
    }

    hal_audio::AudioConfig config = {
        .sample_rate = 16000, //16kHz
        .bits_per_sample = 32, //32 bits sample
        .channels = 1, //Mono
        .dma_buffer_count = 4, //4 buffer count
        .dma_buffer_len = 512 //512 buffer sample

    };

    hal_audio::AudioStatus status = sensor->init(config);

    if(status != hal_audio::AudioStatus::OK)
    {
        ESP_LOGE(TAG,"Failed to initalize sensor");
        delete sensor;
        return;
    }

    //start capturing
    status = sensor->start();

    if(status != hal_audio::AudioStatus::OK)
    {
        ESP_LOGE(TAG, "Failed to start audio sensor");
        sensor->deinit();
        delete sensor;
        return;
    }

    ESP_LOGI(TAG, "Audio sensor initialized successfully");

    xTaskCreatePinnedToCore(
        audio_capture_task,
        "audio_capture",
        8192,
        sensor,
        10,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Audio Task created");
    xTaskCreatePinnedToCore(
        feature_extraction_task,
        "mfcc_extract",
        12288,
        NULL,
        6,   // Medium priority
        NULL,
        1
    );
    
    xTaskCreatePinnedToCore(
        ml_inference_task,
        "ml_inference",
        16384,
        NULL,
        3,   // Lower priority
        NULL,
        1
    );
    
    ESP_LOGI(TAG, "Pipeline started successfully!");
    ESP_LOGI(TAG, "Audio → DSP → MFCC → ML → Results");

    print_memory_info();        
}

