#include "hal_audio/AudioSensorI2S.hpp"
#include "dsp/SignalProcesor.hpp"
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

//global queue for inter-task communication
static QueueHandle_t audio_queue = nullptr;
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


    uint32_t frame_count = 0;

   
    
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

                
                //for mfcc extraction task
                AudioFrame frame{
                        .samples = pcm_buffer,
                        .timestamp = (uint64_t)esp_timer_get_time(),
                        .rms_energy = rms
                };
                /*
                if(xQueueSend(audio_queue, &frame, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Audio queue full, dropping frame");
                    //Restore pcm_buffer size for next read
                    //pcm_buffer = std::move(frame.samples);
                }
                */

                //check for clipping
                if(dsp_processor->hasClipping(pcm_buffer))
                {
                    ESP_LOGW(TAG, "Clipping detected in audio frame");
                }

                //Periodic status log
                if (++frame_count % 50 == 0) {
                ESP_LOGI(TAG, "Frame %lu: %d samples, RMS: %.4f", 
                         frame_count, pcm_buffer.size(), rms);
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

void inference_task(void* pvParameters)
{
    ESP_LOGI(TAG, "Inference task started");
    const size_t sample_rate = 16000;
    const size_t window_size = sample_rate + 1; //1 second window
    std::vector<int16_t> audio_window;
    audio_window.reserve(window_size);

    AudioFrame frame;
    // Placeholder for inference logic
    while(true)
    {
        AudioFrame frame;
        if(xQueueReceive(audio_queue, &frame, portMAX_DELAY) == pdTRUE)
        {
            audio_window.insert(audio_window.end(),
             frame.samples.begin(),
             frame.samples.end()
            );


            if(audio_window.size() >= window_size)
            {
                // Perform inference on received audio frame
                ESP_LOGI(TAG, "Processing %d samples for inference", audio_window.size());

                float window_energy = 0.0f;
                for(int16_t sample : audio_window)
                {
                    float normalized = sample / 32768.0f;
                    window_energy += normalized * normalized;
                }

                window_energy = std::sqrt(window_energy / audio_window.size());

                const float VAD_THRESHOLD = 0.02f; //example threshold

                if(window_energy > VAD_THRESHOLD)
                {
                     ESP_LOGI(TAG, "Sound detected (energy: %.4f) - ready for MFCC extraction", 
                             window_energy);

                    // TODO: Pass to MFCC extractor
                    // features::FeatureExtractor* extractor = ...
                    // extractor->extract(audio_window);
                    
                    // TODO: Pass features to ML classifier
                    // ml::Classifier* classifier = ...
                    // classifier->classify(features);
                    
                }else{
                    ESP_LOGI(TAG, "Silence (energy: %.4f)", window_energy);
                }
                audio_window.clear();
            }
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
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Audio Task created");

    print_memory_info();        
}

