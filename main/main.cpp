#include "hal_audio/AudioSensorI2S.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


extern "C" 
{   
    void app_main(void);
}

using namespace hal_audio;

static const char* TAG = "MAIN";

AudioSensor* createI2SAudioSensor();

void audio_capture_task(void* pvParameters)
{
    AudioSensor* sensor = static_cast<AudioSensor*>(pvParameters);

    const size_t buffer_size = 1024;
    std::vector<int32_t>audio_buffer(buffer_size) ;

    while(true)
    {
        size_t bytes_read = 0;
        AudioStatus status = sensor->read(&audio_buffer,buffer_size,&bytes_read,1000);

        if(status == AudioStatus::OK & bytes_read > 0)
        {
            size_t samples_read = bytes_read/sizeof(int32_t);

            ESP_LOGI(TAG, "Read %d samples",samples_read);
            // TODO: Process audio data (send to DSP pipeline)
            // Example: dsp::process(audio_buffer, samples_read);

        }else if(status != AudioStatus::OK){
            ESP_LOGW(TAG, "Audio read error");
        }
        // Optional: Add small delay if needed
        // vTaskDelay(pdMS_TO_TICKS(10));
    }

    
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Audio catching");

    AudioSensor* sensor = createI2SAudioSensor();

    AudioConfig config = {
        .sample_rate = 16000, //16kHz
        .bits_per_sample = 32, //32 bits sample
        .channels = 1, //Mono
        .dma_buffer_count = 4, //4 buffer count
        .dma_buffer_len = 1024 //1024 buffer sample

    };

    AudioStatus status = sensor->init(config);

    if(status != AudioStatus::OK)
    {
        ESP_LOGE(TAG,"Failed to initalize sensor");
        return;
    }

    //start capturing
    status = sensor->start();

    if(status != AudioStatus::OK)
    {
        ESP_LOGE(TAG, "Failed to start audio sensor");
        return;
    }

    xTaskCreatePinnedToCore(
        audio_capture_task,
        "audio_capture",
        4096,
        sensor,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Audio Task created");
}

