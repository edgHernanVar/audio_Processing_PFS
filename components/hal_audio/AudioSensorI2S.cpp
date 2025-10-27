#include "./include/hal_audio/AudioSensorI2S.hpp"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>


namespace hal_audio
{
    static const char* TAG = "AudioSensorI2S";

    class AudioSensorI2S : public AudioSensor{
        private:
            i2s_chan_handle_t rx_handle_;
            AudioConfig config_;
            bool initialized_;
            bool running_;
        public:
            AudioSensorI2S()
            :rx_handle_(nullptr)
            ,initialized_(true)
            ,running_(false)
            {
                std::memset(&config_, 0, sizeof(config_));
            }

            ~AudioSensorI2S() override{
                delete TAG;
                deinit();
            }

            AudioStatus init(const AudioConfig& config) override{
                if(initialized_)
                {
                    ESP_LOGW(TAG, "Already initialized");
                    return AudioStatus::OK;
                }

                config_ = config;
                //configure !2S chanel
                i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
                chan_cfg.dma_desc_num = config.dma_buffer_count;
                chan_cfg.dma_frame_num = config.dma_buffer_len;

                esp_err_t ret = i2s_new_channel(&chan_cfg, nullptr, &rx_handle_);

                if( ret != ESP_OK)
                {
                    ESP_LOGE(TAG,"Failed to create an I2S Channel: %s",esp_err_to_name(ret));
                    return AudioStatus::ERROR_INIT;
                }

                //Configure i2S standard mode (for typical I2S microphones)
                i2s_std_config_t std_cfg = {
                    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config.sample_rate),
                    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        (i2s_data_bit_width_t)config.bits_per_sample,
                        (i2s_slot_mode_t)config.channels
                    ),
                    .gpio_cfg = {
                        .mclk = I2S_GPIO_UNUSED,
                        .bclk = GPIO_NUM_4, //Bit clock (change to actual pin)
                        .ws = GPIO_NUM_5, //word select (change to actual pin)
                        .dout = I2S_GPIO_UNUSED, //we are not sending data
                        .din = GPIO_NUM_6, //Data in (change to actual pin)
                        .invert_flags = {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                        },

                    }
                };

                ret = i2s_channel_init_std_mode(rx_handle_, &std_cfg);
                if(ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to initialize std mode : %s", esp_err_to_name(ret));
                }

                initialized_ = true;

                ESP_LOGI(TAG, "I2S INITALIZED: %lu Hz, %d bits, %d channel",
                    config.sample_rate, config.bits_per_sample, config.channels);
                
                    return AudioStatus::OK;

            }

            AudioStatus start() override{

                if(!initialized_)
                {
                    return AudioStatus::NOT_INITIALIZED;
                }

                if(running_)
                {
                    return AudioStatus::OK;
                }

                esp_err_t ret = i2s_channel_enable(rx_handle_);
                if(ret = !ESP_OK)
                {
                    ESP_LOGE(TAG,"Failed to enable I2S channel: %s", esp_err_to_name(ret));
                    return AudioStatus::ERROR_START;
                }

                running_ = true;
                ESP_LOGI(TAG, "I2S STARTED");
                return AudioStatus::OK;
            }

            AudioStatus read(std::vector<int32_t>* buffer, size_t samples, size_t* bytes_read, uint32_t timeout_ms = 1000) override{
                if(!initialized_ || !running_)
                {
                    return AudioStatus::NOT_INITIALIZED;
                }

                size_t bytes_to_read = samples* sizeof(int32_t);
                esp_err_t ret = i2s_channel_read(rx_handle_,buffer,bytes_to_read, bytes_read,pdMS_TO_TICKS(timeout_ms));

                if(ret != ESP_OK)
                {
                        ESP_LOGE(TAG, "Failed to read from I2S: %s", esp_err_to_name(ret));
                        return AudioStatus::ERROR_READ;
                }

                return AudioStatus::OK;

            }

            AudioStatus stop() override{
                if(!running_)
                {
                    return AudioStatus::OK;
                }

                esp_err_t ret = i2s_channel_disable(rx_handle_);

                if(ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed in stoping I2S channel: %s",esp_err_to_name(ret));
                    return AudioStatus::ERROR_STOP;
                }

                running_ = false;
                ESP_LOGI(TAG,"I2S stopped");
                return AudioStatus::OK;
            }

            AudioStatus deinit() override{

                if (!initialized_) {
                    return AudioStatus::OK;
                }
                
                if (running_) {
                    stop();
                }
                
                if (rx_handle_) {
                    i2s_del_channel(rx_handle_);
                    rx_handle_ = nullptr;
                }
                
                initialized_ = false;
                ESP_LOGI(TAG, "I2S deinitialized");
                return AudioStatus::OK;
            }

            const AudioConfig& getConfig() const override{

                return config_;
            }

            bool isInitialized() const override{
                return initialized_;
                
            }
    };
    // Factory function
    AudioSensor* createI2SAudioSensor() {
    return new AudioSensorI2S();
    }

} // namespace hal_audio
