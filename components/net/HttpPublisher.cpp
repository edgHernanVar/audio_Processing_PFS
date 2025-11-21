// components/net/HttpPublisher.cpp
#include "net/Publisher.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "cJSON.h"
#include <cstring>
#include <ctime>


extern "C" {
    #include "esp_http_client.h"
}

namespace net {

static const char* TAG = "HttpPublisher";

// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

class HttpPublisher : public Publisher {
private:
    std::vector<WiFiCredentials> credentials_;
    ServerConfig server_config_;
    EventGroupHandle_t wifi_event_group_;
    int current_credential_index_;
    bool initialized_;
    bool connected_;
    bool wifi_started_;
    int retry_count_;
    
public:
    HttpPublisher()
        : wifi_event_group_(nullptr)
        , current_credential_index_(-1)
        , initialized_(false)
        , connected_(false)
        , wifi_started_(false)
        , retry_count_(0)
    {
    }
    
    ~HttpPublisher() override {
        if (wifi_event_group_) {
            vEventGroupDelete(wifi_event_group_);
        }
    }
    
    bool init(
    const std::vector<WiFiCredentials>& credentials,
    const ServerConfig& server_config
    ) override {
    
        ESP_LOGI(TAG, "Initializing HTTP Publisher...");
        
        credentials_ = credentials;
        server_config_ = server_config;
        
        // Initialize NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase();
            ret = nvs_flash_init();
        }
        
        // Create event group
        wifi_event_group_ = xEventGroupCreate();
        
        initialized_ = true;
        ESP_LOGI(TAG, "Publisher initialized");
        return true;
        }

    bool connect(uint32_t timeout_per_network_ms = 30000) override {
    if (!initialized_) {
        return false;
    }
    
    ESP_LOGI(TAG, "Initializing WiFi stack...");
    
    // Initialize network stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    
    // Register event handlers
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                               &HttpPublisher::wifi_event_handler_static, this);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
                               &HttpPublisher::ip_event_handler_static, this);
    
    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // ✅ CRITICAL: Set storage to RAM
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    // ✅ CRITICAL: Set mode BEFORE any config
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // ✅ CRITICAL: Start WiFi BEFORE setting config
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // ✅ CRITICAL: Wait for WiFi to be fully ready
    vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second delay
    
    ESP_LOGI(TAG, "WiFi stack initialized, trying networks...");
    
    // Now try each network
    for (size_t i = 0; i < credentials_.size(); ++i) {
        ESP_LOGI(TAG, "Attempting connection to: %s", credentials_[i].ssid.c_str());
        
        // Clear events
        xEventGroupClearBits(wifi_event_group_, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        retry_count_ = 0;
        
        // Disconnect first if already connected
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Configure network
        wifi_config_t wifi_config = {};
        
        // Copy credentials
        size_t ssid_len = std::min(credentials_[i].ssid.length(), (size_t)sizeof(wifi_config.sta.ssid) - 1);
        size_t pass_len = std::min(credentials_[i].password.length(), (size_t)sizeof(wifi_config.sta.password) - 1);
        
        memcpy(wifi_config.sta.ssid, credentials_[i].ssid.c_str(), ssid_len);
        wifi_config.sta.ssid[ssid_len] = '\0';
        
        memcpy(wifi_config.sta.password, credentials_[i].password.c_str(), pass_len);
        wifi_config.sta.password[pass_len] = '\0';
        
        // Security settings
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        wifi_config.sta.pmf_cfg.capable = true;
        wifi_config.sta.pmf_cfg.required = false;
        wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        
        ESP_LOGI(TAG, "Setting configuration...");
        
        // ✅ Set config (WiFi is already started)
        esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Set config failed: %s", esp_err_to_name(err));
            continue;
        }
        
        // ✅ Connect
        ESP_LOGI(TAG, "Connecting...");
        err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Connect failed: %s", esp_err_to_name(err));
            continue;
        }
        
        // Wait for connection
        ESP_LOGI(TAG, "Waiting for connection (timeout: %lu ms)...", timeout_per_network_ms);
        
        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group_,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(timeout_per_network_ms)
        );
        
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "✅ Connected to %s", credentials_[i].ssid.c_str());
            connected_ = true;
            current_credential_index_ = i;
            return true;
        }
        
        ESP_LOGW(TAG, "❌ Failed to connect to %s", credentials_[i].ssid.c_str());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    ESP_LOGE(TAG, "Failed to connect to any network");
    return false;
}
    
    
    PublishResult publishSoundEvent(
        const std::string& device_id,
        const std::string& label,
        float confidence,
        const std::vector<std::pair<std::string, float>>& alternatives,
        uint32_t duration_ms,
        float rms_energy
    ) override {
        
        PublishResult result;
        
        if (!connected_) {
            ESP_LOGW(TAG, "Not connected to WiFi");
            result.status = PublishStatus::ERROR_NOT_CONNECTED;
            return result;
        }
        
        // Create JSON payload
        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "device_id", device_id.c_str());
        cJSON_AddStringToObject(root, "timestamp", getCurrentTimestamp().c_str());
        cJSON_AddStringToObject(root, "event_type", "sound_detected");
        
        // Event data
        cJSON* event_data = cJSON_CreateObject();
        
        // Classification
        cJSON* classification = cJSON_CreateObject();
        cJSON_AddStringToObject(classification, "label", label.c_str());
        cJSON_AddNumberToObject(classification, "confidence", confidence);
        
        // Alternative labels
        if (!alternatives.empty()) {
            cJSON* alt_array = cJSON_CreateArray();
            for (const auto& alt : alternatives) {
                cJSON* alt_obj = cJSON_CreateObject();
                cJSON_AddStringToObject(alt_obj, "label", alt.first.c_str());
                cJSON_AddNumberToObject(alt_obj, "confidence", alt.second);
                cJSON_AddItemToArray(alt_array, alt_obj);
            }
            cJSON_AddItemToObject(classification, "alternative_labels", alt_array);
        }
        
        cJSON_AddItemToObject(event_data, "classification", classification);
        
        // Audio metrics
        cJSON* audio_metrics = cJSON_CreateObject();
        cJSON_AddNumberToObject(audio_metrics, "duration_ms", duration_ms);
        cJSON_AddNumberToObject(audio_metrics, "sample_rate", 16000);
        cJSON_AddNumberToObject(audio_metrics, "rms_energy", rms_energy);
        cJSON_AddItemToObject(event_data, "audio_metrics", audio_metrics);
        
        cJSON_AddItemToObject(root, "event_data", event_data);
        
        // Convert to string
        char* json_string = cJSON_PrintUnformatted(root);
        
        ESP_LOGI(TAG, "Publishing sound event: %s (%.1f%%)", 
                 label.c_str(), confidence * 100.0f);
        ESP_LOGD(TAG, "JSON: %s", json_string);
        
        // Send HTTP POST
        result = sendHttpPost(json_string, device_id);
        
        // Cleanup
        cJSON_free(json_string);
        cJSON_Delete(root);
        
        return result;
    }
    
    PublishResult publishError(
        const std::string& device_id,
        const std::string& error_code,
        const std::string& severity,
        const std::string& description
    ) override {
        
        PublishResult result;
        
        if (!connected_) {
            ESP_LOGW(TAG, "Not connected to WiFi");
            result.status = PublishStatus::ERROR_NOT_CONNECTED;
            return result;
        }
        
        // Create JSON payload
        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "device_id", device_id.c_str());
        cJSON_AddStringToObject(root, "timestamp", getCurrentTimestamp().c_str());
        cJSON_AddStringToObject(root, "message_type", "error");
        
        // Error object
        cJSON* error = cJSON_CreateObject();
        cJSON_AddStringToObject(error, "code", error_code.c_str());
        cJSON_AddStringToObject(error, "severity", severity.c_str());
        cJSON_AddStringToObject(error, "description", description.c_str());
        cJSON_AddNumberToObject(error, "count", 1);
        cJSON_AddStringToObject(error, "first_occurrence", getCurrentTimestamp().c_str());
        cJSON_AddItemToObject(root, "error", error);
        
        // Convert to string
        char* json_string = cJSON_PrintUnformatted(root);
        
        ESP_LOGE(TAG, "Publishing error: %s - %s", error_code.c_str(), description.c_str());
        
        // Send HTTP POST
        result = sendHttpPost(json_string, device_id);
        
        // Cleanup
        cJSON_free(json_string);
        cJSON_Delete(root);
        
        return result;
    }
    
    bool isConnected() override {
        return connected_;
    }
    
    std::string getCurrentSSID() override {
        if (!connected_ || current_credential_index_ < 0) {
            return "not_connected";
        }
        return credentials_[current_credential_index_].ssid;
    }
    
    int getRSSI() override {
        if (!connected_) {
            return -100;
        }
        
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            return ap_info.rssi;
        }
        return -100;
    }
    
private:
    bool connectToNetwork(const WiFiCredentials& cred, uint32_t timeout_ms) {
        ESP_LOGI(TAG, "Configuring WiFi for: %s", cred.ssid.c_str());
        
        // Clear event bits
        xEventGroupClearBits(wifi_event_group_, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        retry_count_ = 0;
        
        // Validate credentials
        if (cred.ssid.empty() || cred.ssid.length() > 32) {
            ESP_LOGE(TAG, "Invalid SSID");
            return false;
        }
        
        if (cred.password.length() > 64) {
            ESP_LOGE(TAG, "Password too long");
            return false;
        }
        
        // Configure WiFi - Zero initialize
        wifi_config_t wifi_config = {};
        
        // Copy SSID
        strncpy((char*)wifi_config.sta.ssid, cred.ssid.c_str(), sizeof(wifi_config.sta.ssid) - 1);
        wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
        
        // Copy password
        if (!cred.password.empty()) {
            strncpy((char*)wifi_config.sta.password, cred.password.c_str(), sizeof(wifi_config.sta.password) - 1);
            wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';
            wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        } else {
            wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        }
        
        // PMF configuration
        wifi_config.sta.pmf_cfg.capable = true;
        wifi_config.sta.pmf_cfg.required = false;
        
        // Scan configuration
        wifi_config.sta.scan_method = WIFI_FAST_SCAN;
        wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        wifi_config.sta.threshold.rssi = -127;
        
        ESP_LOGI(TAG, "Setting WiFi configuration...");
        
        // Set configuration (WiFi mode was already set in init())
        esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(err));
            return false;
        }
        
        ESP_LOGI(TAG, "Starting WiFi...");
        
        // Start WiFi
        err = esp_wifi_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(err));
            return false;
        }
        
        wifi_started_ = true;
        
        ESP_LOGI(TAG, "Waiting for connection (timeout: %lu ms)...", timeout_ms);
        
        // Wait for connection
        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group_,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdTRUE,  // Clear bits after reading
            pdFALSE, // Wait for either bit
            pdMS_TO_TICKS(timeout_ms)
        );
        
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "✅ Connected!");
            return true;
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGW(TAG, "❌ Connection failed");
        } else {
            ESP_LOGW(TAG, "⏱️  Timeout");
        }
        
        return false;
    }
    
    PublishResult sendHttpPost(const char* json_data, const std::string& device_id) {
        PublishResult result;
        uint64_t start_time = esp_timer_get_time();
        
        esp_http_client_config_t config = {};
        config.url = server_config_.url.c_str();
        config.method = HTTP_METHOD_POST;
        config.timeout_ms = server_config_.timeout_ms;
        
        esp_http_client_handle_t client = esp_http_client_init(&config);
        
        // Set headers
        esp_http_client_set_header(client, "Content-Type", "application/json");

        esp_http_client_set_header(client, "X-Device-ID", device_id.c_str());
         if (!server_config_.device_key.empty()) {

            esp_http_client_set_header(client, "X-Device-Key", server_config_.device_key.c_str());
        }
        
        // Set POST data
        esp_http_client_set_post_field(client, json_data, strlen(json_data));
        
        // Perform request with retries
        esp_err_t err = ESP_FAIL;
        for (uint32_t i = 0; i < server_config_.retry_count; ++i) {
            err = esp_http_client_perform(client);
            
            if (err == ESP_OK) {
                break;
            }
            
            ESP_LOGW(TAG, "HTTP POST retry %d/%d", i + 1, server_config_.retry_count);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        if (err == ESP_OK) {
            result.http_code = esp_http_client_get_status_code(client);
            int content_length = esp_http_client_get_content_length(client);
            
            ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                     result.http_code, content_length);
            
            if (result.http_code >= 200 && result.http_code < 300) {
                result.status = PublishStatus::OK;
            } else {
                result.status = PublishStatus::ERROR_SERVER;
            }
        } else {
            ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
            result.status = PublishStatus::ERROR_HTTP_REQUEST;
        }
        
        result.duration_ms = (esp_timer_get_time() - start_time) / 1000;
        
        esp_http_client_cleanup(client);
        return result;
    }
    
    std::string getCurrentTimestamp() {
        // Simple timestamp (you might want to sync with NTP)
        uint64_t time_us = esp_timer_get_time();
        uint64_t time_s = time_us / 1000000;
        
        char timestamp[32];
        snprintf(timestamp, sizeof(timestamp), "%llu", time_s);
        
        // TODO: Replace with proper ISO 8601 format after NTP sync
        return std::string(timestamp);
    }
    
    // Static event handlers
    static void wifi_event_handler_static(void* arg, esp_event_base_t event_base,
                                         int32_t event_id, void* event_data) {
        HttpPublisher* self = static_cast<HttpPublisher*>(arg);
        self->wifi_event_handler(event_base, event_id, event_data);
    }
    
    static void ip_event_handler_static(void* arg, esp_event_base_t event_base,
                                       int32_t event_id, void* event_data) {
        HttpPublisher* self = static_cast<HttpPublisher*>(arg);
        self->ip_event_handler(event_base, event_id, event_data);
    }
    
    void wifi_event_handler(esp_event_base_t event_base, int32_t event_id, void* event_data) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (retry_count_ < 5) {
                esp_wifi_connect();
                retry_count_++;
                ESP_LOGI(TAG, "Retry connecting to WiFi...");
            } else {
                xEventGroupSetBits(wifi_event_group_, WIFI_FAIL_BIT);
            }
            ESP_LOGI(TAG, "Connect to WiFi failed");
            connected_ = false;
        }
    }
    
    void ip_event_handler(esp_event_base_t event_base, int32_t event_id, void* event_data) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            xEventGroupSetBits(wifi_event_group_, WIFI_CONNECTED_BIT);
            connected_ = true;
            retry_count_ = 0;
        }
    }
};

// Factory function
Publisher* createHttpPublisher() {
    return new HttpPublisher();
}

} // namespace net