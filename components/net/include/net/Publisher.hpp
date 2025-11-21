// components/net/include/net/Publisher.hpp
#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace net {

enum class PublishStatus {
    OK,
    ERROR_NOT_CONNECTED,
    ERROR_INVALID_DATA,
    ERROR_HTTP_REQUEST,
    ERROR_TIMEOUT,
    ERROR_SERVER
};

struct WiFiCredentials {
    std::string ssid;
    std::string password;
};

struct ServerConfig {
    std::string url;              // Full URL: http://192.168.1.100:5000/api/events
    uint32_t timeout_ms;          // Request timeout
    uint32_t retry_count;         // Number of retries on failure

    std::string device_id_header;
    std::string device_key;

};

struct PublishResult {
    PublishStatus status;
    int http_code;                // HTTP response code (200, 404, etc.)
    std::string response;         // Server response body
    uint32_t duration_ms;         // Request duration
    
    PublishResult() 
        : status(PublishStatus::ERROR_NOT_CONNECTED)
        , http_code(0)
        , duration_ms(0) 
    {}
};

class Publisher {
public:
    virtual ~Publisher() = default;
    
    /**
     * @brief Initialize publisher with WiFi credentials
     * @param credentials Vector of WiFi credentials (tries in order)
     * @param server_config Server configuration
     * @return true if initialized successfully
     */
    virtual bool init(
        const std::vector<WiFiCredentials>& credentials,
        const ServerConfig& server_config
    ) = 0;
    
    /**
     * @brief Connect to WiFi (tries all credentials)
     * @param timeout_per_network_ms Timeout per network attempt
     * @return true if connected
     */
    virtual bool connect(uint32_t timeout_per_network_ms = 30000) = 0;
    
    /**
     * @brief Publish sound detection event
     * @param device_id Device identifier
     * @param label Detected sound label
     * @param confidence Confidence score (0-1)
     * @param alternatives Alternative labels with probabilities
     * @param duration_ms Event duration
     * @param rms_energy RMS energy
     * @return Publish result
     */
    virtual PublishResult publishSoundEvent(
        const std::string& device_id,
        const std::string& label,
        float confidence,
        const std::vector<std::pair<std::string, float>>& alternatives,
        uint32_t duration_ms,
        float rms_energy
    ) = 0;
    
    /**
     * @brief Publish error event
     * @param device_id Device identifier
     * @param error_code Error code
     * @param severity Error severity (warning, error, critical)
     * @param description Error description
     * @return Publish result
     */
    virtual PublishResult publishError(
        const std::string& device_id,
        const std::string& error_code,
        const std::string& severity,
        const std::string& description
    ) = 0;
    
    /**
     * @brief Check if connected to WiFi
     */
    virtual bool isConnected() = 0;
    
    /**
     * @brief Get current WiFi SSID
     */
    virtual std::string getCurrentSSID() = 0;
    
    /**
     * @brief Get WiFi signal strength (RSSI)
     */
    virtual int getRSSI() = 0;
};
    Publisher* createHttpPublisher();
} // namespace net