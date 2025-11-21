idf_component_register(
    SRCS "HttpPublisher.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        esp_wifi
        esp_http_client
        esp_event
        nvs_flash
        lwip
        json
)
