// components/ml/ClassifierTFLM.cpp
#include "./include/ml/ClassifierTFLM.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

// TensorFlow Lite Micro headers
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include <algorithm>
#include <cstring>

namespace ml {

static const char* TAG = "ClassifierTFLM";

class ClassifierTFLM : public Classifier {
private:
    // TFLite objects
    const tflite::Model* model_;
    tflite::MicroInterpreter* interpreter_;
    TfLiteTensor* input_;
    TfLiteTensor* output_;
    
    // Memory arena for TFLite
    uint8_t* tensor_arena_;
    size_t tensor_arena_size_;
    
    // Model info
    ModelInfo model_info_;
    std::vector<std::string> class_labels_;
    bool initialized_;
    
    // Input quantization parameters (for INT8 models)
    float input_scale_;
    int32_t input_zero_point_;
    bool is_quantized_;
    
    // Output quantization parameters
    float output_scale_;
    int32_t output_zero_point_;
    
public:
    ClassifierTFLM() 
        : model_(nullptr)
        , interpreter_(nullptr)
        , input_(nullptr)
        , output_(nullptr)
        , tensor_arena_(nullptr)
        , tensor_arena_size_(0)
        , initialized_(false)
        , input_scale_(1.0f)
        , input_zero_point_(0)
        , is_quantized_(false)
        , output_scale_(1.0f)
        , output_zero_point_(0)
    {
        model_info_.model_version = "unknown";
    }
    
    ~ClassifierTFLM() override {
        cleanup();
    }
    
    ClassifierStatus init(
        const unsigned char* model_data,
        size_t model_size,
        const std::vector<std::string>& class_labels
    ) override {
        
        ESP_LOGI(TAG, "Initializing TFLite Micro classifier...");
        ESP_LOGI(TAG, "Model size: %d bytes", model_size);
        ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
        
        if (!model_data || model_size == 0) {
            ESP_LOGE(TAG, "Invalid model data");
            return ClassifierStatus::ERROR_MODEL_LOAD;
        }
        
        class_labels_ = class_labels;
        
        // Load the model
        model_ = tflite::GetModel(model_data);
        if (model_->version() != TFLITE_SCHEMA_VERSION) {
            ESP_LOGE(TAG, "Model schema version %d not supported. Supported version is %d",
                     model_->version(), TFLITE_SCHEMA_VERSION);
            return ClassifierStatus::ERROR_MODEL_LOAD;
        }
        
        ESP_LOGI(TAG, "Model schema version: %d", model_->version());
        
        // Allocate tensor arena in PSRAM (TFLite needs a lot of memory)
        // Start with 100KB, increase if needed
        tensor_arena_size_ = 100 * 1024;  // 100 KB
        tensor_arena_ = (uint8_t*)heap_caps_malloc(
            tensor_arena_size_,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        );
        
        if (!tensor_arena_) {
            ESP_LOGE(TAG, "Failed to allocate tensor arena (%d KB)", 
                     tensor_arena_size_ / 1024);
            return ClassifierStatus::ERROR_ALLOCATION;
        }
        
        ESP_LOGI(TAG, "Allocated tensor arena: %d KB in PSRAM", 
                 tensor_arena_size_ / 1024);
        
        // Create op resolver (add only operations used by your model)
        static tflite::MicroMutableOpResolver<10> resolver;
        
        // Add operations used by typical audio classification models
        resolver.AddConv2D();
        resolver.AddMaxPool2D();
        resolver.AddFullyConnected();
        resolver.AddReshape();
        resolver.AddSoftmax();
        resolver.AddQuantize();
        resolver.AddDequantize();
        resolver.AddRelu();
        resolver.AddLogistic();  // Sigmoid
        
        // Create interpreter
        static tflite::MicroInterpreter static_interpreter(
            model_, 
            resolver, 
            tensor_arena_, 
            tensor_arena_size_
        );
        interpreter_ = &static_interpreter;
        
        // Allocate tensors
        TfLiteStatus allocate_status = interpreter_->AllocateTensors();
        if (allocate_status != kTfLiteOk) {
            ESP_LOGE(TAG, "AllocateTensors() failed");
            cleanup();
            return ClassifierStatus::ERROR_ALLOCATION;
        }
        
        // Get input tensor
        input_ = interpreter_->input(0);
        if (!input_) {
            ESP_LOGE(TAG, "Failed to get input tensor");
            cleanup();
            return ClassifierStatus::ERROR_MODEL_LOAD;
        }
        
        // Get output tensor
        output_ = interpreter_->output(0);
        if (!output_) {
            ESP_LOGE(TAG, "Failed to get output tensor");
            cleanup();
            return ClassifierStatus::ERROR_MODEL_LOAD;
        }
        
        // Extract model info
        extractModelInfo();
        
        // Check if model is quantized
        is_quantized_ = (input_->type == kTfLiteInt8);
        if (is_quantized_) {
            input_scale_ = input_->params.scale;
            input_zero_point_ = input_->params.zero_point;
            output_scale_ = output_->params.scale;
            output_zero_point_ = output_->params.zero_point;
            
            ESP_LOGI(TAG, "Model is INT8 quantized");
            ESP_LOGI(TAG, "Input: scale=%.6f, zero_point=%d", 
                     input_scale_, input_zero_point_);
            ESP_LOGI(TAG, "Output: scale=%.6f, zero_point=%d",
                     output_scale_, output_zero_point_);
        } else {
            ESP_LOGI(TAG, "Model is float32");
        }
        
        // Print memory usage
        size_t used_bytes = interpreter_->arena_used_bytes();
        ESP_LOGI(TAG, "Tensor arena used: %d / %d bytes (%.1f%%)",
                 used_bytes, tensor_arena_size_,
                 100.0f * used_bytes / tensor_arena_size_);
        
        if (used_bytes > tensor_arena_size_ * 0.95f) {
            ESP_LOGW(TAG, "Arena usage > 95%% - consider increasing size");
        }
        
        initialized_ = true;
        ESP_LOGI(TAG, "Classifier initialized successfully");
        
        return ClassifierStatus::OK;
    }
    
    ClassifierStatus classify(
        const float* features,
        size_t feature_size,
        ClassificationResult& result
    ) override {
        
        if (!initialized_) {
            return ClassifierStatus::ERROR_NOT_INITIALIZED;
        }
        
        if (feature_size != static_cast<size_t>(model_info_.input_size)) {
            ESP_LOGE(TAG, "Invalid input size: got %d, expected %d",
                     feature_size, model_info_.input_size);
            return ClassifierStatus::ERROR_INVALID_INPUT;
        }
        
        uint64_t start_time = esp_timer_get_time();
        
        // Copy features to input tensor
        if (is_quantized_) {
            // Quantize float input to INT8
            int8_t* input_data = input_->data.int8;
            for (size_t i = 0; i < feature_size; ++i) {
                float value = features[i];
                int32_t quantized = static_cast<int32_t>(
                    value / input_scale_ + input_zero_point_
                );
                // Clamp to INT8 range
                quantized = std::max((int32_t)-128, std::min((int32_t)127, quantized));
                input_data[i] = static_cast<int8_t>(quantized);
            }
        } else {
            // Direct float copy
            float* input_data = input_->data.f;
            std::memcpy(input_data, features, feature_size * sizeof(float));
        }
        
        // Run inference
        TfLiteStatus invoke_status = interpreter_->Invoke();
        if (invoke_status != kTfLiteOk) {
            ESP_LOGE(TAG, "Invoke() failed");
            return ClassifierStatus::ERROR_INFERENCE_FAILED;
        }
        
        uint64_t end_time = esp_timer_get_time();
        result.inference_time_us = end_time - start_time;
        
        // Extract output
        int num_classes = model_info_.output_size;
        result.probabilities.resize(num_classes);
        
        if (is_quantized_) {
            // Dequantize INT8 output to float
            int8_t* output_data = output_->data.int8;
            for (int i = 0; i < num_classes; ++i) {
                result.probabilities[i] = output_scale_ * 
                    (output_data[i] - output_zero_point_);
            }
        } else {
            // Direct float copy
            float* output_data = output_->data.f;
            std::memcpy(result.probabilities.data(), 
                       output_data, 
                       num_classes * sizeof(float));
        }
        
        // Find predicted class (argmax)
        result.predicted_class = 0;
        result.confidence = result.probabilities[0];
        for (int i = 1; i < num_classes; ++i) {
            if (result.probabilities[i] > result.confidence) {
                result.confidence = result.probabilities[i];
                result.predicted_class = i;
            }
        }
        
        ESP_LOGI(TAG, "Inference: %d µs, Predicted: class %d (%.2f%%)",
                 result.inference_time_us,
                 result.predicted_class,
                 result.confidence * 100.0f);
        
        return ClassifierStatus::OK;
    }
    
    ClassifierStatus classify(
        const std::vector<float>& features,
        ClassificationResult& result
    ) override {
        return classify(features.data(), features.size(), result);
    }
    
    ModelInfo getModelInfo() const override {
        return model_info_;
    }
    
    bool isInitialized() const override {
        return initialized_;
    }
    
    void reset() override {
        // TFLite Micro doesn't need explicit reset
        // State is reset on each inference
    }
    
private:
    void extractModelInfo() {
        // Get input shape
        TfLiteIntArray* input_dims = input_->dims;
        model_info_.input_shape.clear();
        for (int i = 0; i < input_dims->size; ++i) {
            model_info_.input_shape.push_back(input_dims->data[i]);
        }
        
        // Calculate total input size
        model_info_.input_size = 1;
        for (int dim : model_info_.input_shape) {
            model_info_.input_size *= dim;
        }
        
        // Get output size (number of classes)
        TfLiteIntArray* output_dims = output_->dims;
        model_info_.output_size = output_dims->data[output_dims->size - 1];
        
        // Set class labels
        model_info_.class_labels = class_labels_;
        
        ESP_LOGI(TAG, "=== Model Info ===");
        ESP_LOGI(TAG, "Input shape: [%d, %d, %d]", 
                 model_info_.input_shape.size() >= 3 ? model_info_.input_shape[1] : 0,
                 model_info_.input_shape.size() >= 3 ? model_info_.input_shape[2] : 0,
                 model_info_.input_shape.size() >= 4 ? model_info_.input_shape[3] : 1);
        ESP_LOGI(TAG, "Input size: %d features", model_info_.input_size);
        ESP_LOGI(TAG, "Output size: %d classes", model_info_.output_size);
        ESP_LOGI(TAG, "Classes:");
        for (size_t i = 0; i < class_labels_.size(); ++i) {
            ESP_LOGI(TAG, "  [%d] %s", i, class_labels_[i].c_str());
        }
    }
    
    void cleanup() {
        if (tensor_arena_) {
            heap_caps_free(tensor_arena_);
            tensor_arena_ = nullptr;
        }
        
        interpreter_ = nullptr;
        input_ = nullptr;
        output_ = nullptr;
        model_ = nullptr;
        initialized_ = false;
    }
};

// Factory function
Classifier* createClassifier() {
    return new ClassifierTFLM();
}

} // namespace ml