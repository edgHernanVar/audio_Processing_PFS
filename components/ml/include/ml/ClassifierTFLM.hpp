// components/ml/include/ml/Classifier.hpp
#pragma once

#include <vector>
#include <string>
#include <cstdint>

namespace ml {

enum class ClassifierStatus {
    OK,
    ERROR_NOT_INITIALIZED,
    ERROR_INVALID_INPUT,
    ERROR_INFERENCE_FAILED,
    ERROR_MODEL_LOAD,
    ERROR_ALLOCATION
};

struct ClassificationResult {
    std::vector<float> probabilities;    // Probability for each class
    int predicted_class;                 // Index of most likely class
    float confidence;                    // Confidence of prediction (0-1)
    uint32_t inference_time_us;          // Inference time in microseconds
    
    ClassificationResult() 
        : predicted_class(-1)
        , confidence(0.0f)
        , inference_time_us(0) 
    {}
};

struct ModelInfo {
    int input_size;                      // Total input features
    int output_size;                     // Number of classes
    std::vector<int> input_shape;        // Input tensor shape [frames, coeffs]
    std::vector<std::string> class_labels;  // Class names
    std::string model_version;
};

class Classifier {
public:
    virtual ~Classifier() = default;
    
    /**
     * @brief Initialize the classifier with a TFLite model
     * @param model_data Pointer to TFLite model data
     * @param model_size Size of model in bytes
     * @param class_labels Vector of class names
     * @return Status code
     */
    virtual ClassifierStatus init(
        const unsigned char* model_data,
        size_t model_size,
        const std::vector<std::string>& class_labels
    ) = 0;
    
    /**
     * @brief Run inference on MFCC features
     * @param features Pointer to feature data (flattened MFCC matrix)
     * @param feature_size Number of features (frames × coefficients)
     * @param result Output classification result
     * @return Status code
     */
    virtual ClassifierStatus classify(
        const float* features,
        size_t feature_size,
        ClassificationResult& result
    ) = 0;
    
    /**
     * @brief Run inference on feature vector
     * @param features Feature vector
     * @param result Output classification result
     * @return Status code
     */
    virtual ClassifierStatus classify(
        const std::vector<float>& features,
        ClassificationResult& result
    ) = 0;
    
    /**
     * @brief Get model information
     */
    virtual ModelInfo getModelInfo() const = 0;
    
    /**
     * @brief Check if classifier is initialized
     */
    virtual bool isInitialized() const = 0;
    
    /**
     * @brief Reset classifier state
     */
    virtual void reset() = 0;
};
    Classifier* createClassifier();

} // namespace ml