idf_component_register(SRCS "ClassifierTFLM.cpp"
                      "model_data.cpp"
                  INCLUDE_DIRS "include"
                  # Esta línea es la clave:
                  REQUIRES esp-tflite-micro)