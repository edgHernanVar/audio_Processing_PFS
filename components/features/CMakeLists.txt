idf_component_register(
    SRCS 
        "MFCCExtractorESP.cpp"
        "FFTHelperESPDSP.cpp"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES esp-dsp  # For optimized FFT operations
)