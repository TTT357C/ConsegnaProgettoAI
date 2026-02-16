#include "config.hpp"
#include <string>
#include <algorithm>
#include <iostream>

namespace PlannerConfig {

// Global config instance
Config config;

// ====================================================================================================
// Initialization Function Implementation
// ====================================================================================================

void initializeConfig(const std::string& inputFile) {
    // Convert to lowercase for case-insensitive comparison
    std::string lowerInput = inputFile;
    std::transform(lowerInput.begin(), lowerInput.end(), lowerInput.begin(), ::tolower);

    // Check if it's a warehouse instance
    if (lowerInput.find("warehouse") != std::string::npos ||
        lowerInput.find("sortation") != std::string::npos) {
        config = WAREHOUSE_CONFIG;
        std::cout << "[DEBUG] Using WAREHOUSE_CONFIG for input file: " << inputFile << std::endl;
        std::cout << "[DEBUG] Config: PLANNING_WINDOW=" << config.PLANNING_WINDOW 
                  << ", EXECUTION_WINDOW=" << config.EXECUTION_WINDOW
                  << ", CUTOFF_TIME=" << config.CUTOFF_TIME
                  << ", MAX_ITERATIONS=" << config.MAX_ITERATIONS << std::endl;
    } else {
        // Default config for paris, random, game
        config = DEFAULT_CONFIG;
        std::cout << "[DEBUG] Using DEFAULT_CONFIG for input file: " << inputFile << std::endl;
        std::cout << "[DEBUG] Config: PLANNING_WINDOW=" << config.PLANNING_WINDOW 
                  << ", EXECUTION_WINDOW=" << config.EXECUTION_WINDOW
                  << ", CUTOFF_TIME=" << config.CUTOFF_TIME
                  << ", MAX_ITERATIONS=" << config.MAX_ITERATIONS << std::endl;
    }
}

} // namespace PlannerConfig