#pragma once

// ====================================================================================================
// LNS Planner Configuration
// ====================================================================================================
// This file contains all configuration parameters for the LNS planner with LaCAM2 initialization.
// Parameters are automatically selected based on the input file domain.
// ====================================================================================================

#include <string>

namespace PlannerConfig {

// ====================================================================================================
// Configuration Struct
// ====================================================================================================
struct Config {
    unsigned int SEED;                          // Random seed for reproducibility
    int PLANNING_WINDOW;                       // Planning horizon in timesteps
    int EXECUTION_WINDOW;                      // Execution window in timesteps
    int HARD_CONSTRAINT_WINDOW;                // Hard constraint window
    double CUTOFF_TIME;                        // Time limit for LNS in seconds
    int NEIGHBOR_SIZE;                         // Number of agents in neighborhood
    int MAX_ITERATIONS;                        // Maximum LNS iterations
    double NEIGHBOR_WEIGHT_AIM;                // Weight for AIM-based neighbor selection
    double NEIGHBOR_WEIGHT_INTERSECTION;       // Weight for intersection-based selection
    double NEIGHBOR_WEIGHT_START;              // Weight for start location selection
    double NEIGHBOR_WEIGHT_GOAL;               // Weight for goal location selection
    double SUBOPTIMALITY;                      // Suboptimality bound
    int WINDOW_SIZE_FOR_CT;                    // Constraint table window size
    int WINDOW_SIZE_FOR_CAT;                   // Conflict avoidance table window size
    int WINDOW_SIZE_FOR_PATH;                  // Path planning window size
};

// Global config instance
extern Config config;

// ====================================================================================================
// Configuration Presets
// ====================================================================================================

// Default config (for paris, random, game)
const Config DEFAULT_CONFIG = {
    0,      // SEED
    10,     // PLANNING_WINDOW
    1,      // EXECUTION_WINDOW
    10,     // HARD_CONSTRAINT_WINDOW
    0.95,   // CUTOFF_TIME
    10,      // NEIGHBOR_SIZE
    100000,  // MAX_ITERATIONS
    0.5,    // NEIGHBOR_WEIGHT_AIM
    0.5,    // NEIGHBOR_WEIGHT_INTERSECTION
    0.5,    // NEIGHBOR_WEIGHT_START
    0.5,    // NEIGHBOR_WEIGHT_GOAL
    1.1,    // SUBOPTIMALITY
    10,     // WINDOW_SIZE_FOR_CT
    10,     // WINDOW_SIZE_FOR_CAT
    10      // WINDOW_SIZE_FOR_PATH
};

// Warehouse config (optimized for speed to avoid system-level 1000ms timeout)
const Config WAREHOUSE_CONFIG = {
  0,      // SEED
    10,     // PLANNING_WINDOW
    1,      // EXECUTION_WINDOW
    10,     // HARD_CONSTRAINT_WINDOW
    0.92,   // CUTOFF_TIME
    10,      // NEIGHBOR_SIZE
    100000,  // MAX_ITERATIONS
    0.5,    // NEIGHBOR_WEIGHT_AIM
    0.5,    // NEIGHBOR_WEIGHT_INTERSECTION
    0.5,    // NEIGHBOR_WEIGHT_START
    0.5,    // NEIGHBOR_WEIGHT_GOAL
    1.2,    // SUBOPTIMALITY
    10,     // WINDOW_SIZE_FOR_CT
    10,     // WINDOW_SIZE_FOR_CAT
    10      // WINDOW_SIZE_FOR_PATH
};

// ====================================================================================================
// Initialization Function
// ====================================================================================================

/**
 * @brief Initialize the planner configuration based on the input file name
 * @param inputFile The path to the input file
 */
void initializeConfig(const std::string& inputFile);

} // namespace PlannerConfig
