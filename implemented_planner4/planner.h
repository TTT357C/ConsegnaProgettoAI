#ifndef PLANNER
#define PLANNER

// ====================================================================================================
// PLANNER HEADER - Multi-Agent Pathfinding Planner Interface
// ====================================================================================================
//
// This header defines the interface for the default planner implementation.
// The planner uses a hierarchical approach with multiple solvers:
// 1. LNS (Large Neighborhood Search) for complex, large-scale problems
// 2. LaCAM2 for medium-scale problems
// 3. PIBT (Priority Inheritance with Backtracking) as fallback for simple problems
//
// The planner adapts solver selection based on map characteristics and problem complexity.

#include "Types.h"
#include "TrajLNS.h"
#include <random>
#include <memory>
#include <nlohmann/json.hpp>

// Forward declarations to avoid naming conflicts
class HeuristicTable;
namespace LaCAM2 {
    class LaCAM2Solver;
}
namespace LNS {
    class LNSSolver;
}

namespace DefaultPlanner{

    // ====================================================================================================
    // SOLVER COMPONENTS
    // ====================================================================================================

    // LaCAM2 solver for problems with many agents or high density
    extern std::shared_ptr<LaCAM2::LaCAM2Solver> lacam2_solver;
    extern std::shared_ptr<::HeuristicTable> lacam2_heuristics;
    extern std::shared_ptr<std::vector<float>> map_weights;
    extern bool use_lacam2;
    
    // LNS solver using LaCAM2 as initial algorithm (like RAPID team)
    extern std::shared_ptr<LNS::LNSSolver> lns_solver;
    extern bool use_lns;
    extern nlohmann::json config;
    
    // ====================================================================================================
    // PUBLIC INTERFACE
    // ====================================================================================================

    /**
     * @brief Initialize the planner with environment and preprocessing
     * @param preprocess_time_limit Time limit for preprocessing in milliseconds
     * @param env Shared environment containing map and agent information
     */
    void initialize(int preprocess_time_limit, SharedEnvironment* env);

    /**
     * @brief Compute actions for all agents given current environment state
     * @param time_limit Time limit for planning in milliseconds
     * @param actions Output vector to be filled with actions for each agent
     * @param env Shared environment containing current state
     */
    void plan(int time_limit,vector<Action> & actions,  SharedEnvironment* env);


}
#endif