
#ifndef flow_hpp
#define flow_hpp

// ====================================================================================================
// TRAFFIC FLOW OPTIMIZATION HEADER
// ====================================================================================================
//
// This header defines functions for traffic flow optimization using the Frank-Wolfe algorithm.
// The flow optimization minimizes congestion by adjusting agent guide paths to reduce
// conflicts and improve overall traffic efficiency.
//
// Key components:
// - Guide path management (add/remove trajectories)
// - Frank-Wolfe algorithm for flow optimization
// - Distance table computation for path costs
// - Flow metrics and deviation calculations

#include "Types.h"
#include "search.h"
#include "TrajLNS.h"
#include "heuristics.h"

#include <random>
#include <unordered_set>

namespace DefaultPlanner{

// ====================================================================================================
// TRAJECTORY MANAGEMENT
// ====================================================================================================

/**
 * @brief Remove agent's trajectory from flow network
 * @param lns Trajectory LNS structure
 * @param agent Agent index whose trajectory to remove
 */
void remove_traj(TrajLNS& lns, int agent);

/**
 * @brief Add agent's trajectory to flow network
 * @param lns Trajectory LNS structure
 * @param agent Agent index whose trajectory to add
 */
void add_traj(TrajLNS& lns, int agent);

// ====================================================================================================
// FLOW OPTIMIZATION
// ====================================================================================================

/**
 * @brief Compute flow deviation metrics
 * @param lns Trajectory LNS structure
 */
void get_deviation(TrajLNS& lns);

/**
 * @brief Update Frank-Wolfe algorithm metrics
 * @param lns Trajectory LNS structure
 */
void update_fw_metrics(TrajLNS& lns);

/**
 * @brief Execute Frank-Wolfe algorithm for traffic flow optimization
 * @param lns Trajectory LNS structure
 * @param updated Set of agents whose paths were updated
 * @param timelimit Time limit for optimization
 */
void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit);

// ====================================================================================================
// DISTANCE COMPUTATION
// ====================================================================================================

/**
 * @brief Update distance-to-path for agent
 * @param lns Trajectory LNS structure
 * @param i Agent index
 */
void update_dist_2_path(TrajLNS& lns, int i);

/**
 * @brief Initialize distance table for trajectories
 * @param lns Trajectory LNS structure
 * @param amount Number of agents to process
 */
void init_dist_table(TrajLNS& lns, int amount);

/**
 * @brief Update trajectory and distance table for agent
 * @param lns Trajectory LNS structure
 * @param i Agent index
 */
void update_traj(TrajLNS& lns, int i);

}
#endif