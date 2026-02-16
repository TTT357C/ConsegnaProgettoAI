
#ifndef heuristics_hpp
#define heuristics_hpp

// ====================================================================================================
// HEURISTICS HEADER - Distance and Path Cost Computations
// ====================================================================================================
//
// This header defines functions for computing heuristic distances and path costs.
// Provides precomputed distance tables for efficient A* search and flow optimization.
//
// Key components:
// - Heuristic table initialization and lookup
// - Distance-to-path calculations for guide path heuristics
// - Neighbor structure management

#include "Types.h"
#include "utils.h"
#include <queue>
#include "TrajLNS.h"
#include "search_node.h"

namespace DefaultPlanner{

// ====================================================================================================
// HEURISTIC TABLE MANAGEMENT
// ====================================================================================================

/**
 * @brief Initialize global heuristic tables for all locations
 * @param env Shared environment containing map information
 */
void init_heuristics(SharedEnvironment* env);

/**
 * @brief Initialize neighbor structures for the map
 * @param env Shared environment containing map information
 */
void init_neighbor(SharedEnvironment* env);

/**
 * @brief Initialize heuristic table for a specific goal location
 * @param ht Heuristic table to initialize
 * @param env Shared environment
 * @param goal_location Goal location for distance computation
 */
void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

/**
 * @brief Get heuristic distance from source to goal using precomputed table
 * @param ht Heuristic table
 * @param env Shared environment
 * @param source Source location
 * @param ns Neighbor structure
 * @return Heuristic distance
 */
int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns);

/**
 * @brief Get Manhattan distance between two locations
 * @param env Shared environment
 * @param source Source location
 * @param target Target location
 * @return Manhattan distance
 */
int get_h(SharedEnvironment* env, int source, int target);

// ====================================================================================================
// DISTANCE-TO-PATH COMPUTATIONS
// ====================================================================================================

/**
 * @brief Initialize distance-to-path table for a trajectory
 * @param dp Distance-to-path table to initialize
 * @param env Shared environment
 * @param path Trajectory path
 */
void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path);

/**
 * @brief Get distance from source to nearest point on path
 * @param dp Distance-to-path table
 * @param env Shared environment
 * @param source Source location
 * @param ns Neighbor structure
 * @return Pair of (distance, nearest_path_index)
 */
std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

/**
 * @brief Get distance from source to path
 * @param dp Distance-to-path table
 * @param env Shared environment
 * @param source Source location
 * @param ns Neighbor structure
 * @return Distance to path
 */
int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

}
#endif