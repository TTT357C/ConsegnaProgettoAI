
#ifndef pibt_hpp
#define pibt_hpp

// ====================================================================================================
// PIBT HEADER - Priority Inheritance with Backtracking
// ====================================================================================================
//
// This header defines the Priority Inheritance with Backtracking (PIBT) algorithm.
// PIBT is a conflict resolution method for multi-agent pathfinding that uses
// priority inheritance to handle conflicts and backtracking to resolve deadlocks.
//
// Key components:
// - causalPIBT: Main PIBT algorithm with conflict resolution
// - get_gp_h: Get heuristic value from guide path
// - getAction: Convert state transitions to actions
// - moveCheck: Validate action execution considering turning constraints

#include <vector>
#include <list>
#include <unordered_set>
#include <tuple>
#include "Types.h"
#include "utils.h"
#include "heuristics.h"
#include "TrajLNS.h"
#include "utils.h"

namespace DefaultPlanner{

/**
 * @brief Get heuristic value from guide path for agent ai to reach target
 * @param lns Trajectory LNS structure containing guide paths
 * @param ai Agent index
 * @param target Target location
 * @return Heuristic distance value
 */
int get_gp_h(TrajLNS& lns, int ai, int target);

/**
 * @brief Main PIBT algorithm with causal conflict resolution
 * @param curr_id Current agent being planned
 * @param higher_id Higher priority agent causing conflict (-1 if none)
 * @param prev_states Previous states of all agents
 * @param next_states Next states to be computed
 * @param prev_decision Previous decision locations
 * @param decision Current decision locations
 * @param occupied Occupation status of locations
 * @param lns Trajectory LNS structure with guide paths
 * @return True if planning successful
 */
bool causalPIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	 std::vector<State>& next_states,
      std::vector<int>& prev_decision, std::vector<int>& decision, 
	  std::vector<bool>& occupied,TrajLNS& lns
	  );

/**
 * @brief Convert state transition to action
 * @param prev Previous state
 * @param next Next state
 * @return Action to perform
 */
Action getAction(State& prev, State& next);

/**
 * @brief Convert location transition to action considering orientation
 * @param prev Previous state
 * @param next_loc Next location
 * @param env Environment for map information
 * @return Action to perform
 */
Action getAction(State& prev, int next_loc, SharedEnvironment* env);

/**
 * @brief Recursively check if forward move can be executed
 * @param id Agent ID to check
 * @param checked Agents already checked
 * @param decided Decision states
 * @param actions Actions array
 * @param prev_decision Previous decisions
 * @return True if move is valid
 */
bool moveCheck(int id, std::vector<bool>& checked,
		std::vector<DCR>& decided, std::vector<Action>& actions, std::vector<int>& prev_decision);
}
#endif