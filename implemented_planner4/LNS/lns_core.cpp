#include "lns_core.h"
#include "util/Utils.h"
#include "LNS/Parallel/lns_managers.h"
#include <boost/tokenizer.hpp>
#include <algorithm>
#include <random>

// ====================================================================================================
// LARGE NEIGHBORHOOD SEARCH (LNS) CORE IMPLEMENTATION
// ====================================================================================================

/**
 * @brief LNS namespace for Large Neighborhood Search MAPF solver
 * LNS implements a metaheuristic approach that iteratively destroys and repairs
 * parts of the solution to find better conflict-free paths for multi-agent systems.
 */
namespace LNS {

/**
 * @brief Output operator for Path objects
 * Prints the sequence of locations in the path
 * @param os Output stream
 * @param path Path to output
 * @return Output stream
 */
std::ostream& operator<<(std::ostream& os, const Path& path) {
    for (const auto& state : path) {
        os << state.location << "\t";
    }
    return os;
}

/**
 * @brief Check if two paths are identical
 * Compares location sequences of two paths
 * @param p1 First path
 * @param p2 Second path
 * @return True if paths are identical
 */
bool isSamePath(const Path& p1, const Path& p2) {
    if (p1.size() != p2.size()) return false;
    for (unsigned i = 0; i < p1.size(); i++) {
        if (p1[i].location != p2[i].location) return false;
    }
    return true;
}

/**
 * @brief Register a path in the spatiotemporal grid
 * Updates the space-time grid to mark agent occupancy along the path
 * @param agent_id Agent identifier
 * @param _path Path to register (truncated to planning horizon if needed)
 */
void PathT::registerPath(int agent_id, const Path& _path) {
    if (_path.empty()) return;

    // Truncate path to planning horizon if specified
    int horizon = (int)_path.size();
    if (planning_horizon > 0 && planning_horizon + 1 < _path.size())
        horizon = planning_horizon + 1;
    Path path(_path.begin(), _path.begin() + horizon);

    // Mark agent occupancy in space-time grid
    for (int timestep = 0; timestep < (int)path.size(); timestep++) {
        if (space_time_grid[path[timestep].location].size() <= timestep)
            space_time_grid[path[timestep].location].resize(timestep + 1, NO_AGENT);
        space_time_grid[path[timestep].location][timestep] = agent_id;
    }

    // Record goal achievement timestamp
    if (target_timestamps[path.back().location] == MAX_TIMESTEP)
        target_timestamps[path.back().location] = (int)path.size() - 1;
    max_time_horizon = max(max_time_horizon, (int)path.size() - 1);
}

/**
 * @brief Unregister a path from the spatiotemporal grid
 * Removes agent occupancy markings for the given path
 * @param agent_id Agent identifier
 * @param _path Path to unregister
 */
void PathT::unregisterPath(int agent_id, const Path& _path) {
    if (_path.empty()) return;

    // Truncate path to planning horizon if specified
    int horizon = (int)_path.size();
    if (planning_horizon > 0 && planning_horizon + 1 < _path.size())
        horizon = planning_horizon + 1;
    Path path(_path.begin(), _path.begin() + horizon);

    // Remove agent occupancy markings
    for (int timestep = 0; timestep < (int)path.size(); timestep++) {
        if (space_time_grid[path[timestep].location].size() > timestep &&
            space_time_grid[path[timestep].location][timestep] == agent_id)
            space_time_grid[path[timestep].location][timestep] = NO_AGENT;
    }

    // Reset goal timestamp
    target_timestamps[path.back().location] = MAX_TIMESTEP;
}

/**
 * @brief Register a parallel path in the spatiotemporal grid
 * Updates occupancy for paths from parallel LNS components
 * @param agent_id Agent identifier
 * @param _path Parallel path structure to register
 */
void PathT::registerPath(int agent_id, const Parallel::Path& _path) {
    if (_path.nodes.empty()) return;

    // Truncate path to planning horizon if specified
    int horizon = (int)_path.nodes.size();
    if (planning_horizon > 0 && planning_horizon + 1 < _path.nodes.size())
        horizon = planning_horizon + 1;

    auto& path = _path.nodes;
    for (int timestep = 0; timestep < horizon; timestep++) {
        if (space_time_grid[path[timestep].location].size() <= timestep)
            space_time_grid[path[timestep].location].resize(timestep + 1, NO_AGENT);
        space_time_grid[path[timestep].location][timestep] = agent_id;
    }
}

/**
 * @brief Unregister a parallel path from the spatiotemporal grid
 * Removes occupancy markings for parallel path structures
 * @param agent_id Agent identifier
 * @param _path Parallel path structure to unregister
 */
void PathT::unregisterPath(int agent_id, const Parallel::Path& _path) {
    if (_path.nodes.empty()) return;

    // Truncate path to planning horizon if specified
    int horizon = (int)_path.nodes.size();
    if (planning_horizon > 0 && planning_horizon + 1 < _path.nodes.size())
        horizon = planning_horizon + 1;

    auto& path = _path.nodes;
    for (int timestep = 0; timestep < horizon; timestep++) {
        if (space_time_grid[path[timestep].location].size() > timestep &&
            space_time_grid[path[timestep].location][timestep] == agent_id)
            space_time_grid[path[timestep].location][timestep] = NO_AGENT;
    }
}

/**
 * @brief Check if a move is blocked by another agent
 * Tests for vertex conflicts (same location at same time) and edge conflicts
 * (crossing paths between adjacent locations)
 * @param from Starting location
 * @param to Ending location
 * @param arrival_time Time of arrival at destination
 * @return True if move is blocked
 */
bool PathT::isBlocked(int from, int to, int arrival_time) const {
    if (space_time_grid.empty()) return false;

    // Check vertex conflict at destination
    if (space_time_grid[to].size() > arrival_time &&
        space_time_grid[to][arrival_time] != NO_AGENT)
        return true;

    // Check edge conflict (agent moving to/from the same locations)
    if (space_time_grid[to].size() >= arrival_time &&
        space_time_grid[from].size() > arrival_time &&
        !space_time_grid[to].empty() &&
        space_time_grid[to][arrival_time - 1] != NO_AGENT &&
        space_time_grid[from][arrival_time] == space_time_grid[to][arrival_time - 1])
        return true;

    return false;
}

/**
 * @brief Check if a move is blocked, ignoring specified agents
 * Similar to isBlocked but allows ignoring certain agents (useful for neighbor operations)
 * @param from Starting location
 * @param to Ending location
 * @param arrival_time Time of arrival at destination
 * @param ignored_agents Set of agent IDs to ignore in conflict checking
 * @return True if move is blocked by non-ignored agents
 */
bool PathT::isBlocked(int from, int to, int arrival_time, std::vector<int>& ignored_agents) const {
    if (space_time_grid.empty()) return false;

    // Check vertex conflict at destination
    if (space_time_grid[to].size() > arrival_time && space_time_grid[to][arrival_time] != NO_AGENT) {
        bool is_ignored = false;
        for (size_t i = 0; i < ignored_agents.size(); ++i) {
            if (space_time_grid[to][arrival_time] == ignored_agents[i]) {
                is_ignored = true;
                break;
            }
        }
        if (!is_ignored) return true;
    }

    // Check edge conflict
    if (space_time_grid[to].size() >= arrival_time &&
        space_time_grid[from].size() > arrival_time &&
        !space_time_grid[to].empty() &&
        space_time_grid[to][arrival_time - 1] != NO_AGENT &&
        space_time_grid[from][arrival_time] == space_time_grid[to][arrival_time - 1]) {
        bool is_ignored = false;
        for (size_t i = 0; i < ignored_agents.size(); ++i) {
            if (space_time_grid[to][arrival_time - 1] == ignored_agents[i]) {
                is_ignored = true;
                break;
            }
        }
        if (!is_ignored) return true;
    }
    return false;
}

/**
 * @brief Find agents conflicting with a potential move
 * Identifies all agents that would conflict with the given move
 * @param agent_id Agent making the move (to exclude from conflicts)
 * @param conflicts Set to store conflicting agent IDs
 * @param from Starting location
 * @param to Ending location
 * @param arrival_time Time of arrival at destination
 */
void PathT::findConflictingAgents(int agent_id, set<int>& conflicts, int from, int to, int arrival_time) const {
    if (space_time_grid.empty()) return;

    // Add agent occupying destination at arrival time
    if (space_time_grid[to].size() > arrival_time && space_time_grid[to][arrival_time] != NO_AGENT)
        conflicts.insert(space_time_grid[to][arrival_time]);

    // Add agent involved in edge conflict
    if (space_time_grid[to].size() >= arrival_time &&
        space_time_grid[from].size() > arrival_time &&
        space_time_grid[to][arrival_time - 1] != NO_AGENT &&
        space_time_grid[from][arrival_time] == space_time_grid[to][arrival_time - 1])
        conflicts.insert(space_time_grid[from][arrival_time]);
}

/**
 * @brief Get all agents occupying a specific location
 * Collects all agents that visit the given location at any time
 * @param conflicts Set to store agent IDs
 * @param location Location to check
 */
void PathT::get_agents(set<int>& conflicts, int location) const {
    if (location < 0) return;
    for (auto agent : space_time_grid[location]) {
        if (agent >= 0) conflicts.insert(agent);
    }
}

/**
 * @brief Get a sample of agents occupying a location (bounded size)
 * Randomly samples agents visiting a location, preferring recent visits
 * @param conflicts Set to store agent IDs
 * @param neighbor_size Maximum number of agents to return
 * @param location Location to check
 */
void PathT::get_agents(set<int>& conflicts, int neighbor_size, int location) const {
    if (location < 0 || space_time_grid[location].empty()) return;

    int max_t = (int)space_time_grid[location].size() - 1;
    while (space_time_grid[location][max_t] == NO_AGENT && max_t > 0) max_t--;
    if (max_t == 0) return;

    // Start from random timestep and sample backwards/forwards
    int base_time = rand() % max_t;
    if (space_time_grid[location][base_time] != NO_AGENT)
        conflicts.insert(space_time_grid[location][base_time]);

    int offset = 1;
    while (base_time - offset >= 0 || base_time + offset <= max_t) {
        if (base_time - offset >= 0 &&
            space_time_grid[location][base_time - offset] != NO_AGENT) {
            conflicts.insert(space_time_grid[location][base_time - offset]);
            if ((int)conflicts.size() == neighbor_size) return;
        }
        if (base_time + offset <= max_t &&
            space_time_grid[location][base_time + offset] != NO_AGENT) {
            conflicts.insert(space_time_grid[location][base_time + offset]);
            if ((int)conflicts.size() == neighbor_size) return;
        }
        offset++;
    }
}

/**
 * @brief Get earliest time agent can hold/wait at a location
 * Finds the minimum time an agent can stay at a location without conflicts
 * @param location Location to check
 * @param earliest_timestep Minimum allowed timestep
 * @return Earliest available holding time
 */
int LNS::PathT::getHoldingTime(int location, int earliest_timestep) const {
    if (space_time_grid.empty() ||
        (int)space_time_grid[location].size() <= earliest_timestep)
        return earliest_timestep;

    int hold_time = (int)space_time_grid[location].size();
    while (hold_time > earliest_timestep &&
           space_time_grid[location][hold_time - 1] == NO_AGENT)
        hold_time--;
    return hold_time;
}

void LNS::PathTableWC::registerPath(int agent_id, const Path& path) {
    path_cache[agent_id] = &path;
    if (path.empty()) return;
    
    for (int timestep = 0; timestep < (int)path.size(); timestep++) {
        if (conflict_grid[path[timestep].location].size() <= timestep)
            conflict_grid[path[timestep].location].resize(timestep + 1);
        conflict_grid[path[timestep].location][timestep].push_back(agent_id);
    }
    
    if (target_timestamps[path.back().location] == MAX_TIMESTEP)
        target_timestamps[path.back().location] = (int)path.size() - 1;
    max_time_horizon = max(max_time_horizon, (int)path.size() - 1);
}

void LNS::PathTableWC::registerPath(int agent_id) {
    if (path_cache[agent_id] != nullptr) 
        registerPath(agent_id, *path_cache[agent_id]);
}

void LNS::PathTableWC::unregisterPath(int agent_id) {
    const Path& path = *path_cache[agent_id];
    if (path.empty()) return;
    
    for (int timestep = 0; timestep < (int)path.size(); timestep++) {
        if (conflict_grid[path[timestep].location].size() > timestep &&
            std::find(conflict_grid[path[timestep].location][timestep].begin(), 
                     conflict_grid[path[timestep].location][timestep].end(), 
                     agent_id) != conflict_grid[path[timestep].location][timestep].end())
            conflict_grid[path[timestep].location][timestep].remove(agent_id);
    }
    
    target_timestamps[path.back().location] = MAX_TIMESTEP;
    if (max_time_horizon == (int)path.size() - 1) {
        max_time_horizon = 0;
        for (int time : target_timestamps) {
            if (time < MAX_TIMESTEP && time > max_time_horizon) 
                max_time_horizon = time;
        }
    }
}

int LNS::PathTableWC::countFutureConflicts(int location, int time) const {
    if (target_timestamps[location] != MAX_TIMESTEP) return 0;
    
    int total = 0;
    if (!conflict_grid.empty() && (int)conflict_grid[location].size() > time) {
        for (int t = time + 1; t < (int)conflict_grid[location].size(); t++)
            total += (int)conflict_grid[location][t].size();
    }
    return total;
}

int LNS::PathTableWC::countConflicts(int from, int to, int arrival_time) const {
    int total = 0;
    if (!conflict_grid.empty()) {
        if ((int)conflict_grid[to].size() > arrival_time) 
            total += (int)conflict_grid[to][arrival_time].size();
        
        if (from != to && conflict_grid[to].size() >= arrival_time && 
            conflict_grid[from].size() > arrival_time) {
            for (auto agent1 : conflict_grid[to][arrival_time - 1]) {
                for (auto agent2 : conflict_grid[from][arrival_time]) {
                    if (agent1 == agent2) total++;
                }
            }
        }
    }
    if (!target_timestamps.empty() && target_timestamps[to] < arrival_time) 
        total++;
    return total;
}

bool PathTableWC::hasCollisions(int from, int to, int arrival_time) const {
    if (!conflict_grid.empty()) {
        if ((int)conflict_grid[to].size() > arrival_time && 
            !conflict_grid[to][arrival_time].empty()) 
            return true;
        
        if (from != to && conflict_grid[to].size() >= arrival_time && 
            conflict_grid[from].size() > arrival_time) {
            for (auto agent1 : conflict_grid[to][arrival_time - 1]) {
                for (auto agent2 : conflict_grid[from][arrival_time]) {
                    if (agent1 == agent2) return true;
                }
            }
        }
    }
    if (!target_timestamps.empty() && target_timestamps[to] < arrival_time) 
        return true;
    return false;
}

bool LNS::PathTableWC::hasEdgeCollisions(int from, int to, int arrival_time) const {
    if (conflict_grid.empty() || from == to) return false;
    
    if (conflict_grid[to].size() >= arrival_time && 
        conflict_grid[from].size() > arrival_time) {
        for (auto agent1 : conflict_grid[to][arrival_time - 1]) {
            for (auto agent2 : conflict_grid[from][arrival_time]) {
                if (agent1 == agent2) return true;
            }
        }
    }
    return false;
}

int LNS::PathTableWC::getLastCollisionTimestep(int location) const {
    if (conflict_grid.empty()) return -1;
    
    for (int timestep = (int)conflict_grid[location].size() - 1; timestep >= 0; timestep--) {
        if (!conflict_grid[location][timestep].empty()) 
            return timestep;
    }
    return -1;
}

void LNS::PathTableWC::clear() {
    conflict_grid.clear();
    target_timestamps.clear();
    path_cache.clear();
}

// ====================================================================================================
// CONSTRAINT TABLE IMPLEMENTATION
// ====================================================================================================

/**
 * @brief Get maximum timestep across all constraint types
 * Calculates the maximum time horizon considering hard/soft constraints, path lengths, and landmarks
 * @return Maximum timestep value
 */
int LNS::ConstraintTable::getMaxTimestep() const {
    int max_t = max(max(constraint_end_time, penalty_end_time), min_path_length);
    
    if (hard_constraints_table != nullptr) 
        max_t = max(max_t, hard_constraints_table->max_time_horizon);
    
    if (soft_constraints_table != nullptr) 
        max_t = max(max_t, soft_constraints_table->max_time_horizon);
    
    if (max_path_length < MAX_TIMESTEP) 
        max_t = max(max_t, max_path_length);
    
    if (!landmarks.empty()) 
        max_t = max(max_t, landmarks.rbegin()->first);
    
    return max_t;
}

/**
 * @brief Get last timestep with collision at a location
 * Finds the most recent timestep where a collision occurs at the given location
 * @param location Location to check
 * @return Last collision timestep or -1 if none
 */
int LNS::ConstraintTable::getLastCollisionTimestep(int location) const {
    int latest = -1;
    
    if (soft_constraints_table != nullptr) 
        latest = soft_constraints_table->getLastCollisionTimestep(location);
    
    if (!penalty_grid.empty()) {
        for (auto timestep = penalty_grid[location].size() - 1; timestep > latest; timestep--) {
            if (penalty_grid[location][timestep]) 
                return timestep;
        }
    }
    return latest;
}

/**
 * @brief Add hard constraint for edge movement
 * Prevents agent movement between locations during specified time interval
 * @param from Starting location
 * @param to Ending location
 * @param t_min Minimum timestep (inclusive)
 * @param t_max Maximum timestep (exclusive)
 */
void LNS::ConstraintTable::addHardConstraint(size_t from, size_t to, int t_min, int t_max) {
    addHardConstraint(getEdgeIndex(from, to), t_min, t_max);
}

/**
 * @brief Add hard constraint for vertex occupation
 * Prevents agent from occupying a location during specified time interval
 * @param location Location to block
 * @param t_min Minimum timestep (inclusive)
 * @param t_max Maximum timestep (exclusive)
 */
void LNS::ConstraintTable::addHardConstraint(size_t location, int t_min, int t_max) {
    if (location < 0) return;
    
    blocking_constraints[location].emplace_back(t_min, t_max);
    
    if (t_max < MAX_TIMESTEP && t_max > constraint_end_time) {
        constraint_end_time = t_max;
    } else if (t_max == MAX_TIMESTEP && t_min > constraint_end_time) {
        constraint_end_time = t_min;
    }
}

/**
 * @brief Add hard constraints from a complete path
 * Creates constraints that prevent other agents from following the same path
 * @param path Path to convert into hard constraints
 */
void LNS::ConstraintTable::addHardConstraint(const Path& path) {
    int prev_loc = path.front().location;
    int prev_time = 0;
    
    for (int timestep = 0; timestep < (int)path.size(); timestep++) {
        auto curr_loc = path[timestep].location;
        if (prev_loc != curr_loc) {
            addHardConstraint(prev_loc, prev_time, timestep);
            addHardConstraint(curr_loc, prev_loc, timestep, timestep + 1);
            prev_loc = curr_loc;
            prev_time = timestep;
        }
    }
    addHardConstraint(path.back().location, (int)path.size() - 1, MAX_TIMESTEP);
}

/**
 * @brief Add soft constraints from other agents' paths
 * Creates penalty constraints from paths of other agents (excluding specified agent)
 * @param agent Agent to exclude from constraint generation
 * @param paths Vector of all agent paths
 */
void LNS::ConstraintTable::addSoftConstraints(int agent, const vector<Path*>& paths) {
    for (size_t ag = 0; ag < paths.size(); ag++) {
        if (ag == agent || paths[ag] == nullptr) continue;
        addSoftConstraint(*paths[ag]);
    }
}

/**
 * @brief Add soft constraint from a path
 * Creates penalty zones along the path to discourage but not prevent conflicts
 * @param path Path to convert into soft constraints
 */
void LNS::ConstraintTable::addSoftConstraint(const Path& path) {
    if (penalty_grid.empty()) {
        penalty_grid.resize(map_size);
        penalty_goal_times.resize(map_size, MAX_TIMESTEP);
    }
    
    if (penalty_goal_times[path.back().location] == MAX_TIMESTEP)
        penalty_goal_times[path.back().location] = path.size() - 1;
    
    for (auto timestep = (int)path.size() - 1; timestep >= 0; timestep--) {
        int loc = path[timestep].location;
        if (penalty_grid[loc].size() <= timestep) 
            penalty_grid[loc].resize(timestep + 1, false);
        penalty_grid[loc][timestep] = true;
    }
    penalty_end_time = max(penalty_end_time, (int)path.size() - 1);
}

/**
 * @brief Check if vertex is blocked at given time
 * Tests if a location is blocked by hard constraints or landmarks at specified time
 * @param location Location to check
 * @param t Timestep to check
 * @return True if location is blocked
 */
bool LNS::ConstraintTable::isBlocked(size_t location, int t) const {
    if (location < 0) return false;
    
    if (location < map_size) {
        if (hard_constraints_table != nullptr && 
            hard_constraints_table->isBlocked(location, location, t)) 
            return true;
        
        const auto& it = landmarks.find(t);
        if (it != landmarks.end() && it->second != location) 
            return true;
    }

    const auto& it = blocking_constraints.find(location);
    if (it == blocking_constraints.end()) return false;
    
    for (const auto& constraint : it->second) {
        if (constraint.first <= t && t < constraint.second) 
            return true;
    }
    return false;
}

/**
 * @brief Check if edge movement is blocked at given time
 * Tests if movement between locations is blocked by hard constraints
 * @param curr_loc Current location
 * @param next_loc Next location
 * @param next_t Time of movement
 * @return True if edge movement is blocked
 */
bool LNS::ConstraintTable::isBlocked(size_t curr_loc, size_t next_loc, int next_t) const {
    return (hard_constraints_table != nullptr && 
            hard_constraints_table->isBlocked(curr_loc, next_loc, next_t)) ||
           isBlocked(getEdgeIndex(curr_loc, next_loc), next_t);
}

void LNS::ConstraintTable::copy(const ConstraintTable& other) {
    min_path_length = other.min_path_length;
    max_path_length = other.max_path_length;
    num_columns = other.num_columns;
    map_size = other.map_size;
    blocking_constraints = other.blocking_constraints;
    constraint_end_time = other.constraint_end_time;
    penalty_grid = other.penalty_grid;
    penalty_goal_times = other.penalty_goal_times;
    penalty_end_time = other.penalty_end_time;
    landmarks = other.landmarks;
    hard_constraints_table = other.hard_constraints_table;
    soft_constraints_table = other.soft_constraints_table;
}

int LNS::ConstraintTable::countStepConflicts(size_t curr_id, size_t next_id, int next_timestep) const {
    int total = 0;
    
    if (soft_constraints_table != nullptr)
        total = soft_constraints_table->countConflicts(curr_id, next_id, next_timestep);

    if (!penalty_grid.empty()) {
        if (penalty_grid[next_id].size() > next_timestep && 
            penalty_grid[next_id][next_timestep]) 
            total++;
        
        if (curr_id != next_id && 
            penalty_grid[next_id].size() >= next_timestep && 
            penalty_grid[curr_id].size() > next_timestep &&
            penalty_grid[next_id][next_timestep - 1] && 
            penalty_grid[curr_id][next_timestep]) 
            total++;
        
        if (penalty_goal_times[next_id] < next_timestep) 
            total++;
    }
    return total;
}

bool LNS::ConstraintTable::hasStepConflict(size_t curr_id, size_t next_id, int next_timestep) const {
    if (soft_constraints_table != nullptr && 
        soft_constraints_table->hasCollisions(curr_id, next_id, next_timestep)) 
        return true;
    
    if (!penalty_grid.empty()) {
        if (penalty_grid[next_id].size() > next_timestep && 
            penalty_grid[next_id][next_timestep]) 
            return true;
        
        if (curr_id != next_id && 
            penalty_grid[next_id].size() >= next_timestep && 
            penalty_grid[curr_id].size() > next_timestep &&
            penalty_grid[next_id][next_timestep - 1] && 
            penalty_grid[curr_id][next_timestep]) 
            return true;
        
        if (penalty_goal_times[next_id] < next_timestep) 
            return true;
    }
    return false;
}

bool LNS::ConstraintTable::hasEdgeConflict(size_t curr_id, size_t next_id, int next_timestep) const {
    if (curr_id == next_id) return false;
    
    if (soft_constraints_table != nullptr && 
        soft_constraints_table->hasEdgeCollisions(curr_id, next_id, next_timestep)) 
        return true;
    
    return !penalty_grid.empty() && curr_id != next_id && 
           penalty_grid[next_id].size() >= next_timestep &&
           penalty_grid[curr_id].size() > next_timestep && 
           penalty_grid[next_id][next_timestep - 1] && 
           penalty_grid[curr_id][next_timestep];
}

int LNS::ConstraintTable::countFutureConflicts(int location, int t) const {
    int total = 0;
    
    if (soft_constraints_table != nullptr) 
        total = soft_constraints_table->countFutureConflicts(location, t);
    
    if (!penalty_grid.empty()) {
        for (auto timestep = t + 1; timestep < penalty_grid[location].size(); timestep++) {
            total += (int)penalty_grid[location][timestep];
        }
    }
    return total;
}

int LNS::ConstraintTable::getHoldingTime(int location, int earliest_timestep) const {
    int hold_time = earliest_timestep;
    
    if (hard_constraints_table != nullptr) 
        hold_time = hard_constraints_table->getHoldingTime(location, earliest_timestep);
    
    auto it = blocking_constraints.find(location);
    if (it != blocking_constraints.end()) {
        for (auto time_range : it->second) 
            hold_time = max(hold_time, time_range.second);
    }
    
    for (auto landmark : landmarks) {
        if (landmark.second != location) 
            hold_time = max(hold_time, (int)landmark.first + 1);
    }
    return hold_time;
}

}
