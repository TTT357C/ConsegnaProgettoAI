/**
 * @file lns_parallel_core.h
 * @brief Core data structures and utilities for parallel Large Neighborhood Search
 *
 * This file contains the fundamental data structures and classes used in the parallel
 * LNS implementation for multi-agent pathfinding. It includes path representations,
 * agent structures, neighbor definitions, and the time-space A* planner.
 *
 * Key Components:
 * - Path and PathEntry: Spatiotemporal path representations with orientation
 * - Agent: Individual agent data with path and goal information
 * - Neighbor: Solution neighborhood for LNS exploration
 * - TimeSpaceAStarPlanner: Orientation-aware A* search with conflict avoidance
 * - Supporting enums and constants for movement and heuristics
 */

#pragma once
#include "LNS/lns_core.h"
#include "LaCAM2/lacam2_core.hpp"
#include "util/HeuristicTable.h"
#include "util/Utils.h"
#include "boost/heap/pairing_heap.hpp"
#include "boost/unordered_set.hpp"
#include <omp.h>
#include <memory>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <list>

namespace LNS {
namespace Parallel {

// ================================================================================================
// CONSTANTS AND ENUMS
// ================================================================================================

static constexpr int NUM_DIRECTIONS = 5;           ///< Number of possible movement directions (4 cardinal + stay)
static constexpr int NUM_ORIENTATIONS = 4;         ///< Number of possible agent orientations
static constexpr int DIRECTION_EAST = 0;           ///< East movement direction
static constexpr int DIRECTION_SOUTH = 1;          ///< South movement direction
static constexpr int DIRECTION_WEST = 2;           ///< West movement direction
static constexpr int DIRECTION_NORTH = 3;          ///< North movement direction
static constexpr int DIRECTION_STAY = 4;           ///< Stationary (stay) action

/**
 * @enum destroy_heuristic
 * @brief Heuristics for selecting agents to replan in LNS
 */
enum destroy_heuristic {
    RANDOMAGENTS,   ///< Randomly select agents
    RANDOMWALK,     ///< Select agents via random walk from most delayed agent
    INTERSECTION,   ///< Select agents at intersection locations
    DESTORY_COUNT   ///< Number of destroy heuristics (for array sizing)
};

// ================================================================================================
// PATH DATA STRUCTURES
// ================================================================================================

/**
 * @struct PathEntry
 * @brief Single spatiotemporal state in an agent's path
 */
struct PathEntry {
    int location;      ///< Grid location index
    int orientation;   ///< Agent orientation (0-3)
    PathEntry(int location=-1, int orientation=-1): location(location), orientation(orientation) {}
};

/**
 * @struct Path
 * @brief Complete spatiotemporal path for an agent
 */
struct Path {
    std::vector<PathEntry> nodes;  ///< Sequence of path entries
    float path_cost;               ///< Total path cost (distance + time penalties)
    
    inline void clear() {
        path_cost=0;
        nodes.clear();
    }
    inline const PathEntry& operator[](int i) const { return nodes[i]; }
    inline PathEntry& operator[](int i) { return nodes[i]; }
    inline size_t size() const { return nodes.size(); }
    inline bool empty() const { return nodes.empty(); }
    inline const PathEntry& back() const { return nodes.back(); }
    inline PathEntry& back() { return nodes.back(); }
    inline const PathEntry& front() const { return nodes.front(); }
    inline PathEntry& front() { return nodes.front(); }
};

inline std::ostream& operator<<(std::ostream& out, const PathEntry& pe) {
    out << "Loc:" << pe.location << ",Orient:" << pe.orientation;
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Path& path) {
    if (path.nodes.empty()) {
        out << "<Empty Path>" << std::endl;
        return out;
    }
    out << "Path[" << path.nodes.size() << " steps, cost:" << path.path_cost << "]: ";
    for (size_t i = 0; i < path.nodes.size(); ++i) {
        const auto& pe = path.nodes[i];
        out << "(" << pe.location << "," << pe.orientation << ")";
        if (i < path.nodes.size() - 1) out << " -> ";
    }
    out << std::endl;
    return out;
}

// ================================================================================================
// AGENT DATA STRUCTURE
// ================================================================================================

/**
 * @struct Agent
 * @brief Individual agent data with path planning capabilities
 *
 * Represents an agent in the multi-agent pathfinding problem with its current path,
 * start/goal locations, and methods for path cost calculation and delay estimation.
 */
struct Agent {
    int id;                                           ///< Unique agent identifier
    Path path;                                        ///< Current planned path
    const Instance& instance;                         ///< Problem instance reference
    std::shared_ptr<HeuristicTable> HT;              ///< Heuristic distance table
    std::shared_ptr<std::vector<LaCAM2::AgentInfo>> agent_infos; ///< Agent information

    /**
     * @brief Constructor for Agent
     * @param id Agent identifier
     * @param instance Problem instance reference
     * @param HT Heuristic table for distance estimation
     * @param agent_infos Agent information from LaCAM2
     */
    Agent(int id, const Instance& instance, std::shared_ptr<HeuristicTable>& HT, 
          std::shared_ptr<std::vector<LaCAM2::AgentInfo>>& agent_infos): 
        id(id), HT(HT), instance(instance), agent_infos(agent_infos) {}

    /**
     * @brief Copy constructor for Agent
     * @param agent Agent to copy
     */
    Agent(const Agent& agent):
        id(agent.id), HT(agent.HT), instance(agent.instance), 
        agent_infos(agent.agent_infos), path(agent.path) {}

    /// @brief Get agent's start location
    inline int getStartLocation() { return instance.start_locations[id]; }
    /// @brief Get agent's start orientation
    inline int getStartOrientation() { return instance.start_orientations[id]; }
    /// @brief Get agent's goal location
    inline int getGoalLocation() { return instance.goal_locations[id]; }

    /**
     * @brief Calculate cost of moving between two states
     * @param pos_start Starting position
     * @param orient_start Starting orientation
     * @param pos_end Ending position
     * @param orient_end Ending orientation
     * @param HT Heuristic table with map weights
     * @return Movement cost
     */
    static inline float get_action_cost(int pos_start, int orient_start, int pos_end, int orient_end, 
                                        const std::shared_ptr<HeuristicTable>& HT) {
        const auto& map_weights = *(HT->map_weights);
        const int offset = pos_end - pos_start;
        const int weight_base_idx = pos_start * NUM_DIRECTIONS;
        
        if (offset == 0) return map_weights[weight_base_idx + DIRECTION_STAY];
        else if (offset == 1) return map_weights[weight_base_idx + DIRECTION_EAST];
        else if (offset == HT->env.cols) return map_weights[weight_base_idx + DIRECTION_SOUTH];
        else if (offset == -1) return map_weights[weight_base_idx + DIRECTION_WEST];
        else if (offset == -HT->env.cols) return map_weights[weight_base_idx + DIRECTION_NORTH];
        else {
            std::cerr << "[Error] Invalid move from " << pos_start << " to " << pos_end << endl;
            exit(-1);
        }
    }

    /**
     * @brief Calculate estimated path length with optional time limits
     * @param path Path to evaluate
     * @param goal_location Goal location
     * @param HT Heuristic table
     * @param arrival_break Stop when goal is reached
     * @param time_limit Maximum time steps to consider
     * @return Estimated path cost
     */
    inline float getEstimatedPathLength(const Path& path, int goal_location, 
                                       const std::shared_ptr<HeuristicTable>& HT, 
                                       bool arrival_break = false, int time_limit = -1) const {
        if ((*agent_infos)[id].disabled) return 0.0f;

        const int max_steps = (time_limit != -1) ? time_limit : static_cast<int>(path.size()) - 1;
        float accumulated_cost = 0.0f;
        bool has_arrived = false;
        
        for (int step = 0; step < max_steps; ++step) {
            const auto& current = path[step];
            const auto& next = path[step + 1];
            accumulated_cost += get_action_cost(current.location, current.orientation, next.location, next.orientation, HT);
            
            if (current.location == goal_location) {
                has_arrived = true;
                if (arrival_break) break;
            }
        }

        // Add heuristic cost to goal if not yet arrived
        if (!has_arrived && time_limit == -1) {
            const auto& last = path.back();
            accumulated_cost += HT->get(last.location, last.orientation, goal_location);
        }
        
        return accumulated_cost;
    }

    /**
     * @brief Calculate number of delays compared to optimal path
     * @return Delay cost (actual cost - optimal cost)
     */
    inline float getNumOfDelays() const {
        const int start_loc = instance.start_locations[id];
        const int start_orient = instance.start_orientations[id];
        const int goal_loc = instance.goal_locations[id];
        
        const float actual_cost = getEstimatedPathLength(path, goal_loc, HT);
        const float optimal_cost = HT->get(start_loc, start_orient, goal_loc);
        
        return actual_cost - optimal_cost;
    }

    /// @brief Reset agent path
    void reset() { path.clear(); }
};

// ================================================================================================
// NEIGHBOR SOLUTION STRUCTURE
// ================================================================================================

/**
 * @struct Neighbor
 * @brief Solution neighborhood for LNS exploration
 *
 * Represents a subset of agents selected for replanning along with their
 * old and new path solutions, used in the LNS optimization process.
 */
struct Neighbor {
    vector<int> agents;                    ///< IDs of agents in this neighborhood
    float sum_of_costs;                   ///< Total cost of new paths
    float old_sum_of_costs;               ///< Total cost of old paths
    std::map<int, Path> m_paths;          ///< New paths for agents in neighborhood
    std::map<int, Path> m_old_paths;      ///< Old paths for agents in neighborhood
    bool succ = false;                    ///< Whether optimization succeeded
    int selected_neighbor;                ///< Index of destroy heuristic used
    float num_arrived;
    float old_num_arrived;
};

// ================================================================================================
// TIME-SPACE A* STATE AND PLANNER
// ================================================================================================

/**
 * @class TimeSpaceAStarState
 * @brief State representation for time-space A* search with orientation awareness
 *
 * Represents a search state in the time-space A* algorithm, incorporating position,
 * orientation, time step, and conflict information for multi-agent pathfinding.
 * Uses a priority queue with custom comparison for conflict-minimizing search.
 */
class TimeSpaceAStarState {
public:
    int pos;                              ///< Current position on the map
    int orient;                           ///< Current orientation (0=right, 1=down, 2=left, 3=up)
    int t;                                ///< Current time step
    float g;                              ///< Cost from start to current state
    float h;                              ///< Heuristic cost to goal
    float f;                              ///< Total cost (g + h)
    int num_of_conflicts;                 ///< Number of conflicts with other agents
    bool arrived;                         ///< Whether agent has reached goal
    TimeSpaceAStarState* prev;            ///< Pointer to parent state for path reconstruction
    bool closed;                          ///< Whether state has been expanded
    
    /**
     * @struct Compare
     * @brief Custom comparison for priority queue ordering
     *
     * Orders states by: conflicts (ascending), arrival status (arrived first),
     * f-value (ascending), then h-value (ascending) for tie-breaking.
     */
    struct Compare {
        bool operator()(const TimeSpaceAStarState* s1, const TimeSpaceAStarState* s2) const {
            if (s1->num_of_conflicts == s2->num_of_conflicts) {
                if (s1->arrived == s2->arrived) {
                    if (s1->f == s2->f) return s1->h > s2->h;
                    return s1->f > s2->f;
                }
                return s1->arrived < s2->arrived;
            }
            return s1->num_of_conflicts > s2->num_of_conflicts;
        }
    };
    
    /// Handle for priority queue management
    boost::heap::pairing_heap<TimeSpaceAStarState*, boost::heap::compare<Compare>>::handle_type open_list_handle;

    /// @brief Default constructor
    TimeSpaceAStarState(): pos(-1), orient(-1), t(-1), g(-1), h(0), f(-1), 
        num_of_conflicts(0), prev(nullptr), closed(false), arrived(false) {}
    
    /**
     * @brief Parameterized constructor
     * @param pos Current position
     * @param orient Current orientation
     * @param t Current time step
     * @param g Cost from start
     * @param h Heuristic cost to goal
     * @param num_of_conflicts Number of conflicts
     * @param arrived Whether goal reached
     * @param prev Parent state pointer
     */
    TimeSpaceAStarState(int pos, int orient, int t, float g, float h, int num_of_conflicts, 
                       bool arrived, TimeSpaceAStarState* prev):
        pos(pos), orient(orient), t(t), g(g), h(h), f(g+h), num_of_conflicts(num_of_conflicts), 
        prev(prev), closed(false), arrived(arrived) {}

    /**
     * @brief Copy state data from another state
     * @param s Source state to copy from
     */
    void copy(const TimeSpaceAStarState* s) {
        pos = s->pos;
        orient = s->orient;
        t = s->t;
        g = s->g;
        h = s->h;
        f = s->f;
        num_of_conflicts = s->num_of_conflicts;
        arrived = s->arrived;
        prev = s->prev;
    }
    
    /**
     * @struct Hash
     * @brief Hash function for state uniqueness in unordered_set
     *
     * Combines time, position, orientation, and arrival status into a hash value
     * for efficient state lookup and duplicate detection.
     */
    struct Hash {
        std::size_t operator()(const TimeSpaceAStarState* s) const {
            size_t h = std::hash<int>()(s->t<<21+s->pos<<5+s->orient<<1+s->arrived);
            return h;        
        }
    };
    
    /**
     * @struct Equal
     * @brief Equality comparison for state uniqueness
     *
     * Two states are equal if they have the same position, orientation, time, and arrival status.
     */
    struct Equal {
        bool operator()(const TimeSpaceAStarState* s1, const TimeSpaceAStarState* s2) const {
            return s1->pos == s2->pos && s1->orient == s2->orient && 
                   s1->t == s2->t && s1->arrived == s2->arrived;
        }
    };
};

/**
 * @class TimeSpaceAStarPlanner
 * @brief Time-space A* planner with orientation awareness and conflict counting
 *
 * Implements a time-space A* search algorithm that finds conflict-minimizing paths
 * for agents in multi-agent pathfinding scenarios. Incorporates orientation changes,
 * movement costs, and constraint-based conflict avoidance.
 */
class TimeSpaceAStarPlanner {
public:
    Instance& instance;                    ///< Reference to problem instance
    std::shared_ptr<HeuristicTable> HT;   ///< Heuristic table for goal distance estimation
    std::shared_ptr<vector<float>> weights; ///< Movement cost weights for each action
    int execution_window;                 ///< Maximum planning horizon
    /// Priority queue for open states (min-heap by conflicts, arrival, f-value)
    boost::heap::pairing_heap<TimeSpaceAStarState*, boost::heap::compare<TimeSpaceAStarState::Compare>> open_list;
    /// Set of all generated states for duplicate detection
    boost::unordered_set<TimeSpaceAStarState*, TimeSpaceAStarState::Hash, TimeSpaceAStarState::Equal> all_states;
    std::vector<TimeSpaceAStarState*> successors; ///< Generated successor states
    int n_expanded;                       ///< Number of expanded states
    int n_generated;                      ///< Number of generated states
    Path path;                            ///< Final computed path
    static const int n_dirs = 5;          ///< Number of movement directions (4 cardinal + wait)
    static const int n_orients = 4;       ///< Number of possible orientations

    /**
     * @brief Constructor
     * @param instance Problem instance reference
     * @param HT Heuristic table for distance estimation
     * @param weights Movement cost weights
     * @param execution_window Maximum planning horizon
     */
    TimeSpaceAStarPlanner(Instance& instance, std::shared_ptr<HeuristicTable> HT, 
                         std::shared_ptr<vector<float>> weights, int execution_window): 
        instance(instance), HT(HT), weights(weights), execution_window(execution_window) {}

    /**
     * @brief Find conflict-minimizing path using time-space A* search
     * @param start_pos Starting position
     * @param start_orient Starting orientation
     * @param goal_pos Goal position
     * @param constraint_table Constraint table for conflict checking
     * @param time_limiter Time limit for search
     *
     * Performs A* search in time-space with orientation awareness, prioritizing
     * states with fewer conflicts. Explores movement and rotation actions while
     * respecting constraints and map boundaries.
     */
    void findPath(int start_pos, int start_orient, int goal_pos, ConstraintTable& constraint_table, 
                 const TimeLimiter& time_limiter) {
        clear();

        // Initialize start state
        TimeSpaceAStarState* start_state = new TimeSpaceAStarState(start_pos, start_orient, 0, 0, 
            HT->get(start_pos, start_orient, goal_pos), 0, false, nullptr);
        start_state->closed = false;
        start_state->open_list_handle = open_list.push(start_state);
        all_states.insert(start_state);

        // Main search loop
        while (!open_list.empty() && !time_limiter.has_timed_out()) {
            TimeSpaceAStarState* curr = open_list.top();
            open_list.pop();
            curr->closed = true;
            ++n_expanded;

            // Check for goal conditions
            if (execution_window == 1 && curr->pos == goal_pos && curr->t >= 1 && 
                !constraint_table.isBlocked(curr->pos, curr->t)) {
                buildPath(curr, goal_pos);
                return;
            }

            if (curr->t >= constraint_table.path_window) {
                buildPath(curr, goal_pos);
                return;
            }

            // Generate and process successors
            getSuccessors(curr, goal_pos, constraint_table);
            for (auto& next_state: successors) {
                ++n_generated;
                auto iter = all_states.find(next_state);
                if (iter == all_states.end()) {
                    // New state - add to open list
                    all_states.insert(next_state);
                    next_state->closed = false;
                    next_state->open_list_handle = open_list.push(next_state);
                } else {
                    // State already exists - check for better path
                    auto old_state = (*iter);
                    if (next_state->num_of_conflicts < old_state->num_of_conflicts || 
                        (next_state->num_of_conflicts == old_state->num_of_conflicts && 
                         next_state->f < old_state->f)) {
                        old_state->copy(next_state);
                        if (old_state->closed) {
                            old_state->closed = false;
                            old_state->open_list_handle = open_list.push(old_state);
                        } else {
                            open_list.increase(old_state->open_list_handle);
                        }
                    }
                    delete next_state;
                }
            }
        }
    }

    /// @brief Clear all search data structures and reset counters
    void clear() {
        open_list.clear();
        for (auto s: all_states) delete s;
        all_states.clear();
        path.clear();
        n_expanded = 0;
        n_generated = 0;
    }

    /**
     * @brief Reconstruct path from goal state to start
     * @param curr Goal state
     * @param goal_pos Goal position
     */
    void buildPath(TimeSpaceAStarState* curr, int goal_pos) {
        path.clear();
        path.path_cost = curr->f;
        auto s = curr;
        path.nodes.emplace_back(s->pos, s->orient);
        while (s->prev != nullptr) {
            s = s->prev;
            path.nodes.emplace_back(s->pos, s->orient);
        }
        std::reverse(path.nodes.begin(), path.nodes.end());
    }

    /**
     * @brief Generate successor states for current state
     * @param curr Current state
     * @param goal_pos Goal position
     * @param constraint_table Constraint table for conflict checking
     *
     * Generates all valid successor states by considering movement in current
     * orientation direction, rotation actions, and wait actions. Each successor
     * includes updated position, orientation, time, cost, and conflict count.
     */
    void getSuccessors(TimeSpaceAStarState* curr, int goal_pos, ConstraintTable& constraint_table) {
        successors.clear();
        int& cols = instance.num_of_cols;
        int& rows = instance.num_of_rows;
        auto& map = instance.my_map;
        auto& weights_vec = *this->weights;
        int pos = curr->pos;
        int x = pos % cols;
        int y = pos / cols;
        int orient = curr->orient;
        int next_timestep = curr->t + 1;

        int next_pos, weight_idx, next_orient;
        float next_g, next_h;
        int next_num_of_conflicts;
        bool next_arrived;

        // Generate movement successors based on current orientation
        if (orient == 0 && x + 1 < cols) {  // Moving right
            next_pos = pos + 1;
            weight_idx = pos * n_dirs;
            if (map[next_pos] == 0 && !constraint_table.isBlocked(pos, next_pos, next_timestep)) {
                next_orient = orient;
                next_g = curr->g + weights_vec[weight_idx];
                next_h = curr->arrived ? 0 : HT->get(next_pos, next_orient, goal_pos);
                next_num_of_conflicts = curr->num_of_conflicts + 
                    constraint_table.countStepConflicts(curr->pos, next_pos, next_timestep);
                next_arrived = curr->arrived | (next_pos == goal_pos);
                successors.push_back(new TimeSpaceAStarState(next_pos, next_orient, next_timestep, 
                    next_g, next_h, next_num_of_conflicts, next_arrived, curr));
            }
        } else if (orient == 1 && y + 1 < rows) {  // Moving down
            next_pos = pos + cols;
            weight_idx = pos * n_dirs + 1;
            if (map[next_pos] == 0 && !constraint_table.isBlocked(pos, next_pos, next_timestep)) {
                next_orient = orient;
                next_g = curr->g + weights_vec[weight_idx];
                next_h = curr->arrived ? 0 : HT->get(next_pos, next_orient, goal_pos);
                next_num_of_conflicts = curr->num_of_conflicts + 
                    constraint_table.countStepConflicts(curr->pos, next_pos, next_timestep);
                next_arrived = curr->arrived | (next_pos == goal_pos);
                successors.push_back(new TimeSpaceAStarState(next_pos, next_orient, next_timestep, 
                    next_g, next_h, next_num_of_conflicts, next_arrived, curr));
            }
        } else if (orient == 2 && x - 1 >= 0) {  // Moving left
            next_pos = pos - 1;
            weight_idx = pos * n_dirs + 2;
            if (map[next_pos] == 0 && !constraint_table.isBlocked(pos, next_pos, next_timestep)) {
                next_orient = orient;
                next_g = curr->g + weights_vec[weight_idx];
                next_h = curr->arrived ? 0 : HT->get(next_pos, next_orient, goal_pos);
                next_num_of_conflicts = curr->num_of_conflicts + 
                    constraint_table.countStepConflicts(curr->pos, next_pos, next_timestep);
                next_arrived = curr->arrived | (next_pos == goal_pos);
                successors.push_back(new TimeSpaceAStarState(next_pos, next_orient, next_timestep, 
                    next_g, next_h, next_num_of_conflicts, next_arrived, curr));
            }
        } else if (orient == 3 && y - 1 >= 0) {  // Moving up
            next_pos = pos - cols;
            weight_idx = pos * n_dirs + 3;
            if (map[next_pos] == 0 && !constraint_table.isBlocked(pos, next_pos, next_timestep)) {
                next_orient = orient;
                next_g = curr->g + weights_vec[weight_idx];
                next_h = curr->arrived ? 0 : HT->get(next_pos, next_orient, goal_pos);
                next_num_of_conflicts = curr->num_of_conflicts + 
                    constraint_table.countStepConflicts(curr->pos, next_pos, next_timestep);
                next_arrived = curr->arrived | (next_pos == goal_pos);
                successors.push_back(new TimeSpaceAStarState(next_pos, next_orient, next_timestep, 
                    next_g, next_h, next_num_of_conflicts, next_arrived, curr));
            }
        }

        // Generate rotation and wait successors at current position
        if (!constraint_table.isBlocked(pos, pos, next_timestep)) {
            weight_idx = pos * n_dirs + 4;  // Wait action cost
            next_g = curr->g + weights_vec[weight_idx];
            next_num_of_conflicts = curr->num_of_conflicts + 
                constraint_table.countStepConflicts(curr->pos, pos, next_timestep);
            next_arrived = curr->arrived || (pos == goal_pos);

            // Generate states for all possible rotations (left, no rotation, right)
            for (int rot : {1, -1, 0}) {
                next_orient = (orient + rot + n_orients) % n_orients;
                next_h = curr->arrived ? 0.0f : HT->get(pos, next_orient, goal_pos);
                successors.push_back(new TimeSpaceAStarState(pos, next_orient, next_timestep, 
                    next_g, next_h, next_num_of_conflicts, next_arrived, curr));
            }
        }
    }
};

}
}
