/**
 * @file lns_managers.h
 * @brief Parallel LNS Manager Classes for Large Neighborhood Search Optimization
 *
 * This file contains the core manager classes for parallel Large Neighborhood Search (LNS)
 * optimization in multi-agent pathfinding. The managers coordinate between local optimization,
 * neighbor generation, and global search coordination using parallel processing.
 *
 * Key Components:
 * - LocalOptimizer: Handles local search optimization for agent subsets
 * - NeighborGenerator: Generates new solution neighborhoods for exploration
 * - GlobalManager: Coordinates overall parallel LNS search process
 *
 * The implementation uses thread-safe data structures and OpenMP for parallel execution,
 * supporting various destroy heuristics and adaptive neighborhood selection.
 */

#pragma once
#include "lns_parallel_core.h"
#include <algorithm>
#include <cstdlib>

namespace LNS {
namespace Parallel {

// ================================================================================================
// LOCAL OPTIMIZER CLASS
// ================================================================================================

/**
 * @class LocalOptimizer
 * @brief Handles local search optimization for subsets of agents in parallel LNS
 *
 * The LocalOptimizer manages local optimization of agent paths within a neighborhood.
 * It coordinates path planning, constraint management, and cost evaluation for
 * agent subsets during the LNS optimization process.
 */
class LocalOptimizer {
public:
    Instance& instance;                          ///< Reference to problem instance
    PathT path_table;                           ///< Spatiotemporal occupancy table
    std::vector<Agent> agents;                  ///< Agent data with current paths
    std::shared_ptr<HeuristicTable> HT;         ///< Heuristic distance table
    std::shared_ptr<vector<float>> map_weights; ///< Edge weights for planning
    std::shared_ptr<TimeSpaceAStarPlanner> path_planner; ///< Time-space A* planner
    std::mt19937 MT;                            ///< Random number generator
    std::shared_ptr<std::vector<LaCAM2::AgentInfo>> agent_infos; ///< Agent information
    std::vector<Neighbor> updating_queue;       ///< Queue for neighbor updates
    string replan_algo_name;                    ///< Replanning algorithm name
    int window_size_for_CT;                     ///< Window size for constraint table
    int window_size_for_CAT;                    ///< Window size for conflict avoidance table
    int path_window;                            ///< Planning horizon window
    bool has_disabled_agents;                   ///< Whether some agents are disabled

    /**
     * @brief Constructor for LocalOptimizer
     * @param instance Problem instance reference
     * @param agents Vector of agents to optimize
     * @param HT Heuristic table for distance estimation
     * @param map_weights Edge weights for the map
     * @param agent_infos Agent information from LaCAM2
     * @param replan_algo_name Algorithm for replanning ("PP" for Priority Planning)
     * @param sipp Whether to use SIPP (Safe Interval Path Planning)
     * @param window_size_for_CT Constraint table window size
     * @param window_size_for_CAT Conflict avoidance table window size
     * @param path_window Planning horizon length
     * @param execution_window Execution window size
     * @param has_disabled_agents Whether some agents are inactive
     * @param random_seed Random seed for shuffling
     */
    LocalOptimizer(Instance& instance, std::vector<Agent>& agents, std::shared_ptr<HeuristicTable> HT, 
                  std::shared_ptr<vector<float>> map_weights, std::shared_ptr<std::vector<LaCAM2::AgentInfo>> agent_infos,
                  string replan_algo_name, bool sipp, int window_size_for_CT, int window_size_for_CAT, 
                  int path_window, int execution_window, bool has_disabled_agents, int random_seed):
        instance(instance), path_table(instance.map_size, window_size_for_CT), HT(HT), map_weights(map_weights), 
        agent_infos(agent_infos), replan_algo_name(replan_algo_name), window_size_for_CT(window_size_for_CT), 
        window_size_for_CAT(window_size_for_CAT), path_window(path_window), 
        has_disabled_agents(has_disabled_agents), MT(random_seed) {
        path_planner = std::make_shared<TimeSpaceAStarPlanner>(instance, HT, map_weights, execution_window);
        for (int i = 0; i < instance.num_of_agents; ++i) {
            this->agents.emplace_back(i, instance, HT, agent_infos);
        }
    }

    /**
     * @brief Reset the path table for new optimization iteration
     */
    void reset() {
        path_table.reset();
    }

    /**
     * @brief Update agent paths after successful neighbor optimization
     * @param neighbor The neighbor solution to apply
     */
    void update(Neighbor& neighbor) {
        if (neighbor.succ) {
            // Unregister old paths from conflict table
            for (auto& aid: neighbor.agents) {
                path_table.unregisterPath(aid, neighbor.m_old_paths[aid]);
            }
            // Register new paths and update agent data
            for (auto& aid: neighbor.agents) {
                path_table.registerPath(aid, neighbor.m_paths[aid]);
                agents[aid].path = neighbor.m_paths[aid];
            }
        }
    }

    /**
     * @brief Prepare neighbor for optimization by unregistering old paths
     * @param neighbor The neighbor to prepare
     */
    void prepare(Neighbor& neighbor) {
        neighbor.old_sum_of_costs = 0;
        for (auto& aid: neighbor.agents) {
            auto& agent = agents[aid];
            if (replan_algo_name == "PP") neighbor.m_old_paths[aid] = agent.path;
            neighbor.old_sum_of_costs += agent.path.path_cost;
            path_table.unregisterPath(aid, neighbor.m_old_paths[aid]);
        }   
    }

    /**
     * @brief Optimize a neighbor solution using local search
     * @param neighbor The neighbor to optimize
     * @param time_limiter Time limit for optimization
     */
    void optimize(Neighbor& neighbor, const TimeLimiter& time_limiter) {
        prepare(neighbor);
        bool succ = false;
        if (replan_algo_name == "PP") {
            succ = runPP(neighbor, time_limiter);
        } else {
            cerr << "Wrong replanning strategy" << endl;
            exit(-1);
        }
        neighbor.succ = succ;
    }

    /**
     * @brief Run Priority Planning (PP) algorithm for local optimization
     * @param neighbor The neighbor to optimize
     * @param time_limiter Time limit for optimization
     * @return True if optimization succeeded
     */
    bool runPP(Neighbor& neighbor, const TimeLimiter& time_limiter) {
        auto shuffled_agents = neighbor.agents;
        std::shuffle(shuffled_agents.begin(), shuffled_agents.end(), MT);

        if (has_disabled_agents) {
            std::stable_sort(shuffled_agents.begin(), shuffled_agents.end(), [&](int a, int b) {
                return (*agent_infos)[a].disabled < (*agent_infos)[b].disabled;
            });
        }


        int remaining_agents = (int)shuffled_agents.size();
        auto p = shuffled_agents.begin();
        neighbor.sum_of_costs = 0;
        ConstraintTable constraint_table(instance.num_of_cols, instance.map_size, &path_table, nullptr, 
                                        window_size_for_CT, window_size_for_CAT, path_window);

        while (p != shuffled_agents.end()) {
            if (time_limiter.has_timed_out()) break;

            int id = *p;
            int start_pos = instance.start_locations[id];
            int start_orient = instance.start_orientations[id];
            int goal_pos = instance.goal_locations[id];


            path_planner->findPath(start_pos, start_orient, goal_pos, constraint_table, time_limiter);
            neighbor.m_paths[id] = path_planner->path;

            if (neighbor.m_paths[id].empty()) break;

            if (neighbor.m_paths[id].back().location != agents[id].getGoalLocation()) {
                if (neighbor.m_paths[id].size() != constraint_table.path_window + 1) {
                    std::cerr << "agent " << agents[id].id << "'s path length " << neighbor.m_paths[id].size() 
                             << " should be equal to window size for path " << constraint_table.path_window 
                             << " if it doesn't arrive at its goal" << endl;
                    exit(-1);
                } 
            }

            neighbor.m_paths[id].path_cost = agents[id].getEstimatedPathLength(neighbor.m_paths[id], goal_pos, HT);
            neighbor.sum_of_costs += neighbor.m_paths[id].path_cost;

            if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs) {
                neighbor.m_paths[id].clear();
                break;
            }
            remaining_agents--;
            path_table.registerPath(agents[id].id, neighbor.m_paths[id]);
            ++p;
        }

        for (auto& aid: neighbor.agents) {
            path_table.unregisterPath(aid, neighbor.m_paths[aid]);
        }

        if (!neighbor.m_old_paths.empty()) {
            for (auto& aid : neighbor.agents) {
                path_table.registerPath(aid, neighbor.m_old_paths[aid]);
            }
        }

        // Success if all agents replanned and cost improved
        return (remaining_agents == 0 && neighbor.sum_of_costs <= neighbor.old_sum_of_costs);
    }
};

// ================================================================================================
// NEIGHBOR GENERATOR CLASS
// ================================================================================================

/**
 * @class NeighborGenerator
 * @brief Generates new solution neighborhoods for LNS exploration
 *
 * The NeighborGenerator creates diverse neighborhoods by selecting subsets of agents
 * to replan using various destroy heuristics. It supports both fixed-size and
 * adaptive neighborhood selection with tabu lists to avoid cycling.
 */
class NeighborGenerator {
public:
    Instance& instance;                           ///< Problem instance reference
    std::shared_ptr<HeuristicTable> HT;          ///< Heuristic distance table
    PathT& path_table;                           ///< Spatiotemporal occupancy table
    std::vector<Agent>& agents;                  ///< Agent data with current paths
    std::mt19937 MT;                             ///< Random number generator
    std::shared_ptr<std::vector<LaCAM2::AgentInfo>> agent_infos; ///< Agent information
    int neighbor_size;                           ///< Target neighborhood size
    destroy_heuristic destroy_strategy;          ///< Heuristic for agent selection
    bool ALNS;                                   ///< Whether to use Adaptive LNS
    double decay_factor;                         ///< Weight decay factor for ALNS
    double reaction_factor;                      ///< Reaction factor for ALNS
    int num_threads;                             ///< Number of parallel threads
    static const int n_orients = 4;              ///< Number of possible orientations
    vector<double> destroy_weights;              ///< Weights for destroy heuristics
    std::vector<unordered_set<int>> tabu_list_list; ///< Tabu lists for each thread
    list<int> intersections;                     ///< List of intersection locations
    std::vector<std::shared_ptr<Neighbor>> neighbors; ///< Generated neighbor solutions

    /**
     * @brief Constructor for NeighborGenerator
     * @param instance Problem instance reference
     * @param HT Heuristic table for distance estimation
     * @param path_table Spatiotemporal occupancy table
     * @param agents Vector of agents with current paths
     * @param agent_infos Agent information from LaCAM2
     * @param neighbor_size Target size of neighborhoods to generate
     * @param destroy_strategy Heuristic for selecting agents to replan
     * @param ALNS Whether to use Adaptive Large Neighborhood Search
     * @param decay_factor Weight decay factor for ALNS adaptation
     * @param reaction_factor Reaction factor for ALNS adaptation
     * @param num_threads Number of parallel threads
     * @param random_seed Random seed for reproducibility
     */
    NeighborGenerator(Instance& instance, std::shared_ptr<HeuristicTable> HT, PathT& path_table, 
                     std::vector<Agent>& agents, std::shared_ptr<std::vector<LaCAM2::AgentInfo>> agent_infos,
                     int neighbor_size, destroy_heuristic destroy_strategy, bool ALNS, double decay_factor, 
                     double reaction_factor, int num_threads, int random_seed):
        instance(instance), HT(HT), path_table(path_table), agents(agents), agent_infos(agent_infos),
        neighbor_size(neighbor_size), destroy_strategy(destroy_strategy), ALNS(ALNS), decay_factor(decay_factor), 
        reaction_factor(reaction_factor), num_threads(num_threads), MT(random_seed) {
        // Initialize destroy heuristic weights
        destroy_weights.assign(DESTORY_COUNT, 1);
        // Identify intersection locations (degree > 2) for intersection-based heuristics
        for (int i = 0; i < instance.map_size; i++) {
            if (!instance.isObstacle(i) && instance.getDegree(i) > 2) intersections.push_back(i);
        }
        // Initialize tabu lists and neighbor storage for each thread
        tabu_list_list.resize(num_threads);
        neighbors.resize(num_threads);
    }

    /**
     * @brief Reset neighbor generator state for new search iteration
     */
    void reset() {
        destroy_weights.assign(DESTORY_COUNT, 1);
        for (auto& tabu_list: tabu_list_list) tabu_list.clear();
    }

    /**
     * @brief Update destroy heuristic weights based on neighbor performance (ALNS)
     * @param neighbor The neighbor solution that was evaluated
     */
    void update(Neighbor& neighbor) {
        if (ALNS) {
            if (neighbor.old_sum_of_costs > neighbor.sum_of_costs) {
                // Reward successful heuristic with improved weights
                destroy_weights[neighbor.selected_neighbor] = reaction_factor * 
                    (neighbor.old_sum_of_costs - neighbor.sum_of_costs) / neighbor.agents.size() + 
                    (1 - reaction_factor) * destroy_weights[neighbor.selected_neighbor];
            } else {
                // Decay weights for unsuccessful heuristic
                destroy_weights[neighbor.selected_neighbor] = (1 - decay_factor) * destroy_weights[neighbor.selected_neighbor];
            }
        }
    }

    /**
     * @brief Generate neighbors in parallel across multiple threads
     * @param time_limiter Time limit for generation
     */
    void generate_parallel(const TimeLimiter& time_limiter) {
        #pragma omp parallel for
        for (int i = 0; i < num_threads; i++) {
            generate(time_limiter, i);
        }
    }

    /**
     * @brief Generate a single neighbor solution for the given thread
     * @param time_limiter Time limit for generation
     * @param idx Thread index for tabu list management
     * @return Generated neighbor solution
     */
    Neighbor generate(const TimeLimiter& time_limiter, int idx) {
        std::shared_ptr<Neighbor> neighbor_ptr = std::make_shared<Neighbor>();
        Neighbor& neighbor = *neighbor_ptr;
        bool succ = false;

        // Keep generating until successful or timeout
        while (!succ) {
            if (time_limiter.has_timed_out()) break;
            if (ALNS) chooseDestroyHeuristicbyALNS();

            // Generate neighbor based on selected destroy heuristic
            switch (destroy_strategy) {
                case RANDOMWALK:
                    succ = generateNeighborByRandomWalk(neighbor, idx);
                    neighbor.selected_neighbor = 0;
                    break;
                case INTERSECTION:
                    succ = generateNeighborByIntersection(neighbor);
                    neighbor.selected_neighbor = 1;
                    break;
                case RANDOMAGENTS: {
                    // Randomly select agents without replacement
                    auto s = std::set<int>();
                    while (s.size() < neighbor_size) s.insert(rand() % agents.size());
                    for (auto i: s) neighbor.agents.push_back(i);
                    succ = true;
                    neighbor.selected_neighbor = 2;
                    break;
                }
                default:
                    cerr << "Wrong neighbor generation strategy" << endl;
                    exit(-1);
            }
        }
        neighbors[idx] = neighbor_ptr;
        return neighbor;
    }

    /**
     * @brief Select destroy heuristic using roulette wheel selection (ALNS)
     */
    void chooseDestroyHeuristicbyALNS() {
        int selected_neighbor = rouletteWheel();
        switch (selected_neighbor) {
            case 0: destroy_strategy = RANDOMWALK; break;
            case 1: destroy_strategy = INTERSECTION; break;
            case 2: destroy_strategy = RANDOMAGENTS; break;
            default: cerr << "ERROR" << endl; exit(-1);
        }
    }

    /**
     * @brief Perform roulette wheel selection based on destroy heuristic weights
     * @return Index of selected heuristic
     */
    int rouletteWheel() {
        double sum = 0;
        for (const auto& h : destroy_weights) sum += h;
        double r = (double) rand() / RAND_MAX;
        double threshold = destroy_weights[0];
        int selected_neighbor = 0;
        while (threshold < r * sum) {
            selected_neighbor++;
            threshold += destroy_weights[selected_neighbor];
        }
        return selected_neighbor;
    }

    /**
     * @brief Generate neighbor by random walk from most delayed agent
     * @param neighbor Neighbor to populate with selected agents
     * @param idx Thread index for tabu list
     * @return True if neighbor generation succeeded
     */
    bool generateNeighborByRandomWalk(Neighbor& neighbor, int idx) {
        // If neighborhood size >= total agents, select all
        if (neighbor_size >= (int)agents.size()) {
            neighbor.agents.resize(agents.size());
            for (int i = 0; i < (int)agents.size(); i++) neighbor.agents[i] = i;
            return true;
        }

        // Find most delayed agent not in tabu list
        int a = findMostDelayedAgent(idx);
        if (a < 0) return false;
        
        set<int> neighbors_set;
        neighbors_set.insert(a);
        // Perform random walk to find connected agents
        randomWalk(a, 0, neighbors_set, neighbor_size);

        // Extend neighborhood if needed by trying different starting times
        int count = 0;
        while (neighbors_set.size() < neighbor_size && count < 10) {
            int t = rand() % agents[a].path.size();
            randomWalk(a, t, neighbors_set, neighbor_size);
            count++;
            // Randomly select new starting agent from current set
            int idx = rand() % neighbors_set.size();
            int i = 0;
            for (auto n : neighbors_set) {
                if (i == idx) {
                    a = n;  // Note: this should be a = n, not a = i
                    break;
                }
                i++;
            }
        }

        if (neighbors_set.size() < 2) return false;

        neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
        return true;
    }

    /**
     * @brief Generate neighbor by selecting agents at intersection locations
     * @param neighbor Neighbor to populate with selected agents
     * @return True if neighbor generation succeeded
     */
    bool generateNeighborByIntersection(Neighbor& neighbor) {
        set<int> neighbors_set;
        // Start from random intersection
        auto pt = intersections.begin();
        std::advance(pt, rand() % intersections.size());
        int location = *pt;
        // Get agents occupying this location
        path_table.get_agents(neighbors_set, neighbor_size, location);

        // Expand search if needed
        if (neighbors_set.size() < neighbor_size) {
            set<int> closed;
            closed.insert(location);
            std::queue<int> open;
            open.push(location);
            while (!open.empty() && (int) neighbors_set.size() < neighbor_size) {
                int curr = open.front();
                open.pop();
                // BFS from current location
                for (auto next : instance.getNeighbors(curr)) {
                    if (closed.count(next) > 0) continue;
                    open.push(next);
                    closed.insert(next);
                    // Check intersections for more agents
                    if (instance.getDegree(next) >= 3) {
                        path_table.get_agents(neighbors_set, neighbor_size, next);
                        if ((int) neighbors_set.size() == neighbor_size) break;
                    }
                }
            }
        }

        neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
        // Trim to exact size if too many
        if (neighbor.agents.size() > neighbor_size) {
            std::shuffle(neighbor.agents.begin(), neighbor.agents.end(), MT);
            neighbor.agents.resize(neighbor_size);
        }
        return true;
    }

    /**
     * @brief Find the most delayed agent for the given thread
     * @param idx Thread index for tabu list partitioning
     * @return Agent ID of most delayed agent, or -1 if none found
     */
    int findMostDelayedAgent(int idx) {
        int a = -1;
        float max_delays = -1;
        auto& tabu_list = tabu_list_list[idx];
        // Check agents assigned to this thread
        for (int i = 0; i < agents.size(); i++) {
            if (i % num_threads != idx) continue;
            if (tabu_list.find(i) != tabu_list.end()) continue;
            if ((*agent_infos)[i].disabled) {
                tabu_list.insert(i);
                continue;
            }
            float delays = agents[i].getNumOfDelays();
            if (max_delays < delays) {
                a = i;
                max_delays = delays;
            }
        }
        // Reset tabu list if all agents checked
        if (max_delays == 0) {
            tabu_list.clear();
            return -1;
        }
        tabu_list.insert(a);
        if (tabu_list.size() == (agents.size() / num_threads)) tabu_list.clear();
        return a;
    }

    /**
     * @brief Get valid successor states from current position and orientation
     * @param pos Current position
     * @param orient Current orientation
     * @return List of valid (position, orientation) successor pairs
     */
    std::list<std::pair<int,int>> getSuccessors(int pos, int orient) {
        std::list<std::pair<int,int>> successors;
        int& cols = instance.num_of_cols;
        int& rows = instance.num_of_rows;
        auto& map = instance.my_map;
        int x = pos % cols;
        int y = pos / cols;

        int next_pos, next_orient = orient;
        // Forward movement based on orientation
        if (orient == 0 && x + 1 < cols) {
            next_pos = pos + 1;
            if (map[next_pos] == 0) successors.emplace_back(next_pos, next_orient);
        } else if (orient == 1 && y + 1 < rows) {
            next_pos = pos + cols;
            if (map[next_pos] == 0) successors.emplace_back(next_pos, next_orient);
        } else if (orient == 2 && x - 1 >= 0) {
            next_pos = pos - 1;
            if (map[next_pos] == 0) successors.emplace_back(next_pos, next_orient);
        } else if (orient == 3 && y - 1 >= 0) {
            next_pos = pos - cols;
            if (map[next_pos] == 0) successors.emplace_back(next_pos, next_orient);
        }

        next_pos = pos;
        next_orient = (orient + 1 + n_orients) % n_orients;
        successors.emplace_back(next_pos, next_orient);
        next_orient = (orient - 1 + n_orients) % n_orients;
        successors.emplace_back(next_pos, next_orient);
        successors.emplace_back(pos, orient);
        
        return successors;
    }

    void randomWalk(int agent_id, int start_timestep, set<int>& conflicting_agents, int neighbor_size) {
        auto& path = agents[agent_id].path;
        int loc = path[start_timestep].location;
        int orient = path[start_timestep].orientation;
        auto& agent = agents[agent_id];

        float partial_path_cost = agent.getEstimatedPathLength(path, agent.getGoalLocation(), HT, false, start_timestep);
        for (int t = start_timestep; t < path.size(); ++t) {
            auto successors = getSuccessors(loc, orient);
            while (!successors.empty()) {
                int step = rand() % successors.size();
                auto iter = successors.begin();
                advance(iter, step);

                int next_loc = iter->first;
                int next_orient = iter->second;
                
                float action_cost = agent.get_action_cost(loc, orient, next_loc, next_orient, HT);
                float next_h_val = HT->get(next_loc, next_orient, instance.goal_locations[agent_id]);
                if (partial_path_cost + action_cost + next_h_val < path.path_cost) {
                    path_table.findConflictingAgents(agent_id, conflicting_agents, loc, next_loc, t + 1);
                    loc = next_loc;
                    orient = next_orient;
                    partial_path_cost += action_cost;
                    break;
                }
                successors.erase(iter);
            }
            if (successors.empty() || conflicting_agents.size() >= neighbor_size) break;
        }
    }
};

// ================================================================================================
// GLOBAL MANAGER CLASS
// ================================================================================================

/**
 * @class GlobalManager
 * @brief Coordinates LNS optimization across multiple threads
 */
class GlobalManager {
public:
    std::shared_ptr<NeighborGenerator> neighbor_generator;        ///< Single neighbor generator for sync mode
    std::vector<std::shared_ptr<NeighborGenerator>> neighbor_generators; ///< Per-thread generators for async mode
    std::vector<std::shared_ptr<LocalOptimizer>> local_optimizers; ///< Per-thread local optimizers
    float initial_sum_of_costs;                                   ///< Initial solution cost
    float sum_of_costs;                                           ///< Current solution cost
    int num_of_failures;                                          ///< Number of failed optimization attempts
    double average_group_size;                                    ///< Average neighborhood size
    float sum_of_distances;                                        ///< Sum of agent distances
    int window_size_for_CT;                                       ///< Window size for constraint table
    int window_size_for_CAT;                                      ///< Window size for conflict avoidance table
    int path_window;                                              ///< Planning horizon window
    list<IterStats> iteration_stats;                              ///< Statistics for each iteration
    string init_algo_name;                                        ///< Initial solution algorithm name
    string replan_algo_name;                                      ///< Replanning algorithm name
    Instance& instance;                                           ///< Problem instance reference
    PathT path_table;                                             ///< Shared spatiotemporal occupancy table
    std::vector<Agent> agents;                                    ///< Agent data with current paths
    std::shared_ptr<HeuristicTable> HT;                          ///< Heuristic distance table
    std::shared_ptr<vector<float>> map_weights;                  ///< Edge weights for planning
    int num_threads;                                              ///< Number of parallel threads
    std::shared_ptr<std::vector<LaCAM2::AgentInfo>> agent_infos; ///< Agent information
    std::vector<std::vector<Neighbor>> updating_queues;          ///< Queues for async updates
    std::vector<omp_lock_t> updating_queue_locks;                ///< Locks for thread safety
    bool has_disabled_agents;                                     ///< Whether some agents are disabled
    bool async;                                                   ///< Whether to use asynchronous mode

    /**
     * @brief Constructor for GlobalManager
     * @param async Whether to use asynchronous parallel execution
     * @param instance Problem instance reference
     * @param HT Heuristic table for distance estimation
     * @param map_weights Edge weights for the map
     * @param agent_infos Agent information from LaCAM2
     * @param neighbor_size Target size of neighborhoods to generate
     * @param destroy_strategy Heuristic for selecting agents to replan
     * @param ALNS Whether to use Adaptive Large Neighborhood Search
     * @param decay_factor Weight decay factor for ALNS adaptation
     * @param reaction_factor Reaction factor for ALNS adaptation
     * @param init_algo_name Algorithm for initial solution generation
     * @param replan_algo_name Algorithm for replanning
     * @param sipp Whether to use SIPP (Safe Interval Path Planning)
     * @param window_size_for_CT Window size for constraint table
     * @param window_size_for_CAT Window size for conflict avoidance table
     * @param path_window Planning horizon length
     * @param execution_window Execution window size
     * @param has_disabled_agents Whether some agents are inactive
     */
    GlobalManager(bool async, Instance& instance, std::shared_ptr<HeuristicTable> HT, 
                 std::shared_ptr<vector<float>> map_weights, std::shared_ptr<std::vector<LaCAM2::AgentInfo>> agent_infos,
                 int neighbor_size, destroy_heuristic destroy_strategy, bool ALNS, double decay_factor, 
                 double reaction_factor, string init_algo_name, string replan_algo_name, bool sipp,
                 int window_size_for_CT, int window_size_for_CAT, int path_window, int execution_window,
                 bool has_disabled_agents): 
        async(async), instance(instance), path_table(instance.map_size, path_window), HT(HT), 
        map_weights(map_weights), init_algo_name(init_algo_name), replan_algo_name(replan_algo_name),
        window_size_for_CT(window_size_for_CT), window_size_for_CAT(window_size_for_CAT), 
        path_window(path_window), agent_infos(agent_infos), 
        has_disabled_agents(has_disabled_agents), initial_sum_of_costs(MAX_COST), sum_of_costs(MAX_COST),
        num_of_failures(0), average_group_size(0), sum_of_distances(0) {

        // Determine number of threads from environment or OpenMP
        char* num_threads_env = std::getenv("LNS_NUM_THREADS");
        num_threads = (num_threads_env != nullptr) ? std::atoi(num_threads_env) : omp_get_max_threads();
        omp_set_num_threads(num_threads);
        cout << "LNS use " << num_threads << " threads" << endl;

        // Initialize agents
        for (int i = 0; i < instance.num_of_agents; ++i) {
            agents.emplace_back(i, instance, HT, agent_infos);
        }

        // Create local optimizers for each thread
        for (auto i = 0; i < num_threads; ++i) {
            auto local_optimizer = std::make_shared<LocalOptimizer>(instance, agents, HT, map_weights, agent_infos,
                replan_algo_name, sipp, window_size_for_CT, window_size_for_CAT, path_window, 
                execution_window, has_disabled_agents, i * 2023 + 1);
            local_optimizers.push_back(local_optimizer);
        }

        // Create neighbor generators (sync vs async mode)
        if (!async) {
            neighbor_generator = std::make_shared<NeighborGenerator>(instance, HT, path_table, agents, agent_infos,
                neighbor_size, destroy_strategy, ALNS, decay_factor, reaction_factor, num_threads, 0);
        } else {
            for (auto i = 0; i < num_threads; ++i) {
                auto neighbor_gen = std::make_shared<NeighborGenerator>(instance, HT, 
                    local_optimizers[i]->path_table, local_optimizers[i]->agents, agent_infos, neighbor_size, 
                    destroy_strategy, ALNS, decay_factor, reaction_factor, num_threads, i * 2023 + 1314);
                neighbor_generators.push_back(neighbor_gen);
            }
            // Initialize async communication structures
            updating_queues.resize(num_threads);
            updating_queue_locks.resize(num_threads);
            for (auto& lock: updating_queue_locks) omp_init_lock(&lock);
        }
    }

    /**
     * @brief Destructor - clean up OpenMP locks
     */
    ~GlobalManager() {
        if (async) {
            for (auto& lock: updating_queue_locks) omp_destroy_lock(&lock);
        }
    }

    /**
     * @brief Reset the global manager state for new search iteration
     */
    void reset() {
        initial_sum_of_costs = MAX_COST;
        sum_of_costs = MAX_COST;
        num_of_failures = 0;
        average_group_size = 0;  
        sum_of_distances = 0;
        iteration_stats.clear();
        path_table.reset();
        for (auto& agent: agents) agent.reset();

        if (!async) {
            neighbor_generator->reset();
        } else {
            #pragma omp parallel for
            for (auto& neighbor_generator: neighbor_generators) neighbor_generator->reset();
            for (int i = 0; i < num_threads; ++i) {
                omp_set_lock(&(updating_queue_locks[i]));
                updating_queues[i].clear();
                omp_unset_lock(&(updating_queue_locks[i]));
            }
        }

        #pragma omp parallel for
        for (auto& local_optimizer: local_optimizers) local_optimizer->reset();
    }

    /**
     * @brief Update solution with a neighbor, optionally rechecking validity
     * @param neighbor The neighbor solution to apply
     * @param recheck Whether to validate the solution before applying
     */
    void update(Neighbor& neighbor, bool recheck) {
        if (!neighbor.succ) return;

        if (recheck) {
            g_timer.record_timepoint("manager_update_s");
            g_timer.record_timepoint("recheck_s");

            // Validate that new paths don't conflict with existing reservations
            bool valid = true;
            for (auto& aid: neighbor.agents) {
                auto& path = neighbor.m_paths[aid];
                for (int i = 0; i < path.size() - 1; ++i) {
                    int from = path[i].location;
                    int to = path[i+1].location;
                    int to_time = i + 1;
                    if (path_table.isBlocked(from, to, to_time, neighbor.agents)) {
                        valid = false;
                        break;
                    }
                }
                if (!valid) break;
            }

            if (!valid) {
                neighbor.succ = false;
                return;
            }

            // Check if solution actually improves cost
            float old_sum_of_costs = 0;
            for (auto& aid: neighbor.agents) old_sum_of_costs += agents[aid].path.path_cost;

            if (old_sum_of_costs <= neighbor.sum_of_costs) {
                neighbor.succ = false;
                return;
            }

            neighbor.old_sum_of_costs = old_sum_of_costs;
            for (auto& id: neighbor.agents) neighbor.m_old_paths[id] = agents[id].path;
            g_timer.record_duration("recheck_s", "recheck");
        } else {
            g_timer.record_timepoint("init_manager_update_s");
        }

        update(neighbor);

        if (recheck) {
            g_timer.record_duration("manager_update_s", "manager_update");
        } else {
            g_timer.record_duration("init_manager_update_s", "init_manager_update");
        }
    }

    /**
     * @brief Apply neighbor solution to global state
     * Updates agent paths, path table, and solution cost
     * @param neighbor The neighbor solution to apply
     */
    void update(Neighbor& neighbor) {
        g_timer.record_timepoint("path_table_delete_s");
        // Remove old paths from conflict table
        for (auto& aid: neighbor.agents) {
            path_table.unregisterPath(aid, neighbor.m_old_paths[aid]);
        }
        g_timer.record_duration("path_table_delete_s", "path_table_delete");

        g_timer.record_timepoint("path_table_insert_s");
        // Update agent paths and register new paths
        for (auto& aid: neighbor.agents) {
            agents[aid].path = neighbor.m_paths[aid];
            path_table.registerPath(aid, neighbor.m_paths[aid]);
        }
        g_timer.record_duration("path_table_insert_s", "path_table_insert");

        // Update global solution cost
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
    }

    /**
     * @brief Execute the main LNS search algorithm
     * @param time_limiter Time limit for the search
     * @return True if a valid solution was found
     */
    bool run(TimeLimiter& time_limiter) {
        return async ? _run_async(time_limiter) : _run(time_limiter);
    }

    /**
     * @brief Asynchronous parallel LNS execution
     * Each thread runs its own neighbor generation and optimization loop
     * @param time_limiter Time limit for the search
     * @return True if execution completed successfully
     */
    bool _run_async(TimeLimiter& time_limiter) {
        // Initialize statistics and solution
        initial_sum_of_costs = 0;
        sum_of_costs = 0;
        num_of_failures = 0;
        average_group_size = 0;
        iteration_stats.clear();

        g_timer.record_timepoint("lns_init_sol_s");
        // Calculate sum of straight-line distances for reference
        sum_of_distances = 0;
        for (const auto& agent : agents) {
            sum_of_distances += HT->get(instance.start_locations[agent.id], instance.goal_locations[agent.id]);
        }

        // Get initial solution from LaCAM2
        Neighbor init_neighbor;
        init_neighbor.agents.resize(agents.size());
        for (int i = 0; i < agents.size(); ++i) init_neighbor.agents[i] = i;
        getInitialSolution(init_neighbor);
        update(init_neighbor, false);

        // Update all local optimizers with initial solution
        #pragma omp parallel for
        for (int i = 0; i < num_threads; ++i) {
            local_optimizers[i]->update(init_neighbor);
        }

        bool runtime = g_timer.record_duration("lns_init_sol_s", "lns_init_sol");
        double elapse = time_limiter.get_elapsed_time();
        iteration_stats.emplace_back(agents.size(), initial_sum_of_costs, runtime, init_algo_name);

    
        if (!init_neighbor.succ) {
            cerr << "Failed to get initial solution." << endl;
            exit(-1);
        }

        g_timer.record_timepoint("lns_opt_s");

        // Main asynchronous optimization loop
        #pragma omp parallel for
        for (int i = 0; i < num_threads; ++i) {
            while (!time_limiter.has_timed_out()) {
                // Generate neighbor for this thread
                Neighbor neighbor = neighbor_generators[i]->generate(time_limiter, i);
                if (time_limiter.has_timed_out()) break;

                // Optimize the neighbor locally
                local_optimizers[i]->optimize(neighbor, time_limiter);
                if (time_limiter.has_timed_out()) break;

                // Critical section for global updates
                #pragma omp critical
                {
                    if (!time_limiter.has_timed_out()) {
                        neighbor_generators[i]->update(neighbor);
                        update(neighbor, true);

                        if (!neighbor.succ) {
                            ++num_of_failures;
                        } else {
                            // Broadcast successful update to all threads
                            for (int j = 0; j < num_threads; ++j) {
                                updating_queues[j].push_back(neighbor);
                            }
                        }

                        // Process pending updates for this thread
                        local_optimizers[i]->updating_queue = updating_queues[i];
                        updating_queues[i].clear();

                        elapse = time_limiter.get_elapsed_time();
                        
                        iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, elapse, replan_algo_name);
                    }
                }

                if (time_limiter.has_timed_out()) break;
                // Apply pending updates to local optimizer
                for (auto& _neighbor: local_optimizers[i]->updating_queue) {
                    if (time_limiter.has_timed_out()) break;
                    local_optimizers[i]->update(_neighbor);
                }
                local_optimizers[i]->updating_queue.clear();
            }
        }
        g_timer.record_duration("lns_opt_s", "lns_opt");

        // Calculate average neighborhood size
        average_group_size = -iteration_stats.front().num_of_agents;
        for (const auto& data : iteration_stats) average_group_size += data.num_of_agents;
        if (average_group_size > 0) average_group_size /= (double)(iteration_stats.size() - 1);

        return true;
    }

    /**
     * @brief Synchronous parallel LNS execution
     * Centralized neighbor generation with parallel optimization
     * @param time_limiter Time limit for the search
     * @return True if execution completed successfully
     */
    bool _run(TimeLimiter& time_limiter) {
        // Initialize statistics and solution
        initial_sum_of_costs = 0;
        sum_of_costs = 0;
        num_of_failures = 0;
        average_group_size = 0;
        iteration_stats.clear();

        g_timer.record_timepoint("lns_init_sol_s");
        // Calculate sum of straight-line distances
        sum_of_distances = 0;
        for (const auto& agent : agents) {
            sum_of_distances += HT->get(instance.start_locations[agent.id], instance.goal_locations[agent.id]);
        }

        // Get initial solution
        Neighbor init_neighbor;
        init_neighbor.agents.resize(agents.size());
        for (int i = 0; i < agents.size(); ++i) init_neighbor.agents[i] = i;
        getInitialSolution(init_neighbor);
        update(init_neighbor, false);

        // Update all local optimizers
        #pragma omp parallel for
        for (int i = 0; i < num_threads; ++i) {
            local_optimizers[i]->update(init_neighbor);
        }

        bool runtime = g_timer.record_duration("lns_init_sol_s", "lns_init_sol");
        double elapse = time_limiter.get_elapsed_time();
        iteration_stats.emplace_back(agents.size(), initial_sum_of_costs, runtime, init_algo_name);


        if (!init_neighbor.succ) {
            cerr << "Failed to get initial solution." << endl;
            exit(-1);
        }

        g_timer.record_timepoint("lns_opt_s");
        // Main synchronous optimization loop
        while (!time_limiter.has_timed_out()) {
            // Generate neighbors in parallel
            g_timer.record_timepoint("neighbor_generate_s");
            neighbor_generator->generate_parallel(time_limiter);
            g_timer.record_duration("neighbor_generate_s", "neighbor_generate");

            if (time_limiter.has_timed_out()) break;

            // Optimize neighbors in parallel
            g_timer.record_timepoint("loc_opt_s");
            #pragma omp parallel for
            for (auto i = 0; i < neighbor_generator->neighbors.size(); ++i) {
                auto& neighbor_ptr = neighbor_generator->neighbors[i];
                auto& neighbor = *neighbor_ptr;
                local_optimizers[i]->optimize(neighbor, time_limiter);
            }
            g_timer.record_duration("loc_opt_s", "loc_opt");

            if (time_limiter.has_timed_out()) break;

            // Update ALNS weights
            g_timer.record_timepoint("neighbor_update_s");
            for (auto& neighbor_ptr: neighbor_generator->neighbors) {
                if (time_limiter.has_timed_out()) break;
                auto& neighbor = *neighbor_ptr;
                neighbor_generator->update(neighbor);
            }
            g_timer.record_duration("neighbor_update_s", "neighbor_update");

            if (time_limiter.has_timed_out()) break;

            // Apply successful updates globally
            for (auto& neighbor_ptr: neighbor_generator->neighbors) {
                if (time_limiter.has_timed_out()) break;
                auto& neighbor = *neighbor_ptr;
                update(neighbor, true);

                // Update all local optimizers
                #pragma omp parallel for
                for (int i = 0; i < num_threads; ++i) {
                    local_optimizers[i]->update(neighbor);
                }

                if (!neighbor.succ) ++num_of_failures;

                elapse = time_limiter.get_elapsed_time();
                iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, elapse, replan_algo_name);
            }
        }
        g_timer.record_duration("lns_opt_s", "lns_opt");

        // Calculate average neighborhood size
        average_group_size = -iteration_stats.front().num_of_agents;
        for (const auto& data : iteration_stats) average_group_size += data.num_of_agents;
        if (average_group_size > 0) average_group_size /= (double)(iteration_stats.size() - 1);

        return true;
    }

    /**
     * @brief Initialize solution with precomputed paths from LaCAM2
     * Validates path lengths and computes initial solution cost
     * @param neighbor Neighbor structure to populate with initial solution
     */
    void getInitialSolution(Neighbor& neighbor) {
        neighbor.old_sum_of_costs = 0;
        neighbor.sum_of_costs = 0;
        for (int i = 0; i < agents.size(); ++i) {
            // Validate path constraints for non-goal reaching agents
            if (agents[i].path.back().location != instance.goal_locations[i] && 
                agents[i].path.size() <= window_size_for_CT) {
                cerr << "A precomputed agent " << i << "'s path " << agents[i].path.size()
                     << " should be longer than window size for CT " << window_size_for_CT
                     << " unless it arrives at its goal:" << agents[i].path.back().location
                     << " vs " << instance.goal_locations[i] << endl;
                exit(-1);
            }

            if (agents[i].path.back().location != instance.goal_locations[i] && 
                agents[i].path.size() != path_window + 1) {
                cerr << "we require agent either arrives at its goal earlier or has a planned path of length path_window. " 
                     << agents[i].path.size() << " vs " << path_window << endl;
                exit(-1);
            }

            neighbor.sum_of_costs += agents[i].path.path_cost;
            neighbor.m_paths[i] = agents[i].path;
            neighbor.m_old_paths[i] = Path();
        }

        neighbor.succ = true;
        initial_sum_of_costs = neighbor.sum_of_costs;
    }

    /**
     * @brief Get descriptive name for the solver configuration
     * @return String describing the LNS solver with init and replan algorithms
     */
    string getSolverName() const { return "LNS(" + init_algo_name + ";" + replan_algo_name + ")"; }
};

}
}
