#pragma once
#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <map>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <utility>
#include <queue>
#include <random>
#include <memory>
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include "util/HeuristicTable.h"
#include "util/ActionModel.h"
#include "LaCAM2/lacam2_core.hpp"
#include "SharedEnv.h"
#include "States.h"
#include "config.hpp"
#include <nlohmann/json.hpp>

using boost::heap::pairing_heap;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;
using std::list;
using std::set;
using std::map;
using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;

// Boundary values for algorithm constraints
#define MAX_TIMESTEP  INT_MAX
#define MAX_COST FLT_MAX / 2
#define MAX_NODES INT_MAX / 2
#define NO_AGENT -1

namespace LNS {

// Represents a discrete waypoint in an agent's trajectory
struct PathEntry {
    int location = -1;
    explicit PathEntry(int loc = -1) { location = loc; }
};

typedef vector<PathEntry> Path;

// Statistics collector for iteration metrics and performance
struct IterStats {
    int sum_of_costs;
    double runtime;
    int num_of_agents;
    string algorithm;
    int sum_of_costs_lowerbound;
    int num_of_colliding_pairs;
    IterStats(int num_of_agents, int sum_of_costs, double runtime, const string& algorithm,
                   int sum_of_costs_lowerbound = 0, int num_of_colliding_pairs = 0) :
            num_of_agents(num_of_agents), sum_of_costs(sum_of_costs), runtime(runtime),
            sum_of_costs_lowerbound(sum_of_costs_lowerbound), algorithm(algorithm),
            num_of_colliding_pairs(num_of_colliding_pairs) {}
};

struct PIBTPPS_option {
    int windowSize;
    bool winPIBTSoft;
    int timestepLimit;
};

namespace Parallel {
    struct PathNode;
    struct PathEntry;
    struct Path;
    class GlobalManager;
}

// Manages spatiotemporal occupancy and conflict detection across agent trajectories
class PathT {
public:
    // Core state tracking
    int max_time_horizon = 0;  // Latest timestep with active path
    int planning_horizon = -1;        // Planning horizon constraint (-1 for infinite)
    vector<vector<int>> space_time_grid;  // Spatiotemporal grid: [location][time] -> agent_id
    vector<int> target_timestamps;               // Goal achievement timestamps per location
    
    // Constructor initializes spatiotemporal data structures
    explicit PathT(int map_size = 0, int window_size=-1) 
        : space_time_grid(map_size), target_timestamps(map_size, MAX_TIMESTEP), planning_horizon(window_size) {}
    
    // Path management operations
    void registerPath(int agent_id, const Path& path);
    void unregisterPath(int agent_id, const Path& path);
    void registerPath(int agent_id, const Parallel::Path& path);
    void unregisterPath(int agent_id, const Parallel::Path& path);
    bool isBlocked(int from, int to, int to_time) const;
    bool isBlocked(int from, int to, int to_time, std::vector<int>& ignored_agents) const;
    void get_agents(set<int>& conflicting_agents, int location) const;
    void get_agents(set<int>& conflicting_agents, int neighbor_size, int location) const;
    void findConflictingAgents(int agent_id, set<int>& conflicting_agents, int from, int to, int to_time) const;
    int getHoldingTime(int location, int earliest_timestep) const;
    
    // Reset all tracked data
    void reset() { 
        auto map_size = space_time_grid.size(); 
        space_time_grid.clear(); 
        space_time_grid.resize(map_size); 
        target_timestamps.assign(map_size, MAX_TIMESTEP); 
        max_time_horizon = 0; 
    }
};

// Multi-agent occupancy tracker with weighted collision scoring
// Supports soft constraint evaluation for optimization
class PathTableWC {
public:
    // Temporal tracking state
    int max_time_horizon = 0;
    vector<vector<list<int>>> conflict_grid;  // Multi-occupancy grid: [loc][time] -> agents
    vector<int> target_timestamps;                     // Per-location goal timestamps
    
    // Initialize collision tracking structures
    explicit PathTableWC(int map_size = 0, int num_of_agents = 0) 
        : conflict_grid(map_size), target_timestamps(map_size, MAX_TIMESTEP), path_cache(num_of_agents, nullptr) {}
    
    // Path modification and query operations
    void registerPath(int agent_id, const Path& path);
    void registerPath(int agent_id);
    void unregisterPath(int agent_id);
    int countFutureConflicts(int location, int time) const;
    int countConflicts(int from, int to, int to_time) const;
    bool hasCollisions(int from, int to, int to_time) const;
    bool hasEdgeCollisions(int from, int to, int to_time) const;
    int getLastCollisionTimestep(int location) const;
    void clear();
    
    // Path retrieval
    const Path* getPath(int agent_id) const { return path_cache[agent_id]; }
    
    void reset() { 
        auto map_size = conflict_grid.size(); 
        conflict_grid.clear(); 
        conflict_grid.resize(map_size); 
        target_timestamps.assign(map_size, MAX_TIMESTEP); 
        max_time_horizon = 0; 
    }
    
private:
    vector<const Path*> path_cache;  // Cached pointers to agent paths
};

// Manages both hard constraints (blocking) and soft constraints (penalties)
// for path planning
class ConstraintTable {
public:
    // Path length boundaries
    int min_path_length = 0;
    int max_path_length = MAX_TIMESTEP;
    size_t num_columns;
    size_t map_size;
    const PathT* hard_constraints_table;       // Hard blocking constraints
    const PathTableWC* soft_constraints_table;     // Soft penalty constraints
    int hard_constraint_window = MAX_TIMESTEP;
    int soft_constraint_window = MAX_TIMESTEP;
    int path_window = MAX_TIMESTEP;

    // Constructors and destructor
    ConstraintTable(size_t num_columns, size_t map_size, 
            const PathT* hard_table = nullptr,
            const PathTableWC* soft_table = nullptr, 
            int hard_window = MAX_TIMESTEP, 
            int soft_window = MAX_TIMESTEP,
            int path_win = MAX_TIMESTEP)
        : num_columns(num_columns), map_size(map_size), 
          hard_constraints_table(hard_table), soft_constraints_table(soft_table),
          hard_constraint_window(hard_window), soft_constraint_window(soft_window), 
          path_window(path_win) {}
    ConstraintTable(const ConstraintTable& other) { copy(other); }
    ~ConstraintTable() = default;

    // Constraint manipulation methods
    void copy(const ConstraintTable& other);
    void addHardConstraint(const Path& path);
    void addHardConstraint(size_t loc, int t_min, int t_max);
    void addHardConstraint(size_t from, size_t to, int t_min, int t_max);
    void addSoftConstraints(int agent, const vector<Path*>& paths);
    void addSoftConstraint(const Path& path);
    int getHoldingTime(int location, int earliest_timestep) const;
    int getMaxTimestep() const;
    int getLastCollisionTimestep(int location) const;
    bool isBlocked(size_t loc, int t) const;
    bool isBlocked(size_t curr_loc, size_t next_loc, int next_t) const;
    int countStepConflicts(size_t curr_id, size_t next_id, int next_timestep) const;
    bool hasStepConflict(size_t curr_id, size_t next_id, int next_timestep) const;
    bool hasEdgeConflict(size_t curr_id, size_t next_id, int next_timestep) const;
    int countFutureConflicts(int loc, int t) const;
    
    // Utility methods
    void init(const ConstraintTable& other) { copy(other); }
    void clear() { blocking_constraints.clear(); landmarks.clear(); penalty_grid.clear(); }

protected:
    friend class ReservationTable;
    
    // Hard constraint storage
    typedef unordered_map<size_t, list<pair<int, int>>> BlockingMap;
    BlockingMap blocking_constraints;  // [location/edge] -> list of (start_time, end_time)
    int constraint_end_time = 0;
    
    // Soft constraint storage
    typedef vector<vector<bool>> PenaltyGrid;
    PenaltyGrid penalty_grid;  // [location][time] -> has collision
    int penalty_end_time = 0;
    vector<int> penalty_goal_times;    // Goal times for soft constraints
    map<int, size_t> landmarks;      // Required waypoints: time -> location

    // Encode edge as unique index for constraint storage
    inline size_t getEdgeIndex(size_t from, size_t to) const { 
        return (1 + from) * map_size + to; 
    }
};

typedef tuple<int, int, bool> Interval;

class ReservationTable {
public:
    const ConstraintTable& constraint_table;

    ReservationTable(const ConstraintTable& constraint_table, int goal_location) :
        constraint_table(constraint_table), goal_location(goal_location), sit(constraint_table.map_size) {}

    list<tuple<int, int, int, bool, bool>> get_safe_intervals(int from, int to, int lower_bound, int upper_bound);
    Interval get_first_safe_interval(size_t location);
    bool find_safe_interval(Interval& interval, size_t location, int t_min);

private:
    int goal_location;
    typedef vector<list<Interval>> SIT;
    SIT sit;
    
    void insert2SIT(int location, int t_min, int t_max);
    void insertSoftConstraint2SIT(int location, int t_min, int t_max);
    void updateSIT(int location);
    int get_earliest_arrival_time(int from, int to, int lower_bound, int upper_bound) const;
    int get_earliest_no_collision_arrival_time(int from, int to, const Interval& interval, int lower_bound, int upper_bound) const;
};

class Instance {
public:
    int num_of_cols;
    int num_of_rows;
    int map_size;

    Instance() = default;
    Instance(const string& map_fname, const string& agent_fname, 
        int num_of_agents = 0, int num_of_rows = 0, int num_of_cols = 0, int num_of_obstacles = 0, int warehouse_width = 0);
    Instance(const SharedEnvironment& env);

    void set_starts_and_goals(const SharedEnvironment& env);
    void set_starts_and_goals(const std::vector<::State>& starts, const std::vector<::State>& goals);
    string getMapFile() const { return map_fname; }
    vector<int> getStarts() const { return start_locations; }
    vector<int> getGoals() const { return goal_locations; }

    inline bool isObstacle(int loc) const { return my_map[loc]; }
    inline bool validMove(int curr, int next) const {
        if (next < 0 || next >= map_size) return false;
        if (my_map[next]) return false;
        return getManhattanDistance(curr, next) < 2;
    }
    list<int> getNeighbors(int curr) const;

    inline int linearizeCoordinate(int row, int col) const { return (this->num_of_cols * row + col); }
    inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
    inline int getColCoordinate(int id) const { return id % this->num_of_cols; }
    inline pair<int, int> getCoordinate(int id) const { return make_pair(id / this->num_of_cols, id % this->num_of_cols); }
    inline int getCols() const { return num_of_cols; }

    inline int getManhattanDistance(int loc1, int loc2) const {
        int loc1_x = getRowCoordinate(loc1);
        int loc1_y = getColCoordinate(loc1);
        int loc2_x = getRowCoordinate(loc2);
        int loc2_y = getColCoordinate(loc2);
        return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
    }

    static inline int getManhattanDistance(const pair<int, int>& loc1, const pair<int, int>& loc2) {
        return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    }

    int getDegree(int loc) const {
        if (loc < 0 || loc >= map_size || my_map[loc]) return 0;
        int degree = 0;
        if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols]) degree++;
        if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols]) degree++;
        if (loc % num_of_cols > 0 && !my_map[loc - 1]) degree++;
        if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1]) degree++;
        return degree;
    }

    int getDefaultNumberOfAgents() const { return num_of_agents; }
    string getInstanceName() const { return agent_fname; }

    vector<bool> my_map;
    string map_fname;
    string agent_fname;
    int num_of_agents;
    vector<int> start_locations;
    vector<int> start_orientations;
    vector<int> goal_locations;
    bool nathan_benchmark = true;

    bool loadMap();
    void printMap() const;
    void saveMap() const;
    bool loadAgents();
    void saveNathan() const;
    void generateConnectedRandomGrid(int rows, int cols, int obstacles);
    void generateRandomAgents(int warehouse_width);
    bool addObstacle(int obstacle);
    bool isConnected(int start, int goal);
    int randomWalk(int loc, int steps) const;

    friend class SingleAgentSolver;
};

class LNSSolver {
public:
    CompetitionActionModelWithRotate action_model;
    std::mt19937* MT;
    int total_feasible_timestep = 0;
    int timestep = 0;

    std::shared_ptr<Parallel::GlobalManager> lns;
    std::shared_ptr<Instance> instance;
    std::shared_ptr<HeuristicTable> HT;
    std::shared_ptr<std::vector<float>> map_weights;
    std::shared_ptr<vector<LaCAM2::AgentInfo>> agent_infos;

    LaCAM2::Executor executor;
    LaCAM2::SlowExecutor slow_executor;
    std::shared_ptr<LaCAM2::LaCAM2Solver> lacam2_solver;

    double max_plan_time = 0;
    std::vector<::Path> planning_paths;
    std::vector<::Path> execution_paths;
    int executed_step = 0;
    bool need_new_execution_paths = false;
    int execution_window;
    int planning_window;
    int num_task_completed = 0;
    int max_task_completed;

    LNSSolver(const std::shared_ptr<HeuristicTable>& HT, SharedEnvironment* env,
        std::shared_ptr<std::vector<float>>& map_weights,
        std::shared_ptr<LaCAM2::LaCAM2Solver>& lacam2_solver, int max_task_completed);

    ~LNSSolver() { delete MT; }

    void initialize(const SharedEnvironment& env);
    void observe(const SharedEnvironment& env);
    void plan(const SharedEnvironment& env, int time_limit_ms = -1);
    void get_step_actions(const SharedEnvironment& env, vector<Action>& actions);
    Action get_action_from_states(const State& state, const State& next_state);
};

} // namespace LNS
