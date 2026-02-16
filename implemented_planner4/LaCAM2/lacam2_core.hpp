// ====================================================================================================
// LaCAM2 Core Header File
// ====================================================================================================
// This header file defines the core components of the LaCAM2 MAPF solver.
// It includes data structures for graphs, configurations, instances, and solver classes.
// ====================================================================================================

#pragma once
#include <array>
#include <chrono>
#include <climits>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <queue>
#include <random>
#include <regex>
#include <set>
#include <stack>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <memory>
#include "States.h"
#include "common.h"
#include "SharedEnv.h"
#include "ActionModel.h"
#include "util/ActionModel.h"
#include "util/HeuristicTable.h"
#include "util/Utils.h"
#include "config.hpp"
#include "nlohmann/json.hpp"

namespace LaCAM2 {

using Time = std::chrono::steady_clock;
using uint = unsigned int;

struct Deadline {
  const Time::time_point t_s;
  const double time_limit_ms;
  Deadline(double _time_limit_ms = 0);
  double elapsed_ms() const;
  double elapsed_ns() const;
};

double elapsed_ms(const Deadline* deadline);
double elapsed_ns(const Deadline* deadline);
bool is_expired(const Deadline* deadline);
float get_random_float(std::mt19937* MT, float from = 0, float to = 1);

void info(const int level, const int verbose);

template <typename Head, typename... Tail>
void info(const int level, const int verbose, Head&& head, Tail&&... tail) {
  if (verbose < level) return;
  std::cout << head;
  info(level, verbose, std::forward<Tail>(tail)...);
}

// ====================================================================================================
// Vertex Struct
// ====================================================================================================
// Represents a node in the graph with unique id, index, and list of neighboring vertices.
// ====================================================================================================

struct Vertex {
  const uint id;
  const uint index;
  std::vector<Vertex*> neighbor;
  Vertex(uint _id, uint _index);
};
using Vertices = std::vector<Vertex*>;

// ====================================================================================================
// Config Struct
// ====================================================================================================
// Represents the configuration of all agents with their locations, orientations, and arrival status.
// ====================================================================================================

struct Config {
  std::vector<Vertex*> locs;
  std::vector<int> orients;
  std::vector<bool> arrivals;

  Config(int N) {
    locs.resize(N, nullptr);
    orients.resize(N, -1);
    arrivals.resize(N, false);
  }

  inline size_t size() const { return locs.size(); }
  
  inline bool all_arrived() const {
    for (int i=0;i<arrivals.size();++i) {
      if (!arrivals[i]) return false;
    }
    return true;
  }

  struct ConfigEqual {
    bool operator()(const Config& C1, const Config& C2) const {
        const auto N = C1.size();
        for (size_t i = 0; i < N; ++i) {
          if (C1.locs[i]->id != C2.locs[i]->id) return false;
          if (C1.orients[i] != C2.orients[i]) return false;
          if (C1.arrivals[i] != C2.arrivals[i]) return false;
        }
        return true;
    }
  };
};

// ====================================================================================================
// Graph Struct
// ====================================================================================================
// Represents the environment graph with vertices, dimensions, and loading from file or environment.
// ====================================================================================================

struct Graph {
  Vertices V;
  Vertices U;
  uint width;
  uint height;
  Graph();
  Graph(const std::string& filename);
  Graph(const SharedEnvironment & env);
  ~Graph();
  uint size() const;
};

bool is_same_config(const Config& C1, const Config& C2);

struct ConfigHasher {
  uint operator()(const Config& C) const;
};

std::ostream& operator<<(std::ostream& os, const Vertex* v);
std::ostream& operator<<(std::ostream& os, const Config& config);

// ====================================================================================================
// AgentInfo Struct
// ====================================================================================================
// Stores information about each agent including goal location, elapsed time, and status.
// ====================================================================================================

struct AgentInfo {
    int goal_location;
    float elapsed;
    float tie_breaker;
    int id;
    int stuck_order;
    bool disabled;
    AgentInfo():id(-1),goal_location(-1),elapsed(-1),tie_breaker(-1), stuck_order(0), disabled(false) {};
};

// ====================================================================================================
// Instance Struct
// ====================================================================================================
// Represents a MAPF instance with graph, start/goal configurations, and agent information.
// ====================================================================================================

struct Instance {
  const Graph & G;
  Config starts;
  Config goals;
  const uint N;
  vector<AgentInfo> & agent_infos;
  int planning_window=-1;
  std::vector<::Path> * precomputed_paths;

  Instance(
    const Graph & G,
    const std::vector<std::pair<uint,int> >& start_indexes,
    const std::vector<std::pair<uint,int> >& goal_indexes,
    std::vector<AgentInfo> & agent_infos,
    int planning_window=-1,
    std::vector<::Path> * precomputed_paths=nullptr
  );
  ~Instance() {}

  void set_starts_and_goals(std::vector<::State> * starts, std::vector<::State> * goals);
  bool is_valid(const int verbose = 0) const;
};

using Solution = std::vector<Config>;
std::ostream& operator<<(std::ostream& os, const Solution& solution);

enum Objective { OBJ_NONE, OBJ_MAKESPAN, OBJ_SUM_OF_LOSS };
std::ostream& operator<<(std::ostream& os, const Objective objective);

struct Agent {
  const uint id;
  Vertex* v_now;
  Vertex* v_next;
  int o_next;
  Agent(uint _id) : id(_id), v_now(nullptr), v_next(nullptr), o_next(-1) {}
};
using Agents = std::vector<Agent*>;

struct LNode {
  std::vector<uint> who;
  std::vector<std::tuple<Vertex*,int > > where;
  const uint depth;
  LNode(LNode* parent, uint i, const std::tuple<Vertex*,int > & t);
  LNode(): who(), where(), depth(0) {}
};

struct HNode {
  static uint HNODE_CNT;
  const Config C;
  HNode* parent;
  std::set<HNode*> neighbor;
  uint d;
  float g;
  float h;
  float f;
  std::vector<float> priorities;
  std::vector<uint> order;
  std::queue<LNode*> search_tree;

  HNode(const Config& _C, const std::shared_ptr<HeuristicTable> & HT, Instance * ins, HNode* _parent, float _g,
        float _h, const uint _d, bool disable_agent_goals);
  ~HNode();
};
using HNodes = std::vector<HNode*>;

// ====================================================================================================
// Executor Class
// ====================================================================================================
// Handles the execution of agent actions, resolving conflicts and updating states.
// ====================================================================================================

class Executor {
public:
    int rows;
    int cols;
    unordered_map<int,int> reservation_table;
    vector<bool> executed;
    const vector<State> * curr_states;
    const vector<State> * planned_next_states;
    vector<State> * next_states;

    Executor(const SharedEnvironment * _env):rows(_env->rows), cols(_env->cols){};
    Executor(int _rows, int _cols): rows(_rows), cols(_cols) {};   

    void execute(const vector<State> * _curr_states, const vector<State> * _planned_next_states, vector<State> * _next_states);
    void execute_agent(int agent_idx, int root_agent_idx);
    int get_neighbor_orientation(int loc1,int loc2);
};

// ====================================================================================================
// SlowExecutor Class
// ====================================================================================================
// A slower executor that handles orientation changes explicitly for agents.
// ====================================================================================================

class SlowExecutor {
public:
    const SharedEnvironment * env;
    const vector<State> * curr_states;
    const vector<State> * planned_next_states;
    vector<State> * next_states;

    SlowExecutor(const SharedEnvironment * _env):env(_env){};

    void execute(const vector<State> * _curr_states, const vector<State> * _planned_next_states, vector<State> * _next_states);
    int get_next_orientation(int curr_orient, int target_orient);
    int get_neighbor_orientation(int loc1,int loc2);
};

// ====================================================================================================
// Planner Struct
// ====================================================================================================
// The core planner class that implements the LaCAM2 algorithm for solving MAPF instances.
// ====================================================================================================

struct Planner {
  Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;
  bool use_swap;
  bool use_orient_in_heuristic;
  bool use_external_executor;
  int MC_idx;
  const Objective objective;
  const float RESTART_RATE;
  const uint N;
  const uint V_size;
  std::shared_ptr<HeuristicTable> HT;
  std::shared_ptr<std::vector<float> > map_weights;
  uint loop_cnt;
  std::vector<std::array<Vertex*, 5> > C_next;
  std::vector<float> tie_breakers;
  Agents A;
  Agents occupied_now;
  Agents occupied_next;
  bool disable_agent_goals;
  Executor executor;

  Planner(Instance* _ins, const std::shared_ptr<HeuristicTable> & HT, const std::shared_ptr<std::vector<float> > & map_weights, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          const Objective _objective = OBJ_NONE,
          const float _restart_rate = 0.001f,
          bool use_swap=false,
          bool use_orient_in_heuristic=false,
          bool disable_agent_goals=true);
  ~Planner();

  Solution solve(std::string& additional_info);
  std::vector<std::tuple<Vertex *,int> > get_successors(Vertex *v, int orient);
  void expand_lowlevel_tree(HNode* H, LNode* L);
  void rewrite(HNode* H_from, HNode* T, HNode* H_goal, std::stack<HNode*>& OPEN);
  float get_edge_cost(const Config& C1, const Config& C2);
  float get_edge_cost(HNode* H_from, HNode* H_to);
  float get_h_value(const Config& C);
  bool get_new_config(HNode* H, LNode* L);
  bool funcPIBT(Agent* ai, HNode * H);
  Agent* swap_possible_and_required(Agent* ai);
  bool is_swap_required(const uint pusher, const uint puller, Vertex* v_pusher_origin, Vertex* v_puller_origin);
  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin);
  float get_cost_move(int pst,int ped);

  template <typename... Body>
  void solver_info(const int level, Body&&... body) {
    if (verbose < level) return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt
              << "  node_cnt:" << std::setw(8) << HNode::HNODE_CNT << "\t";
    info(level, verbose, (body)...);
  }
};

bool is_feasible_solution(const Instance& ins, const Solution& solution, const int verbose = 0);
float get_makespan(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT, const Solution& solution);
int get_path_cost(const Solution& solution, uint i);
int get_sum_of_costs(const Solution& solution);
float get_sum_of_loss(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT, const Solution& solution);
float get_makespan_lower_bound(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT);
float get_sum_of_costs_lower_bound(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT);
void print_stats(const int verbose, const Instance& ins, const std::shared_ptr<HeuristicTable>& HT,
                 const Solution& solution, const double comp_time_ms);
void make_log(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT, const Solution& solution,
              const std::string& output_name, const double comp_time_ms,
              const std::string& map_name, const int seed,
              const std::string& additional_info, const bool log_short = false);

// ====================================================================================================
// LaCAM2Solver Class
// ====================================================================================================
// The main solver class that integrates planning, execution, and action generation for LaCAM2.
// ====================================================================================================

class LaCAM2Solver {
public:
    std::vector<Path> paths;
    CompetitionActionModelWithRotate action_model;
    std::mt19937* MT;
    bool need_replan = true;
    int total_feasible_timestep = 0;
    int timestep = 0;
    void initialize(const SharedEnvironment & env);
    void plan(const SharedEnvironment & env, std::vector<Path> * precomputed_paths=nullptr, std::vector<::State> * starts=nullptr, std::vector<::State> * goals=nullptr);
    void get_step_actions(const SharedEnvironment & env, vector<Action> & actions);

    bool flag = false;
    std::shared_ptr<Graph> G;
    std::shared_ptr<HeuristicTable> HT;
    std::shared_ptr<std::vector<float> > map_weights;
    std::shared_ptr<std::vector<AgentInfo> > agent_infos;
    int max_agents_in_use;
    bool disable_corner_target_agents;
    bool disable_agent_goals;
    int execution_window;
    int planning_window;
    int num_task_completed=0;
    int max_task_completed;
    Executor executor;
    SlowExecutor slow_executor;

    Instance build_instance(const SharedEnvironment & env, std::vector<Path> * precomputed_paths=nullptr);
    float get_action_cost(int pst, int ost, int ped, int oed);
    float eval_solution(const Instance & instance, const Solution & solution);
    int get_neighbor_orientation(int loc1,int loc2);
    void solution_convert(const SharedEnvironment & env, Solution & solution, std::vector<Path> & paths);
    void disable_agents(const SharedEnvironment & env);
    void clear(const SharedEnvironment & env);

    LaCAM2Solver(const std::shared_ptr<HeuristicTable> & HT, SharedEnvironment * env, std::shared_ptr<std::vector<float> > & map_weights, int max_agents_in_use, bool disable_corner_target_agents,
     int max_task_completed);
    ~LaCAM2Solver();

    Action get_action_from_states(const State & state, const State & next_state);
};

int get_neighbor_orientation(const Graph & G, int loc1, int loc2, int default_value=5);
std::set<int> load_tabu_locs(string fp);

template <typename T>
T read_param_json(const nlohmann::json& config, const std::string& key, T default_value = T()) {
    if (config.contains(key)) {
        return config[key].get<T>();
    }
    return default_value;
}

int get_o_dist(int o1, int o2);

} // namespace LaCAM2
