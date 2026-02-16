// ====================================================================================================
// LaCAM2 Core Implementation File
// ====================================================================================================
// This file contains the implementation of the core components of the LaCAM2 MAPF solver.
// It includes implementations for graph handling, configurations, executors, and utility functions.
// ====================================================================================================

#include "lacam2_core.hpp"
#include <algorithm>

namespace LaCAM2 {

Deadline::Deadline(double _time_limit_ms) : t_s(Time::now()), time_limit_ms(_time_limit_ms) {}


double elapsed_ms(const Deadline* deadline) {
  if (deadline == nullptr) return 0;
  return std::chrono::duration_cast<std::chrono::milliseconds>(Time::now() - deadline->t_s).count();
}

double elapsed_ns(const Deadline* deadline) {
  if (deadline == nullptr) return 0;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() - deadline->t_s).count();
}

bool is_expired(const Deadline* deadline) {
  if (deadline == nullptr) return false;
  return elapsed_ms(deadline) > deadline->time_limit_ms;
}

float get_random_float(std::mt19937* MT, float from, float to) {
  return std::uniform_real_distribution<float> (from, to)(*MT);
}

void info(const int level, const int verbose) { std::cout << std::endl; }

Vertex::Vertex(uint _id, uint _index) : id(_id), index(_index), neighbor(Vertices()) {}

Graph::Graph() : V(Vertices()), width(0), height(0) {}

Graph::~Graph() {
  for (auto& v : V) if (v != nullptr) delete v;
  V.clear();
}

Graph::Graph(const SharedEnvironment & env): V(Vertices()), width(env.cols), height(env.rows) {
  U = Vertices(width * height, nullptr);
  std::cout<<"graph size "<<U.size()<<std::endl;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int idx=i*width+j;
      int loc = env.map[idx];
      if (loc==1) continue;
      auto v = new Vertex(V.size(), idx);
      V.push_back(v);
      U[idx] = v;
    }
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      auto v = U[width * y + x];
      if (v == nullptr) continue;
      if (x > 0) {
        auto u = U[width * y + (x - 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      if (x < width - 1) {
        auto u = U[width * y + (x + 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      if (y < height - 1) {
        auto u = U[width * (y + 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      if (y > 0) {
        auto u = U[width * (y - 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
    }
  }
}

static const std::regex r_height = std::regex(R"(height\s(\d+))");
static const std::regex r_width = std::regex(R"(width\s(\d+))");
static const std::regex r_map = std::regex(R"(map)");

/**
 * @brief Constructor for Graph from file
 * Loads a graph from a standard MAPF benchmark file format
 * @param filename Path to the map file
 */
Graph::Graph(const std::string& filename) : V(Vertices()), width(0), height(0) {
  std::ifstream file(filename);
  if (!file) {
    std::cout << "file " << filename << " is not found." << std::endl;
    return;
  }
  std::string line;
  std::smatch results;

  // Parse header information
  while (getline(file, line)) {
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    if (std::regex_match(line, results, r_height)) height = std::stoi(results[1].str());
    if (std::regex_match(line, results, r_width)) width = std::stoi(results[1].str());
    if (std::regex_match(line, results, r_map)) break;
  }

  U = Vertices(width * height, nullptr);

  // Parse map data
  uint y = 0;
  while (getline(file, line)) {
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    for (uint x = 0; x < width; ++x) {
      char s = line[x];
      if (s == 'T' or s == '@') continue;  // Skip obstacles
      auto index = width * y + x;
      auto v = new Vertex(V.size(), index);
      V.push_back(v);
      U[index] = v;
    }
    ++y;
  }
  file.close();

  // Build neighbor relationships
  for (uint y = 0; y < height; ++y) {
    for (uint x = 0; x < width; ++x) {
      auto v = U[width * y + x];
      if (v == nullptr) continue;
      // Add neighbors in 4 cardinal directions
      if (x > 0) {
        auto u = U[width * y + (x - 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      if (x < width - 1) {
        auto u = U[width * y + (x + 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      if (y < height - 1) {
        auto u = U[width * (y + 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      if (y > 0) {
        auto u = U[width * (y - 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
    }
  }
}

/**
 * @brief Get number of vertices in graph
 * @return Number of valid vertices (excluding obstacles)
 */
uint Graph::size() const { return V.size(); }

/**
 * @brief Check if two configurations are identical
 * Compares positions, orientations, and arrival status for all agents
 * @param C1 First configuration
 * @param C2 Second configuration
 * @return True if configurations are identical
 */
bool is_same_config(const Config& C1, const Config& C2) {
  const auto N = C1.size();
  for (size_t i = 0; i < N; ++i) {
    if (C1.locs[i]->id != C2.locs[i]->id) return false;
    if (C1.orients[i] != C2.orients[i]) return false;
    if (C1.arrivals[i] != C2.arrivals[i]) return false;
  }
  return true;
}

/**
 * @brief Hash function for Config
 * Generates a hash value for configuration-based state lookup
 * @param C Configuration to hash
 * @return Hash value
 */
uint ConfigHasher::operator()(const Config& C) const {
  uint hash = C.size();
  for (auto i=0;i<C.size();++i) {
    hash ^= C.locs[i]->id<<3 + C.orients[i]<<1 + C.arrivals[i] + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  }
  return hash;
}

/**
 * @brief Output operator for Vertex
 * @param os Output stream
 * @param v Vertex to output
 * @return Output stream
 */
std::ostream& operator<<(std::ostream& os, const Vertex* v) {
  os << v->index;
  return os;
}

/**
 * @brief Output operator for Config
 * @param os Output stream
 * @param config Configuration to output
 * @return Output stream
 */
std::ostream& operator<<(std::ostream& os, const Config& config) {
  os << "<";
  const auto N = config.size();
  for (size_t i = 0; i < N; ++i) {
    if (i > 0) os << ",";
    os << "(" <<std::setw(5) << config.locs[i]<<","<< config.orients[i]<<","<<config.arrivals[i]<<")";
  }
  os << ">";
  return os;
}

/**
 * @brief Constructor for MAPF Instance
 * Creates a complete MAPF problem instance with graph, starts, goals, and agent information
 * @param G Graph representing the environment
 * @param start_indexes Starting positions and orientations for agents
 * @param goal_indexes Goal positions and orientations for agents
 * @param agent_infos Additional agent information
 * @param planning_window Maximum planning horizon
 * @param precomputed_paths Optional precomputed paths for agents
 */
Instance::Instance(
    const Graph & G,
    const std::vector<std::pair<uint,int> >& start_indexes,
    const std::vector<std::pair<uint,int> >& goal_indexes,
    std::vector<AgentInfo>& agent_infos,
    int planning_window,
    std::vector<::Path> * _precomputed_paths
    ): G(G), starts(agent_infos.size()), goals(agent_infos.size()), N(agent_infos.size()),
       agent_infos(agent_infos), planning_window(planning_window), precomputed_paths(_precomputed_paths) {
  // Initialize start and goal configurations
  for (int i=0;i<N;++i) {
    starts.locs[i]=G.U[start_indexes[i].first];
    starts.orients[i]=start_indexes[i].second;
    goals.locs[i]=G.U[goal_indexes[i].first];
    goals.orients[i]=goal_indexes[i].second;
  }
}

/**
 * @brief Set custom start and goal states
 * Overrides the default start and goal configurations
 * @param starts_ptr Pointer to new start states
 * @param goals_ptr Pointer to new goal states
 */
void Instance::set_starts_and_goals(std::vector<::State> * starts_ptr, std::vector<::State> * goals_ptr) {
  auto & starts=*starts_ptr;
  auto & goals=*goals_ptr;

  for (int i=0;i<N;++i) {
    this->starts.locs[i]=G.U[starts[i].location];
    this->starts.orients[i]=starts[i].orientation;
    this->goals.locs[i]=G.U[goals[i].location];
    this->goals.orients[i]=goals[i].orientation;
  }
}

/**
 * @brief Validate instance consistency
 * Checks that the instance has valid dimensions and configurations
 * @param verbose Verbosity level for error messages
 * @return True if instance is valid
 */
bool Instance::is_valid(const int verbose) const {
  if (N != starts.size() || N != goals.size()) {
    info(1, verbose, "invalid N, check instance");
    return false;
  }
  return true;
}

/**
 * @brief Output operator for Solution
 * Prints the complete solution showing paths for all agents
 * @param os Output stream
 * @param solution Solution to output
 * @return Output stream
 */
std::ostream& operator<<(std::ostream& os, const Solution& solution) {
  auto N = solution.front().size();
  for (size_t i = 0; i < N; ++i) {
    os << std::setw(5) << i << ":";
    for (size_t k = 0; k < solution[i].size(); ++k) {
      if (k > 0) os << "->";
      os << "(" << std::setw(5) << solution[i].locs[k]->index <<"," << solution[i].orients[k] << ")"; 
    }
    os << std::endl;
  }
  return os;
}

// ====================================================================================================
// EXECUTOR CLASS IMPLEMENTATION
// ====================================================================================================

/**
 * @brief Execute agent actions and resolve conflicts
 * Main execution method that takes planned actions and produces actual next states
 * after resolving any conflicts between agents
 * @param _curr_states Current states of all agents
 * @param _planned_next_states Planned next states for all agents
 * @param _next_states Output actual next states after conflict resolution
 */
void Executor::execute(const vector<State> * _curr_states, const vector<State> * _planned_next_states, vector<State> * _next_states) {
    assert(_curr_states->size()==_planned_next_states->size());
    assert(_curr_states->size()==_next_states->size());

    curr_states=_curr_states;
    planned_next_states=_planned_next_states;
    next_states=_next_states;

    executed.clear();
    executed.resize(curr_states->size(),false);
    reservation_table.clear();
    // Reserve current positions
    for (int i=0;i<curr_states->size();++i) {
        reservation_table[(*curr_states)[i].location]=i;
    }

    // Execute all agents starting from root agents
    for (int i=0;i<curr_states->size();++i) {
        if (!executed[i]) {
            execute_agent(i,i);
        }
    }
}

/**
 * @brief Execute a single agent's action with conflict resolution
 * Recursively resolves conflicts by executing blocking agents first
 * @param agent_idx Index of agent to execute
 * @param root_agent_idx Root agent in the execution chain (for cycle detection)
 */
void Executor::execute_agent(int agent_idx, int root_agent_idx) {
    assert(!executed[agent_idx]);

    const auto & curr_state=(*curr_states)[agent_idx];
    const auto & planned_next_state=(*planned_next_states)[agent_idx];
    auto & next_state=(*next_states)[agent_idx];

    // Handle rotation-only moves (no position change)
    if (curr_state.location==planned_next_state.location) {
        next_state.location=curr_state.location;
        if (planned_next_state.orientation!=-1) {
            next_state.orientation=planned_next_state.orientation;
        } else {
            next_state.orientation=curr_state.orientation;
        }
        next_state.timestep=curr_state.timestep+1;
    } else {
        // Handle movement to new position
        int curr_orient=curr_state.orientation;
        int expected_orient=get_neighbor_orientation(curr_state.location,planned_next_state.location);
        
        if (curr_orient==expected_orient) {
            // Agent is facing the correct direction for movement
            auto iter=reservation_table.find(planned_next_state.location);
            if (iter!=reservation_table.end()) {
                // Target position is occupied
                int parent_agent_idx=iter->second;
                if (parent_agent_idx!=root_agent_idx && !executed[parent_agent_idx]) {
                    // Execute the occupying agent first to potentially free the position
                    execute_agent(parent_agent_idx,root_agent_idx);
                }
                if (parent_agent_idx==root_agent_idx || (*next_states)[parent_agent_idx].location!=(*curr_states)[parent_agent_idx].location) {
                    // Position is now free or occupied by root agent moving away
                    next_state.location=planned_next_state.location;
                    next_state.orientation=curr_state.orientation;
                    next_state.timestep=curr_state.timestep+1;
                } else {
                    // Position still occupied, stay in place
                    next_state.location=curr_state.location;
                    if (planned_next_state.orientation!=-1) {
                        next_state.orientation=planned_next_state.orientation;
                    } else {
                        next_state.orientation=curr_state.orientation;
                    }
                    next_state.timestep=curr_state.timestep+1;
                }
            } else {
                // Target position is free
                next_state.location=planned_next_state.location;
                next_state.orientation=curr_state.orientation;
                next_state.timestep=curr_state.timestep+1;
            }
        } else {
            // Agent needs to rotate to face the correct direction
            int d1=(curr_orient+4-expected_orient)%4;
            int d2=(expected_orient+4-curr_orient)%4;

            int next_orient=-1;
            // Choose shorter rotation direction
            if (d1<d2) {
                next_orient=(curr_orient-1+4)%4;
            } else {
                next_orient=(curr_orient+1+4)%4;
            }
            
            next_state.location=curr_state.location;
            if (planned_next_state.orientation!=-1) {
                next_state.orientation=planned_next_state.orientation;
            } else {
                next_state.orientation=next_orient;
            }
            next_state.timestep=curr_state.timestep+1;
        }
    }

    executed[agent_idx]=true;
}

/**
 * @brief Get orientation when moving between adjacent locations
 * @param loc1 Starting location
 * @param loc2 Ending location
 * @return Orientation (0=right, 1=down, 2=left, 3=up)
 */
int Executor::get_neighbor_orientation(int loc1,int loc2) {
    if (loc1+1==loc2) return 0;
    if (loc1+cols==loc2) return 1;
    if (loc1-1==loc2) return 2;
    if (loc1-cols==loc2) return 3;
    std::cerr<<"executor: loc1 and loc2 are not neighbors: "<<loc1<<", "<<loc2<<std::endl;
    exit(-1);
}

// ====================================================================================================
// SLOW EXECUTOR CLASS IMPLEMENTATION
// ====================================================================================================

/**
 * @brief Execute actions with simplified conflict resolution
 * Slower but simpler executor that handles orientation changes before movement
 * @param _curr_states Current states
 * @param _planned_next_states Planned next states
 * @param _next_states Output next states
 */
void SlowExecutor::execute(const vector<State> * _curr_states, const vector<State> * _planned_next_states, vector<State> * _next_states) {
    assert(_curr_states->size()==_planned_next_states->size());
    assert(_curr_states->size()==_next_states->size());

    curr_states=_curr_states;
    planned_next_states=_planned_next_states;
    next_states=_next_states;
    int n_agents=curr_states->size();

    // Check if all agents are properly oriented for their moves
    bool all_oriented=true;
    for (int i=0;i<n_agents;++i) {
        if ((*curr_states)[i].location!=(*planned_next_states)[i].location) {
            int expected_orient=get_neighbor_orientation((*curr_states)[i].location,(*planned_next_states)[i].location);
            if ((*curr_states)[i].orientation!=expected_orient) {
                all_oriented=false;
                break;
            }
        }
    }

    // Execute actions based on orientation status
    for (int i=0;i<n_agents;++i) {
        if ((*curr_states)[i].location==(*planned_next_states)[i].location) {
            // Rotation-only move
            (*next_states)[i].location=(*curr_states)[i].location;
            (*next_states)[i].orientation=(*curr_states)[i].orientation;
            (*next_states)[i].timestep=(*curr_states)[i].timestep+1;
        } else {
            int expected_orient=get_neighbor_orientation((*curr_states)[i].location,(*planned_next_states)[i].location);
            if (all_oriented) {
                // All agents can move since they're properly oriented
                (*next_states)[i].location=(*planned_next_states)[i].location;
                (*next_states)[i].orientation=expected_orient;
                (*next_states)[i].timestep=(*curr_states)[i].timestep+1;
            } else {
                // Some agents need to rotate first
                int next_orient=get_next_orientation((*curr_states)[i].orientation,expected_orient);
                (*next_states)[i].location=(*curr_states)[i].location;
                (*next_states)[i].orientation=next_orient;
                (*next_states)[i].timestep=(*curr_states)[i].timestep+1;
            }
        }
    }
}

/**
 * @brief Get next orientation in rotation towards target
 * @param curr_orient Current orientation
 * @param target_orient Target orientation
 * @return Next orientation in shortest rotation path
 */
int SlowExecutor::get_next_orientation(int curr_orient, int target_orient) {
    if (curr_orient==target_orient) return curr_orient;

    int next_orient1=(curr_orient+1+4)%4;
    int next_orient2=(curr_orient-1+4)%4;

    int d1=(next_orient1-target_orient+4)%4;
    int d2=(next_orient2-target_orient+4)%4;
    if (d1<d2) {
        return next_orient1;
    } else {
        return next_orient2;
    }
}

/**
 * @brief Get orientation between adjacent locations
 * @param loc1 Starting location
 * @param loc2 Ending location
 * @return Orientation (0=right, 1=down, 2=left, 3=up)
 */
int SlowExecutor::get_neighbor_orientation(int loc1,int loc2) {
    if (loc1+1==loc2) return 0;
    if (loc1+env->cols==loc2) return 1;
    if (loc1-1==loc2) return 2;
    if (loc1-env->cols==loc2) return 3;
    std::cerr<<"executor: loc1 and loc2 are not neighbors: "<<loc1<<", "<<loc2<<std::endl;
    exit(-1);
}

// ====================================================================================================
// UTILITY FUNCTIONS
// ====================================================================================================

/**
 * @brief Check if solution is feasible (placeholder)
 * @param ins MAPF instance
 * @param solution Solution to check
 * @param verbose Verbosity level
 * @return True (always returns true for now)
 */
bool is_feasible_solution(const Instance& ins, const Solution& solution, const int verbose) {
  return true;
}

/**
 * @brief Calculate makespan of solution
 * Makespan is the maximum completion time over all agents
 * @param ins MAPF instance
 * @param HT Heuristic table
 * @param solution Solution to evaluate
 * @return Makespan value
 */
float get_makespan(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT, const Solution& solution) {
  if (solution.empty()) return 0;
  float c = 0;
  for (auto i=0;i<ins.N;++i) {
    c = std::max(c,(float)(solution.size()-1) + HT->get(solution.back().locs[i]->index,solution.back().orients[i],ins.goals.locs[i]->index));
  }
  return c;
}

/**
 * @brief Get path cost for single agent
 * @param solution Complete solution
 * @param i Agent index
 * @return Cost of agent's path
 */
int get_path_cost(const Solution& solution, uint i) {
  const auto makespan = solution.size();
  const auto g = solution.back().locs[i];
  auto c = makespan;
  while (c > 0 && solution[c - 1].locs[i] == g) --c;
  return c;
}

/**
 * @brief Calculate sum of costs for all agents
 * @param solution Solution to evaluate
 * @return Sum of individual path costs
 */
int get_sum_of_costs(const Solution& solution) {
  if (solution.empty()) return 0;
  int c = 0;
  const auto N = solution.front().size();
  for (size_t i = 0; i < N; ++i) c += get_path_cost(solution, i);
  return c;
}

/**
 * @brief Calculate sum of loss (SOC + remaining distance)
 * @param ins MAPF instance
 * @param HT Heuristic table
 * @param solution Solution to evaluate
 * @return Sum of loss value
 */
float get_sum_of_loss(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT, const Solution& solution) {
  if (solution.empty()) return 0;
  float c = 0;
  const auto N = solution.front().size();
  const auto T = solution.size();
  for (size_t i = 0; i < N; ++i) {
    c+= (float)(solution.size()-1) + HT->get(solution.back().locs[i]->index,solution.back().orients[i],ins.goals.locs[i]->index);
  }
  return c;
}

/**
 * @brief Get lower bound for makespan
 * @param ins MAPF instance
 * @param HT Heuristic table
 * @return Makespan lower bound
 */
float get_makespan_lower_bound(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT) {
  float c = 0;
  for (size_t i = 0; i < ins.N; ++i) {
    c = std::max(c, HT->get(ins.starts.locs[i]->index,ins.goals.locs[i]->index));
  }
  return c;
}

/**
 * @brief Get lower bound for sum of costs
 * @param ins MAPF instance
 * @param HT Heuristic table
 * @return Sum of costs lower bound
 */
float get_sum_of_costs_lower_bound(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT) {
  float c = 0;
  for (size_t i = 0; i < ins.N; ++i) {
    c += HT->get(ins.starts.locs[i]->index,ins.goals.locs[i]->index);
  }
  return c;
}

/**
 * @brief Print solution statistics
 * @param verbose Verbosity level
 * @param ins MAPF instance
 * @param HT Heuristic table
 * @param solution Solution to analyze
 * @param comp_time_ms Computation time
 */
void print_stats(const int verbose, const Instance& ins, const std::shared_ptr<HeuristicTable>& HT,
                 const Solution& solution, const double comp_time_ms) {
  auto ceil = [](float x) { return std::ceil(x * 100) / 100; };
  const auto makespan = get_makespan(ins,HT,solution);
  const auto makespan_lb = get_makespan_lower_bound(ins, HT);
  const auto sum_of_costs_lb = get_sum_of_costs_lower_bound(ins, HT);
  const auto sum_of_loss = get_sum_of_loss(ins,HT,solution);
  info(1, verbose, "solved: ", comp_time_ms, "ms", "\tmakespan: ", makespan,
       " (lb=", makespan_lb, ", ratio=", ceil((float)makespan / makespan_lb), ")",
       "\tsum_of_loss: ", sum_of_loss, " (lb=", sum_of_costs_lb,
       ", ratio=", ceil((float)sum_of_loss / sum_of_costs_lb), ")");
}

static const std::regex r_map_name = std::regex(R"(.+/(.+))");

/**
 * @brief Create solution log file
 * @param ins MAPF instance
 * @param HT Heuristic table
 * @param solution Solution to log
 * @param output_name Output filename
 * @param comp_time_ms Computation time
 * @param map_name Map filename
 * @param seed Random seed used
 * @param additional_info Additional solver information
 * @param log_short Whether to create short log
 */
void make_log(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT, const Solution& solution,
              const std::string& output_name, const double comp_time_ms,
              const std::string& map_name, const int seed,
              const std::string& additional_info, const bool log_short) {
  std::smatch results;
  const auto map_recorded_name = (std::regex_match(map_name, results, r_map_name)) ? results[1].str() : map_name;

  auto get_x = [&](int k) { return k % ins.G.width; };
  auto get_y = [&](int k) { return k / ins.G.width; };
  std::ofstream log;
  log.open(output_name, std::ios::out);
  log << "agents=" << ins.N << "\n";
  log << "map_file=" << map_recorded_name << "\n";
  log << "solver=planner\n";
  log << "solved=" << !solution.empty() << "\n";
  log << "makespan=" << get_makespan(ins,HT,solution) << "\n";
  log << "makespan_lb=" << get_makespan_lower_bound(ins, HT) << "\n";
  log << "sum_of_loss=" << get_sum_of_loss(ins,HT,solution) << "\n";
  log << "sum_of_loss_lb=" << get_sum_of_costs_lower_bound(ins, HT) << "\n";
  log << "comp_time=" << comp_time_ms << "\n";
  log << "seed=" << seed << "\n";
  log << additional_info;
  if (log_short) return;
  log << "starts=";
  for (size_t i = 0; i < ins.N; ++i) {
    auto k = ins.starts.locs[i]->index;
    log << "(" << get_x(k) << "," << get_y(k) << "),";
  }
  log << "\ngoals=";
  for (size_t i = 0; i < ins.N; ++i) {
    auto k = ins.goals.locs[i]->index;
    log << "(" << get_x(k) << "," << get_y(k) << "),";
  }
  log << "\nsolution=\n";
  for (size_t t = 0; t < solution.size(); ++t) {
    log << t << ":";
    auto C = solution[t];
    for (auto v : C.locs) {
      log << "(" << get_x(v->index) << "," << get_y(v->index) << "),";
    }
    log << "\n";
  }
  log.close();
}

/**
 * @brief Output operator for Objective enum
 * @param os Output stream
 * @param obj Objective to output
 * @return Output stream
 */
std::ostream& operator<<(std::ostream& os, const Objective obj) {
  if (obj == OBJ_NONE) {
    os << "none";
  } else if (obj == OBJ_MAKESPAN) {
    os << "makespan";
  } else if (obj == OBJ_SUM_OF_LOSS) {
    os << "sum_of_loss";
  }
  return os;
}

int get_neighbor_orientation(const Graph & G, int loc1, int loc2, int default_value) {
    if (loc1+1==loc2) return 0;
    if (loc1+G.width==loc2) return 1;
    if (loc1-1==loc2) return 2;
    if (loc1-G.width==loc2) return 3;
    if (loc1==loc2) return default_value;
    std::cerr<<"loc1 and loc2 are not neighbors: "<<loc1<<", "<<loc2<<std::endl;
    exit(-1);
}

int get_o_dist(int o1, int o2) {
  return std::min((o2-o1+4)%4,(o1-o2+4)%4);
}

} // namespace LaCAM2
