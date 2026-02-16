// ====================================================================================================
// LaCAM2 Planner Implementation File
// ====================================================================================================
// This file contains the implementation of the planner and solver logic for LaCAM2.
// It includes the main solving algorithm, node expansions, and action planning.
// ====================================================================================================

#include "lacam2_core.hpp"

namespace LaCAM2 {

uint HNode::HNODE_CNT = 0;

// ====================================================================================================
// LOW-LEVEL NODE (LNode) IMPLEMENTATION
// ====================================================================================================

/**
 * @brief Constructor for low-level search tree node
 * @param parent Parent node in the search tree
 * @param i Agent index being moved
 * @param t Tuple of (vertex, orientation) for the move
 */
LNode::LNode(LNode* parent, uint i, const std::tuple<Vertex*,int > & t)
    : who(), where(), depth(parent == nullptr ? 0 : parent->depth + 1) {
  if (parent != nullptr) {
    who = parent->who;
    who.push_back(i);
    where = parent->where;
    where.push_back(t);
  }
}

// ====================================================================================================
// HIGH-LEVEL NODE (HNode) IMPLEMENTATION
// ====================================================================================================

/**
 * @brief Constructor for high-level search node
 * @param _C Current configuration of all agents
 * @param HT Heuristic table for distance computation
 * @param ins MAPF instance
 * @param _parent Parent node in high-level search
 * @param _g Cost from start to this node
 * @param _h Heuristic cost to goal
 * @param _d Depth in search tree
 * @param disable_agent_goals Whether to disable goal constraints for some agents
 */
HNode::HNode(const Config& _C, const std::shared_ptr<HeuristicTable> & HT, Instance * ins, HNode* _parent, float _g,
             float _h, const uint _d, bool disable_agent_goals)
    : C(_C), parent(_parent), neighbor(), g(_g), h(_h), d(_d), f(g + h),
      priorities(C.size()), order(C.size(), 0), search_tree(std::queue<LNode*>()) {
  ++HNODE_CNT;

  const auto N = C.size();

  if (parent != nullptr) parent->neighbor.insert(this);

  // Disable goals for specified agents
  for (int aid=0;aid<N;++aid) {
    if ((disable_agent_goals && ins->agent_infos[aid].disabled)) {
      ins->goals.locs[aid]=C.locs[aid];
    }
  }

  // Compute agent priorities for ordering
  std::vector<std::tuple<bool,bool,bool,float,float,float,int> > scores;
  for (int i=0;i<N;++i) {
    const AgentInfo & a=ins->agent_infos[i];
    bool disabled=a.disabled;
    bool arrived=C.arrivals[i];
    bool precomputed = ins->precomputed_paths!=nullptr && (*(ins->precomputed_paths))[i].size()>(d+1);
    float h=HT->get(C.locs[i]->index,C.orients[i],ins->goals.locs[i]->index);
    float elapse=a.elapsed;
    float tie_breaker=a.tie_breaker; 
    scores.emplace_back(disabled,arrived,precomputed,elapse,h,tie_breaker,i);
  }

  // Sort agents by priority scores using early_time strategy
  std::sort(scores.begin(),scores.end(),[&](std::tuple<bool,bool,bool,float,float,float,int> &score1, std::tuple<bool,bool,bool,float,float,float,int> &score2) {
    if (std::get<0>(score1)!=std::get<0>(score2)) return std::get<0>(score1)<std::get<0>(score2);
    if (std::get<1>(score1)!=std::get<1>(score2)) return std::get<1>(score1)<std::get<1>(score2);
    if (std::get<2>(score1)!=std::get<2>(score2)) return std::get<2>(score1)>std::get<2>(score2);
    // early_time strategy: prioritize earliest time (higher is better)
    if (std::get<3>(score1)!=std::get<3>(score2)) return std::get<3>(score1)>std::get<3>(score2);
    if (std::get<4>(score1)!=std::get<4>(score2)) return std::get<4>(score1)<std::get<4>(score2);
    return std::get<5>(score1)>std::get<5>(score2);
  });

  for (int i=0;i<N;++i) {
    order[i]=std::get<6>(scores[i]);
  }

  // Initialize search tree with root node
  search_tree.push(new LNode());
}

HNode::~HNode() {
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(Instance* _ins, const std::shared_ptr<HeuristicTable> & HT, const std::shared_ptr<std::vector<float> > & map_weights, const Deadline* _deadline,
                 std::mt19937* _MT, const int _verbose,
                 const Objective _objective, const float _restart_rate, bool use_swap, bool use_orient_in_heuristic, bool disable_agent_goals)
    : ins(_ins), deadline(_deadline), MT(_MT), verbose(_verbose), objective(_objective), RESTART_RATE(_restart_rate),
      N(ins->N), V_size(ins->G.size()), HT(HT), map_weights(map_weights), loop_cnt(0), C_next(N),
      tie_breakers(V_size, 0), A(N, nullptr), occupied_now(V_size, nullptr), occupied_next(V_size, nullptr),
      use_swap(use_swap), use_orient_in_heuristic(use_orient_in_heuristic),
      executor(_ins->G.height,_ins->G.width), disable_agent_goals(disable_agent_goals) {}

Planner::~Planner() {}

Solution Planner::solve(std::string& additional_info) {
  solver_info(1, "start search");

  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  auto OPEN = std::stack<HNode*>();
  auto EXPLORED = std::unordered_map<Config, HNode*, ConfigHasher, Config::ConfigEqual>();
  auto H_init = new HNode(ins->starts, HT, ins, nullptr, 0, get_h_value(ins->starts), 0, disable_agent_goals);
  OPEN.push(H_init);
  EXPLORED[H_init->C] = H_init;

  std::vector<Config> solution;
  auto C_new = Config(N);
  HNode* H_goal = nullptr;

  int d_max=INT_MAX/2;
  if (ins->planning_window>0) {
    d_max=ins->planning_window;
  }

  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    auto H = OPEN.top();

    if (H->d>=d_max) {
      H_goal=H;
      break;
    }

    if (H->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    if (H_goal != nullptr && H->f >= H_goal->f) {
      OPEN.pop();
      continue;
    }

    if (H_goal == nullptr && H->C.all_arrived()) {
      H_goal = H;
      solver_info(1, "found solution, cost: ", H->g);
      if (objective == OBJ_NONE) break;
      continue;
    }

    auto L = H->search_tree.front();
    H->search_tree.pop();
    expand_lowlevel_tree(H, L);

    const auto res = get_new_config(H, L);
    delete L;
    if (!res) continue;

      std::vector<::State> curr_states;
      std::vector<::State> planned_next_states;
      std::vector<::State> next_states;
      
      curr_states.reserve(N);
      planned_next_states.reserve(N);
      next_states.reserve(N);

      for (int i=0;i<N;++i){
        curr_states.emplace_back(H->C.locs[i]->index,0,H->C.orients[i]);
        planned_next_states.emplace_back(A[i]->v_next->index,-1,-1);
        next_states.emplace_back(-1,-1,-1);
      }

      // Always use external executor
      for (int i=0;i<N;++i) {
        C_new.locs[i] = A[i]->v_next;
        C_new.orients[i] = get_neighbor_orientation(ins->G,A[i]->v_now->index,A[i]->v_next->index,H->C.orients[i]);
        C_new.arrivals[i] = H->C.arrivals[i] | (A[i]->v_next->index==ins->goals.locs[i]->index);
      }

    // Create new high-level node for the expanded configuration
    const auto H_new = new HNode(C_new, HT, ins, H, H->g + get_edge_cost(H->C, C_new), get_h_value(C_new), H->d + 1, disable_agent_goals);
    if (H_goal == nullptr || H_new->f < H_goal->f) OPEN.push(H_new);
  }

  // Extract solution path if goal found
  if (H_goal != nullptr) {
    auto H = H_goal;
    while (H != nullptr) {
      solution.push_back(H->C);
      H = H->parent;
    }
    std::reverse(solution.begin(), solution.end());
  }

  // Report solution status
  if (H_goal != nullptr && OPEN.empty()) {
    solver_info(1, "solved optimally, objective: ", objective);
  } else if (H_goal != nullptr) {
    solver_info(1, "solved sub-optimally, objective: ", objective);
  } else if (OPEN.empty()) {
    solver_info(1, "no solution");
  } else {
    solver_info(1, "timeout");
  }

  // Add solver statistics to output
  additional_info += "optimal=" + std::to_string(H_goal != nullptr && OPEN.empty()) + "\n";
  additional_info += "objective=" + std::to_string(objective) + "\n";
  additional_info += "loop_cnt=" + std::to_string(loop_cnt) + "\n";
  additional_info += "num_node_gen=" + std::to_string(EXPLORED.size()) + "\n";

  // Clean up memory
  for (auto a : A) delete a;
  for (auto itr : EXPLORED) delete itr.second;

  return solution;
}

/**
 * @brief Rewrite function for updating costs in the search graph
 * Updates g-values and f-values when a better path is found
 * @param H_from Source node for cost propagation
 * @param H_to Target node for cost propagation
 * @param H_goal Current best goal node
 * @param OPEN Priority queue for search
 */
void Planner::rewrite(HNode* H_from, HNode* H_to, HNode* H_goal, std::stack<HNode*>& OPEN) {
  H_from->neighbor.insert(H_to);

  std::queue<HNode*> Q({H_from});
  while (!Q.empty()) {
    auto n_from = Q.front();
    Q.pop();
    for (auto n_to : n_from->neighbor) {
      auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
      if (g_val < n_to->g) {
        if (n_to == H_goal)
          solver_info(1, "cost update: ", n_to->g, " -> ", g_val);
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        n_to->d = n_from->d + 1;
        Q.push(n_to);
        if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push(n_to);
      }
    }
  }
}

/**
 * @brief Calculate edge cost between two configurations
 * @param C1 First configuration
 * @param C2 Second configuration
 * @return Cost of transitioning from C1 to C2
 */
float Planner::get_edge_cost(const Config& C1, const Config& C2) {
  if (objective == OBJ_SUM_OF_LOSS) {
    float cost = 0;
    for (uint i = 0; i < N; ++i) {
      if ((!C1.arrivals[i] || !C2.arrivals[i])) {
        cost += 1;
      }
    }
    return cost;
  }
  return 1;
}

/**
 * @brief Calculate edge cost between two high-level nodes
 * @param H_from Source node
 * @param H_to Target node
 * @return Cost of edge between nodes
 */
float Planner::get_edge_cost(HNode* H_from, HNode* H_to) {
  return get_edge_cost(H_from->C, H_to->C);
}

/**
 * @brief Calculate heuristic value for a configuration
 * @param C Current configuration
 * @return Heuristic estimate to goal
 */
float Planner::get_h_value(const Config& C) {
  float cost = 0;
  if (objective == OBJ_MAKESPAN) {
    for (auto i = 0; i < N; ++i) cost = std::max(cost, HT->get(C.locs[i]->index, C.orients[i], ins->goals.locs[i]->index)*(float)(1-C.arrivals[i]));
  } else if (objective == OBJ_SUM_OF_LOSS) {
    for (auto i = 0; i < N; ++i) cost += HT->get(C.locs[i]->index, C.orients[i], ins->goals.locs[i]->index)*(float)(1-C.arrivals[i]);
  }
  return cost;
}

/**
 * @brief Get valid successor states for a vertex and orientation
 * Implements spatial search for movement in grid with orientation constraints
 * @param v Current vertex
 * @param orient Current orientation (0=right, 1=down, 2=left, 3=up)
 * @return Vector of (vertex, orientation) tuples for valid moves
 */
std::vector<std::tuple<Vertex *,int> > Planner::get_successors(Vertex *v, int orient) {
  std::vector<std::tuple<Vertex *,int> > successors;
  int pos=v->index;
  int rows=ins->G.height;
  int cols=ins->G.width;
  int x=pos%cols;
  int y=pos/cols;

  int next_pos;
  // Move in current orientation direction
  if (orient==0) {
      if (x+1<cols){
          next_pos=pos+1;
          auto nv=ins->G.U[next_pos];
          if (nv!=nullptr) {
              successors.emplace_back(nv,orient);
          }
      }
  } else if (orient==1) {
      if (y+1<rows) {
          next_pos=pos+cols;
          auto nv=ins->G.U[next_pos];
          if (nv!=nullptr) {
              successors.emplace_back(nv,orient);
          }
      }
  } else if (orient==2) {
      if (x-1>=0) {
          next_pos=pos-1;
          auto nv=ins->G.U[next_pos];
          if (nv!=nullptr) {
              successors.emplace_back(nv,orient);
          }
      }
  } else if (orient==3) {
      if (y-1>=0) {
          next_pos=pos-cols;
          auto nv=ins->G.U[next_pos];
          if (nv!=nullptr) {
              successors.emplace_back(nv,orient);
          }
      }
  } else {
      std::cerr<<"spatial search in heuristics: invalid orient: "<<orient<<std::endl;
      exit(-1);
  }

  int next_orient;
  const int n_orients=4;

  // Add rotation moves (turn left/right)
  next_orient=(orient+1+n_orients)%n_orients;
  successors.emplace_back(v,next_orient);

  next_orient=(orient-1+n_orients)%n_orients;
  successors.emplace_back(v,next_orient);

  // Add wait action (stay in place)
  successors.emplace_back(v,orient);

  return successors;
}

/**
 * @brief Expand the low-level search tree for agent planning
 * Generates successor nodes in the low-level search tree by considering
 * all possible moves for the next agent in the ordering
 * @param H Current high-level node
 * @param L Current low-level node to expand
 */
void Planner::expand_lowlevel_tree(HNode* H, LNode* L) {
  if (L->depth >= N) return;
  const auto i = H->order[L->depth];

  auto successors=get_successors(H->C.locs[i],H->C.orients[i]);

  if (MT != nullptr) std::shuffle(successors.begin(), successors.end(), *MT);
  for (auto s : successors) H->search_tree.push(new LNode(L, i, s));
}

/**
 * @brief Generate new configuration from low-level search tree node
 * Applies the sequence of moves in the low-level tree to create a new
 * configuration, checking for conflicts and applying PIBT when needed
 * @param H Current high-level node
 * @param L Low-level node containing move sequence
 * @return True if valid configuration generated, false if conflicts
 */
bool Planner::get_new_config(HNode* H, LNode* L) {
  // Reset agent states and occupancy
  for (auto a : A) {
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }

    a->v_now = H->C.locs[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // Apply planned moves from low-level tree
  for (uint k = 0; k < L->depth; ++k) {
    const auto i = L->who[k];
    const auto l = std::get<0>(L->where[k])->id;

    if (occupied_next[l] != nullptr) return false;
    auto l_pre = H->C.locs[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id) return false;

    A[i]->v_next = std::get<0>(L->where[k]);
    occupied_next[l] = A[i];
  }

  // Apply PIBT for agents without planned moves
  for (auto k : H->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a,H)) return false;
  }
  return true;
}

/**
 * @brief Get movement cost between two positions using map weights
 * @param pst Starting position index
 * @param ped Ending position index
 * @return Cost of the move based on map weights
 */
float Planner::get_cost_move(int pst, int ped) {
  if (ped-pst==1) {
    return (*map_weights)[pst*5+0];  // Right move
  } else if (ped-pst==ins->G.width) {
    return (*map_weights)[pst*5+1];  // Down move
  } else if (ped-pst==-1) {
    return (*map_weights)[pst*5+2];  // Left move
  } else if (ped-pst==-ins->G.width) {
    return (*map_weights)[pst*5+3];  // Up move
  } else if (ped-pst==0) {
    return (*map_weights)[pst*5+4];  // Wait action
  }
  else {
    std::cerr<<"invalid move: "<<pst<<" "<<ped<<std::endl;
    exit(-1);
  }
}

/**
 * @brief Priority Inheritance with Backtracking (PIBT) algorithm
 * Resolves conflicts by finding alternative paths for agents when their
 * preferred moves are blocked. Uses priority inheritance to coordinate
 * agent movements and backtracking to explore alternatives.
 * @param ai Agent that needs to find a move
 * @param H Current high-level search node
 * @return True if a valid move was found, false otherwise
 */
bool Planner::funcPIBT(Agent* ai, HNode * H) {
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // Generate all possible moves (neighbors + stay)
  for (auto k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);
  }
  C_next[i][K] = ai->v_now;
  tie_breakers[ai->v_now->id] = get_random_float(MT);

  std::vector<std::tuple<int, float, float, Vertex *> > scores;

  int o0=H->C.orients[i];
  float cost_rot=(*map_weights)[ai->v_now->index*5+4];
  // Evaluate each possible move
  for (int k=0;k<=K;++k) {
    auto & v=C_next[i][k];
    int o1=get_neighbor_orientation(ins->G,ai->v_now->index,v->index,o0);
    int o_dist1=get_o_dist(o0,o1);
    float cost1=(float)o_dist1*cost_rot+get_cost_move(ai->v_now->index,v->index);
    float d1=HT->get(v->index,o1,ins->goals.locs[i]->index)+cost1;
    int pre_d1=1;
    // Check if move follows precomputed path
    if (ins->precomputed_paths!=nullptr){
      auto & path=(*ins->precomputed_paths)[i];
      int j=H->d;
      if (j<path.size()-1 && path[j].location==ai->v_now->index && path[j].orientation==o0) {
          if (path[j+1].orientation==o1) {
            pre_d1=0;
          }
        }
    }
    scores.emplace_back(pre_d1, d1, o_dist1, v);
  }

  // Sort moves by priority (precomputed path, heuristic distance, orientation distance)
  std::sort(scores.begin(),scores.end(),[&](const std::tuple<int, float, float, Vertex *> & a, const std::tuple<int, float, float, Vertex *> & b) {
    if (std::get<0>(a)!=std::get<0>(b)) return std::get<0>(a)<std::get<0>(b);
    if (std::get<1>(a)!=std::get<1>(b)) return std::get<1>(a)<std::get<1>(b);
    return std::get<2>(a)<std::get<2>(b);
  });

  for (int k=0;k<=K;++k) {
    C_next[i][k]=std::get<3>(scores[k]);
  }

  // Check for possible swap with another agent
  Agent* swap_agent=nullptr;
  if (use_swap) {
    swap_agent = swap_possible_and_required(ai);
    if (swap_agent != nullptr)
      std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
  }

  // Try each move in priority order
  for (auto k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];

    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    occupied_next[u->id] = ai;
    ai->v_next = u;

    // Recursively resolve conflicts for displaced agent
    if (ak != nullptr && ak != ai && ak->v_next == nullptr && !funcPIBT(ak,H))
      continue;

    // Handle agent swapping if enabled
    if (use_swap) {
      if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
          occupied_next[ai->v_now->id] == nullptr) {
        swap_agent->v_next = ai->v_now;
        occupied_next[swap_agent->v_next->id] = swap_agent;
      }
    }
    return true;
  }

  // No valid move found, stay in place
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

/**
 * @brief Check if agent swapping is possible and required
 * Determines if another agent can be swapped with the current agent
 * to resolve a deadlock situation
 * @param ai Agent that wants to move
 * @return Agent that can be swapped, or nullptr if no swap possible
 */
Agent* Planner::swap_possible_and_required(Agent* ai) {
  const auto i = ai->id;
  if (C_next[i][0] == ai->v_now) return nullptr;

  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
      is_swap_possible(aj->v_now, ai->v_now)) {
    return aj;
  }

  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0]) &&
        is_swap_possible(C_next[i][0], ai->v_now)) {
      return ak;
    }
  }

  return nullptr;
}

/**
 * @brief Check if swapping two agents is required for progress
 * Determines if a swap between pusher and puller agents is necessary
 * based on their goal distances and blocking situations
 * @param pusher Agent doing the pushing
 * @param puller Agent being pulled
 * @param v_pusher_origin Pusher's current position
 * @param v_puller_origin Puller's current position
 * @return True if swap is required
 */
bool Planner::is_swap_required(const uint pusher, const uint puller,
                               Vertex* v_pusher_origin, Vertex* v_puller_origin) {
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (
      HT->get(v_puller->index, ins->goals.locs[pusher]->index) < HT->get(v_pusher->index, ins->goals.locs[pusher]->index)
    ) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals.locs[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  return (HT->get(v_pusher->index, ins->goals.locs[puller]->index) < HT->get(v_puller->index, ins->goals.locs[puller]->index)) &&
          (HT->get(v_pusher->index, ins->goals.locs[pusher]->index) == 0 || HT->get(v_puller->index, ins->goals.locs[pusher]->index) < HT->get(v_pusher->index, ins->goals.locs[pusher]->index));
}

/**
 * @brief Check if swapping two agents is physically possible
 * Verifies that a swap operation can be executed without creating
 * additional conflicts or deadlocks
 * @param v_pusher_origin Pusher's current position
 * @param v_puller_origin Puller's current position
 * @return True if swap is possible
 */
bool Planner::is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin) {
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (v_puller != v_pusher_origin) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals.locs[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return true;
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

/**
 * @brief Load tabu locations from JSON file
 * Reads a list of forbidden locations that agents should avoid
 * @param fp File path to JSON file containing tabu locations
 * @return Set of tabu location indices
 */
std::set<int> load_tabu_locs(string fp) {
    std::ifstream f(fp);
    try {
        auto tabu_locs = nlohmann::json::parse(f);
        std::set<int> tabu_locs_set;
        for (int i = 0; i < tabu_locs.size(); ++i) {
            int loc = tabu_locs[i].get<int>();
            tabu_locs_set.insert(loc);
        }
        return tabu_locs_set;
    } catch (nlohmann::json::parse_error error) {
        std::cout << "Failed to load " << fp << std::endl;
        std::cout << "Message: " << error.what() << std::endl;
        exit(1);
    }
}

// ====================================================================================================
// LACAM2 SOLVER CLASS IMPLEMENTATION
// ====================================================================================================

/**
 * @brief Constructor for LaCAM2 solver
 * Initializes the solver with heuristic table, environment, and configuration
 * @param HT Heuristic table for distance computation
 * @param env Shared environment containing map and agent information
 * @param map_weights Optional map weights for weighted distance
 * @param max_agents_in_use Maximum number of agents to use (others disabled)
 * @param disable_corner_target_agents Whether to disable agents targeting corners
 * @param max_task_completed Maximum tasks to complete
 * @param config JSON configuration object
 */
LaCAM2Solver::LaCAM2Solver(const std::shared_ptr<HeuristicTable> & HT, SharedEnvironment * env, std::shared_ptr<std::vector<float> > & map_weights, int max_agents_in_use, bool disable_corner_target_agents,
     int max_task_completed):
        HT(HT), map_weights(map_weights), action_model(env),executor(env),slow_executor(env),
        MT(new std::mt19937(PlannerConfig::config.SEED)),
        max_agents_in_use(max_agents_in_use), disable_corner_target_agents(disable_corner_target_agents),
        max_task_completed(max_task_completed) {
        planning_window = PlannerConfig::config.PLANNING_WINDOW;
        execution_window = PlannerConfig::config.EXECUTION_WINDOW;
        disable_agent_goals = true;  // Hardcoded
}

/**
 * @brief Destructor for LaCAM2 solver
 * Cleans up random number generator
 */
LaCAM2Solver::~LaCAM2Solver(){ delete MT; }

/**
 * @brief Initialize solver with environment
 * Sets up agent paths and information structures
 * @param env Shared environment
 */
void LaCAM2Solver::initialize(const SharedEnvironment &env) {
    paths.resize(env.num_of_agents);
    agent_infos = std::make_shared<std::vector<AgentInfo>>(env.num_of_agents);
    for (int i = 0; i < env.num_of_agents; ++i) {
        (*agent_infos)[i].id = i;
    }
    G = std::make_shared<Graph>(env);
}

/**
 * @brief Disable agents based on configuration strategy
 * Selectively disables agents to reduce problem complexity
 * @param env Shared environment
 */
void LaCAM2Solver::disable_agents(const SharedEnvironment &env) {
    string strategy = "none";  // Default strategy - no agent disabling

    int disabled_agents_num;
    if (strategy == "uniform") {
        // Randomly disable agents to reach max_agents_in_use
        std::vector<int> agents_ids;
        for (int i = 0; i < env.num_of_agents; ++i) {
            agents_ids.push_back(i);
        }
        std::shuffle(agents_ids.begin(), agents_ids.end(), *MT);

        disabled_agents_num = env.num_of_agents - max_agents_in_use;
        for (int i = 0; i < disabled_agents_num; ++i) {
            (*agent_infos)[agents_ids[i]].disabled = true;
        }
    } else if (strategy == "tabu_locs") {
        // Disable agents in tabu locations
        string tabu_locs_fp = "";  // No tabu locations file by default
        auto tabu_locs = load_tabu_locs(tabu_locs_fp);

        std::vector<int> agents_ids;
        for (int i = 0; i < env.num_of_agents; ++i) {
            if (tabu_locs.find(env.curr_states[i].location) == tabu_locs.end()) {
                agents_ids.push_back(i);
            }
        }
        std::shuffle(agents_ids.begin(), agents_ids.end(), *MT);

        std::vector<int> disabled_agents_ids;
        disabled_agents_num = env.num_of_agents - max_agents_in_use;
        disabled_agents_num = std::min(disabled_agents_num, (int)agents_ids.size());
        for (int i = 0; i < disabled_agents_num; ++i) {
            (*agent_infos)[agents_ids[i]].disabled = true;
            disabled_agents_ids.push_back(agents_ids[i]);
        }
    }

    std::cout << "strategy: " << strategy << " #disabled agents: " << disabled_agents_num << std::endl;
}

/**
 * @brief Build MAPF instance from environment state
 * Creates an Instance object with current agent positions, goals, and constraints
 * @param env Shared environment
 * @param precomputed_paths Optional precomputed paths for agents
 * @return MAPF instance ready for planning
 */
Instance LaCAM2Solver::build_instance(const SharedEnvironment &env, std::vector<Path> *precomputed_paths) {
    auto starts = vector<std::pair<uint, int>>();
    auto goals = vector<std::pair<uint, int>>();

    int ctr = 0;
    for (int i = 0; i < env.num_of_agents; ++i) {
        starts.emplace_back(env.curr_states[i].location, env.curr_states[i].orientation);
        int goal_location;
        if (env.goal_locations[i].empty()) {
            goal_location = env.curr_states[i].location;
        } else {
            goal_location = env.goal_locations[i][0].first;
        }
        auto &agent_info = (*agent_infos)[i];
        if (disable_corner_target_agents) {
            int g_x = goal_location % env.cols;
            int g_y = goal_location / env.cols;
            if (G->U[goal_location]->neighbor.size() <= 1) {
                agent_info.disabled = true;
                ++ctr;
            }
        }
        if (disable_agent_goals && agent_info.disabled) {
            goal_location = env.curr_states[i].location;
        }
        goals.emplace_back(goal_location, -1);
        if (goal_location != agent_info.goal_location) {
            agent_info.goal_location = goal_location;
            agent_info.elapsed = 0;
            agent_info.tie_breaker = get_random_float(MT, 0, 1);
            agent_info.stuck_order = 0;
        } else {
            agent_info.elapsed += 1;
        }
    }
    return Instance(*G, starts, goals, *agent_infos, PlannerConfig::config.PLANNING_WINDOW, precomputed_paths);
}

/**
 * @brief Get orientation when moving between two adjacent locations
 * @param loc1 Starting location index
 * @param loc2 Ending location index
 * @return Orientation (0=right, 1=down, 2=left, 3=up)
 */
int LaCAM2Solver::get_neighbor_orientation(int loc1, int loc2) {
    if (loc1 + 1 == loc2) return 0;
    if (loc1 + G->width == loc2) return 1;
    if (loc1 - 1 == loc2) return 2;
    if (loc1 - G->width == loc2) return 3;
    std::cerr << "loc1 and loc2 are not neighbors: " << loc1 << ", " << loc2 << std::endl;
    exit(-1);
}

/**
 * @brief Get cost of action between two states
 * @param pst Starting position
 * @param ost Starting orientation
 * @param ped Ending position
 * @param oed Ending orientation
 * @return Action cost based on map weights
 */
float LaCAM2Solver::get_action_cost(int pst, int ost, int ped, int oed) {
    auto &map_weights = *(HT->map_weights);

    int offset = ped - pst;
    if (offset == 0) {
        return map_weights[pst * 5 + 4];  // Wait action
    } else if (offset == 1) {
        return map_weights[pst * 5 + 0];  // Right move
    } else if (offset == HT->env.cols) {
        return map_weights[pst * 5 + 1];  // Down move
    } else if (offset == -1) {
        return map_weights[pst * 5 + 2];
    } else if (offset == -HT->env.cols) {
        return map_weights[pst * 5 + 3];
    } else {
        std::cerr << "invalid move" << std::endl;
        exit(-1);
    }
}

float LaCAM2Solver::eval_solution(const Instance &instance, const Solution &solution) {
    float cost = 0;
    for (int aid = 0; aid < instance.N; ++aid) {
        bool arrived = false;
        for (int i = 0; i < solution.size() - 1; ++i) {
            if (solution[i].arrivals[aid]) {
                arrived = true;
                break;
            }
            int loc = solution[i].locs[aid]->index;
            int orient = solution[i].orients[aid];
            int next_loc = solution[i + 1].locs[aid]->index;
            int next_orient = solution[i + 1].orients[aid];
            cost += get_action_cost(loc, orient, next_loc, next_orient);
        }
        if (!arrived) {
            int loc = solution.back().locs[aid]->index;
            int orient = solution.back().orients[aid];
            cost += HT->get(loc, orient, instance.goals.locs[aid]->index);
        }
    }
    return cost;
}

void LaCAM2Solver::solution_convert(const SharedEnvironment &env, Solution &solution, std::vector<Path> &_paths) {
    int num_steps = 0;
    int N = _paths.size();
    int planning_window = PlannerConfig::config.PLANNING_WINDOW;

    auto &curr_config = solution[0];

    std::vector<::State> curr_states;
    for (int aid = 0; aid < N; ++aid) {
        curr_states.emplace_back(curr_config.locs[aid]->index, 0, curr_config.orients[aid]);
        _paths[aid].push_back(curr_states[aid]);
    }

    for (int i = 1; i < solution.size(); ++i) {
        auto &next_config = solution[i];
        while (true) {
            std::vector<::State> planned_next_states;
            std::vector<::State> next_states;

            planned_next_states.reserve(N);
            next_states.reserve(N);

            for (int i = 0; i < N; ++i) {
                planned_next_states.emplace_back(next_config.locs[i]->index, -1, -1);
                next_states.emplace_back(-1, -1, -1);
            }

            executor.execute(&curr_states, &planned_next_states, &next_states);

            curr_states = next_states;
            ++num_steps;

            for (int aid = 0; aid < N; ++aid) {
                _paths[aid].push_back(next_states[aid]);
            }

            if (num_steps >= planning_window) break;

            bool arrived = true;
            for (int aid = 0; aid < N; ++aid) {
                if (planned_next_states[aid].location != next_states[aid].location) {
                    arrived = false;
                    break;
                }
            }

            if (arrived) break;
        }

        if (num_steps >= planning_window) break;
    }
}

/**
 * @brief Main planning function for LaCAM2 solver
 * Plans paths for all agents from current state to goals
 * @param env Shared environment containing current state
 * @param precomputed_paths Optional precomputed paths to follow
 * @param starts Optional custom starting states
 * @param goals Optional custom goal states
 */
void LaCAM2Solver::plan(const SharedEnvironment &env, std::vector<Path> *precomputed_paths, std::vector<::State> *starts, std::vector<::State> *goals) {
    // Disable agents on first timestep if configured
    if (env.curr_timestep == 0 && max_agents_in_use < env.num_of_agents) {
        disable_agents(env);
    }

    if (need_replan) {
        const int verbose = 10;
        const int time_limit_sec = 2;
        auto instance = build_instance(env, precomputed_paths);
        
        // Override starts and goals if provided
        if (starts != nullptr) {
            if (goals == nullptr) {
                std::cerr << "not supported now! goals must be specified as well" << std::endl;
                exit(-1);
            }
            instance.set_starts_and_goals(starts, goals);
        }

        // Validate precomputed paths
        if (precomputed_paths != nullptr) {
            for (int i = 0; i < env.num_of_agents; ++i) {
                if ((*precomputed_paths)[i].size() == 0 || (*precomputed_paths)[i][0].location != instance.starts.locs[i]->index) {
                    std::cerr << "agent " << i << " has zero-length precomputed paths or initial states are not the same!" << std::endl;
                    std::cerr << "size: " << (*precomputed_paths)[i].size() << std::endl;
                    std::cerr << "states: " << (*precomputed_paths)[i][0] << " vs " << instance.starts.locs[i] << std::endl;
                    exit(-1);
                }
            }
        }

        const auto deadline = Deadline(time_limit_sec * 1000);
        bool use_swap = false;
        bool use_orient_in_heuristic = true;  // Hardcoded to rotation-aware

        float best_cost = FLT_MAX;
        Solution best_solution;

        // Create and run planner
        auto planner = Planner(&instance, HT, map_weights, &deadline, MT, 0, LaCAM2::OBJ_SUM_OF_LOSS, 0.0F,
                               use_swap, use_orient_in_heuristic, disable_agent_goals);
        auto additional_info = std::string("");
        auto solution = planner.solve(additional_info);
        auto cost = eval_solution(instance, solution);
        if (cost < best_cost) {
            best_cost = cost;
            best_solution = solution;
        }

        // Convert solution to executable paths using external executor
        solution_convert(env, best_solution, paths);
    }
}

void LaCAM2Solver::get_step_actions(const SharedEnvironment &env, vector<Action> &actions) {
    assert(actions.size() == env.num_of_agents);

    if (num_task_completed >= max_task_completed) {
        for (int i = 0; i < env.num_of_agents; ++i) {
            actions.at(i) = Action::W;
        }
    } else {
        for (int i = 0; i < env.num_of_agents; ++i) {
            if (paths[i].size() <= timestep + 1) {
                std::cerr << "wierd error for agent " << i << ". path length: " << paths[i].size() << ", " << "timestep+1: " << timestep + 1 << std::endl;
                assert(false);
            }
            actions.at(i) = get_action_from_states(paths[i][timestep], paths[i][timestep + 1]);

            // Debug: Check if agent is reaching goal
            if (timestep == 0 && i < 3 && !env.goal_locations[i].empty()) {
                std::cerr << "Agent " << i << " path from " << paths[i][0].location 
                          << " to " << paths[i].back().location 
                          << ", goal: " << env.goal_locations[i][0].first << std::endl;
            }

            if (!env.goal_locations[i].empty() and paths[i][timestep + 1].location == env.goal_locations[i][0].first) {
                ++num_task_completed;
            }
        }
    }

    if (!action_model.validate_actions(env.curr_states, actions)) {
        std::cerr << "planed actions are not valid in timestep " << timestep + 1 << "!" << std::endl;
#ifdef DEV
        exit(-1);
#else
        actions.resize(env.num_of_agents, Action::W);
#endif
    } else {
        timestep += 1;
    }

    std::cout << "timestep: " << timestep << " " << paths[0].size() << std::endl;

    if (timestep + planning_window - execution_window == paths[0].size() - 1) {
        need_replan = true;
    } else {
        need_replan = false;
    }

    std::cout << "need_replan" << need_replan << std::endl;

    if (need_replan) {
        for (int i = 0; i < env.num_of_agents; ++i) {
            paths[i].clear();
            timestep = 0;
        }
    }
}

void LaCAM2Solver::clear(const SharedEnvironment & env) {
    int num_of_agents=paths.size();

    paths.clear();
    paths.resize(env.num_of_agents);
    
    need_replan = true;
    total_feasible_timestep = 0;
    timestep = 0;
    delete MT;
    MT = new std::mt19937(PlannerConfig::config.SEED);

    // Reset agent information
    for (int i=0;i<env.num_of_agents;++i) {
        auto & agent_info=(*agent_infos)[i];
        agent_info.goal_location=-1;
        agent_info.elapsed=-1;
        agent_info.tie_breaker=-1;
        agent_info.stuck_order=0;
    }
}

/**
 * @brief Convert state transition to action
 * Determines the action needed to move from current state to next state
 * @param state Current state
 * @param next_state Next state
 * @return Action to execute
 */
Action LaCAM2Solver::get_action_from_states(const State & state, const State & next_state){
#ifndef NO_ROT
    // Handle rotation and movement actions
    if (state.location==next_state.location){
        if (state.orientation==next_state.orientation) {
            return Action::W;  // Wait
        } else if ((state.orientation-next_state.orientation+4)%4==3) {
            return Action::CR;  // Clockwise rotation
        } else if ((state.orientation-next_state.orientation+4)%4==1) {
            return Action::CCR;  // Counter-clockwise rotation
        } else {
            assert(false);
            return Action::W;
        }
    } else {
        return Action::FW;  // Forward movement
    }
#else 
    assert(state.timestep+1==next_state.timestep);
    
    // Find action for grid movement (no rotation)
    for (int i=0;i<5;++i) {
        if (state.location+action_model.moves[i]==next_state.location) {
            return static_cast<Action>(i);
        }
    }

    std::cerr<<"Cannot get action from invalid movement between state "<<state<<" and "<<next_state<<std::endl;
    exit(-1);
#endif
}

} // namespace LaCAM2
