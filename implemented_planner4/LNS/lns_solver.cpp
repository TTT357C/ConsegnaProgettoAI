#include "lns_core.h"
#include "util/Utils.h"
#include "LNS/Parallel/lns_managers.h"
#include <boost/tokenizer.hpp>
#include <algorithm>
#include <random>

// ====================================================================================================
// LARGE NEIGHBORHOOD SEARCH (LNS) SOLVER IMPLEMENTATION
// ====================================================================================================

/**
 * @brief LNS namespace for Large Neighborhood Search MAPF solver
 * Contains the main LNSSolver class and supporting utilities for MAPF optimization
 */
namespace LNS {

/**
 * @brief Safely read parameter from JSON configuration with default fallback
 * @tparam T Parameter type
 * @param config JSON configuration object
 * @param key Parameter key to read
 * @param default_value Default value if key not found
 * @return Parameter value or default
 */
template<typename T>
T read_param_json(const nlohmann::json& config, const std::string& key, T default_value = T()) {
    if (config.contains(key)) return config[key].get<T>();
    return default_value;
}

/**
 * @brief Construct Instance from SharedEnvironment
 * Initializes map and agent data from the shared environment state
 * @param env Shared environment containing current MAPF state
 */
LNS::Instance::Instance(const SharedEnvironment& env) {
    num_of_rows = env.rows;
    num_of_cols = env.cols;
    map_size = num_of_rows * num_of_cols;
    my_map.resize(map_size, false);

    for (int i = 0; i < map_size; ++i) {
        my_map[i] = (bool)env.map[i];
    }

    num_of_agents = env.num_of_agents;
    start_locations.resize(num_of_agents);
    start_orientations.resize(num_of_agents);
    goal_locations.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; ++i) {
        start_locations[i] = env.curr_states[i].location;
        start_orientations[i] = env.curr_states[i].orientation;
        if (env.goal_locations[i].empty()) {
            goal_locations[i] = env.curr_states[i].location;
        } else {
            goal_locations[i] = env.goal_locations[i][0].first;
        }
    }
}

/**
 * @brief Update agent start and goal positions from environment
 * Synchronizes instance data with current environment state
 * @param env Shared environment with updated agent positions
 */
void LNS::Instance::set_starts_and_goals(const SharedEnvironment& env) {
    for (int i = 0; i < num_of_agents; ++i) {
        start_locations[i] = env.curr_states[i].location;
        start_orientations[i] = env.curr_states[i].orientation;
        if (env.goal_locations[i].empty()) {
            goal_locations[i] = env.curr_states[i].location;
        } else {
            goal_locations[i] = env.goal_locations[i][0].first;
        }
    }
}

/**
 * @brief Set agent start and goal positions from state vectors
 * Updates instance with provided start and goal states
 * @param starts Vector of start states
 * @param goals Vector of goal states
 */
void LNS::Instance::set_starts_and_goals(const std::vector<::State>& starts, const std::vector<::State>& goals) {
    for (int i = 0; i < num_of_agents; ++i) {
        start_locations[i] = starts[i].location;
        start_orientations[i] = starts[i].orientation;
        goal_locations[i] = goals[i].location;
    }
}

/**
 * @brief Construct Instance from map and agent files with generation fallback
 * Loads map and agent data from files, generates random instances if files missing
 * @param map_fname Map file path
 * @param agent_fname Agent file path
 * @param num_of_agents Number of agents (0 for auto-detect)
 * @param num_of_rows Map rows (0 for auto-detect)
 * @param num_of_cols Map columns (0 for auto-detect)
 * @param num_of_obstacles Number of obstacles (0 for auto-detect)
 * @param warehouse_width Warehouse width for agent generation
 */
LNS::Instance::Instance(const string& map_fname, const string& agent_fname,
    int num_of_agents, int num_of_rows, int num_of_cols, int num_of_obstacles, int warehouse_width) :
    map_fname(map_fname), agent_fname(agent_fname), num_of_agents(num_of_agents) {
    bool succ = loadMap();
    if (!succ) {
        if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 &&
            num_of_obstacles < num_of_rows * num_of_cols) {
            generateConnectedRandomGrid(num_of_rows, num_of_cols, num_of_obstacles);
            saveMap();
        } else {
            cerr << "Unable to locate map file: " << map_fname << endl;
            exit(-1);
        }
    }

    succ = loadAgents();
    if (!succ) {
        if (num_of_agents > 0) {
            generateRandomAgents(warehouse_width);
            saveNathan();
        } else {
            cerr << "Agent configuration file missing: " << agent_fname << endl;
            exit(-1);
        }
    }
}

/**
 * @brief Perform random walk from starting location
 * Moves agent randomly for specified number of steps, respecting map constraints
 * @param curr Starting location
 * @param steps Number of random walk steps
 * @return Final location after random walk
 */
int LNS::Instance::randomWalk(int curr, int steps) const {
    for (int walk = 0; walk < steps; walk++) {
        list<int> l = getNeighbors(curr);
        vector<int> next_locations(l.cbegin(), l.cend());
        auto rng = std::default_random_engine{};
        std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
        for (int next : next_locations) {
            if (validMove(curr, next)) {
                curr = next;
                break;
            }
        }
    }
    return curr;
}

/**
 * @brief Generate random agent start and goal positions
 * Creates random but valid start/goal assignments with connectivity constraints
 * Supports both general random placement and warehouse-style configurations
 * @param warehouse_width Width parameter for warehouse-style agent placement (0 for general random)
 */
void LNS::Instance::generateRandomAgents(int warehouse_width) {
    cout << "Generating " << num_of_agents << " randomized start/goal positions" << endl;
    vector<bool> starts(map_size, false);
    vector<bool> goals(map_size, false);
    start_locations.resize(num_of_agents);
    goal_locations.resize(num_of_agents);

    if (warehouse_width == 0) {
        // General random placement - ensure connectivity between start and goal
        int k = 0;
        while (k < num_of_agents) {
            int x = rand() % num_of_rows, y = rand() % num_of_cols;
            int start = linearizeCoordinate(x, y);
            if (my_map[start] || starts[start]) continue;

            x = rand() % num_of_rows;
            y = rand() % num_of_cols;
            int goal = linearizeCoordinate(x, y);
            while (my_map[goal] || goals[goal]) {
                x = rand() % num_of_rows;
                y = rand() % num_of_cols;
                goal = linearizeCoordinate(x, y);
            }
            if (!isConnected(start, goal)) continue;

            start_locations[k] = start;
            starts[start] = true;
            goal_locations[k] = goal;
            goals[goal] = true;
            k++;
        }
    } else {
        // Warehouse-style placement - agents start/goal in designated warehouse zones
        int k = 0;
        while (k < num_of_agents) {
            int x = rand() % num_of_rows, y = rand() % warehouse_width;
            if (k % 2 == 0) y = num_of_cols - y - 1;
            int start = linearizeCoordinate(x, y);
            if (starts[start]) continue;
            start_locations[k] = start;
            starts[start] = true;
            k++;
        }
        k = 0;
        while (k < num_of_agents) {
            int x = rand() % num_of_rows, y = rand() % warehouse_width;
            if (k % 2 == 1) y = num_of_cols - y - 1;
            int goal = linearizeCoordinate(x, y);
            if (goals[goal]) continue;
            goal_locations[k] = goal;
            goals[goal] = true;
            k++;
        }
    }
}

/**
 * @brief Add obstacle to map while maintaining connectivity
 * Attempts to place an obstacle, ensuring the map remains connected
 * Uses connectivity checks between boundary-adjacent locations
 * @param obstacle Location to place obstacle
 * @return True if obstacle successfully added without disconnecting map
 */
bool LNS::Instance::addObstacle(int obstacle) {
    if (my_map[obstacle]) return false;
    my_map[obstacle] = true;
    int obstacle_x = getRowCoordinate(obstacle);
    int obstacle_y = getColCoordinate(obstacle);
    int x[4] = {obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1};
    int y[4] = {obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y};
    int start = 0;
    int goal = 1;
    while (start < 3 && goal < 4) {
        if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 || y[start] >= num_of_cols ||
            my_map[linearizeCoordinate(x[start], y[start])])
            start++;
        else if (goal <= start)
            goal = start + 1;
        else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 || y[goal] >= num_of_cols ||
            my_map[linearizeCoordinate(x[goal], y[goal])])
            goal++;
        else if (isConnected(linearizeCoordinate(x[start], y[start]), linearizeCoordinate(x[goal], y[goal]))) {
            start = goal;
            goal++;
        } else {
            my_map[obstacle] = false;
            return false;
        }
    }
    return true;
}

/**
 * @brief Check if two locations are connected via valid paths
 * Performs BFS to determine if a path exists between start and goal locations
 * @param start Starting location
 * @param goal Goal location
 * @return True if locations are connected
 */
bool LNS::Instance::isConnected(int start, int goal) {
    std::queue<int> open;
    vector<bool> closed(map_size, false);
    open.push(start);
    closed[start] = true;
    while (!open.empty()) {
        int curr = open.front();
        open.pop();
        if (curr == goal) return true;
        for (int next : getNeighbors(curr)) {
            if (closed[next]) continue;
            open.push(next);
            closed[next] = true;
        }
    }
    return false;
}

/**
 * @brief Generate connected random grid with obstacles
 * Creates a grid map with random obstacles while ensuring connectivity
 * Adds border walls and randomly places obstacles that don't disconnect the map
 * @param rows Interior grid rows
 * @param cols Interior grid columns
 * @param obstacles Number of obstacles to place
 */
void LNS::Instance::generateConnectedRandomGrid(int rows, int cols, int obstacles) {
    cout << "Creating " << rows << "x" << cols << " grid containing " << obstacles << " obstacles" << endl;
    int i, j;
    num_of_rows = rows + 2;
    num_of_cols = cols + 2;
    map_size = num_of_rows * num_of_cols;
    my_map.resize(map_size, false);

    // Add border walls
    i = 0;
    for (j = 0; j < num_of_cols; j++) my_map[linearizeCoordinate(i, j)] = true;
    i = num_of_rows - 1;
    for (j = 0; j < num_of_cols; j++) my_map[linearizeCoordinate(i, j)] = true;
    j = 0;
    for (i = 0; i < num_of_rows; i++) my_map[linearizeCoordinate(i, j)] = true;
    j = num_of_cols - 1;
    for (i = 0; i < num_of_rows; i++) my_map[linearizeCoordinate(i, j)] = true;

    // Add random obstacles while maintaining connectivity
    i = 0;
    while (i < obstacles) {
        int loc = rand() % map_size;
        if (addObstacle(loc)) {
            printMap();
            i++;
        }
    }
}

/**
 * @brief Load map from file
 * Reads map data from file, supporting both Nathan benchmark and standard formats
 * @return True if map loaded successfully
 */
bool LNS::Instance::loadMap() {
    using namespace boost;
    using namespace std;
    ifstream myfile(map_fname.c_str());
    if (!myfile.is_open()) return false;
    string line;
    tokenizer<char_separator<char>>::iterator beg;
    getline(myfile, line);
    if (line[0] == 't') {
        // Nathan benchmark format
        nathan_benchmark = true;
        char_separator<char> sep(" ");
        getline(myfile, line);
        tokenizer<char_separator<char>> tok(line, sep);
        beg = tok.begin();
        beg++;
        num_of_rows = atoi((*beg).c_str());
        getline(myfile, line);
        tokenizer<char_separator<char>> tok2(line, sep);
        beg = tok2.begin();
        beg++;
        num_of_cols = atoi((*beg).c_str());
        getline(myfile, line);
    } else {
        nathan_benchmark = false;
        char_separator<char> sep(",");
        tokenizer<char_separator<char>> tok(line, sep);
        beg = tok.begin();
        num_of_rows = atoi((*beg).c_str());
        beg++;
        num_of_cols = atoi((*beg).c_str());
    }
    map_size = num_of_cols * num_of_rows;
    my_map.resize(map_size, false);
    for (int i = 0; i < num_of_rows; i++) {
        getline(myfile, line);
        for (int j = 0; j < num_of_cols; j++) {
            my_map[linearizeCoordinate(i, j)] = (line[j] != '.');
        }
    }
    myfile.close();
    return true;
}

/**
 * @brief Print map to console
 * Displays the map layout with @ for obstacles and . for free cells
 */
void LNS::Instance::printMap() const {
    for (int i = 0; i < num_of_rows; i++) {
        for (int j = 0; j < num_of_cols; j++) {
            if (this->my_map[linearizeCoordinate(i, j)])
                cout << '@';
            else
                cout << '.';
        }
        cout << endl;
    }
}

/**
 * @brief Save map to file
 * Writes map data to file in CSV format (rows,cols followed by map grid)
 */
void LNS::Instance::saveMap() const {
    ofstream myfile;
    myfile.open(map_fname);
    if (!myfile.is_open()) {
        cout << "Failed to write map data to: " << map_fname << endl;
        return;
    }
    myfile << num_of_rows << "," << num_of_cols << endl;
    for (int i = 0; i < num_of_rows; i++) {
        for (int j = 0; j < num_of_cols; j++) {
            if (my_map[linearizeCoordinate(i, j)])
                myfile << "@";
            else
                myfile << ".";
        }
        myfile << endl;
    }
    myfile.close();
}

/**
 * @brief Load agent configurations from file
 * Reads agent start and goal positions from agent file
 * @return True if agents loaded successfully
 */
bool LNS::Instance::loadAgents() {
    using namespace std;
    using namespace boost;

    string line;
    ifstream myfile(agent_fname.c_str());
    if (!myfile.is_open()) return false;

    getline(myfile, line);
    if (nathan_benchmark) {
        if (num_of_agents == 0) {
            cerr << "Agent count must be positive" << endl;
            exit(-1);
        }
        start_locations.resize(num_of_agents);
        goal_locations.resize(num_of_agents);
        char_separator<char> sep("\t");
        for (int i = 0; i < num_of_agents; i++) {
            getline(myfile, line);
            if (line.empty()) {
                cerr << "Insufficient agents in instance: only " << i << " found" << endl;
                exit(-1);
            }
            tokenizer<char_separator<char>> tok(line, sep);
            tokenizer<char_separator<char>>::iterator beg = tok.begin();
            beg++; beg++; beg++; beg++;
            int col = atoi((*beg).c_str());
            beg++;
            int row = atoi((*beg).c_str());
            start_locations[i] = linearizeCoordinate(row, col);
            beg++;
            col = atoi((*beg).c_str());
            beg++;
            row = atoi((*beg).c_str());
            goal_locations[i] = linearizeCoordinate(row, col);
        }
    } else {
        char_separator<char> sep(",");
        tokenizer<char_separator<char>> tok(line, sep);
        tokenizer<char_separator<char>>::iterator beg = tok.begin();
        num_of_agents = atoi((*beg).c_str());
        start_locations.resize(num_of_agents);
        goal_locations.resize(num_of_agents);
        for (int i = 0; i < num_of_agents; i++) {
            getline(myfile, line);
            tokenizer<char_separator<char>> col_tok(line, sep);
            tokenizer<char_separator<char>>::iterator c_beg = col_tok.begin();
            pair<int, int> curr_pair;
            int row = atoi((*c_beg).c_str());
            c_beg++;
            int col = atoi((*c_beg).c_str());
            start_locations[i] = linearizeCoordinate(row, col);
            c_beg++;
            row = atoi((*c_beg).c_str());
            c_beg++;
            col = atoi((*c_beg).c_str());
            goal_locations[i] = linearizeCoordinate(row, col);
        }
    }
    myfile.close();
    return true;
}

/**
 * @brief Save agent configurations in Nathan benchmark format
 * Writes agent start and goal positions to file in Nathan benchmark format
 */
void LNS::Instance::saveNathan() const {
    ofstream myfile;
    myfile.open(agent_fname);
    if (!myfile.is_open()) {
        cout << "Unable to persist agent data to: " << agent_fname << endl;
        return;
    }
    myfile << "version 1" << endl;
    for (int i = 0; i < num_of_agents; i++)
        myfile << i << "\t" << map_fname << "\t" << this->num_of_cols << "\t" << this->num_of_rows << "\t"
               << getColCoordinate(start_locations[i]) << "\t" << getRowCoordinate(start_locations[i]) << "\t"
               << getColCoordinate(goal_locations[i]) << "\t" << getRowCoordinate(goal_locations[i]) << "\t" << 0
               << endl;
    myfile.close();
}

/**
 * @brief Get valid neighboring locations
 * Returns list of locations adjacent to current location (up, down, left, right)
 * Only includes valid moves that don't go through obstacles or out of bounds
 * @param curr Current location
 * @return List of valid neighboring locations
 */
list<int> LNS::Instance::getNeighbors(int curr) const {
    list<int> neighbors;
    int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
    for (int next : candidates) {
        if (validMove(curr, next)) neighbors.emplace_back(next);
    }
    return neighbors;
}

// ====================================================================================================
// RESERVATION TABLE IMPLEMENTATION
// ====================================================================================================

/**
 * @brief Insert time interval into Safe Interval Table (SIT)
 * Adds a safe time interval for a location to the reservation table
 * @param location Location to add interval for
 * @param t_min Minimum timestep (inclusive)
 * @param t_max Maximum timestep (exclusive)
 */
void LNS::ReservationTable::insert2SIT(int location, int t_min, int t_max) {
    if (t_min < 0 || t_min >= t_max || sit[location].empty()) return;
    for (auto it = sit[location].begin(); it != sit[location].end();) {
        if (t_min >= get<1>(*it))
            ++it;
        else if (t_max <= get<0>(*it))
            break;
        else if (get<0>(*it) < t_min && get<1>(*it) <= t_max) {
            (*it) = make_tuple(get<0>(*it), t_min, get<2>(*it));
            ++it;
        } else if (t_min <= get<0>(*it) && t_max < get<1>(*it)) {
            (*it) = make_tuple(t_max, get<1>(*it), get<2>(*it));
            break;
        } else if (get<0>(*it) < t_min && t_max < get<1>(*it)) {
            sit[location].insert(it, make_tuple(get<0>(*it), t_min, get<2>(*it)));
            (*it) = make_tuple(t_max, get<1>(*it), get<2>(*it));
            break;
        } else {
            it = sit[location].erase(it);
        }
    }
}

/**
 * @brief Insert soft constraint into Safe Interval Table
 * Modifies safe intervals to account for soft constraints (penalties)
 * Maintains interval integrity while marking constrained periods
 * @param location Location to modify
 * @param t_min Minimum timestep (inclusive)
 * @param t_max Maximum timestep (exclusive)
 */
void LNS::ReservationTable::insertSoftConstraint2SIT(int location, int t_min, int t_max) {
    if (t_min < 0 || t_min >= t_max || sit[location].empty()) return;
    for (auto it = sit[location].begin(); it != sit[location].end(); ++it) {
        if (t_min >= get<1>(*it) || get<2>(*it))
            continue;
        else if (t_max <= get<0>(*it))
            break;

        auto i_min = get<0>(*it);
        auto i_max = get<1>(*it);
        if (i_min < t_min && i_max <= t_max) {
            if (it != sit[location].end() && std::next(it) != sit[location].end() &&
                (location != goal_location || i_max != constraint_table.min_path_length) &&
                i_max == get<0>(*std::next(it)) && get<2>(*std::next(it))) {
                (*it) = make_tuple(i_min, t_min, false);
                ++it;
                (*it) = make_tuple(t_min, get<1>(*it), true);
            } else {
                sit[location].insert(it, make_tuple(i_min, t_min, false));
                (*it) = make_tuple(t_min, i_max, true);
            }
        } else if (t_min <= i_min && t_max < i_max) {
            if (it != sit[location].begin() &&
                (location != goal_location || i_min != constraint_table.min_path_length) &&
                i_min == get<1>(*std::prev(it)) && get<2>(*std::prev(it))) {
                (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), t_max, true);
            } else {
                sit[location].insert(it, make_tuple(i_min, t_max, true));
            }
            (*it) = make_tuple(t_max, i_max, false);
        } else if (i_min < t_min && t_max < i_max) {
            sit[location].insert(it, make_tuple(i_min, t_min, false));
            sit[location].insert(it, make_tuple(t_min, t_max, true));
            (*it) = make_tuple(t_max, i_max, false);
        } else {
            if (it != sit[location].begin() &&
                (location != goal_location || i_min != constraint_table.min_path_length) &&
                i_min == get<1>(*std::prev(it)) && get<2>(*std::prev(it))) {
                if (it != sit[location].end() && std::next(it) != sit[location].end() &&
                    (location != goal_location || i_max != constraint_table.min_path_length) &&
                    i_max == get<0>(*std::next(it)) && get<2>(*std::next(it))) {
                    (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), get<1>(*std::next(it)), true);
                    sit[location].erase(std::next(it));
                    it = sit[location].erase(it);
                } else {
                    (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), i_max, true);
                    it = sit[location].erase(it);
                }
                --it;
            } else {
                if (it != sit[location].end() && std::next(it) != sit[location].end() &&
                    (location != goal_location || i_max != constraint_table.min_path_length) &&
                    i_max == get<0>(*std::next(it)) && get<2>(*std::next(it))) {
                    (*it) = make_tuple(i_min, get<1>(*std::next(it)), true);
                    sit[location].erase(std::next(it));
                } else {
                    (*it) = make_tuple(i_min, i_max, true);
                }
            }
        }
    }
}

/**
 * @brief Update Safe Interval Table for a location
 * Initializes and populates the SIT with safe time intervals considering all constraints
 * Handles both vertex and edge constraints, hard and soft constraints
 * @param location Location to update SIT for
 */
void LNS::ReservationTable::updateSIT(int location) {
    if (!sit[location].empty()) return;
    if (location == goal_location) {
        if (constraint_table.min_path_length > constraint_table.max_path_length) {
            sit[location].emplace_back(0, 0, false);
            return;
        }
        if (0 < constraint_table.min_path_length) {
            sit[location].emplace_back(0, constraint_table.min_path_length, false);
        }
        if (constraint_table.min_path_length >= 0)
            sit[location].emplace_back(constraint_table.min_path_length,
                                       min(constraint_table.max_path_length + 1, MAX_TIMESTEP), false);
    } else {
        sit[location].emplace_back(0, min(constraint_table.max_path_length, MAX_TIMESTEP - 1) + 1, false);
    }

    // Insert hard constraints from other agents' paths
    if (constraint_table.hard_constraints_table != nullptr && !constraint_table.hard_constraints_table->space_time_grid.empty()) {
        if (location < constraint_table.map_size) {
            int T = min((int)constraint_table.hard_constraints_table->space_time_grid[location].size(),
                        constraint_table.hard_constraint_window + 1);
            for (int t = 0; t < T; t++) {
                if (constraint_table.hard_constraints_table->space_time_grid[location][t] != NO_AGENT) {
                    insert2SIT(location, t, t + 1);
                }
            }
            T = min(MAX_TIMESTEP, constraint_table.hard_constraint_window + 1);
            if (constraint_table.hard_constraints_table->target_timestamps[location] < T)
                insert2SIT(location, constraint_table.hard_constraints_table->target_timestamps[location], T + 1);
        } else {
            auto from = location / constraint_table.map_size - 1;
            auto to = location % constraint_table.map_size;
            if (from != to) {
                int t_max = (int)min(constraint_table.hard_constraints_table->space_time_grid[from].size(),
                                     constraint_table.hard_constraints_table->space_time_grid[to].size() + 1);
                t_max = min(t_max, constraint_table.hard_constraint_window + 1);
                for (int t = 1; t < t_max; t++) {
                    if (constraint_table.hard_constraints_table->space_time_grid[to][t - 1] != NO_AGENT &&
                        constraint_table.hard_constraints_table->space_time_grid[to][t - 1] ==
                            constraint_table.hard_constraints_table->space_time_grid[from][t]) {
                        insert2SIT(location, t, t + 1);
                    }
                }
            }
        }
    }

    // Insert blocking constraints
    const auto& it = constraint_table.blocking_constraints.find(location);
    if (it != constraint_table.blocking_constraints.end()) {
        for (auto time_range : it->second) insert2SIT(location, time_range.first, time_range.second);
    }

    // Insert landmark constraints
    if (location < constraint_table.map_size) {
        for (auto landmark : constraint_table.landmarks) {
            if (landmark.second != location) {
                insert2SIT(location, landmark.first, landmark.first + 1);
            }
        }
    }

    // Insert soft constraints from other agents' paths
    if (constraint_table.soft_constraints_table != nullptr &&
        !constraint_table.soft_constraints_table->conflict_grid.empty()) {
        if (location < constraint_table.map_size) {
            for (int t = 0; t < (int)constraint_table.soft_constraints_table->conflict_grid[location].size(); t++) {
                if (!constraint_table.soft_constraints_table->conflict_grid[location][t].empty()) {
                    insertSoftConstraint2SIT(location, t, t + 1);
                }
            }
            if (constraint_table.soft_constraints_table->target_timestamps[location] < MAX_TIMESTEP)
                insertSoftConstraint2SIT(location, constraint_table.soft_constraints_table->target_timestamps[location],
                                         MAX_TIMESTEP);
        } else {
            auto from = location / constraint_table.map_size - 1;
            auto to = location % constraint_table.map_size;
            if (from != to) {
                int t_max = (int)min(constraint_table.soft_constraints_table->conflict_grid[from].size(),
                                     constraint_table.soft_constraints_table->conflict_grid[to].size() + 1);
                for (int t = 1; t < t_max; t++) {
                    bool found = false;
                    for (auto a1 : constraint_table.soft_constraints_table->conflict_grid[to][t - 1]) {
                        for (auto a2 : constraint_table.soft_constraints_table->conflict_grid[from][t]) {
                            if (a1 == a2) {
                                insertSoftConstraint2SIT(location, t, t + 1);
                                found = true;
                                break;
                            }
                            if (found) break;
                        }
                    }
                }
            }
        }
    }

    if (!constraint_table.penalty_grid.empty()) {
        for (auto t = 0; t < constraint_table.penalty_grid[location].size(); t++) {
            if (constraint_table.penalty_grid[location][t]) insertSoftConstraint2SIT(location, t, t + 1);
        }
        if (constraint_table.penalty_goal_times[location] < MAX_TIMESTEP)
            insertSoftConstraint2SIT(location, constraint_table.penalty_goal_times[location], MAX_TIMESTEP);
    }
}

list<tuple<int, int, int, bool, bool>> LNS::ReservationTable::get_safe_intervals(int from, int to, int lower_bound,
                                                                             int upper_bound) {
    list<tuple<int, int, int, bool, bool>> rst;
    if (lower_bound >= upper_bound) return rst;

    if (sit[to].empty()) updateSIT(to);

    for (auto interval : sit[to]) {
        if (lower_bound >= get<1>(interval))
            continue;
        else if (upper_bound <= get<0>(interval))
            break;
        auto t1 = get_earliest_arrival_time(from, to, max(lower_bound, get<0>(interval)),
                                            min(upper_bound, get<1>(interval)));
        if (t1 < 0)
            continue;
        else if (get<2>(interval)) {
            rst.emplace_back(get<1>(interval), t1, get<1>(interval), true, false);
        } else {
            auto t2 = get_earliest_no_collision_arrival_time(from, to, interval, t1, upper_bound);
            if (t1 == t2)
                rst.emplace_back(get<1>(interval), t1, get<1>(interval), false, false);
            else if (t2 < 0)
                rst.emplace_back(get<1>(interval), t1, get<1>(interval), false, true);
            else {
                rst.emplace_back(get<1>(interval), t1, t2, false, true);
                rst.emplace_back(get<1>(interval), t2, get<1>(interval), false, false);
            }
        }
    }
    return rst;
}

Interval LNS::ReservationTable::get_first_safe_interval(size_t location) {
    if (sit[location].empty()) updateSIT(location);
    return sit[location].front();
}

bool LNS::ReservationTable::find_safe_interval(Interval& interval, size_t location, int t_min) {
    if (t_min >= min(constraint_table.max_path_length, MAX_TIMESTEP - 1) + 1) return false;
    if (sit[location].empty()) updateSIT(location);
    for (auto& i : sit[location]) {
        if ((int)get<0>(i) <= t_min && t_min < (int)get<1>(i)) {
            interval = Interval(t_min, get<1>(i), get<2>(i));
            return true;
        } else if (t_min < (int)get<0>(i))
            break;
    }
    return false;
}

int LNS::ReservationTable::get_earliest_arrival_time(int from, int to, int lower_bound, int upper_bound) const {
    int T = min(MAX_TIMESTEP, constraint_table.hard_constraint_window);
    for (auto t = lower_bound; t < upper_bound; t++) {
        if (t > T || !constraint_table.isBlocked(from, to, t)) return t;
    }
    return -1;
}

int LNS::ReservationTable::get_earliest_no_collision_arrival_time(int from, int to, const Interval& interval,
                                                              int lower_bound, int upper_bound) const {
    for (auto t = max(lower_bound, get<0>(interval)); t < min(upper_bound, get<1>(interval)); t++) {
        if (!constraint_table.hasEdgeConflict(from, to, t)) return t;
    }
    return -1;
}

// ====================================================================================================
// LNS SOLVER IMPLEMENTATION
// ====================================================================================================

/**
 * @brief Constructor for Large Neighborhood Search solver
 * Initializes LNS solver with all required components for MAPF optimization
 * @param HT Heuristic distance table for A* search
 * @param env Shared environment with current MAPF state
 * @param map_weights Edge weights for the map
 * @param lacam2_solver LaCAM2 solver for initial solution generation
 * @param max_task_completed Maximum number of tasks to complete
 */
LNS::LNSSolver::LNSSolver(const std::shared_ptr<HeuristicTable>& HT, SharedEnvironment* env,
                     std::shared_ptr<std::vector<float>>& map_weights,
                     std::shared_ptr<LaCAM2::LaCAM2Solver>& lacam2_solver, int max_task_completed)
    : HT(HT), map_weights(map_weights), action_model(env), executor(env), slow_executor(env),
      MT(new std::mt19937(PlannerConfig::config.SEED)), lacam2_solver(lacam2_solver),
      agent_infos(lacam2_solver->agent_infos), max_task_completed(max_task_completed) {}

/**
 * @brief Initialize LNS solver with environment state
 * Sets up initial paths and solver components based on current environment
 * @param env Shared environment containing current agent states
 */
void LNS::LNSSolver::initialize(const SharedEnvironment& env) {
    execution_paths.resize(env.num_of_agents);
    planning_paths.resize(env.num_of_agents);

    planning_window = PlannerConfig::config.HARD_CONSTRAINT_WINDOW;
    execution_window = PlannerConfig::config.EXECUTION_WINDOW;
}

/**
 * @brief Observe environment state and update execution tracking
 * Monitors agent movements and determines if new execution paths are needed
 * Updates executed_step counter and checks for plan completion
 * @param env Current shared environment state
 */
void LNS::LNSSolver::observe(const SharedEnvironment& env) {
    if (execution_paths[0].size() == 0) {
        // Initialize execution paths with current states
        for (int i = 0; i < env.num_of_agents; ++i) {
            execution_paths[i].push_back(env.curr_states[i]);
            planning_paths[i].push_back(env.curr_states[i]);
        }
        executed_step = 0;
    } else {
        // Check if agents followed planned paths
        bool match = true;
        for (int i = 0; i < execution_paths.size(); ++i) {
            if (executed_step + 1 >= execution_paths[i].size()) {
                cerr << "Execution index out of bounds: " << executed_step << " exceeds path length " << execution_paths[i].size()
                     << endl;
                exit(-1);
            }
            if (execution_paths[i][executed_step + 1].location != env.curr_states[i].location ||
                execution_paths[i][executed_step + 1].orientation != env.curr_states[i].orientation) {
                match = false;
            }
        }
        if (match) ++executed_step;
    }

    // Check if we need new execution paths
    if (executed_step == execution_paths[0].size() - 1) {
        need_new_execution_paths = true;
    } else {
        need_new_execution_paths = false;
    }
}

/**
 * @brief Main planning function for LNS solver
 * Generates conflict-free paths using LaCAM2 initialization and LNS optimization
 * Handles path extension, replanning, and execution path generation
 * @param env Current shared environment state
 */
void LNS::LNSSolver::plan(const SharedEnvironment& env, int time_limit_ms) {

    double time_limit = (time_limit_ms > 0) 
        ? (time_limit_ms / 1000.0) * PlannerConfig::config.CUTOFF_TIME
        : PlannerConfig::config.CUTOFF_TIME;
        
    // Extract current starts and goals from environment
    std::vector<::State> starts;
    std::vector<::State> goals;

    for (int i = 0; i < env.num_of_agents; ++i) {
        starts.emplace_back(execution_paths[i].back().location, -1, execution_paths[i].back().orientation);
        if (env.goal_locations[i].empty()) {
            goals.emplace_back(env.curr_states[i].location, -1, -1);
        } else {
            goals.emplace_back(env.goal_locations[i][0].first, -1, -1);
        }
    }

    // Extend planning paths if needed
    if (planning_paths[0].size() < planning_window + 1) {
        // Always use LaCAM2 (hardcoded)
        lacam2_solver->clear(env);
            vector<::Path> precomputed_paths;
            precomputed_paths.resize(env.num_of_agents);
            for (int i = 0; i < env.num_of_agents; ++i) {
                if (planning_paths[i][0].location != execution_paths[i].back().location ||
                    planning_paths[i][0].orientation != execution_paths[i].back().orientation) {
                    cerr << "State mismatch detected for agent " << i << " - current position conflicts with plan" << endl;
                    exit(-1);
                }
                for (int j = 0; j < planning_paths[i].size(); ++j) {
                    precomputed_paths[i].emplace_back(planning_paths[i][j]);
                    if (j > 1 && (env.goal_locations[i].empty() ||
                                  planning_paths[i][j].location == env.goal_locations[i][0].first)) {
                        break;
                    }
                }
            }

            lacam2_solver->plan(env, &precomputed_paths, &starts, &goals);

            for (int i = 0; i < env.num_of_agents; ++i) {
                planning_paths[i].clear();
                for (int j = 0; j < lacam2_solver->paths[i].size(); ++j) {
                    planning_paths[i].emplace_back(lacam2_solver->paths[i][j].location, j,
                                                   lacam2_solver->paths[i][j].orientation);
                }
            }
    }

    // Initialize LNS components on first timestep
    if (env.curr_timestep == 0) {
        instance = std::make_shared<Instance>(env);
        lns = std::make_shared<Parallel::GlobalManager>(
            true, *instance, HT, map_weights, agent_infos, PlannerConfig::config.NEIGHBOR_SIZE,
            Parallel::destroy_heuristic::RANDOMWALK, true, 0.01, 0.01, "LaCAM2",
            "PP", false, PlannerConfig::config.HARD_CONSTRAINT_WINDOW,
            PlannerConfig::config.WINDOW_SIZE_FOR_CAT, PlannerConfig::config.WINDOW_SIZE_FOR_PATH,
            execution_window, lacam2_solver->max_agents_in_use != env.num_of_agents);
    }

    // Reset LNS state and update goals
    lns->reset();
    instance->set_starts_and_goals(starts, goals);

    // Update agent paths in LNS with current planning paths
    for (int i = 0; i < lns->agents.size(); i++) {
        if (lns->agents[i].id != i) {
            cerr << "Agent sequence initialization error - incorrect ordering detected" << endl;
            exit(-1);
        }
        lns->agents[i].path.clear();
        for (int j = 0; j < planning_paths[i].size(); ++j) {
            lns->agents[i].path.nodes.emplace_back(planning_paths[i][j].location, planning_paths[i][j].orientation);
        }
        if (env.goal_locations[i].empty()) {
            lns->agents[i].path.path_cost = 0;
        } else {
            lns->agents[i].path.path_cost =
                lns->agents[i].getEstimatedPathLength(lns->agents[i].path, env.goal_locations[i][0].first, HT);
        }
    }

    // Execute LNS optimization with time limit
    TimeLimiter time_limiter(time_limit);
    bool succ = lns->run(time_limiter);

    // Extend paths to planning window if using single-step execution
    if (execution_window == 1) {
        for (int i = 0; i < lns->agents.size(); ++i) {
            if (lns->agents[i].path.size() < planning_window + 1) {
                lns->agents[i].path.nodes.resize(planning_window + 1, lns->agents[i].path.back());
            }
        }
    }

    // Update planning paths with LNS results
    for (int i = 0; i < planning_paths.size(); ++i) {
        auto& path = planning_paths[i];
        auto& new_path = lns->agents[i].path;
        path.clear();
        for (int j = 0; j < new_path.size(); ++j) {
            path.emplace_back(new_path[j].location, j, new_path[j].orientation);
        }
    }
}

/**
 * @brief Generate execution actions for current timestep
 * Extracts next actions from execution paths and handles task completion
 * Validates path consistency and generates appropriate actions
 * @param env Current shared environment state
 * @param actions Output vector to store computed actions for each agent
 */
void LNS::LNSSolver::get_step_actions(const SharedEnvironment& env, vector<Action>& actions) {
    // Generate new execution paths if needed
    if (need_new_execution_paths) {
        for (int i = 0; i < env.num_of_agents; ++i) {
            execution_paths[i].clear();
            for (int j = 0; j < execution_window + 1; ++j) {
                execution_paths[i].emplace_back(planning_paths[i][j].location, j, planning_paths[i][j].orientation);
            }
            if (execution_paths[i].size() != execution_window + 1) {
                std::cerr << "Execution trajectory length validation failed" << std::endl;
                exit(-1);
            }
            executed_step = 0;
        }

        // Update planning paths by removing executed portion
        for (int i = 0; i < env.num_of_agents; ++i) {
            planning_paths[i].erase(planning_paths[i].begin(), planning_paths[i].begin() + execution_window);
            if (planning_paths[i].size() != planning_window + 1 - execution_window) {
                std::cerr << "Plan trajectory for agent " << i << " has invalid length: " << planning_paths[i].size()
                          << " (expected: " << planning_window + 1 - execution_window << ")" << std::endl;
                std::cerr << "Planning trajectory dimension mismatch" << std::endl;
                exit(-1);
            }
        }
    }

    // Generate actions or wait if all tasks completed
    if (num_task_completed >= max_task_completed) {
        for (int i = 0; i < env.num_of_agents; ++i) {
            actions.at(i) = Action::W;
        }
    } else {
        for (int i = 0; i < env.num_of_agents; ++i) {
            if (execution_paths[i].size() <= executed_step + 1) {
                cerr << "Unexpected path error for agent " << i << ": trajectory length " << execution_paths[i].size() << ", "
                     << "executed_plan_step+1: " << executed_step + 1 << endl;
                exit(-1);
            }

            // Validate current state matches execution path
            if (execution_paths[i][executed_step].location != env.curr_states[i].location ||
                execution_paths[i][executed_step].orientation != env.curr_states[i].orientation) {
                cerr << "Agent " << i << " position/orientation mismatch with execution plan" << endl;
                exit(-1);
            }

            // Generate action from state transition
            actions.at(i) = get_action_from_states(execution_paths[i][executed_step], execution_paths[i][executed_step + 1]);

            // Check for task completion
            if (!env.goal_locations[i].empty() &&
                execution_paths[i][executed_step + 1].location == env.goal_locations[i][0].first) {
                ++num_task_completed;
            }
        }
    }
}

/**
 * @brief Convert state transition to action
 * Determines the action required to move from current state to next state
 * Handles rotation and forward movement in orientation-aware planning
 * @param state Current agent state
 * @param next_state Target state to reach
 * @return Action to execute (FW, CR, CCR, W)
 */
Action LNS::LNSSolver::get_action_from_states(const State& state, const State& next_state) {
#ifndef NO_ROT
    // Handle rotation-only moves
    if (state.location == next_state.location) {
        if (state.orientation == next_state.orientation) {
            return Action::W;  // Wait
        } else if ((state.orientation - next_state.orientation + 4) % 4 == 3) {
            return Action::CR;  // Clockwise rotation
        } else if ((state.orientation - next_state.orientation + 4) % 4 == 1) {
            return Action::CCR;  // Counter-clockwise rotation
        }
    } else {
        return Action::FW;  // Forward movement
    }
#else
    // Handle movement without rotation
    for (int i = 0; i < 5; ++i) {
        if (state.location + action_model.moves[i] == next_state.location) {
            return static_cast<Action>(i);
        }
    }
    cerr << "Unable to derive valid action from state transition: " << state << " -> " << next_state << endl;
    exit(-1);
#endif
    return Action::W;
}

}
