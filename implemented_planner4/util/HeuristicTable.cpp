#include "util/HeuristicTable.h"

// ====================================================================================================
// UTIL::SPATIAL Namespace Implementation
// ====================================================================================================
// This file contains the implementation of spatial A* search and heuristic table computation.
// It provides efficient precomputation of distances for MAPF planning.
// ====================================================================================================

namespace UTIL {
namespace SPATIAL {

// ====================================================================================================
// AnytimeAStar::get_successors - Generate successor states
// ====================================================================================================
// Generates all possible successor states from the current state.
// Handles both orientation-aware and orientation-ignorant modes.
// ====================================================================================================
void AnytimeAStar::get_successors(State *curr)
{
    clear_successors();  // Reset successors list
    if (curr->orientation == -1)  // No orientation mode
    {
        int pos = curr->pos;
        int x = pos % (env.cols);  // Current x coordinate
        int y = pos / (env.cols);  // Current y coordinate

        // Try moving right
        if (x + 1 < env.cols)
        {
            int next_pos = pos + 1;
            if (env.map[next_pos] == 0)  // Check if empty
            {
                int weight_idx = pos * n_dirs;  // Weight index for right move
                add_successor(next_pos, -1, curr->g + weights[weight_idx], 0, curr);
            }
        }

        // Try moving down
        if (y + 1 < env.rows)
        {
            int next_pos = pos + env.cols;
            if (env.map[next_pos] == 0)
            {
                int weight_idx = pos * n_dirs + 1;  // Weight index for down move
                add_successor(next_pos, -1, curr->g + weights[weight_idx], 0, curr);
            }
        }

        // Try moving left
        if (x - 1 >= 0)
        {
            int next_pos = pos - 1;
            if (env.map[next_pos] == 0)
            {
                int weight_idx = pos * n_dirs + 2;  // Weight index for left move
                add_successor(next_pos, -1, curr->g + weights[weight_idx], 0, curr);
            }
        }

        // Try moving up
        if (y - 1 >= 0)
        {
            int next_pos = pos - env.cols;
            if (env.map[next_pos] == 0)
            {
                int weight_idx = pos * n_dirs + 3;  // Weight index for up move
                add_successor(next_pos, -1, curr->g + weights[weight_idx], 0, curr);
            }
        }
    }
    else  // Orientation-aware mode
    {
        int pos = curr->pos;
        int x = pos % (env.cols);
        int y = pos / (env.cols);
        int orient = curr->orientation;

        int next_pos;
        int weight_idx;
        if (orient == 0)  // Facing right
        {
            if (x + 1 < env.cols)
            {
                next_pos = pos + 1;
                weight_idx = pos * n_dirs;  // Forward move
                if (env.map[next_pos] == 0)
                {
                    add_successor(next_pos, orient, curr->g + weights[weight_idx], 0, curr);
                }
            }
        }
        else if (orient == 1)  // Facing down
        {
            if (y + 1 < env.rows)
            {
                next_pos = pos + env.cols;
                weight_idx = pos * n_dirs + 1;
                if (env.map[next_pos] == 0)
                {
                    add_successor(next_pos, orient, curr->g + weights[weight_idx], 0, curr);
                }
            }
        }
        else if (orient == 2)  // Facing left
        {
            if (x - 1 >= 0)
            {
                next_pos = pos - 1;
                weight_idx = pos * n_dirs + 2;
                if (env.map[next_pos] == 0)
                {
                    add_successor(next_pos, orient, curr->g + weights[weight_idx], 0, curr);
                }
            }
        }
        else if (orient == 3)  // Facing up
        {
            if (y - 1 >= 0)
            {
                next_pos = pos - env.cols;
                weight_idx = pos * n_dirs + 3;
                if (env.map[next_pos] == 0)
                {
                    add_successor(next_pos, orient, curr->g + weights[weight_idx], 0, curr);
                }
            }
        }
        else
        {
            std::cerr << "spatial search in heuristics: invalid orient: " << orient << endl;
            exit(-1);
        }

        // Rotation actions
        int next_orient;
        weight_idx = pos * n_dirs + 4;  // Rotation weight

        next_orient = (orient + 1 + n_orients) % n_orients;  // Clockwise
        add_successor(pos, next_orient, curr->g + weights[weight_idx], 0, curr);

        next_orient = (orient - 1 + n_orients) % n_orients;  // Counter-clockwise
        add_successor(pos, next_orient, curr->g + weights[weight_idx], 0, curr);

        add_successor(pos, orient, curr->g + weights[weight_idx], 0, curr);  // Wait
    }
}

// ====================================================================================================
// AnytimeAStar::add_state - Add a new state to the search
// ====================================================================================================
// Creates and initializes a new state in the all_states array.
// Returns pointer to the new state.
// ====================================================================================================
State *AnytimeAStar::add_state(int pos, int orient, float g, float h, State *prev)
{
    int index;
    if (orient == -1)  // No orientation
    {
        index = pos;
    }
    else  // With orientation
    {
        index = pos * n_orients + orient;
    }
    State *s = all_states + index;
    if (s->pos != -1)  // State already exists
    {
        exit(-1);
    }

    s->pos = pos;
    s->orientation = orient;
    s->g = g;
    s->h = h;
    s->f = g + h;
    s->prev = prev;

    ++n_states;  // Increment state count
    return s;
}

// ====================================================================================================
// AnytimeAStar::search_for_all_weighted - Perform weighted A* search to all reachable states
// ====================================================================================================
// Runs weighted A* search (f = g + weight * h) from start to find all reachable states.
// Weighted A* is faster than standard A* but still admissible.
// Optimized: check time every 1024 iterations instead of every iteration (reduces clock_gettime overhead)
// ====================================================================================================
void AnytimeAStar::search_for_all_weighted(int start_pos, int start_orient, TimePoint end_time, float weight)
{
    State *start = add_state(start_pos, start_orient, 0, 0, nullptr);
    open_list->push(start);

    int iteration = 0;
    const int time_check_interval = 4096;  // Check time every N iterations to reduce overhead
    bool has_time_limit = (end_time != TimePoint::max());
    TimePoint cached_time = std::chrono::steady_clock::now();  // Cache initial time

    while (!open_list->empty())
    {
        // Cache time checks: only update cached_time every 1024 iterations
        if (has_time_limit && (iteration & (time_check_interval - 1)) == 0) {
            cached_time = std::chrono::steady_clock::now();
            if (cached_time > end_time) {
                break;
            }
        }
        ++iteration;
        
        State *curr = open_list->pop();
        curr->closed = true;

        get_successors(curr);
        for (int i = 0; i < n_successors; ++i)
        {
            State *next = successors + i;
            int index;
            if (next->orientation == -1)
            {
                index = next->pos;
            }
            else
            {
                index = next->pos * n_orients + next->orientation;
            }
            if ((all_states + index)->pos == -1)
            {
                // For weighted A*, use f = g + weight * h
                float weighted_f = next->g + weight * next->h;
                State *new_state = add_state(next->pos, next->orientation, next->g, next->h, next->prev);
                new_state->f = weighted_f;  // Override the f value set in add_state
                new_state->closed = false;
                open_list->push(new_state);
            }
            else
            {
                auto old_state = all_states + index;
                if (next->g < old_state->g)
                {
                    old_state->copyStateData(next);
                    // Update f value for weighted A*
                    old_state->f = next->g + weight * next->h;
                    if (old_state->closed)
                    {
                        old_state->closed = false;
                        open_list->push(old_state);
                    }
                    else
                    {
                        open_list->increase(old_state);
                    }
                }
            }
        }
    }
}

// ====================================================================================================
// AnytimeAStar::search_for_all - Perform standard A* search to all reachable states
// ====================================================================================================
// Runs standard A* search (f = g + h) from start to find all reachable states.
// This is the most accurate but potentially slower than weighted A*.
// Optimized: check time every 1024 iterations instead of every iteration
// ====================================================================================================
void UTIL::SPATIAL::AnytimeAStar::search_for_all(int start_pos, int start_orient, TimePoint end_time)
{
    State *start = add_state(start_pos, start_orient, 0, 0, nullptr);
    open_list->push(start);

    int iteration = 0;
    const int time_check_interval = 1024;  // Check time every N iterations to reduce overhead
    bool has_time_limit = (end_time != TimePoint::max());
    TimePoint cached_time = std::chrono::steady_clock::now();  // Cache initial time

    while (!open_list->empty())
    {
        // Cache time checks: only update cached_time every 1024 iterations
        if (has_time_limit && (iteration & (time_check_interval - 1)) == 0) {
            cached_time = std::chrono::steady_clock::now();
            if (cached_time > end_time) {
                break;
            }
        }
        ++iteration;
        
        State *curr = open_list->pop();
        curr->closed = true;

        get_successors(curr);
        for (int i = 0; i < n_successors; ++i)
        {
            State *next = successors + i;
            int index;
            if (next->orientation == -1)
            {
                index = next->pos;
            }
            else
            {
                index = next->pos * n_orients + next->orientation;
            }
            if ((all_states + index)->pos == -1)
            {
                // For standard A*, use f = g + h (weight = 1.0)
                State *new_state = add_state(next->pos, next->orientation, next->g, next->h, next->prev);
                open_list->push(new_state);
            }
            else
            {
                auto old_state = all_states + index;
                if (next->g < old_state->g)
                {
                    old_state->copyStateData(next);
                    if (old_state->closed)
                    {
                        old_state->closed = false;
                        open_list->push(old_state);
                    }
                    else
                    {
                        open_list->increase(old_state);
                    }
                }
            }
        }
    }
}

}  // namespace SPATIAL
}  // namespace UTIL

// ====================================================================================================
// HeuristicTable Class Implementation
// ====================================================================================================
// The HeuristicTable class manages precomputed heuristic distances for efficient MAPF planning.
// It supports both rotation-aware and rotation-ignorant heuristics.
// ====================================================================================================

// ====================================================================================================
// HeuristicTable::HeuristicTable - Constructor
// ====================================================================================================
// Initializes the heuristic table with the given environment and map weights.
// Sets up data structures for storing heuristics.
// ====================================================================================================
HeuristicTable::HeuristicTable(SharedEnvironment *_env, const std::shared_ptr<std::vector<float>> &map_weights) 
    : env(*_env), action_model(_env), map_weights(map_weights)
{
    // Always use rotation-aware mode
    n_orientations = 4;

    loc_size = 0;  // Count empty locations
    for (int i = 0; i < env.map.size(); ++i)
    {
        if (!env.map[i])  // If not obstacle
        {
            ++loc_size;
        }
    }

    empty_locs = new int[loc_size];  // Array of empty locations
    loc_idxs = new int[env.map.size()];  // Location to index mapping

    std::fill(loc_idxs, loc_idxs + env.map.size(), -1);  // Initialize to -1

    int loc_idx = 0;
    for (int loc = 0; loc < env.map.size(); ++loc)  // Build mappings
    {
        if (!env.map[loc])
        {
            empty_locs[loc_idx] = loc;
            loc_idxs[loc] = loc_idx;
            ++loc_idx;
        }
    }

    state_size = loc_size * n_orientations;  // Total state size
    main_heuristics = new float[loc_size * loc_size];  // Main heuristic table
    std::fill(main_heuristics, main_heuristics + loc_size * loc_size, MAX_HEURISTIC);  // Initialize to max
    
    sub_heuristics = new float[state_size * loc_size];  // Sub-heuristics for orientations
};

// ====================================================================================================
// HeuristicTable::~HeuristicTable - Destructor
// ====================================================================================================
// Cleans up dynamically allocated memory.
// ====================================================================================================
HeuristicTable::~HeuristicTable()
{
    delete[] empty_locs;
    delete[] loc_idxs;
    delete[] main_heuristics;
    delete[] sub_heuristics;
}

// ====================================================================================================
// HeuristicTable::compute_weighted_heuristics - Compute all pairwise heuristics
// ====================================================================================================
// Uses anytime approach: first initializes all heuristics with Manhattan distance (fast),
// then refines them with A* search using remaining time.
// Parallelized using OpenMP for performance.
// ====================================================================================================
void HeuristicTable::compute_weighted_heuristics(TimePoint end_time)
{
    int n_threads = omp_get_max_threads();
    cout << "number of threads used for heuristic computation: " << n_threads << endl;
    
    bool has_time_limit = (end_time != TimePoint::max());
    if (has_time_limit) {
        cout << "Using anytime heuristics: Manhattan initialization + A* refinement" << endl;
    } else {
        cout << "Using complete A* search (no time limit)" << endl;
    }
    
    auto start = std::chrono::steady_clock::now();
    
    // ========================================
    // PHASE 1: Initialize ALL heuristics with Manhattan distance (very fast)
    // ========================================
    cout << "Phase 1: Initializing all " << loc_size << " locations with Manhattan distance..." << std::flush;
    #pragma omp parallel for
    for (int start_loc_idx = 0; start_loc_idx < loc_size; ++start_loc_idx)
    {
        int start_loc = empty_locs[start_loc_idx];
        for (int target_loc_idx = 0; target_loc_idx < loc_size; ++target_loc_idx)
        {
            int target_loc = empty_locs[target_loc_idx];
            float manhattan = compute_manhattan_distance(start_loc, target_loc);
            
            size_t main_idx = start_loc_idx * loc_size + target_loc_idx;
            main_heuristics[main_idx] = manhattan;
            
            // Initialize sub-heuristics (orientation penalty)
            for (int orient = 0; orient < n_orientations; ++orient)
            {
                size_t sub_idx = (start_loc_idx * loc_size + target_loc_idx) * n_orientations + orient;
                // Small penalty for non-optimal orientation (rough estimate)
                sub_heuristics[sub_idx] = (manhattan > 0) ? 0.5f : 0.0f;
            }
        }
    }
    
    auto phase1_end = std::chrono::steady_clock::now();
    double phase1_time = std::chrono::duration<double>(phase1_end - start).count();
    cout << " done in " << phase1_time << "s" << endl;
    
    // Check if we have time for refinement
    if (has_time_limit && std::chrono::steady_clock::now() > end_time) {
        cout << "No time remaining for BFS/A* refinement. Using Manhattan heuristics only." << endl;
        return;
    }
    
    // ========================================
    // PHASE 2: BFS refinement (fast, wall-aware)
    // Only for paris and brc maps - these have irregular shapes where Manhattan is highly inaccurate
    // ========================================
    bool use_bfs = false;
    string map_name_lower = env.map_name;
    std::transform(map_name_lower.begin(), map_name_lower.end(), map_name_lower.begin(), ::tolower);
    if (map_name_lower.find("paris") != string::npos || map_name_lower.find("brc") != string::npos) {
        use_bfs = true;
    }
    
    if (use_bfs) {
        auto phase2_start = std::chrono::steady_clock::now();
        cout << "Phase 2: BFS refinement (wall-aware, fast) - irregular map detected..." << endl;
        refine_with_bfs(end_time, has_time_limit, start, phase2_start);
        
        // Check if we have time for A* refinement
        if (has_time_limit && std::chrono::steady_clock::now() >= end_time) {
            cout << "No time remaining for A* refinement. Using BFS heuristics." << endl;
            return;
        }
    } else {
        cout << "Phase 2: Skipping BFS refinement (rectangular map, Manhattan is accurate)" << endl;
    }
    
    // ========================================
    // PHASE 3: A* without rotation (faster, 1/4 state space)
    // ========================================
    auto phase3_start = std::chrono::steady_clock::now();
    cout << "Phase 3: A* refinement without rotation (more accurate)..." << endl;
    
    // Allocate memory for parallel A* computation
    float *values = new float[n_threads * n_orientations * state_size];
    UTIL::SPATIAL::AnytimeAStar **planners = new UTIL::SPATIAL::AnytimeAStar *[n_threads];
    for (int i = 0; i < n_threads; ++i)
    {
        planners[i] = new UTIL::SPATIAL::AnytimeAStar(env, n_orientations, *map_weights);
    }
    
    refine_with_no_rotation_astar(values, planners, end_time, has_time_limit, start, phase3_start);
    
    // ========================================
    // PHASE 4: A* with rotation (most accurate, 4× state space)
    // ========================================
    if (has_time_limit && std::chrono::steady_clock::now() >= end_time) {
        cout << "Time limit reached. Using rotation-free A* heuristics." << endl;
        delete[] values;
        for (int i = 0; i < n_threads; ++i) { delete planners[i]; }
        delete planners;
        return;
    }
    
    auto phase4_start = std::chrono::steady_clock::now();
    cout << "Phase 4: A* refinement with rotation (most accurate)..." << endl;
    refine_with_weighted_astar(values, planners, end_time, 1.0f, has_time_limit, start, phase4_start);

    delete[] values;  // Clean up
    for (int i = 0; i < n_threads; ++i)
    {
        delete planners[i];
    }
    delete planners;
}

// ====================================================================================================
// HeuristicTable::_compute_weighted_heuristics - Compute heuristics for one start location
// ====================================================================================================
// Internal method to compute heuristics from a single start location to all others.
// Handles both rotation-aware and rotation-ignorant cases.
// Uses anytime A* with time limits - falls back to Manhattan distance for uncomputed states.
// ====================================================================================================
void HeuristicTable::_compute_heuristics_no_rotation(
    int start_loc_idx,
    float *values,
    UTIL::SPATIAL::AnytimeAStar *planner,
    TimePoint end_time)
{
    int start_loc = empty_locs[start_loc_idx];
    
    // Use orientation = -1 for rotation-free A*
    planner->reset();
    planner->search_for_all(start_loc, -1, end_time);
    
    // Update main heuristics from rotation-free search
    // In no-rotation mode, state index = location (orientation is -1)
    for (int loc_idx = 0; loc_idx < loc_size; ++loc_idx)
    {
        int target_loc = empty_locs[loc_idx];
        UTIL::SPATIAL::State *target_state = planner->all_states + target_loc;
        
        if (target_state->pos != -1)  // State was reached
        {
            size_t main_idx = start_loc_idx * loc_size + loc_idx;
            float old_value = main_heuristics[main_idx];
            float new_value = target_state->g;
            
            // Only update if A* found a better (lower) value
            if (new_value < old_value)
            {
                main_heuristics[main_idx] = new_value;
            }
        }
    }
}

void HeuristicTable::_compute_weighted_heuristics(
    int start_loc_idx,
    float *values,
    UTIL::SPATIAL::AnytimeAStar *planner,
    TimePoint end_time)
{
    int start_loc = empty_locs[start_loc_idx];  // Get start location
    // Always use rotation-aware mode
    {
        std::fill(values, values + n_orientations * state_size, MAX_HEURISTIC);  // Initialize values
        TimePoint cached_time = std::chrono::steady_clock::now();  // Cache time for this location
        
        for (int start_orient = 0; start_orient < n_orientations; ++start_orient)  // For each start orientation
        {
            // Check time limit before each orientation (like competition framework)
            // Cache time check to avoid repeated expensive calls
            if (end_time != TimePoint::max()) {
                cached_time = std::chrono::steady_clock::now();
                if (cached_time > end_time) {
                    break;  // Stop if time limit exceeded
                }
            }
            
            planner->reset();
            planner->search_for_all(start_loc, start_orient, end_time);  // Search with orientation and end time
            
            for (int loc_idx = 0; loc_idx < loc_size; ++loc_idx)  // For each target location
            {
                for (int orient = 0; orient < n_orientations; ++orient)  // For each target orientation
                {
                    int loc = empty_locs[loc_idx];
                    UTIL::SPATIAL::State *state = planner->all_states + loc * n_orientations + orient;  // Get state
                    float cost = state->g;
                    
                    // If unreachable or not computed, use Manhattan distance as fallback
                    if (cost == -1)
                    {
                        cost = compute_manhattan_distance(start_loc, loc);
                        // Add orientation penalty for fallback (rough estimate)
                        if (cost < MAX_HEURISTIC - 3.0f) {
                            cost += 1.0f;  // Conservative penalty for orientation differences
                        }
                    }
                    
                    if (cost > MAX_HEURISTIC)  // Sanity check
                    {
                        cost = MAX_HEURISTIC;
                    }

                    size_t value_idx = start_orient * state_size + loc_idx * n_orientations + orient;  // Value index
                    values[value_idx] = cost;  // Store cost

                    size_t main_idx = start_loc_idx * loc_size + loc_idx;  // Main heuristic index
                    if (cost < main_heuristics[main_idx])  // Update minimum
                    {
                        main_heuristics[main_idx] = cost;
                    }
                }
            }
        }

        // Compute sub-heuristics (orientation-dependent costs)
        for (int start_orient = 0; start_orient < n_orientations; ++start_orient)
        {
            for (int loc_idx = 0; loc_idx < loc_size; ++loc_idx)
            {
                float cost = MAX_HEURISTIC;  // Minimum cost for this start orientation
                for (int orient = 0; orient < n_orientations; ++orient)
                {
                    size_t value_idx = start_orient * state_size + loc_idx * n_orientations + orient;
                    auto value = values[value_idx];
                    if (value < cost)
                    {
                        cost = value;
                    }
                }
                size_t sub_idx = (start_loc_idx * loc_size + loc_idx) * n_orientations + start_orient;  // Sub-heuristic index
                size_t main_idx = start_loc_idx * loc_size + loc_idx;
                float diff = cost - main_heuristics[main_idx];  // Difference from main heuristic
                if (diff < 0)  // Sanity check
                {
                    diff = 0;  // Clamp to zero instead of error
                }

                if (diff > MAX_HEURISTIC)  // Sanity check
                {
                    diff = MAX_HEURISTIC;
                }
                sub_heuristics[sub_idx] = diff;  // Store sub-heuristic
            }
        }
    }
}

// ====================================================================================================
// HeuristicTable::_compute_weighted_heuristics_weighted - Compute heuristics with weighted A*
// ====================================================================================================
// Uses weighted A* (f = g + weight * h) for faster heuristic computation.
// Weight > 1.0 makes A* expand fewer nodes but still provides admissible heuristics.
// ====================================================================================================
void HeuristicTable::_compute_weighted_heuristics_weighted(
    int start_loc_idx,
    float *values,
    UTIL::SPATIAL::AnytimeAStar *planner,
    TimePoint end_time,
    float weight)
{
    int start_loc = empty_locs[start_loc_idx];
    // Always use rotation-aware mode
    {
        std::fill(values, values + n_orientations * state_size, MAX_HEURISTIC);
        TimePoint cached_time = std::chrono::steady_clock::now();  // Cache time for this location
        
        for (int start_orient = 0; start_orient < n_orientations; ++start_orient)
        {
            // Cache time check to avoid repeated expensive calls
            cached_time = std::chrono::steady_clock::now();
            if (cached_time > end_time) {
                break;
            }
            
            planner->reset();
            planner->search_for_all_weighted(start_loc, start_orient, end_time, weight);
            
            for (int loc_idx = 0; loc_idx < loc_size; ++loc_idx)
            {
                for (int orient = 0; orient < n_orientations; ++orient)
                {
                    int loc = empty_locs[loc_idx];
                    UTIL::SPATIAL::State *state = planner->all_states + loc * n_orientations + orient;
                    float cost = state->g;
                    
                    if (cost == -1)
                    {
                        cost = compute_manhattan_distance(start_loc, loc);
                        if (cost < MAX_HEURISTIC - 3.0f) {
                            cost += 1.0f;
                        }
                    }
                    
                    if (cost > MAX_HEURISTIC) {
                        cost = MAX_HEURISTIC;
                    }

                    size_t value_idx = start_orient * state_size + loc_idx * n_orientations + orient;
                    values[value_idx] = cost;

                    size_t main_idx = start_loc_idx * loc_size + loc_idx;
                    if (cost < main_heuristics[main_idx])
                    {
                        main_heuristics[main_idx] = cost;
                    }
                }
            }
        }

        // Compute sub-heuristics (orientation-dependent costs)
        for (int start_orient = 0; start_orient < n_orientations; ++start_orient)
        {
            for (int loc_idx = 0; loc_idx < loc_size; ++loc_idx)
            {
                float cost = MAX_HEURISTIC;
                for (int orient = 0; orient < n_orientations; ++orient)
                {
                    size_t value_idx = start_orient * state_size + loc_idx * n_orientations + orient;
                    auto value = values[value_idx];
                    if (value < cost)
                    {
                        cost = value;
                    }
                }
                size_t sub_idx = (start_loc_idx * loc_size + loc_idx) * n_orientations + start_orient;
                size_t main_idx = start_loc_idx * loc_size + loc_idx;
                float diff = cost - main_heuristics[main_idx];
                if (diff < 0) {
                    diff = 0;
                }

                if (diff > MAX_HEURISTIC) {
                    diff = MAX_HEURISTIC;
                }
                sub_heuristics[sub_idx] = diff;
            }
        }
    }
}

// ====================================================================================================
// HeuristicTable::dump_main_heuristics - Dump heuristics to CSV file
// ====================================================================================================
// Outputs the main heuristic table for a given start location to a CSV file.
// Useful for debugging and visualization.
// ====================================================================================================
void HeuristicTable::dump_main_heuristics(int start_loc, string file_path_prefix)
{
    int start_loc_idx = loc_idxs[start_loc];  // Get start location index
    if (start_loc_idx == -1)  // Invalid location
    {
        std::cerr << "error: start_loc_idx==-1" << endl;
        exit(-1);
    }
    string file_path = file_path_prefix + "_" + std::to_string(start_loc) + ".main_heuristics";  // File path
    std::ofstream fout(file_path);  // Open file
    for (int i = 0; i < env.rows; ++i)  // For each row
    {
        for (int j = 0; j < env.cols; ++j)  // For each column
        {
            int target_loc = i * env.cols + j;  // Target location
            int target_loc_idx = loc_idxs[target_loc];  // Target index
            float h = MAX_HEURISTIC;  // Default heuristic
            if (target_loc_idx != -1)  // If valid location
            {
                h = main_heuristics[start_loc_idx * loc_size + target_loc_idx];  // Get heuristic
            }
            fout << h;  // Write to file
            if (j != env.cols - 1)
            {
                fout << ",";  // Comma separator
            }
        }
        fout << endl;  // New line
    }
}

// ====================================================================================================
// HeuristicTable::get - Get heuristic between two locations
// ====================================================================================================
// Returns the precomputed heuristic distance between two locations.
// Returns MAX_HEURISTIC if either location is invalid.
// ====================================================================================================
float HeuristicTable::get(int loc1, int loc2)
{
    int loc_idx1 = loc_idxs[loc1];  // Index for loc1
    int loc_idx2 = loc_idxs[loc2];  // Index for loc2

    if (loc_idx1 == -1 || loc_idx2 == -1)  // Invalid locations
    {
        return MAX_HEURISTIC;
    }

    size_t idx = loc_idx1 * loc_size + loc_idx2;  // Table index
    return main_heuristics[idx];  // Return heuristic
}

// ====================================================================================================
// HeuristicTable::get - Get heuristic with orientation
// ====================================================================================================
// Returns the heuristic distance considering start orientation.
// Falls back to location-only heuristic if rotations are not considered.
// ====================================================================================================
float HeuristicTable::get(int loc1, int orient1, int loc2)
{
    // Always use rotation-aware heuristic
    int loc_idx1 = loc_idxs[loc1];  // Index for loc1
    int loc_idx2 = loc_idxs[loc2];  // Index for loc2

    if (loc_idx1 == -1 || loc_idx2 == -1)  // Invalid locations
    {
        return MAX_HEURISTIC;
    }

    size_t main_idx = loc_idx1 * loc_size + loc_idx2;  // Main heuristic index
    size_t sub_idx = (loc_idx1 * loc_size + loc_idx2) * n_orientations + orient1;  // Sub-heuristic index
    return main_heuristics[main_idx] + sub_heuristics[sub_idx];  // Combined heuristic
}

// ====================================================================================================
// HeuristicTable::_compute_heuristics_bfs - BFS for one start location
// ====================================================================================================
// Computes accurate distances from one start location using BFS (breadth-first search).
// BFS is much faster than A* and respects walls, making it perfect for irregular maps.
// ====================================================================================================
void HeuristicTable::_compute_heuristics_bfs(int start_loc_idx, TimePoint end_time)
{
    int start_loc = empty_locs[start_loc_idx];
    
    // Simple BFS using deque
    std::deque<std::pair<int, float>> queue;
    std::vector<bool> visited(env.rows * env.cols, false);
    
    queue.push_back({start_loc, 0.0f});
    visited[start_loc] = true;
    
    while (!queue.empty())
    {
        auto [curr_loc, curr_cost] = queue.front();
        queue.pop_front();
        
        // Check time limit periodically
        if (queue.size() % 1000 == 0 && std::chrono::steady_clock::now() > end_time) {
            return;
        }
        
        // Update heuristic if this location is in our empty_locs list
        int curr_loc_idx = loc_idxs[curr_loc];
        if (curr_loc_idx != -1)
        {
            size_t main_idx = start_loc_idx * loc_size + curr_loc_idx;
            
            // Always update - BFS is wall-aware and more accurate than Manhattan
            // BFS distances are typically >= Manhattan since walls require detours
            main_heuristics[main_idx] = curr_cost;
        }
        
        // Expand neighbors (4-connected grid)
        int row = curr_loc / env.cols;
        int col = curr_loc % env.cols;
        
        // Try all 4 directions
        int drow[] = {-1, 1, 0, 0};
        int dcol[] = {0, 0, -1, 1};
        
        for (int dir = 0; dir < 4; ++dir)
        {
            int new_row = row + drow[dir];
            int new_col = col + dcol[dir];
            
            if (new_row < 0 || new_row >= env.rows || new_col < 0 || new_col >= env.cols)
                continue;
            
            int neighbor_loc = new_row * env.cols + new_col;
            
            // Check if neighbor is traversable and not visited
            if (env.map[neighbor_loc] == 0 && !visited[neighbor_loc])
            {
                visited[neighbor_loc] = true;
                queue.push_back({neighbor_loc, curr_cost + 1.0f});
            }
        }
    }
}

// ====================================================================================================
// HeuristicTable::refine_with_bfs - Refine all heuristics using BFS
// ====================================================================================================
// Performs parallel BFS from all start locations to compute wall-aware distances.
// This is much faster than A* and ensures all locations have accurate heuristics.
// ====================================================================================================
void HeuristicTable::refine_with_bfs(TimePoint end_time, bool has_time_limit, TimePoint start, TimePoint phase_start)
{
    int ctr = 0;
    int step = 100;
    bool time_limit_exceeded = false;

#pragma omp parallel for schedule(dynamic, 1)
    for (int loc_idx = 0; loc_idx < loc_size; ++loc_idx)
    {
        if (has_time_limit) {
            #pragma omp flush(time_limit_exceeded)
            if (time_limit_exceeded || std::chrono::steady_clock::now() > end_time) {
                #pragma omp atomic write
                time_limit_exceeded = true;
                continue;
            }
        }
        
        _compute_heuristics_bfs(loc_idx, end_time);

#pragma omp critical
        {
            ++ctr;
            if (ctr % step == 0)
            {
                auto now = std::chrono::steady_clock::now();
                double elapse = std::chrono::duration<double>(now - start).count();
                double phase_elapse = std::chrono::duration<double>(now - phase_start).count();
                cout << ctr << "/" << loc_size << " refined with BFS in " << phase_elapse << "s (total: " << elapse << "s)" << endl;
                
                if (has_time_limit) {
                    double remaining = std::chrono::duration<double>(end_time - now).count();
                    if (remaining < 1.0) {
                        cout << "Less than 1s remaining, stopping BFS refinement early" << endl;
                        #pragma omp atomic write
                        time_limit_exceeded = true;
                    }
                }
            }
        }
    }
    
    auto final_time = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration<double>(final_time - start).count();
    
    if (time_limit_exceeded) {
        cout << "BFS refinement stopped early. Refined " << ctr << "/" << loc_size << " locations in " << total_time << "s." << endl;
    } else {
        cout << "All " << loc_size << " locations refined with BFS in " << total_time << "s." << endl;
    }
}

// ====================================================================================================
// HeuristicTable::compute_manhattan_distance - Compute Manhattan distance between locations
// ====================================================================================================
// Calculates the Manhattan (L1) distance between two grid locations.
// Used as admissible heuristic fallback when A* cannot reach a location.
// ====================================================================================================
float HeuristicTable::compute_manhattan_distance(int loc1, int loc2) const
{
    int x1 = loc1 / env.cols;
    int y1 = loc1 % env.cols;
    int x2 = loc2 / env.cols;
    int y2 = loc2 % env.cols;
    return abs(x1 - x2) + abs(y1 - y2);
}

// ====================================================================================================
// HeuristicTable::refine_with_weighted_astar - Refine heuristics using weighted A*
// ====================================================================================================
// Performs parallel weighted A* refinement on locations that still have Manhattan heuristics.
// Uses the specified weight for faster computation while maintaining admissibility.
// ====================================================================================================
void HeuristicTable::refine_with_no_rotation_astar(float *values, UTIL::SPATIAL::AnytimeAStar **planners, TimePoint end_time, bool has_time_limit, TimePoint start, TimePoint phase_start)
{
    int ctr = 0;
    int step = 100;
    bool time_limit_exceeded = false;

#pragma omp parallel for schedule(dynamic, 1)
    for (int loc_idx = 0; loc_idx < loc_size; ++loc_idx)
    {
        if (has_time_limit) {
            #pragma omp flush(time_limit_exceeded)
            if (time_limit_exceeded || std::chrono::steady_clock::now() > end_time) {
                #pragma omp atomic write
                time_limit_exceeded = true;
                continue;
            }
        }
        
        int thread_id = omp_get_thread_num();
        int s_idx = thread_id * n_orientations * state_size;
        _compute_heuristics_no_rotation(loc_idx, values + s_idx, planners[thread_id], end_time);

#pragma omp critical
        {
            ++ctr;
            if (ctr % step == 0)
            {
                auto now = std::chrono::steady_clock::now();
                double elapse = std::chrono::duration<double>(now - start).count();
                double phase_elapse = std::chrono::duration<double>(now - phase_start).count();
                cout << ctr << "/" << loc_size << " refined with A* (no rotation) in " << phase_elapse << "s (total: " << elapse << "s)" << endl;
                
                if (has_time_limit) {
                    double remaining = std::chrono::duration<double>(end_time - now).count();
                    if (remaining < 1.0) {
                        cout << "Less than 1s remaining, stopping refinement early" << endl;
                        #pragma omp atomic write
                        time_limit_exceeded = true;
                    }
                }
            }
        }
    }
    
    auto final_time = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration<double>(final_time - start).count();
    
    if (time_limit_exceeded) {
        cout << "A* (no rotation) refinement stopped early. Refined " << ctr << "/" << loc_size << " locations in " << total_time << "s." << endl;
    } else {
        cout << "All " << loc_size << " locations refined with A* (no rotation) in " << total_time << "s." << endl;
    }
}

void HeuristicTable::refine_with_weighted_astar(float *values, UTIL::SPATIAL::AnytimeAStar **planners, TimePoint end_time, float weight, bool has_time_limit, TimePoint start, TimePoint phase_start)
{
    int ctr = 0;
    int step = 100;
    bool time_limit_exceeded = false;

#pragma omp parallel for schedule(dynamic, 1)
    for (int loc_idx = 0; loc_idx < loc_size; ++loc_idx)
    {
        if (has_time_limit) {
            #pragma omp flush(time_limit_exceeded)
            if (time_limit_exceeded || std::chrono::steady_clock::now() > end_time) {
                #pragma omp atomic write
                time_limit_exceeded = true;
                continue;
            }
        }
        
        int thread_id = omp_get_thread_num();
        int s_idx = thread_id * n_orientations * state_size;
        _compute_weighted_heuristics_weighted(loc_idx, values + s_idx, planners[thread_id], end_time, weight);

#pragma omp critical
        {
            ++ctr;
            if (ctr % step == 0)
            {
                auto now = std::chrono::steady_clock::now();
                double elapse = std::chrono::duration<double>(now - start).count();
                double phase_elapse = std::chrono::duration<double>(now - phase_start).count();
                cout << ctr << "/" << loc_size << " refined with A* (rotation, w=" << weight << ") in " << phase_elapse << "s (total: " << elapse << "s)" << endl;
                
                if (has_time_limit) {
                    double time_remaining = std::chrono::duration<double>(end_time - now).count();
                    if (time_remaining < 1.0) {
                        cout << "Less than 1s remaining, stopping refinement early." << endl;
                        #pragma omp atomic write
                        time_limit_exceeded = true;
                    }
                }
            }
        }
    }
    
    auto final_time = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration<double>(final_time - start).count();
    
    if (time_limit_exceeded) {
        cout << "Weighted A* (w=" << weight << ") refinement stopped early. Refined " << ctr << "/" << loc_size << " locations in " << total_time << "s." << endl;
    } else {
        cout << "All " << loc_size << " locations refined with weighted A* (w=" << weight << ") in " << total_time << "s." << endl;
    }
}

// ====================================================================================================
// HeuristicTable::preprocess - Load or compute heuristics
// ====================================================================================================
// Checks if precomputed heuristics exist on disk, loads them if available,
// otherwise computes them from scratch with anytime A* respecting competition time limit.
// ====================================================================================================
void HeuristicTable::preprocess(string suffix, int preprocess_time_limit_ms)
{
    string fname = env.map_name.substr(0, env.map_name.size() - 4);  // Map name without extension
    string folder = env.file_storage_path;  // Storage folder
    if (folder[folder.size() - 1] != boost::filesystem::path::preferred_separator)  // Ensure trailing separator
    {
        folder += boost::filesystem::path::preferred_separator;
    }
    string fpath;  // File path
    // Always use rotation-aware file name
    fpath = folder + fname + "_weighted_heuristics_v4_" + suffix + ".gz";

    if (boost::filesystem::exists(fpath))  // If file exists
    {
        load(fpath);  // Load from disk
    }
    else  // Otherwise compute
    {
        // Create end time like competition framework does
        TimePoint end_time = TimePoint::max();  // Default: no time limit
        if (preprocess_time_limit_ms > 0) {
            end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(preprocess_time_limit_ms);
            cout << "Preprocess time limit: " << preprocess_time_limit_ms << "ms" << endl;
        }
        compute_weighted_heuristics(end_time);  // Compute heuristics with end time
    }
}

// ====================================================================================================
// HeuristicTable::save - Save heuristics to compressed file
// ====================================================================================================
// Serializes the heuristic tables to a compressed binary file for later reuse.
// Uses zlib compression for efficient storage.
// ====================================================================================================
void HeuristicTable::save(const string &fpath)
{
    std::ofstream fout;
    fout.open(fpath, std::ios::binary | std::ios::out);  // Open binary file

    boost::iostreams::filtering_streambuf<boost::iostreams::output> outbuf;  // Compression buffer
    outbuf.push(boost::iostreams::zlib_compressor());  // Zlib compressor
    outbuf.push(fout);  // Output to file

    std::ostream out(&outbuf);  // Output stream

    out.write((char *)&loc_size, sizeof(int));  // Write location size
    out.write((char *)empty_locs, sizeof(int) * loc_size);  // Write empty locations
    out.write((char *)main_heuristics, sizeof(float) * loc_size * loc_size);  // Write main heuristics
    out.write((char *)sub_heuristics, sizeof(float) * state_size * loc_size);  // Write sub-heuristics

    boost::iostreams::close(outbuf);  // Close compression
    fout.close();  // Close file
}

// ====================================================================================================
// HeuristicTable::load - Load heuristics from compressed file
// ====================================================================================================
// Deserializes the heuristic tables from a compressed binary file.
// Verifies compatibility with current environment.
// ====================================================================================================
void HeuristicTable::load(const string &fpath)
{
    std::ifstream fin;
    fin.open(fpath, std::ios::binary | std::ios::in);  // Open binary file

    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;  // Decompression buffer
    inbuf.push(boost::iostreams::zlib_decompressor());  // Zlib decompressor
    inbuf.push(fin);  // Input from file

    std::istream in(&inbuf);  // Input stream

    int _loc_size;  // Read location size
    in.read((char *)&_loc_size, sizeof(int));

    if (_loc_size != loc_size)  // Verify size matches
    {
        cerr << "the sizes of empty locations don't match!" << endl;
        exit(-1);
    }

    int *_empty_locs = new int[loc_size];  // Read empty locations
    in.read((char *)_empty_locs, sizeof(int) * loc_size);

    for (auto i = 0; i < loc_size; ++i)  // Verify empty locations match
    {
        if (_empty_locs[i] != empty_locs[i])
        {
            cerr << "the empty locations don't match!" << endl;
            exit(-1);
        }
    }

    in.read((char *)main_heuristics, sizeof(float) * loc_size * loc_size);  // Read main heuristics
    in.read((char *)sub_heuristics, sizeof(float) * state_size * loc_size);  // Read sub-heuristics

    delete[] _empty_locs;  // Clean up

}
