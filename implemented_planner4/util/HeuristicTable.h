#pragma once
#include "common.h"
#include "SharedEnv.h"
#include <omp.h>
#include <chrono>
#include <cstdlib>
#include <random>
#include "util/ActionModel.h"
#include "boost/filesystem.hpp"
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include "boost/format.hpp"
#include "util/Utils.h"

#define MAX_HEURISTIC FLT_MAX/16  // Maximum heuristic value to avoid overflow

// ====================================================================================================
// UTIL::SPATIAL Namespace - Spatial A* and Heuristic Table Implementation
// ====================================================================================================
// This namespace contains classes for spatial pathfinding and heuristic computation.
// It includes a custom A* implementation for grid-based environments with weighted edges,
// and a heuristic table that precomputes distances between all pairs of locations.
// Supports both rotation-aware and rotation-ignorant modes.
// ====================================================================================================

namespace UTIL {
namespace SPATIAL {

// ====================================================================================================
// State Struct - Represents a state in the search space
// ====================================================================================================
// Optimized layout: group hot data together for better cache line utilization
struct State {
    float f;        // Total cost (g + h) - most accessed in comparisons
    float g;        // Cost from start
    float h;        // Heuristic cost
    int pos;        // Position index
    int orientation;     // Orientation (0-3 for directions, -1 for no orientation)
    int heap_index;  // Index in the heap for priority queue
    State * prev;   // Previous state in path
    bool closed;    // Whether this state is closed in A*

    State(): f(-1), g(-1), h(0), pos(-1), orientation(-1), heap_index(-1), prev(nullptr), closed(false) {};  // Default constructor
    State(int pos, int orient, float g, float h, State * prev): f(g+h), g(g), h(h), pos(pos), orientation(orient), heap_index(-1), prev(prev), closed(false) {};  // Parameterized constructor

    // Copy state data (optimized order matching struct layout)
    void copyStateData(const State * state) {
        f = state->f;
        g = state->g;
        h = state->h;
        pos = state->pos;
        orientation = state->orientation;
        prev = state->prev;
    }
};

// ====================================================================================================
// OpenList Class - Priority queue for A* search
// ====================================================================================================
class OpenList {
public:
    State ** heap;  // Array of state pointers

    // Compare two states for priority (with branch prediction hints)
    inline bool is_better(const State * __restrict__ s1, const State * __restrict__ s2) {
        // Most comparisons have different f values - optimize for that case
        if (__builtin_expect(s1->f != s2->f, 1)) {
            return s1->f < s2->f;  // Lower f is better
        }
        return s1->h < s2->h;  // Tie-break on heuristic
    }

    int max_size;  // Maximum size of the heap
    int size;      // Current size

    OpenList(int _max_size): max_size(_max_size), size(0) {  // Constructor
        heap = new State * [max_size];
    };

    ~OpenList(){  // Destructor
        delete [] heap;
    };

    // Swap two states in the heap (optimized with fewer memory operations)
    inline void swap(State * __restrict__ s, State * __restrict__ p) {
        int i = s->heap_index;
        int j = p->heap_index;
        heap[i] = p;
        heap[j] = s;
        s->heap_index = j;
        p->heap_index = i;
    }

    // Move state up the 4-ary heap (optimized: fewer levels, better cache locality)
    inline void move_up(State *s) {
        int heap_index = s->heap_index;
        while (heap_index > 0) {
            int parent_heap_index = (heap_index - 1) / 4;  // 4-ary heap parent
            if (is_better(heap[parent_heap_index], heap[heap_index])) {
                break;
            }
            swap(s, heap[parent_heap_index]);
            heap_index = parent_heap_index;
        }
    }

    // Move state down the 4-ary heap (optimized: find best of 4 children at once)
    inline void move_down(State * s) {
        int heap_index = s->heap_index;
        while (true) {
            int first_child = heap_index * 4 + 1;  // First child in 4-ary heap
            if (first_child >= size) break;  // No children
            
            // Find best child among up to 4 children
            int best_child = first_child;
            int last_child = (first_child + 4 < size) ? first_child + 4 : size;
            
            for (int child = first_child + 1; child < last_child; ++child) {
                if (is_better(heap[child], heap[best_child])) {
                    best_child = child;
                }
            }
            
            if (is_better(heap[heap_index], heap[best_child])) {
                break;
            }
            swap(s, heap[best_child]);
            heap_index = best_child;
        }
    }

    // Push a state onto the heap
    void push(State * s) {
        heap[size] = s;
        s->heap_index = size;
        ++size;
        move_up(s);
    }

    // Pop the best state from the heap
    State * pop() {
        State * ret = heap[0];
        --size;
        heap[0] = heap[size];
        heap[0]->heap_index = 0;
        move_down(heap[0]);
        return ret;
    }

    // Get the top state without popping
    State * top() {
        return heap[0];
    }

    // Increase priority of a state (when cost decreases)
    void increase(State * s) {
        move_up(s);
    }   

    // Check if heap is empty
    inline bool empty() {
        return size==0;
    }

    // Check if heap is full
    inline bool full() {
        return size==max_size;
    }

    // Clear the heap
    void clear() {
        size = 0;
    }
};

// ====================================================================================================
// SpatialAStar Class - A* search implementation for spatial heuristics
// ====================================================================================================
class AnytimeAStar
{
public:
    // Constructor
    AnytimeAStar(const SharedEnvironment &env, int n_orients, const std::vector<float> &weights) 
        : env(env), n_orients(n_orients), weights(weights)
    {
        max_states = env.rows * env.cols * n_orients;  // Maximum possible states
        n_states = 0;
        open_list = new OpenList(max_states);
        all_states = new State[max_states];
        max_successors = 8;  // Maximum successors per state
        successors = new State[max_successors];
        reset();
    };

    // Reset the search state
    void reset()
    {
        open_list->clear();
        n_states = 0;
        for (int i = 0; i < max_states; ++i)
        {
            if (all_states[i].pos != -1)
            {
                all_states[i].pos = -1;
                all_states[i].orientation = -1;
                all_states[i].g = -1;
                all_states[i].h = 0;
                all_states[i].f = -1;
                all_states[i].prev = nullptr;
            }
        }
        n_successors = 0;
    }

    // Destructor
    ~AnytimeAStar()
    {
        delete open_list;
        delete[] all_states;
        delete[] successors;
    }

    int n_orients;      // Number of orientations
    int n_states;       // Current number of states
    int max_states;     // Maximum states
    OpenList *open_list; // Priority queue
    State *all_states;   // Array of all states
    const int n_dirs = 5; // Number of directions (4 moves + wait)

    int n_successors;     // Current number of successors
    int max_successors;   // Maximum successors
    State *successors;    // Array of successor states

    const SharedEnvironment &env;  // Reference to environment
    const std::vector<float> &weights;  // Edge weights

    // Clear successors list
    void clear_successors() {
        n_successors = 0;
    }

    // Add a successor state
    void add_successor(int pos, int orient, float g, float h, State *prev)
    {
        successors[n_successors].pos = pos;
        successors[n_successors].orientation = orient;
        successors[n_successors].g = g;
        successors[n_successors].h = h;
        successors[n_successors].f = g + h;
        successors[n_successors].prev = prev;
        ++n_successors;
    }

    // Get successors for current state
    void get_successors(State *curr);
    
    // Add a new state to the search
    State *add_state(int pos, int orient, float g, float h, State *prev);
    
    // Search for all reachable states from start (with optional end time)
    void search_for_all(int start_pos, int start_orient, TimePoint end_time = TimePoint::max());
    
    // Search for all reachable states with weighted A* (f = g + weight * h)
    void search_for_all_weighted(int start_pos, int start_orient, TimePoint end_time, float weight);
};

}
}

// ====================================================================================================
// HeuristicTable Class - Precomputed distance heuristics
// ====================================================================================================
// This class precomputes and stores heuristic distances between all pairs of locations.
// It uses spatial A* to compute weighted distances, considering or ignoring rotations.
// The table can be saved/loaded from disk for efficiency.
// ====================================================================================================
class HeuristicTable {
public:
    const SharedEnvironment & env;  // Reference to environment
    CompetitionActionModelWithRotate action_model;  // Action model
    float * main_heuristics;  // Main heuristic table (location to location)
    float * sub_heuristics;   // Sub-heuristics for orientations
    int * empty_locs;         // Array of empty location indices
    int * loc_idxs;           // Mapping from location to index
    int n_orientations=4;     // Number of orientations (always 4 for rotation-aware)
    size_t loc_size=0;        // Number of empty locations
    size_t state_size;        // Total state size

    std::shared_ptr<std::vector<float> > map_weights;  // Map weights

    // Constructor
    HeuristicTable(SharedEnvironment * _env, const std::shared_ptr<std::vector<float> > & map_weights);
    
    // Destructor
    ~HeuristicTable();

    // Compute weighted heuristics (with optional end time)
    void compute_weighted_heuristics(TimePoint end_time);
    
    // Internal computation for one start location
    void _compute_weighted_heuristics(int start_loc_idx, float * values, UTIL::SPATIAL::AnytimeAStar * planner, TimePoint end_time);
    
    // Internal computation with weighted A* for faster refinement
    void _compute_weighted_heuristics_weighted(int start_loc_idx, float * values, UTIL::SPATIAL::AnytimeAStar * planner, TimePoint end_time, float weight);
    
    // Internal computation without rotation awareness (4× faster)
    void _compute_heuristics_no_rotation(int start_loc_idx, float * values, UTIL::SPATIAL::AnytimeAStar * planner, TimePoint end_time);
    
    // Refine heuristics with BFS (fast, wall-aware)
    void refine_with_bfs(TimePoint end_time, bool has_time_limit, TimePoint start, TimePoint phase_start);
    
    // Internal BFS computation for one start location
    void _compute_heuristics_bfs(int start_loc_idx, TimePoint end_time);
    
    // Refine heuristics with rotation-free A* (faster than rotation-aware)
    void refine_with_no_rotation_astar(float *values, UTIL::SPATIAL::AnytimeAStar **planners, TimePoint end_time, bool has_time_limit, TimePoint start, TimePoint phase_start);
    
    // Refine heuristics with weighted A* (multi-level anytime)
    void refine_with_weighted_astar(float *values, UTIL::SPATIAL::AnytimeAStar **planners, TimePoint end_time, float weight, bool has_time_limit, TimePoint start, TimePoint phase_start);
    
    // Compute Manhattan distance as fallback heuristic
    float compute_manhattan_distance(int loc1, int loc2) const;
    
    // Dump main heuristics to file
    void dump_main_heuristics(int start_loc, string file_path_prefix);
    
    // Get heuristic between two locations
    float get(int loc1, int loc2);
    
    // Get heuristic with orientation
    float get(int loc1, int orient1, int loc2);

    // Preprocess: load or compute heuristics (with optional time limit in milliseconds)
    void preprocess(string suffix="", int preprocess_time_limit_ms = -1);
    
    // Save heuristics to file
    void save(const string & fpath);
    
    // Load heuristics from file
    void load(const string & fpath);
};
