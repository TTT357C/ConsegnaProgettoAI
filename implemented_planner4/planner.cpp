#include "planner.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "pibt.h"
#include "flow.h"
#include "const.h"
#include <fstream>
#include <iostream>

// Include LaCAM2 and utilities after planner.h to avoid naming conflicts
#include "LaCAM2/lacam2_core.hpp"
#include "util/HeuristicTable.h"
#include "LNS/lns_core.h"
#include "config.hpp"

namespace DefaultPlanner{

    // LaCAM2 solver and related structures (using global HeuristicTable from util)
    std::shared_ptr<LaCAM2::LaCAM2Solver> lacam2_solver;
    std::shared_ptr<::HeuristicTable> lacam2_heuristics;
    std::shared_ptr<std::vector<float>> map_weights;

    // LNS solver
    std::shared_ptr<LNS::LNSSolver> lns_solver;
    bool use_lns = false;

    //default planner data
    std::vector<int> decision;
    std::vector<int> prev_decision;
    std::vector<double> p;
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<double> p_copy;
    std::vector<bool> occupied;
    std::vector<DCR> decided;
    std::vector<bool> checked;
    std::vector<bool> require_guide_path;
    std::vector<int> dummy_goals;
    TrajLNS trajLNS;
    std::mt19937 mt1;

    // ====================================================================================================
    // INITIALIZATION SECTION
    // ====================================================================================================

    /**
     * @brief Default planner initialization
     *
     * @param preprocess_time_limit time limit for preprocessing in milliseconds
     * @param env shared environment object
     *
     * The initialization function initializes the default planner data structures and heuristics tables.
     */
    void initialize(int preprocess_time_limit, SharedEnvironment* env){
            //initialise all required data structures
            assert(env->num_of_agents != 0);
            p.resize(env->num_of_agents);
            decision.resize(env->map.size(), -1);
            prev_states.resize(env->num_of_agents);
            next_states.resize(env->num_of_agents);
            decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
            occupied.resize(env->map.size(),false);
            checked.resize(env->num_of_agents,false);
            ids.resize(env->num_of_agents);
            require_guide_path.resize(env->num_of_agents,false);
            for (int i = 0; i < ids.size();i++){
                ids[i] = i;
            }

            // initialise the heuristics tables containers
            init_heuristics(env);
            mt1.seed(0);
            srand(0);

            new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
            trajLNS.init_mem();

            //assign intial priority to each agent
            std::shuffle(ids.begin(), ids.end(), mt1);
            for (int i = 0; i < ids.size();i++){
                p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
            }
            p_copy = p;

            std::cout << "Agents: " << env->num_of_agents << ", Map: " << env->rows << "x" << env->cols << std::endl;

            
            // Other maps: use LNS
            use_lns = true;
            std::cout << "Using LNS planner" << std::endl;
            
            // Initialize map weights
                map_weights = std::make_shared<std::vector<float>>(env->rows * env->cols * 5, 1.0f);
                
                // Always use rotation-aware heuristics (hardcoded)
                lacam2_heuristics = std::make_shared<::HeuristicTable>(env, map_weights);
                
                // Allocate only 60% of preprocess time for heuristics, leave 40% for other preprocessing
                int heuristic_time_limit_ms = preprocess_time_limit * 0.9;
                lacam2_heuristics->preprocess("all_one", heuristic_time_limit_ms);  // Use suffix matching map_weights initialization with time limit
                
                // Create LaCAM2 solver first (needed by LNS)
                int max_agents_in_use = env->num_of_agents;
                bool disable_corner_target_agents = false;
                int max_task_completed = 1000000;
                
                lacam2_solver = std::make_shared<LaCAM2::LaCAM2Solver>(
                    lacam2_heuristics, env, map_weights, 
                    max_agents_in_use, disable_corner_target_agents,
                    max_task_completed
                );
                lacam2_solver->initialize(*env);
                
                // Create LNS solver
                if (use_lns) {
                    lns_solver = std::make_shared<LNS::LNSSolver>(
                        lacam2_heuristics, env, map_weights,
                        lacam2_solver, max_task_completed
                    );
                    lns_solver->initialize(*env);
                    std::cout << "LNS solver initialized with LaCAM2" << std::endl;
                } else {
                    std::cout << "LaCAM2 solver initialized" << std::endl;
                }
            
            return;
    };

    // ====================================================================================================
    // PLANNING SECTION
    // ====================================================================================================

    /**
     * @brief Default planner plan function - Main planning algorithm
     *
     * This is the core planning function that orchestrates the multi-agent pathfinding.
     * The function implements a hierarchical planning approach:
     * 1. Try LNS solver for large/complex problems
     * 2. Fall back to LaCAM2 for medium problems
     * 3. Fall back to PIBT (Priority Inheritance with Backtracking) for small/simple problems
     *
     * The PIBT implementation includes:
     * - Traffic flow optimization using Frank-Wolfe algorithm
     * - Guide path computation for congestion minimization
     * - Priority-based conflict resolution
     * - Turning action handling with post-processing
     *
     * @param time_limit Time limit for planning in milliseconds
     * @param actions Vector to be filled with computed actions for each agent
     * @param env Shared environment containing map, agents, and current state
     *
     * Key features:
     * - Adaptive solver selection based on map type and size
     * - Time-aware planning with preprocessing budget allocation
     * - Goal assignment handling with dummy goals for unassigned agents
     * - Incremental guide path updates for efficiency
     * - Priority adjustments for deadlock-prone situations
     * - Recursive action validation for turning constraints
     *
     * Note: The default planner ignores turning action costs in optimization,
     * post-processing turning actions as additional delays on top of the base plan.
     */
    void plan(int time_limit,vector<Action> & actions, SharedEnvironment* env){
        auto plan_start = std::chrono::steady_clock::now();
        std::cout << "[DEBUG PLAN] Timestep " << env->curr_timestep << ", Time limit: " << time_limit << "ms" << std::endl;

        // Try LNS solver first for complex problems
        if (use_lns && lns_solver) {
            std::cout << "[DEBUG PLAN] Starting LNS solver..." << std::endl;
            auto lns_start = std::chrono::steady_clock::now();
            try {
                lns_solver->observe(*env);
                auto observe_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lns_start).count();
                std::cout << "[DEBUG PLAN] LNS observe took " << observe_time << "ms" << std::endl;
                
                auto plan_start_lns = std::chrono::steady_clock::now();
                lns_solver->plan(*env, time_limit);
                auto plan_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - plan_start_lns).count();
                std::cout << "[DEBUG PLAN] LNS plan took " << plan_time << "ms" << std::endl;
                
                actions.clear();
                actions.resize(env->num_of_agents);
                lns_solver->get_step_actions(*env, actions);
                
                auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - plan_start).count();
                std::cout << "[DEBUG PLAN] LNS succeeded, total time: " << total_time << "ms" << std::endl;
                return;
            } catch (const std::exception& e) {
                auto fail_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lns_start).count();
                std::cerr << "[DEBUG PLAN] LNS failed after " << fail_time << "ms: " << e.what() << std::endl;
                // Fall through to default PIBT planner
            }
        } else {
            std::cout << "[DEBUG PLAN] LNS not available, using fallback planner" << std::endl;
        }

        // Fallback to PIBT: calculate timing constraints
        std::cout << "[DEBUG PLAN] Starting PIBT fallback planner" << std::endl;
        TimePoint start_time = std::chrono::steady_clock::now();
        //cap the time for distance to goal heuristic table initialisation to half of the given time_limit;
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
        TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
        std::cout << "[DEBUG PLAN] PIBT time budget: " << (time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE) << "ms" << std::endl; 

        // Initialize dummy goals for agents without assigned goals (use starting positions)
        if (env->curr_timestep == 0){
            dummy_goals.resize(env->num_of_agents);
            for(int i=0; i<env->num_of_agents; i++)
            {
                dummy_goals.at(i) = env->curr_states.at(i).location;
            }
        }

        // Reset previous decisions
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);

        // Update agent status and prepare for planning
        int count = 0;
        for(int i=0; i<env->num_of_agents; i++)
        {
            // Initialize heuristic tables for goal locations within time budget
            if ( ( std::chrono::steady_clock::now() < end_time) ){
                for(int j=0; j<env->goal_locations[i].size(); j++)
                {
                    int goal_loc = env->goal_locations[i][j].first;
                        if (trajLNS.heuristics.at(goal_loc).empty()){
                            init_heuristic(trajLNS.heuristics[goal_loc],env,goal_loc);
                            count++;
                        }
                }
            }
            

            // Set goal location for each agent
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                p[i] = p_copy[i];
            }
            else{
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
            }

            // Check if agent needs guide path update
            require_guide_path[i] = false;
            if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
                    require_guide_path[i] = true;
            
            // Check if agent completed previous action (turning might still be in progress)
            assert(env->curr_states[i].location >=0);
            prev_states[i] = env->curr_states[i];
            next_states[i] = State();
            prev_decision[env->curr_states[i].location] = i; 
            if (decided[i].loc == -1){
                decided[i].loc = env->curr_states[i].location;
                assert(decided[i].state == DONE::DONE);
            }
            if (prev_states[i].location == decided[i].loc){
                decided[i].state = DONE::DONE;
            }
            if (decided[i].state == DONE::NOT_DONE){
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc,-1,-1);
            }

            // Reset PIBT priority if agent reached previous goal and switched to new goal
            if(require_guide_path[i])
                p[i] = p_copy[i];
            else if (!env->goal_locations[i].empty())
                p[i] = p[i]+1;

            // Give priority bonus to agents in deadend locations
            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1){
                p[i] = p[i] + 10;
            }
            
        }

        // Compute congestion-minimized guide paths for agents that need updates
        for (int i = 0; i < env->num_of_agents;i++){
            if (std::chrono::steady_clock::now() >end_time)
                break;
            if (require_guide_path[i]){
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);
                update_traj(trajLNS, i);
            }
        }

        // Iterate and optimize guide paths using Frank-Wolfe algorithm
        std::unordered_set<int> updated;
        frank_wolfe(trajLNS, updated,end_time);

        // Sort agents by current priority for PIBT
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return p.at(a) > p.at(b);
            }
        );

        // Compute targeted next locations using PIBT with guide path heuristics
        for (int i : ids){
            if (decided[i].state == DONE::NOT_DONE){
                continue;
            }
            if (next_states[i].location==-1){
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalPIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, trajLNS);
            }
        }
        
        // Post-process targeted locations to turning/moving actions
        actions.resize(env->num_of_agents);
        for (int id : ids){
            // Clear decision table for agents with new next_states
            if (next_states.at(id).location!= -1)
                decision.at(next_states.at(id).location) = -1;
            // Record new decisions as not done yet
            if (next_states.at(id).location >=0){
                decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
            }

            // Convert targeted location to action (turning or moving)
            actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
            checked.at(id) = false;

        }

        // Recursively validate FW actions - if agent cannot move forward due to turning,
        // all agents behind must also wait
        for (int id=0;id < env->num_of_agents ; id++){
            if (!checked.at(id) && actions.at(id) == Action::FW){
                moveCheck(id,checked,decided,actions,prev_decision);
            }
        }



        // Update previous states for next planning cycle
        prev_states = next_states;
        return;

    };
}