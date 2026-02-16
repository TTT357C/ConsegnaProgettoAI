

#include "flow.h"

#include <random>
#include <unordered_set>

namespace DefaultPlanner{

std::mt19937 g(0);

// ====================================================================================================
// TRAJECTORY FLOW MANAGEMENT
// ====================================================================================================

/**
 * @brief Remove agent's trajectory from the flow network
 *
 * Decrements flow counters for each edge in the agent's path.
 * Updates sum-of-costs (SOC) metric.
 *
 * @param lns Trajectory LNS structure
 * @param agent Agent index whose trajectory to remove
 */
void remove_traj(TrajLNS& lns, int agent){
    lns.soc -= lns.trajs[agent].size() - 1;
    if (lns.trajs[agent].size() <= 1){
        return;
    }
    int loc, prev_loc, diff, d, to;

    to = lns.trajs[agent].size();

    for (int j = 1; j < to; j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);

        lns.flow[prev_loc].d[d] -= 1;
    }
}

/**
 * @brief Add agent's trajectory to the flow network
 *
 * Increments flow counters for each edge in the agent's path.
 * Updates last replan time and sum-of-costs (SOC) metric.
 *
 * @param lns Trajectory LNS structure
 * @param agent Agent index whose trajectory to add
 */
void add_traj(TrajLNS& lns, int agent){

    // Update last replan time for agent
    lns.fw_metrics[agent].last_replan_t = lns.env->curr_timestep;

    lns.soc += lns.trajs[agent].size() - 1;
    if (lns.trajs[agent].size() <= 1){
        return;
    }
    int loc, prev_loc, diff, d;
    for (int j = 1; j < lns.trajs[agent].size(); j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);

        lns.flow[prev_loc].d[d] += 1;

    }
}

// ====================================================================================================
// FLOW OPTIMIZATION METRICS
// ====================================================================================================

/**
 * @brief Compute deviation metrics for all agents
 *
 * Calculates how far each agent has deviated from their guide path.
 * Agents with higher deviation are prioritized for replanning.
 *
 * @param lns Trajectory LNS structure
 */
void get_deviation(TrajLNS& lns){
    lns.deviation_agents.clear();
    for  (int i=0; i< lns.env->num_of_agents;i++){
        if (lns.traj_dists[i].empty() || lns.trajs[i].empty())
            continue;

        std::pair<int,int> dists =  get_source_2_path(lns.traj_dists[i], lns.env, lns.env->curr_states[i].location, &(lns.neighbors));

        if (dists.first > 0){
            lns.deviation_agents.emplace_back(dists.first, i);
        }
    }

    std::sort(lns.deviation_agents.begin(), lns.deviation_agents.end(),
        [](std::pair<int,int>& a, std::pair<int,int>& b)
		{
            return a.first > b.first;
        });
    return;
}

/**
 * @brief Update Frank-Wolfe metrics for all agents
 *
 * Computes deviation and other metrics used by the Frank-Wolfe algorithm
 * to determine replanning priority.
 *
 * @param lns Trajectory LNS structure
 */
void update_fw_metrics(TrajLNS& lns){
    for  (int i=0; i< lns.env->num_of_agents;i++){
        lns.fw_metrics[i].id = i;
        lns.fw_metrics[i].rand = rand();
        lns.fw_metrics[i].deviation = 0;
        if (lns.traj_dists[i].empty() || lns.trajs[i].empty())
            continue;
        std::pair<int,int> dists =  get_source_2_path(lns.traj_dists[i], lns.env, lns.env->curr_states[i].location, &(lns.neighbors));
        assert(dists.first >= 0 );
        lns.fw_metrics[i].deviation = dists.first;
    }
    return;
}

// ====================================================================================================
// FRANK-WOLFE ALGORITHM
// ====================================================================================================

/**
 * @brief Execute Frank-Wolfe algorithm for traffic flow optimization
 *
 * Iteratively replans agent paths in order of deviation from guide paths.
 * Prioritizes agents with highest deviation, then by time since last replan,
 * then randomly. Continues until time limit is reached.
 *
 * @param lns Trajectory LNS structure
 * @param updated Set of agents whose paths were updated
 * @param timelimit Time limit for optimization
 */
void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit){
    update_fw_metrics(lns);
    std::vector<FW_Metric> replan_order = lns.fw_metrics;
    assert(replan_order.size() == lns.env->num_of_agents);

    // Sort agents by replanning priority: deviation > time since last replan > random
    std::sort(replan_order.begin(), replan_order.end(),
    [](FW_Metric& a, FW_Metric& b)
    {
        if (a.deviation > b.deviation)
            return true;
        else if ( a.deviation < b.deviation)
            return false;
        
        if (a.last_replan_t < b.last_replan_t)
            return true;
        else if (a.last_replan_t > b.last_replan_t)
            return false;

        return a.rand > b.rand;
    });

    int count=0;
    int a, index;
    while (std::chrono::steady_clock::now() < timelimit){
        index = count%lns.env->num_of_agents;
        a = replan_order[index].id;
        count++;
        if (lns.traj_dists[a].empty() || lns.trajs[a].empty()){
            continue;
        }
        remove_traj(lns,a);
        update_traj(lns,a);
        
    }
    return;

}

// ====================================================================================================
// DISTANCE COMPUTATION
// ====================================================================================================

/**
 * @brief Update distance-to-path table for agent
 * @param lns Trajectory LNS structure
 * @param i Agent index
 */
void update_dist_2_path(TrajLNS& lns, int i){
    init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
}

/**
 * @brief Initialize distance tables for trajectories
 *
 * Computes distance-to-path tables for agents that need them,
 * up to the specified amount.
 *
 * @param lns Trajectory LNS structure
 * @param amount Maximum number of agents to process
 */
void init_dist_table(TrajLNS& lns, int amount){

    int count = 0;
    for (int i=0 ; i <lns.env->num_of_agents; i++){
        if (count >= amount){
            break;
        }
        if(!lns.trajs[i].empty() && lns.trajs[i].size() == get_heuristic(lns.heuristics[lns.trajs[i].back()], lns.env,lns.trajs[i].front(),&(lns.neighbors)))
            continue;
        if(!lns.trajs[i].empty() && lns.traj_dists[i].empty()){
            init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
            count++;
            lns.dist2path_inited++;
        }

    }
}

/**
 * @brief Update trajectory and distance table for agent
 *
 * Replans the path for an agent from current location to goal,
 * then updates the flow network and distance tables.
 *
 * @param lns Trajectory LNS structure
 * @param i Agent index
 */
void update_traj(TrajLNS& lns, int i){
    int start = lns.env->curr_states[i].location;
    int goal = lns.tasks[i];
    lns.goal_nodes[i] = astar(lns.env,lns.flow, lns.heuristics[goal],lns.trajs[i],lns.mem,start,goal, &(lns.neighbors));
    add_traj(lns,i);
    update_dist_2_path(lns,i);
}

}
