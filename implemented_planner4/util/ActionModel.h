#pragma once
#include <string>
#include <vector>
#include "Grid.h"
#include "States.h"
#include "Logger.h"
#include "SharedEnv.h"
#include "../inc/ActionModel.h"  // Explicit path to avoid circular include

// ====================================================================================================
// CompetitionActionModelWithRotate Class Declaration
// ====================================================================================================
// This class defines the action model for agents in a MAPF competition environment.
// It supports actions like moving forward (FW), rotating clockwise (CR), counter-clockwise (CCR), and waiting (W).
// The model computes the resulting states after actions and provides neighbor generation for pathfinding.
// There are two implementations: one with rotation support and one without (controlled by NO_ROT macro).
// ====================================================================================================

#ifndef NO_ROT

class CompetitionActionModelWithRotate
{
public:
    list<std::tuple<std::string,int,int,int>> validation_errors;  // List to store validation errors

    CompetitionActionModelWithRotate(){};  // Default constructor
    CompetitionActionModelWithRotate(SharedEnvironment * env): shared_env(env),grid_rows(env->rows),grid_cols(env->cols){  // Constructor with environment
        direction_offsets[0] = 1;      // Right
        direction_offsets[1] = grid_cols;   // Down
        direction_offsets[2] = -1;     // Left
        direction_offsets[3] = -grid_cols;  // Up
    };

    bool validate_actions(const vector<State>& prev, const vector<Action> & action) {
        // Simple validation - just return true for now
        // More sophisticated validation can be added here if needed
        validation_errors.clear();
        return true;
    }

    // Compute resulting states for multiple agents
    vector<State> compute_next_states(const vector<State>& prev, const vector<Action> & action){
        vector<State> next(prev.size());
        for (size_t i = 0 ; i < prev.size(); i ++){
            next[i] = apply_action(prev[i], action[i]);
        }
        return next;
    };

    // Get location neighbors with orientation
    vector<State> get_adjacent_states(const State & curr,bool move_orient=true) {
        vector<State> neighbors;

        int x=curr.location%shared_env->cols;  // Current x position
        int y=curr.location/shared_env->cols;  // Current y position

        for (int i=0;i<4;++i) {  // For each direction
            int new_location=curr.location+direction_offsets[i];  // Calculate new location
            int new_orientation;
            
            if (move_orient) {
                new_orientation=i;  // Set orientation to direction
            } else {
                new_orientation=0;  // Default orientation
            }

            // Check boundaries
            if ((x==shared_env->cols-1 && i==0) || (y==shared_env->rows-1 && i==1) || (x==0 && i==2) || (y==0 && i==3)) {
                continue;  // Skip if out of bounds
            }

            if (shared_env->map[new_location]) {  // Check if obstacle
                continue;
            }

            neighbors.push_back(State(new_location,curr.timestep+1,new_orientation));  // Add neighbor
        }

        return neighbors;
    }

    SharedEnvironment * shared_env=nullptr;  // Pointer to shared environment
    int grid_rows;  // Number of rows
    int grid_cols;  // Number of columns
    int direction_offsets[4];  // Movement offsets for each direction

    // Compute resulting state after an action
    State apply_action(const State & prev, Action action, bool check=false)
    {
        int new_location = prev.location;
        int new_orientation = prev.orientation;
        if (action == Action::FW)  // Forward move
        {
            new_location = new_location += direction_offsets[prev.orientation];  // Move in current orientation
            if (check) {  // If checking validity
                int x=prev.location%shared_env->cols;
                int y=prev.location/shared_env->cols;

                // Check boundaries
                if ((x==shared_env->cols-1 && prev.orientation==0) || (y==shared_env->rows-1 && prev.orientation==1) || (x==0 && prev.orientation==2) || (y==0 && prev.orientation==3)) {
                    return State(-1,prev.timestep+1,new_orientation=prev.orientation);  // Invalid move
                }

                if (shared_env->map[new_location]) {  // Check obstacle
                    return State(-1,prev.timestep+1,new_orientation=prev.orientation);  // Invalid move
                }
            }
        }
        else if (action == Action::CR)  // Clockwise rotation
        {
            new_orientation = (prev.orientation + 1) % 4;  // Rotate right
        }
        else if (action == Action::CCR)  // Counter-clockwise rotation
        {
            new_orientation = (prev.orientation - 1) % 4;  // Rotate left
            if (new_orientation == -1)
                new_orientation = 3;  // Wrap around
        }

        return State(new_location, prev.timestep + 1, new_orientation);  // Return new state
    }
};

#else  // NO_ROT defined


#endif
