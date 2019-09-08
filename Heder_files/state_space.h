#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include "State_data.h"
#include "lts.h"

typedef boost::unordered_map<std::vector<std::string>, std::vector<State_Data*> > hash_map;

struct State_space
{
    State_space():lts("Connectivity", "aut"), without_self_loop_lts("No_Self_Loop", "aut") {}

    void first_state(State_Data *state)
    {
//        if(start != nullptr)
//        {
//            delete start;
//            delete current_state;
//        }
        start = state;
        current_state = state;

        state->stateId = size;
        size++;
    }

    void add_state(State_Data *state, std::string &move_inf)
    {

        std::vector<State_Data*> same_arr_faces_names_state =
                find_state_by_arr_faces_names(*state->faces_names);

        if(same_arr_faces_names_state.empty())
        {
            same_arr_faces_names_state.push_back(state);
			state->stateId = size;

			lts.addState(current_state, state);
			without_self_loop_lts.addState(current_state, state);

            addState(state, same_arr_faces_names_state, move_inf);

            return;

        }else
        {
            if(check_duplicate_state(same_arr_faces_names_state, state))
            {
                if(check_next_states_of_current(state))
                {
                    current_state = state;
                    return;
                }else
                {
					lts.addTransition(current_state, state);

					if (state != current_state)
						without_self_loop_lts.addTransition(current_state, state);
					else
						without_self_loop_lts.Self_Loop();

                    current_state->next.push_back(make_pair(move_inf, state));
                    current_state = state;
					num_transition++;

                    return;
                }
            }else
            {
                same_arr_faces_names_state.push_back(state);
				state->stateId = size;

				lts.addState(current_state, state);
				without_self_loop_lts.addState(current_state, state);

                addState(state, same_arr_faces_names_state, move_inf);


                return;
            }
        }
    }

    void addState(State_Data *state, std::vector<State_Data*> same_arr_faces_names_state,
                  std::string transition_lable)
    {
        state_list_indexed_by_arr_faces_names[*state->faces_names] =
                same_arr_faces_names_state;

        current_state->next.push_back(make_pair(transition_lable, state));
        current_state = state;

        size++;
        num_transition++;
    }

    std::vector<State_Data*> find_state_by_arr_faces_names(
            std::vector<std::string> faces_curves_name)
    {
        return state_list_indexed_by_arr_faces_names[faces_curves_name];
    }

    bool check_duplicate_state(std::vector<State_Data*> &same_faces_names, State_Data *&state)
    {
        bool check_duplicate = false;
        std::vector<State_Data*>::iterator state_;
		for (state_ = same_faces_names.begin(); state_ != same_faces_names.end();
			++state_)
			if ((*state_)->connectivity_matrix == state->connectivity_matrix && 
				(*state_)->disk_sequences == state->disk_sequences)
				{
					delete state;
					state = *state_;
					return true;
				}

        return check_duplicate;
    }

    bool check_next_states_of_current(State_Data *state)
    {
        std::vector<std::pair<std::string, State_Data*>>::iterator next_state;
        for(next_state = current_state->next.begin(); next_state != current_state->next.end();
            ++next_state)
        {
            if( next_state->second == state )
                return true;
        }

        return false;
    }

    void change_current_state(State_Data *state)
    {
        current_state = state;
    }

    State_Data* getCurrent_state()
    {
        return current_state;
    }

	int getSize()
	{
		return size;
	}

    void creatLTS()
    {
        lts.createLTS(size, num_transition);
		without_self_loop_lts.createLTS(size, num_transition);
    }

private:
    State_Data *start;
    State_Data *current_state;
    hash_map state_list_indexed_by_arr_faces_names;

	LTS lts;
	LTS without_self_loop_lts;

    int num_transition=0;
    int size=0;
};




#endif // STATE_SPACE_H
