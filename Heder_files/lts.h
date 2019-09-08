#ifndef LTS_H
#define LTS_H

#include "M_Utility.h"


struct LTS
{
	LTS(std::string file_name_, std::string format)
	{
		output_format = format;  file_name = file_name_;
	}

    void addState(State_Data *current, State_Data *next)
    {
        std::string connectivity_status = "No_con";
        if(utility.connectivity_check(next->connectivity_matrix))
            connectivity_status = "con";

        file_data = file_data
                        + "(" + std::to_string(current->stateId) + ",\""
                        + connectivity_status + "\", "
                        + std::to_string(next->stateId) + ")" + "\n ";
    }

	void addTransition(State_Data *current, State_Data *next) { addState(current, next); }

    void createLTS(int num_state, int num_transition)
    {
        file_data = "des (0," + std::to_string(num_transition-Num_Self_Loop_Transition) + ","
                              + std::to_string(num_state) + ")\n" + file_data;

        std::ofstream out((file_name + ".aut").c_str());
        out << file_data;
        out.close();

		std::cout << "LTS file created." << std::endl;
    }

	void Self_Loop() { Num_Self_Loop_Transition++; }

private:
    std::string file_data="";
    Utility utility;
    std::string output_format;
	std::string file_name;
	int Num_Self_Loop_Transition=0;
};


#endif // LTS_H
