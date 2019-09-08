#ifndef DISPATCH_H
#define DISPATCH_H

#include "Environment.h"
#include <boost\filesystem.hpp>
#include <windows.h>
#include <ctime>

#include "alpha.h"

class Dispatch
{
public:
    Dispatch(Environment *env);

	void Turn_taking();

private:
    void init();
    void Save(std::vector<Status> &status ,int bufferId);
    std::vector<Status> retrieve(int bufferId);
    void log(std::string &log);

    Status record_status(Status *prev, std::vector<Robot_mem>::iterator it, Robot_mem new_mem);
    std::pair<double, Movement_Direction *> Random_turn_Algorithm();

    std::string progress_tex;

    int Number_of_Robot;
    int bufferId_exist_on_disk [2] = {0,0};
    double buffer_size;
    bool buffering = false;

    Environment *environment;

    std::vector<Status> next_states;
    std::vector<Status> buffer;
    std::vector<Robot> *robots;

	std::string time_difference(std::clock_t &start);
	void print_inf(BigRat &turn, std::clock_t &start, std::clock_t &next_state_change);
	bool exit_();

    void Save_situation(Status &status);
    Status get_situation();
};

#endif // DISPATCH_H
