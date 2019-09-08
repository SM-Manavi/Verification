#ifndef ALPHA_H
#define ALPHA_H

#include "Environment.h"
#include <chrono>

class Alpha
{
public:
    Alpha(Environment *environment_):environment(environment_) {}
    Status init_status()
    {
		std::vector<Movement_Direction*> dir_temp = { new Left_Direction() , new Right_Direction(),
			new Down_Direction() };
		int idx = 0;

        Status status;
        std::vector<Robot_mem> robots_status;
        std::vector<Robot>* robots = environment->getRobots();
        for(auto robot = robots->begin() ; robot != robots->end() ; ++robot )
        {
            Robot_mem mem;
            mem.setRobot(&*robot);
			//mem.setCurrent_dir(random_direction());
			mem.setCurrent_dir(dir_temp[idx]);
			idx++;
            mem.setCurrent_neighbor(environment->getNum_of_neighbor(&(*robot)));
            mem.setPosition(robot->get_position());


            robots_status.push_back(mem);
        }

        status.setStatus(environment->current_state(), &robots_status);

        return status;
    }

	std::vector<Status> turn_taking_init()
	{
		std::vector<Status> status;
		std::vector<Robot_mem> robots_status;
		std::vector<Robot>* robots = environment->getRobots();
		for (auto robot = robots->begin(); robot != robots->end(); ++robot)
		{
			Robot_mem mem;
			mem.setRobot(&*robot);
			mem.setCurrent_dir(random_direction());
			mem.setCurrent_neighbor(environment->getNum_of_neighbor(&(*robot)));
			mem.setPosition(robot->get_position());


			robots_status.push_back(mem);
		}

		Status temp;
		do {
			temp.setStatus(environment->current_state(), &robots_status);
			status.push_back(temp);
		} while (std::next_permutation(robots_status.begin(), robots_status.end()));

		return status;
	}

    std::vector<std::pair<double, Movement_Direction*>> Alph_Dexion(Robot_mem &mem,
            int alpha, double distance)
    {
            std::vector<Movement_Direction*> direction;
			int new_neighbor = environment->getNum_of_neighbor(mem.getRobot());

            if (mem.getMode() == "forward" && new_neighbor >= alpha)
            {
                    direction.push_back(mem.getCurrent_dir());
            }
            else if (mem.getMode() == "forward" && new_neighbor < alpha)
            {
                    direction.push_back(mem.getCurrent_dir()->turn_180());
                    mem.setMode("coherence");
            }
            else if (mem.getMode() == "coherence" && new_neighbor < alpha)
            {
                    direction.push_back(mem.getCurrent_dir());
            }
            else if (mem.getMode() == "coherence" && new_neighbor >= alpha)
            {
                    direction.push_back(mem.getCurrent_dir()->turn_180());
                    mem.setMode("forward");
            }

            std::vector<Movement_Direction*> temp;
            for (auto dir = direction.begin(); dir != direction.end();)
            {
            if (Avoidance(mem.getRobot(), (*dir)->get_segmet_path_on_direction(
                            distance, mem.getPosition())))
                    {
                            std::vector<Movement_Direction*> turn90 = (*dir)->turn_90();
                bool turn90_1 = Avoidance(mem.getRobot(), turn90.front()->get_segmet_path_on_direction(
                                    distance, mem.getPosition()));
                bool turn90_2 = Avoidance(mem.getRobot(), turn90.back()->get_segmet_path_on_direction(
                                    distance, mem.getPosition()));
                            if (turn90_1 && turn90_2)
                            {
                    if (!Avoidance(mem.getRobot(), (*dir)->turn_180()->get_segmet_path_on_direction(
                                            distance, mem.getPosition())))
                                    {
                                            temp.push_back((*dir)->turn_180());
                                    }

                            }
                            else if (turn90_1) {
                                    temp.push_back(turn90.back());
                            }
                            else if (turn90_2) {
                                    temp.push_back(turn90.front());
                            }
                            else {
                                    temp.insert(temp.end(), turn90.begin(), turn90.end());
                            }
                            dir = direction.erase(dir);
                    }
                    else
                    {
                            ++dir;
                    }

            }

            direction.insert(direction.end(), temp.begin(), temp.end());
            direction = unique_(direction);

            std::vector<std::pair<double, Movement_Direction*>> move;

            for(auto it = direction.begin() ; it != direction.end() ; ++it)
                move.push_back(std::make_pair(distance, *it));

            return move;

    }

	std::vector<std::pair<double, Movement_Direction*>> Alpha_(Robot_mem &mem, int alpha, double distance)
	{
		std::vector<std::pair<double, Movement_Direction*>> move;
		Movement_Direction* dir;
		int new_neighbor = environment->getNum_of_neighbor(mem.getRobot());

		if (new_neighbor < mem.getCurrent_neighbor() && new_neighbor < alpha)
			dir = mem.getCurrent_dir()->turn_180();
		else if (new_neighbor > mem.getCurrent_neighbor() && new_neighbor >= alpha)
			dir = random_turn(mem.getCurrent_dir());
		else
			dir = mem.getCurrent_dir();

		//if (Avoidance(mem.getRobot(), dir->get_segmet_path_on_direction(0.1, mem.getPosition())))
		Segment_2 path = dir->get_segmet_path_on_direction(distance, mem.getRobot()->get_position());
		if(path.source() == environment->get_valid_destination(path.source(), path.target()))
		{
			std::vector<Movement_Direction*> turn90 = dir->turn_90();
			bool turn_left = false, turn_right = false;

			path = turn90.front()->get_segmet_path_on_direction(distance, mem.getRobot()->get_position());
			if (path.source() == environment->get_valid_destination(path.source(), path.target()))
				turn_left = true;

			path = turn90.back()->get_segmet_path_on_direction(distance, mem.getRobot()->get_position());
			if (path.source() == environment->get_valid_destination(path.source(), path.target()))
				turn_right = true;

			if (turn_left && !turn_right)
			{
				delete dir;
				dir = toDirecrion_type(turn90.back()->get_direction_name());
			}
			else if (turn_right && !turn_left)
			{
				delete dir;
				dir = toDirecrion_type(turn90.front()->get_direction_name());
			}
			else if (turn_left && turn_right)
			{
				Movement_Direction* turn_180 = dir->turn_180();
				path = turn_180->get_segmet_path_on_direction(distance, mem.getRobot()->get_position());
				if (path.source() != environment->get_valid_destination(path.source(), path.target()))
				{
					delete dir;
					dir = turn_180;
				}
				else
					distance = 0;
			}
			else
			{
				std::srand(std::chrono::system_clock::now().time_since_epoch().count());
				int x = (rand() % 2);

				if (x == 0)
				{
					delete dir;
					dir = toDirecrion_type(turn90.front()->get_direction_name());
				}
				else
				{
					delete dir;
					dir = toDirecrion_type(turn90.back()->get_direction_name());
				}
			}

			for (auto p : turn90)
				delete p;

		}

		move.push_back(std::make_pair(distance, dir));

		mem.setCurrent_neighbor(new_neighbor);

		return move;
	}

    std::pair<double, std::vector<Movement_Direction*>> Alph_Algo(Robot_mem &mem,
                                                               int alpha, double distance)
    {
        std::vector<Movement_Direction*> direction;
        int new_neighbor = environment->getNum_of_neighbor(mem.getRobot());
        int last_neighbor = mem.getCurrent_neighbor();

        if(new_neighbor < last_neighbor && new_neighbor < alpha)
            direction.push_back(mem.getCurrent_dir()->turn_180());
        else if(new_neighbor >= alpha)
            direction = mem.getCurrent_dir()->turn_90();
        else
            direction.push_back(mem.getCurrent_dir());

        std::vector<Movement_Direction*> temp;
        for (auto dir = direction.begin(); dir != direction.end();)
        {
            if (Avoidance(mem.getRobot(), (*dir)->get_segmet_path_on_direction(
                distance, mem.getPosition())))
            {
                std::vector<Movement_Direction*> turn90 = (*dir)->turn_90();
                bool turn90_1 = Avoidance(mem.getRobot(), turn90.front()->get_segmet_path_on_direction(
                    distance, mem.getPosition()));
                bool turn90_2 = Avoidance(mem.getRobot(), turn90.back()->get_segmet_path_on_direction(
                    distance, mem.getPosition()));
                if (turn90_1 && turn90_2)
                {
                    if (!Avoidance(mem.getRobot(), (*dir)->turn_180()->get_segmet_path_on_direction(
                        distance, mem.getPosition())))
                    {
                        temp.push_back((*dir)->turn_180());
                    }

                }
                else if (turn90_1) {
                    temp.push_back(turn90.back());
                }
                else if (turn90_2) {
                    temp.push_back(turn90.front());
                }
                else {
                    temp.insert(temp.end(), turn90.begin(), turn90.end());
                }
                dir = direction.erase(dir);
            }
            else
            {
                ++dir;
            }

        }

        direction.insert(direction.end(), temp.begin(), temp.end());
        direction = unique_(direction);

        return std::make_pair(distance, direction);
    }

    Robot_mem Refresh_mem(Robot_mem &previous, Point_2 &new_position, Movement_Direction* new_dir)
    {
        Robot_mem mem;

		mem.setRobot(previous.getRobot());
        mem.setCurrent_dir(new_dir);
        mem.setPosition(new_position);
		mem.setCurrent_neighbor(previous.getCurrent_neighbor());

        return mem;
    }

	void Refresh_mem_TT(Robot_mem &mem, Point_2 &new_position, Movement_Direction* new_dir)
	{
		mem.setCurrent_dir(new_dir);
		mem.setPosition(new_position);
	}

private:
    Environment *environment;


    std::vector<Movement_Direction*> unique_ (std::vector<Movement_Direction*> &direction)
    {
        std::vector<Movement_Direction*> newVec;
        for(auto dir = direction.begin() ; dir != direction.end() ; ++dir)
        {
            bool check = false;
            for(auto new_ = newVec.begin() ; new_ != newVec.end() ; ++new_)
            {
                if((*new_)->get_direction_name() == (*dir)->get_direction_name())
                {
                    check = true;
                    break;
                }
            }

            if(!check)
                newVec.push_back(*dir);
        }

        return newVec;
    }

    bool Avoidance(Robot* robot, Segment_2 path)
    {
        if(environment->check_collision(robot, path.target()))
            return true;

        return false;
    }

	Movement_Direction* random_turn(Movement_Direction* current)
	{
		std::vector<Movement_Direction*> all_possible_dir;
		all_possible_dir = current->turn_90();
		all_possible_dir.push_back(current->turn_180());

		std::srand(std::chrono::system_clock::now().time_since_epoch().count());
		int x = (rand() % 3); //Between 0-2

		Movement_Direction* rand_dir = toDirecrion_type(all_possible_dir[x]->get_direction_name());

		for (auto p : all_possible_dir)
			delete p;

		return rand_dir;
	}


	Movement_Direction* random_direction()
	{
		int x; //Between 0-3
		std::srand(std::chrono::system_clock::now().time_since_epoch().count());
		x = (rand() % 4);

		if(x==0)
			return new Left_Direction();
		else if(x==1)
			return new Right_Direction();
		else if(x==2)
			return new Up_Direction();
		else if(x==3)
			return new Down_Direction();
	}


};




#endif // ALPHA_H
