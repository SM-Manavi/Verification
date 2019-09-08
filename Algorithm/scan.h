#ifndef SCAN_H
#define SCAN_H

#include "Environment.h"
#include <chrono>

class SCAN{

public:

    SCAN(Environment *environment_):environment(environment_) {}
    Status init_status()
    {
		std::vector<Movement_Direction*> dir_temp = { new Up_Direction() , new Left_Direction(),
			new Right_Direction(), new Down_Direction(), new Right_Direction(), new Right_Direction() };
		int idx = 0;

        Status status;
        std::vector<Robot_mem> robots_status;
        std::vector<Robot>* robots = environment->getRobots();
        for (auto robot = robots->begin(); robot != robots->end(); ++robot)
        {
            Robot_mem mem;
            mem.setRobot(&*robot);
			//mem.setCurrent_dir(random_direction());

			mem.setCurrent_dir(dir_temp[idx]);
			idx++;

            mem.setPosition(robot->get_position());

            robots_status.push_back(mem);
        }

        status.setStatus(environment->current_state(), &robots_status);

        return status;
    }

	std::vector<Status> turn_taking_init()
	{
		std::vector<Movement_Direction*> dir_temp = { new Up_Direction() , new Left_Direction(),
													  new Right_Direction(), new Down_Direction(), 
													  new Right_Direction(), new Right_Direction() };
		int idx = 0;

		std::vector<Status> status;
		std::vector<Robot_mem> robots_status;
		std::vector<Robot>* robots = environment->getRobots();
		for (auto robot = robots->begin(); robot != robots->end(); ++robot)
		{
			Robot_mem mem;
			mem.setRobot(&*robot);
			//mem.setCurrent_dir(random_direction());
			mem.setCurrent_dir(dir_temp[idx]);
			idx++;

			mem.setPosition(robot->get_position());
			std::cout << mem.getCurrent_dir()->get_direction_name() << "   ,   " << std::endl;
			robots_status.push_back(mem);
		}

		Status temp;
		do {
			temp.setStatus(environment->current_state(), &robots_status);
			status.push_back(temp);
		} while (std::next_permutation(robots_status.begin(), robots_status.end()));

		return status;
	}

    bool check_move_condition(Robot_mem &mem, int &k)
    {
        Connectivity_hash_map mat = environment->get_connectivity_matrix();

        std::vector<std::string> robot_neighbor = mat[mem.getRobot()->get_Name()];
        bool move = 1;

		if (!robot_neighbor.empty())
			for (auto it = robot_neighbor.begin(); it != robot_neighbor.end(); ++it)
			{
				std::vector<std::string> neighbor_of_neighbor;
				neighbor_of_neighbor = mat[*it];
				if (intersection(robot_neighbor, neighbor_of_neighbor).size() < k)
				{
					move = 0;
					break;
				}
			}
		else
			move = 0;

        return move;
    }

    std::vector<std::pair<double, Movement_Direction*>> SCAN_Algo(Robot_mem &mem,
                                                                  int k, double distance)
    {
        bool move = check_move_condition(mem, k);

		if (move)
		{
			//return all_direciton(&mem, distance);
			//return mobility_algo_1(&mem, distance);
			return mobility_algo_2(mem, distance);
		}

        std::vector<std::pair<double, Movement_Direction*>> freez;
        return freez;
    }

    Robot_mem Refresh_mem(Robot_mem &previous, Point_2 &new_position, Movement_Direction* new_dir)
    {
        Robot_mem mem;
        mem.setPosition(new_position);
		mem.setCurrent_dir(new_dir);
        mem.setRobot(previous.getRobot());

        return mem;
    }

	Robot_mem Refresh_mem_TT(Robot_mem &mem, Point_2 &new_position, Movement_Direction* new_dir)
	{
		mem.setCurrent_dir(new_dir);
		mem.setPosition(new_position);

		return mem;
	}


private:
    Environment *environment;
	
	Point_2 target = Point_2(200,200);

    std::vector<std::string> intersection(std::vector<std::string> &v1,
                                          std::vector<std::string> &v2){
        std::vector<std::string> v3;

        std::sort(v1.begin(), v1.end());
        std::sort(v2.begin(), v2.end());

        std::set_intersection(v1.begin(),v1.end(),
                              v2.begin(),v2.end(),
                              back_inserter(v3));
        return v3;
    }


	std::pair<double, double> distance_in_xy_to_destination(Point_2 &current_pos, Point_2 &destination)
	{
		BigRat x_dis = destination.x() - current_pos.x();
		BigRat y_dis = destination.y() - current_pos.y();

		return std::make_pair(x_dis.doubleValue(), y_dis.doubleValue());
	}

	std::vector<std::pair<double, Movement_Direction*>> mobility_algo_1(Robot_mem *mem, double &distance)
	{
		std::vector<std::pair<double, Movement_Direction*>> result;

		std::pair<double, double> distance_to_target = distance_in_xy_to_destination(mem->getPosition(), target);

		double distance_to_target_abs_x = abs(distance_to_target.first);
		double distance_to_target_abs_y = abs(distance_to_target.second);

		if (distance_to_target_abs_x > 0 && distance_to_target_abs_y > 0)
		{
			std::srand(std::chrono::system_clock::now().time_since_epoch().count());
			int x = (rand() % 2);
			if (x == 0)
				if (distance_to_target.first > 0)
					result.push_back(std::make_pair(distance_to_target_abs_x, new Right_Direction()));
				else
					result.push_back(std::make_pair(distance_to_target_abs_x, new Left_Direction()));
			else
				if (distance_to_target.second > 0)
					result.push_back(std::make_pair(distance_to_target_abs_y, new Up_Direction()));
				else
					result.push_back(std::make_pair(distance_to_target_abs_y, new Down_Direction()));
		}
		else if (distance_to_target_abs_x > 0)
		{
			if (distance_to_target.first > 0)
				result.push_back(std::make_pair(distance_to_target_abs_x, new Right_Direction()));
			else
				result.push_back(std::make_pair(distance_to_target_abs_x, new Left_Direction()));
		}
		else if(distance_to_target_abs_y > 0)
		{
			if (distance_to_target.second > 0)
				result.push_back(std::make_pair(distance_to_target_abs_y, new Up_Direction()));
			else
				result.push_back(std::make_pair(distance_to_target_abs_y, new Down_Direction()));
		}

		return result;
	}

	std::vector<std::pair<double, Movement_Direction*>> mobility_algo_2(Robot_mem &mem, double &distance)
	{
		std::vector<std::pair<double, Movement_Direction*>> move;
		Movement_Direction* dir = toDirecrion_type(mem.getCurrent_dir()->get_direction_name());

		Segment_2 path = dir->get_segmet_path_on_direction(distance, mem.getRobot()->get_position());
		if (path.source() == environment->get_valid_destination(path.source(), path.target()))
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
		return move;
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

		if (x == 0)
			return new Left_Direction();
		else if (x == 1)
			return new Right_Direction();
		else if (x == 2)
			return new Up_Direction();
		else if (x == 3)
			return new Down_Direction();
	}


    std::vector<std::pair<double, Movement_Direction*>> all_direciton(Robot_mem *mem, double &distance)
    {

        std::vector<Movement_Direction*> directions = { new Left_Direction(), new Right_Direction(),
                                                        new Up_Direction(),   new Down_Direction() };
        std::vector<std::pair<double, Movement_Direction*>> movements;

        for(auto dir = directions.begin() ; dir != directions.end() ; ++dir)
        {
            //std::pair<double, Movement_Direction*> move =
            //        Collision_Avoidance(*mem, std::make_pair(distance, *dir));
            //if(move.first != 0)
            //    movements.push_back(move);
        }

        return movements;
    }

    std::pair<double, Movement_Direction*> Collision_Avoidance(Robot_mem &mem,
                                                               std::pair<double, Movement_Direction*> &move)
    {
        /*Segment_2 seg_path = move.second->get_segmet_path_on_direction(move.first, mem.getPosition());
        std::pair<bool, Point_2> collistion = environment->check_collision(mem.getRobot(),
                                                                           seg_path.target(),
                                                                           move.first);

        if(collistion.first)
        {
            double distance = sqrt(CGAL::squared_distance(mem.getPosition(), collistion.second).doubleValue());
            if(distance <= 0.05)
                return std::make_pair(0, move.second);
            else
                return std::make_pair(distance - 0.05, move.second);
        }
        else
            return move;*/
    }

};


#endif // SCAN_H
