#ifndef MYALGO_H
#define MYALGO_H

#include "Environment.h"

typedef boost::unordered_map<std::string, std::vector<Robot_mem*>> Neighbor_Matrix;

class MyAlgo
{
public:
    MyAlgo(Environment *environment_, Point_2 dest, int Num_Robot,
           double min_step_size_, double max_step_size_):environment(environment_)
    {
        destination.push_back(dest);
		Number_of_robots = Num_Robot;
        max_step = max_step_size_;
		min_step = min_step_size_;
		calculate_distance_matrix(environment->getRadius()+1, min_step);
    }

    MyAlgo(Environment *environment_, std::vector<Point_2> dest, int Num_Robot,
           double min_step_size_, double max_step_size_):environment(environment_)
    {
        destination = dest;
        Number_of_robots = Num_Robot;
        max_step = max_step_size_;
        min_step = min_step_size_;
        calculate_distance_matrix(environment->getRadius()+1, min_step);
    }

    Status init_status()
    {
        int dest_idx=0;
        Status status;
        std::vector<Robot_mem> robots_status;
        std::vector<Robot>* robots = environment->getRobots();
		for (auto robot = robots->begin(); robot != robots->end(); ++robot)
        {
            Robot_mem mem;
            mem.setRobot(&*robot);
            mem.setPosition(robot->get_position());

            if(destination.size()!=Number_of_robots)
                mem.setDestination(destination.front());
            else
            {
                mem.setDestination(destination[dest_idx]);
                dest_idx++;
            }

            robots_status.push_back(mem);
        }

        status.setStatus(environment->current_state(), &robots_status);
        preComputation(&robots_status);

        return status;
    }

	void preComputation (std::vector<Robot_mem> *all_robots)
    {
		std::vector<Robot_mem*> neighbor_inf;
        for(auto it1 = all_robots->begin() ; it1 != all_robots->end() ; ++it1)
        {
            for(auto it2 = all_robots->begin() ; it2 != all_robots->end() ; ++it2)
            {
                if(it1->getRobot() == it2->getRobot())
                    continue;

                if(it1->getRobot()->communication_check(*it2->getRobot()))
                    neighbor_inf.push_back(&*it2);
            }

            neighbor_mat[it1->getRobot()->get_Name()] = neighbor_inf;
            neighbor_inf.clear();
        }
    }

	std::vector<std::pair<double, Movement_Direction*>> Algo(Robot_mem &mem, std::vector<Robot_mem> *all)
    {
		std::vector<Robot_mem*> neighbor_inf;
		neighbor_inf = neighbor_mat[mem.getRobot()->get_Name()];
		double Num_of_neighbor = neighbor_inf.size();
		if (Num_of_neighbor == 0)
		{
			std::pair<double, double> xy_distance = 
                distance_in_xy_to_destination(mem.getPosition(), mem.getDestination());

			return move(&mem, all, xy_distance.first, xy_distance.second);
		}

		//Robot_mem* farthest_to_me = find_farthest_neighbor_to_2(mem);
		//std::pair<double, double> energy_ = energy(mem, *farthest_to_me);

		//std::pair<double, double> xy_distance = safe_move(mem.getPosition(), destination,
		//	farthest_to_me->getPosition(), farthest_to_me->getRobot()->getDisk().get_radius());
		std::pair<double, double> xy_distance = safe_move_for_all_neighbor(mem, 
													mem.getRobot()->getDisk().get_radius());


        return move(&mem, all, xy_distance.first, xy_distance.second);
    }

    std::vector<std::pair<double, Movement_Direction*>> Algo_Dynamic_Graph(Robot_mem &mem,
                                                                           std::vector<Robot_mem> *all)
	{
		preComputation(all);
		return Algo(mem, all);
	}

    std::vector<std::pair<double, Movement_Direction*>> Algo_Static_Graph(Robot_mem &mem,
                                                                          std::vector<Robot_mem> *all)
	{
		//Robot just preserve initiale connections.
		return Algo(mem, all);
	}

    Robot_mem Refresh_mem(Robot_mem previous, Point_2 &new_position, Point_2 new_destination)
    {
        Robot_mem mem;

        mem.setPosition(new_position);
        mem.setDestination(new_destination);
        mem.setRobot(previous.getRobot());

        return mem;
    }

private:
    Environment *environment;
	int Number_of_robots;
    Neighbor_Matrix neighbor_mat;
    std::vector<Point_2> destination;
    double min_step, max_step; //Maximum and minimum distance that a robot can move in each turn.
	std::vector<std::vector<double>> distance_mat;

    double calculate_weight(Point_2 &current_pos, Point_2 &destination, double w)
    {
        return L_shape_distance(current_pos, destination)*w;
    }

	double L_shape_distance(Point_2 &current_pos, Point_2 &destination)
	{
		std::pair<double, double> xy = distance_in_xy_to_destination(current_pos, destination);

		return  (abs(xy.first)+ abs(xy.second));
	}

	std::pair<double, double> distance_in_xy_to_destination(Point_2 &current_pos, Point_2 &destination)
	{
		BigRat x_dis = destination.x() - current_pos.x();
		BigRat y_dis = destination.y() - current_pos.y();

		return std::make_pair(x_dis.doubleValue(), y_dis.doubleValue());
	}

	void calculate_distance_matrix(int max, double step_size)
	{
		Point_2 center(0, 0);
		int temp = (max + step_size) / step_size;
        double i = 0, j = 0;
        for (int idx_i = 0 ; idx_i <= temp; idx_i++)
        {
			std::vector<double> y_;
            for (int idx_j = 0 ; idx_j <= temp; idx_j++)
			{
				Point_2 p(i, j);
				//double dist = static_cast<float>(static_cast<int>
				//		((sqrt(CGAL::squared_distance(center, p).doubleValue()) + 0.05) * 10.)) / 10.;
				double dist = sqrt(CGAL::squared_distance(center, p).doubleValue());
				y_.push_back(dist);
				j += step_size;
			}
			distance_mat.push_back(y_);
			i += step_size;
            j = 0;
		}
	}

	Robot_mem* find_farthest_neighbor_to(Robot_mem &robot)
	{
		std::vector<Robot_mem*> neighbor_inf;
		Robot_mem* farthest_neighbor;
		neighbor_inf = neighbor_mat[robot.getRobot()->get_Name()];
		double dist = 0;
		for (auto it = neighbor_inf.begin(); it != neighbor_inf.end(); ++it)
		{
			double temp = L_shape_distance(robot.getPosition(), (*it)->getPosition());
			if (temp > dist)
			{
				dist = temp;
				farthest_neighbor = *it;
			}
		}

		return farthest_neighbor;
	}

	Robot_mem* find_farthest_neighbor_to_2(Robot_mem &robot)
	{
		std::vector<Robot_mem*> neighbor_inf;
		Robot_mem* farthest_neighbor;
		neighbor_inf = neighbor_mat[robot.getRobot()->get_Name()];
		double dist = 0;

		for (auto it = neighbor_inf.begin(); it != neighbor_inf.end(); ++it)
		{
            double temp = L_shape_distance((*it)->getPosition(), (*it)->getDestination());
			if (temp > dist)
			{
				dist = temp;
				farthest_neighbor = *it;
			}
		}


		//double energy = max_step;
		//double farthest_xy_dist = L_shape_distance(robot.getPosition(),
		//	farthest_neighbor->getPosition());
		//for (auto it = neighbor_inf.begin(); it != neighbor_inf.end(); ++it)
		//{
		//	double xy_dist = L_shape_distance(robot.getPosition(), (*it)->getPosition());

		//	double temp = xy_dist - farthest_xy_dist;

		//	if (temp <= 0)
		//	{
		//		energy = 0;
		//		break;
		//	}
		//	else if (temp < energy)
		//		energy = temp;
		//}

		return farthest_neighbor;
	}

	std::pair<double, double> energy(Robot_mem &robot, Robot_mem &farthest_neighbor)
	{
		std::vector<Robot_mem*> neighbor_inf;
		neighbor_inf = neighbor_mat[robot.getRobot()->get_Name()];
		double x_energy = max_step, y_energy = max_step;

		std::pair<double, double> farthest_xy_dist = distance_in_xy_to_destination(robot.getPosition(),
			farthest_neighbor.getPosition());
		for (auto it = neighbor_inf.begin(); it != neighbor_inf.end(); ++it)
		{
			std::pair<double, double> xy_dist = distance_in_xy_to_destination(robot.getPosition(),
				(*it)->getPosition());
			double xtemp = xy_dist.first - farthest_xy_dist.first;
			double ytemp = xy_dist.second - farthest_xy_dist.second;

			if (xtemp <= 0)
				x_energy = 0;
			else if (xtemp < x_energy)
				x_energy = xtemp;

			if (ytemp <= 0)
				y_energy = 0;
			else if (ytemp < y_energy)
				y_energy = ytemp;

			if (x_energy <= 0 && y_energy <= 0)
			{
				x_energy = 0;
				y_energy = 0;
				break;
			}

		}
		
		return std::make_pair(x_energy, y_energy);
	}

	std::pair<double, double> safe_move_for_all_neighbor(Robot_mem &robot, BigRat radius)
	{
		std::pair<double, double> safe_move_;
		std::vector<Robot_mem*> neighbor_inf;
		neighbor_inf = neighbor_mat[robot.getRobot()->get_Name()];
		double dist = 0;

		std::pair<double, double> xy_dist_to_destination =
            distance_in_xy_to_destination(robot.getPosition(), robot.getDestination());
		int x_sign = xy_dist_to_destination.first / abs(xy_dist_to_destination.first);
		int y_sign = xy_dist_to_destination.second / abs(xy_dist_to_destination.second);

		std::pair<double, double> xy_safe_moving_dist;
		xy_safe_moving_dist.first = max_step * x_sign;
		xy_safe_moving_dist.second = max_step * y_sign;

		int k = 0;
		if (neighbor_inf.size() >= 6)
		{
			std::cout << "Yes   ";
			k = 3;
		}

		for (auto it = neighbor_inf.begin(); it != neighbor_inf.end(); ++it)
		{
			int random = (rand() % (1 + 1 - 0)) + 0;
			if (random == 0)
				if (k > 0)
				{
					k--;
					continue;
				}

            safe_move_ = safe_move(robot.getPosition(), robot.getDestination(),
                                   (*it)->getPosition(), radius);

			if (x_sign < 0)
			{
				if (safe_move_.first > xy_safe_moving_dist.first)
					xy_safe_moving_dist.first = safe_move_.first;
			}else
				if (safe_move_.first < xy_safe_moving_dist.first)
					xy_safe_moving_dist.first = safe_move_.first;


			if (y_sign < 0)
			{
				if (safe_move_.second > xy_safe_moving_dist.second)
					xy_safe_moving_dist.second = safe_move_.second;
			}
			else
				if (safe_move_.second < xy_safe_moving_dist.second)
					xy_safe_moving_dist.second = safe_move_.second;		
		}

		return xy_safe_moving_dist;
	}


	std::pair<double, double> safe_move(Point_2 &current, Point_2 &destination, 
										Point_2 &farthest_neighbor, BigRat &radius)
	{
		// Assume farthest_neighbor located at the origin point. Accordingly 
		// shift current and destination.
		Point_2 new_current = Point_2(current.x() - farthest_neighbor.x(),
									  current.y() - farthest_neighbor.y());	
		Point_2 new_destination = Point_2(destination.x() - farthest_neighbor.x(),
										  destination.y() - farthest_neighbor.y());

		std::pair<double, double> xy_dist_to_destination = 
			distance_in_xy_to_destination(new_current, new_destination);

		std::pair<double, double> xy_safe_moving_dist;
		xy_safe_moving_dist.first = 0;
		xy_safe_moving_dist.second = 0;

		// Calculate safe distance in horizontal direction that preserve connection
		// between robot and his farthest neighbor.
		double size = abs(xy_dist_to_destination.first);
		int sign = +1;
		if (new_destination.x().doubleValue() >= new_current.x().doubleValue())
			sign = +1;
		else
			sign = -1;

		int y_index = abs(new_current.y().doubleValue() / min_step)+ 0.05;

		for(double i = min_step ; i <= size ; i += min_step)
		{
			int index = abs (floor(((new_current.x().doubleValue() + (sign * i)) / min_step)));
			double distance = distance_mat[index][y_index];

			if (distance > radius-0.2)
			{
				double modulo_ = fmod(new_current.x().doubleValue(), min_step);
				if (modulo_ != 0 && xy_safe_moving_dist.first <= min_step)
					xy_safe_moving_dist.first = 0;

				break;
			}
			else
				xy_safe_moving_dist.first = i;
		}

		//xy_safe_moving_dist.first -= 0.11;
		xy_safe_moving_dist.first = static_cast<float>(static_cast<int>
			((xy_safe_moving_dist.first + 0.05) * 10.)) / 10.;

		xy_safe_moving_dist.first *= sign;


		// Calculate safe distance in vertical direction that preserve connection
		// between robot and his farthest neighbor.
		size = abs(xy_dist_to_destination.second);
		if (new_destination.y().doubleValue() >= new_current.y().doubleValue())
			sign = +1;
		else
			sign = -1;

		int x_index = abs(new_current.x().doubleValue() / min_step) + 0.05;

		for (double i = min_step; i <= size; i += min_step)
		{
			int index = abs(floor(((new_current.y().doubleValue() + (sign * i)) / min_step)));
			double distance = distance_mat[x_index][index];

			if (distance > radius-0.2)
			{
				double modulo_ = fmod(new_current.y().doubleValue(), min_step);
				if (modulo_ !=0 && xy_safe_moving_dist.second <= min_step)
					xy_safe_moving_dist.second = 0;

				break;
			}
			else
				xy_safe_moving_dist.second = i;
		}

		//xy_safe_moving_dist.second -= 0.11;
		xy_safe_moving_dist.second = static_cast<float>(static_cast<int>
			((xy_safe_moving_dist.second + 0.05) * 10.)) / 10.;

		xy_safe_moving_dist.second *= sign;


		
		return xy_safe_moving_dist;
	}

    std::vector<std::pair<double, Movement_Direction*>> move(Robot_mem* mem, std::vector<Robot_mem> *all,
                                                             double x_ , double y_)
	{
		std::vector<std::pair<double, Movement_Direction*>> move_, temp;

		if (abs(x_) > max_step)
			x_ = (x_ / abs(x_)) * max_step;
        if (abs(y_) > max_step)
            y_ = (y_ / abs(y_)) * max_step;


		move_.push_back(Avoidance(mem, all, std::make_pair(abs(x_), specify_dir(x_, "horizontal"))));
		move_.push_back(Avoidance(mem, all, std::make_pair(abs(y_), specify_dir(y_, "vertical"))));

		if (move_.front().first != 0 && move_.back().first != 0)
		{
			if (move_.front().first > move_.back().first)
				move_.erase(move_.end()-1);
			else if(move_.front().first < move_.back().first)
				move_.erase(move_.begin());
			else
			{
				int random = (rand() % (1 + 1 - 0)) + 0;
				if (random == 0)
					move_.erase(move_.end() - 1);
				else
					move_.erase(move_.begin());
			}
		}
		else if (move_.front().first != 0)
		{
			move_.erase(move_.end() - 1);
		}
		else if (move_.back().first != 0)
		{
			move_.erase(move_.begin());
		}

		return move_;
	}

	Movement_Direction* specify_dir(double &value, std::string type)
	{
		if (type == "vertical")
			if (value > 0)
				return new Up_Direction();
			else
				return new Down_Direction();
		else if (type == "horizontal")
			if (value > 0)
				return new Right_Direction();
			else
				return new Left_Direction();
	}

    std::pair<double, Movement_Direction*> Avoidance(Robot_mem* mem, std::vector<Robot_mem> *all,
		std::pair<double, Movement_Direction*> &move)
	{
		std::pair<double, Movement_Direction*> collision_avoidance;
		if (move.first != 0)
		{
			Segment_2 seg = move.second->get_segmet_path_on_direction(move.first, mem->getPosition());
			std::vector<Point_2> collision_points;
            for (auto it = all->begin(); it != all->end(); ++it)
			{
                if (mem->getRobot() != it->getRobot() && seg.has_on(it->getPosition()))
                    collision_points.push_back(it->getPosition());
			}
			move.second->sort_event(collision_points);
			if (collision_points.size() == 0)
				collision_avoidance = move;
			else
                collision_avoidance = Collision_Avoidance(mem->getPosition(),
                                                          collision_points.front(), move);
		}
		else
			collision_avoidance = move;

		return collision_avoidance;
	}

	std::pair<double, Movement_Direction*> Collision_Avoidance(Point_2 &current, Point_2 collision,
		std::pair<double, Movement_Direction*> &move)
	{
		double distance = sqrt(CGAL::squared_distance(current, collision).doubleValue());
        if(distance <= 0.05)
            return std::make_pair(0, move.second);
        else
            return std::make_pair(distance - 0.05, move.second);
	}
};




#endif // MYALGO_H
