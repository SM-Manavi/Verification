#ifndef MYALGO_H
#define MYALGO_H

#include "Environment.h"

typedef boost::unordered_map<std::string, std::vector<std::pair<Robot*, double>>> Neighbor_Matrix;

class MyAlgo
{
public:
    MyAlgo(Environment *environment_, Point_2 dest):environment(environment_)
    {
        destination = dest;
    }

    Status init_status()
    {
        Status status;
        std::vector<Robot_mem> robots_status;
        std::vector<Robot>* robots = environment->getRobots();
		for (auto robot = robots->begin(); robot != robots->end(); ++robot)
		{

		}

        return status;
    }

    void get_neighbor_inf(std::vector<Robot_mem> *all_robots)
    {
		std::vector<std::pair<Robot_mem*, double>> neighbor_inf;
        for(auto it1 = all_robots->begin() ; it1 != all_robots->end() ; ++it1)
        {
            for(auto it2 = all_robots->begin() ; it2 != all_robots->end() ; ++it2)
            {
                if(it1->getRobot() == it2->getRobot())
                    continue;

                if(it1->getRobot()->communication_check(*it2->getRobot()))
                    neighbor_inf.push_back(std::make_pair(&*it2, 0));
            }

            neighbor_mat[it1->getRobot()->get_Name()] = neighbor_inf;
            neighbor_inf.clear();
        }

        //for(auto it = neighbor_mat.be ; it != neighbor_mat.end() ; ++it)
        //{
        //    for(auto neighbor = it->second ; neighbor != it->second)
        //}

        //double Num_neighbor = neighbor.size();
        //double w = Num_neighbor/it1->getNumber_of_robots();
        //for(auto it = neighbor.begin() ; it != neighbor.end() ; ++it)
        //    neighbor_inf.push_back(make_pair(&*it, calculate_weight((*it)->getPosition(), destination, w)) );


        //return neighbor_inf;
    }

    std::pair<double, std::vector<Movement_Direction*>> Algo(Robot_mem &mem, std::vector<Robot_mem> *all_robots)
    {
        //double temp_weight = clculate_weight(mem.getPosition(), destination, environment->mem.getNumber_of_robots());
    }

private:
    Environment *environment;
    Neighbor_Matrix neighbor_mat;
    Point_2 destination;

    double calculate_weight(Point_2 &current_pos, Point_2 &destination, double w)
    {
        BigRat x_dis = destination.x() - current_pos.x();
		BigRat y_dis = destination.y() - current_pos.y();

        double weight = abs(x_dis.doubleValue()) + abs(y_dis.doubleValue());

        return weight*w;

    }

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
};




#endif // MYALGO_H
