#include "dispatch.h"

Dispatch::Dispatch(Environment *env):environment(env)
{
	Number_of_Robot = environment->getRobots()->size();
    buffer_size = 50000000;
}

void Dispatch::init()
{
    // log.txt exist then creat new log.txt and delete
	std::ofstream logfile("log.txt");
	logfile.close();

	const char* path = "Buffer";
	boost::filesystem::path dir(path);
	if (boost::filesystem::create_directory(dir))
	{
		std::cerr << "Directory Created: Buffer" << std::endl;
	}
}

void Dispatch::Turn_taking()
{
	init();

	int temp = environment->getNum_of_State();
	std::clock_t next_state_change = std::clock();

	std::string log_;

	Alpha alpha(environment);
	next_states = alpha.turn_taking_init();

	int it_size = 3000;

	std::clock_t start = std::clock();
	BigRat i = 1;
	bool exit = false;
	while (!exit)
	{
		for (auto situation = next_states.begin() ; situation != next_states.end() ; situation++)
		{
			environment->jump(*situation);

			log_ += "Turn : " + i.get_str() + "\n";
			std::vector<Robot_mem>* robots_stat = situation->getRobot_status();
			for (auto robot = robots_stat->begin(); robot != robots_stat->end(); ++robot)
			{
				log_ += robot->getRobot()->get_Name() + "  :\n" +
					toStr(robot->getRobot()->get_position()) + "\n";

				std::vector<std::pair<double, Movement_Direction*>> movement =
					alpha.Alpha_(*robot, 2, 10);

				if (movement.empty())
					log_ += robot->getCurrent_dir()->get_direction_name() + "---> Freez \n";

				
				for (auto it = movement.begin(); it != movement.end(); ++it)
				{
					int j = environment->move(&*robot, 2, it->first, it->second);

					alpha.Refresh_mem_TT(*robot, robot->getRobot()->get_position(), it->second);

					log_ += robot->getCurrent_dir()->get_direction_name() + "  (" + std::to_string(j) + ")  --->  "
							+ toStr(robot->getRobot()->get_position()) + "\n";
				}

				if (GetAsyncKeyState(VK_ESCAPE))
					exit = exit_();
				if (exit)
					break;

				if (temp != environment->getNum_of_State())
				{
					next_state_change = std::clock();
					temp = environment->getNum_of_State();
				}

				print_inf(i, start, next_state_change);

			}

				log(log_);

			i++;

			if (exit)
				break;
		}
	}
}

std::string Dispatch::time_difference(std::clock_t &start)
{
	double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	std::string  time = "Time :  ";
	time += std::to_string((int)(duration / 60)) + "m ";
	duration = fmod(duration, 60);
	time += std::to_string((int)(duration)) + "s";

	return time;
}

void Dispatch::print_inf(BigRat &turn, std::clock_t &start, std::clock_t &next_state_change)
{
	std::cout << time_difference(start)
		<< "  Turn : " << turn
		<< "  Number of State :  " << environment->getNum_of_State()
		<< "  " << time_difference(next_state_change) << " "
		<< "  Next State Size : " << next_states.size()
		<< "  Buffer Size : " << buffer.size() << "      ";
	std::printf("\r");
}

bool Dispatch::exit_()
{
	std::cout << std::endl;
	while (true)
	{
		std::cout << "Do you want to exit ? (y/n)  ";
		std::string a;
		std::cin >> a;
		if (a == "y" || a == "Y")
			return true;
		else if (a == "n" || a == "N")
			return false;
		std::cout << "\r";
	}
}

void Dispatch::Save_situation(Status &status)
{
    if(!buffering)
    {
        if(next_states.size() < buffer_size)
            next_states.push_back(status);
        else {
            buffering = true;
            Save_situation(status);
        }
    }else {
        if(buffer.size() == buffer_size)
        {
            Save(buffer, bufferId_exist_on_disk[1]);
			bufferId_exist_on_disk[1]++;
            buffer.clear();
            buffer.push_back(status);
        }else {
            buffer.push_back(status);
        }
    }

}

Status Dispatch::get_situation()
{

    if(next_states.size() > 0)
    {
        Status status = next_states.front();
        next_states.erase(next_states.begin());
        return status;
    }
    else if (next_states.size() == 0)
    {
        next_states = retrieve(bufferId_exist_on_disk[0]);
        bufferId_exist_on_disk[0]++;
        buffering = false;

        Status status = next_states.front();
        next_states.erase(next_states.begin());
        return status;
    }

}

Status Dispatch::record_status(Status *prev, std::vector<Robot_mem>::iterator it,
                               Robot_mem new_mem)
{
    Robot_mem temp = *it;
    *it = new_mem;
    Status new_status = *prev;
    *it = temp;
    new_status.setState(environment->current_state());
    return new_status;
}

void Dispatch::Save(std::vector<Status> &status ,int buffer_num)
{
    std::string file_name = ".\\Buffer\\buff" + std::to_string(buffer_num);
    std::ofstream ofs(file_name, std::ios::binary);

    std::vector<Robot_mem> inf;
    for(auto it = status.begin() ; it != status.end() ; ++it)
    {
        inf = *it->getRobot_status();
		for(auto it = inf.begin() ; it != inf.end() ; ++it)
			ofs.write(reinterpret_cast<char*>(&*it), sizeof(Robot_mem));
    }

    ofs.close();

    //std::cout << "Save : " << status.size() << std::endl;
}


std::vector<Status> Dispatch::retrieve(int buffer_num)
{
    std::string file_name = ".\\Buffer\\buff" + std::to_string(buffer_num);
    std::ifstream ifs(file_name, std::ios::binary);
	
    std::vector<Status> read;
	std::vector<Robot_mem> vec;
    Robot_mem inf = Robot_mem();

	while (ifs.read(reinterpret_cast<char*>(&inf), sizeof(Robot_mem)))
	{
		vec.push_back(inf);
		if (vec.size() == Number_of_Robot)
		{
			Status status;
			//status.setStatus(vec.front().getCurrent_state(), &vec);
			read.push_back(status);
			vec.clear();
		}
	}

    ifs.close();

	std::remove(file_name.c_str());
	vec.clear();
    return read;
}

void Dispatch::log(std::string &log)
{
	std::ofstream logfile;
	logfile.open("log.txt", std::ios_base::app);
	logfile << "\n" << log;

	logfile.close();

	log = "";
}





