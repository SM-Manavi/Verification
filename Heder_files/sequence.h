#ifndef SEQUENCE_H
#define SEQUENCE_H
#include <Robot.h>

struct Sequence
{
    Sequence(std::string robot_name_) { robot_name = robot_name_; }

    std::string get_robot_name() { return robot_name; }

    void set_left_sequence(std::vector<std::string> &left_seq_)
    {
        left_seq = left_seq_;
        remove_useless(left_seq);
    }
    std::vector<std::string> get_left_sequence() { return left_seq; }

    void set_right_sequence(std::vector<std::string> &right_seq_)
    {
        right_seq = right_seq_;
        remove_useless(right_seq);
    }
    std::vector<std::string> get_right_sequence() { return right_seq; }

    void set_up_sequence(std::vector<std::string> &up_seq_)
    {
        up_seq = up_seq_;
        remove_useless(up_seq);
    }
    std::vector<std::string> get_up_sequence() { return up_seq; }

    void set_down_sequence(std::vector<std::string> &down_seq_)
    {
        down_seq = down_seq_;
        remove_useless(down_seq);
    }
    std::vector<std::string> get_down_sequence() { return down_seq; }

    void remove_useless(std::vector<std::string> &vec)
    {
        vec.erase(std::remove(vec.begin(),vec.end(),""), vec.end());
//        vec.erase(std::remove_if(vec.begin(), vec.end(),
//                                 [](std::string s){if num_of_comma(s)==1
//                                                      return true; else return false;}));
    }

    bool operator ==(const Sequence &second_seq)const
    {
		if(second_seq.robot_name != robot_name)
		{
			printf("**************");
			exit(EXIT_FAILURE);
		}

        if(left_seq == second_seq.left_seq && right_seq == second_seq.right_seq
                && up_seq == second_seq.up_seq && down_seq == second_seq.down_seq)
            return true;
        else
            return false;
    }

    void print()
    {
        std::cout << robot_name << " : " << std::endl;
		std::cout << "Left Sequence  : " ; print_seq(left_seq);
		std::cout << "Right Sequence : " ; print_seq(right_seq);
		std::cout << "Up Sequence    : " ; print_seq(up_seq);
		std::cout << "Down Sequence  : " ; print_seq(down_seq);
		std::cout << std::endl;
    }

    void print_seq(std::vector<std::string> sequence)
    {
        std::vector<std::string>::iterator seq;
        for (seq = sequence.begin(); seq != sequence.end(); ++seq)
            std::cout << "--->" << *seq;
        std::cout << std::endl;
    }


private:
    std::string robot_name;

    std::vector<std::string> left_seq;
    std::vector<std::string> right_seq;

    std::vector<std::string> up_seq;
    std::vector<std::string> down_seq;

};

#endif // SEQUENCE_H
