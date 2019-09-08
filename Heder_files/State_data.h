#ifndef STATE_DATA_H
#define STATE_DATA_H

#include "sequence.h"

#include <boost/unordered_map.hpp>

typedef boost::unordered_map<std::string, std::vector<std::string>> Connectivity_hash_map;

struct State_Data
{
    State_Data() {}
    ~State_Data() { delete faces_names;}

    int stateId;

    std::vector<std::string> *faces_names;

    Connectivity_hash_map connectivity_matrix;

    std::vector<Sequence> disk_sequences;

    std::vector< std::pair<std::string, State_Data*> > next;

    bool operator ==(const State_Data &state)
    {
        if(*faces_names == *(state.faces_names) &&
                connectivity_matrix == state.connectivity_matrix &&
                disk_sequences == state.disk_sequences )
            return true;
        else
            return false;
    }

//    baray zamani ke robot ha roy path harkat mikonand.
//    std::vector<Point_2> forward, backward;

//    braye zamani ke mikhahim az maping estefade konim va dar hard zakhire konim.
//    std::vector<int> Next_state;

};


#endif // STATE_DATA_H
