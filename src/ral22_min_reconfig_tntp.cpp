#include "definitions.h"
#include <vector>
#include "ral22_min_reconfig_tntp.h"

void min_reconfig_tntp(std::vector<ContinuousTrackingMotion> CTM, std::vector<std::vector<int> >& all_topo_motions)
{
    int cost = 10000;

    int last_tspoint_index = -1;
    for(auto iter = CTM.begin(); iter != CTM.end(); ++iter)
    {
        if(iter->last_point_index_ > last_tspoint_index)
            last_tspoint_index = iter->last_point_index_;
    }

    std::list<std::vector<int> > temp_all_topo_solutions;
    std::vector<int> temp;
    temp.push_back(0);
    temp_all_topo_solutions.push_back(temp);

    while(!temp_all_topo_solutions.empty())
    {
        std::vector<int> temp = temp_all_topo_solutions.front();
        temp_all_topo_solutions.pop_front();

        int last_index_of_temp = temp.back();

        if(CTM[last_index_of_temp].last_point_index_ == last_tspoint_index)
        {
            if(temp.size() < cost)
            {
                all_topo_motions.clear();
                all_topo_motions.push_back(temp);
                cost = temp.size();
            }
            else if(temp.size() == cost)
            {
                all_topo_motions.push_back(temp);
            }
        }

        // We will find all possible subsequent CTMs
        for(auto iter = CTM.begin(); iter != CTM.end(); ++iter)
        {
            if(last_index_of_temp+1 >= iter->first_point_index_ && last_index_of_temp + 1 <= iter->last_point_index_)
            {
                std::vector<int> temptemp = temp;
                temptemp.push_back(iter - CTM.begin());
                temp_all_topo_solutions.push_back(temptemp);
            }
        }

    }    

}

void all_possible_tntp(std::vector<ContinuousTrackingMotion> CTM, std::vector<std::vector<int> >& all_possible_solutions, int next_CTM_index)
{
    int cost = 10000;

    int last_tspoint_index = -1;
    for(auto iter = CTM.begin(); iter != CTM.end(); ++iter)
    {
        if(iter->last_point_index_ > last_tspoint_index)
            last_tspoint_index = iter->last_point_index_;
    }

    std::list<std::vector<int> > temp_all_possible_solutions;
    std::vector<int> temp;
    temp.push_back(next_CTM_index);
    temp_all_possible_solutions.push_back(temp);

    while(!temp_all_possible_solutions.empty())
    {
        std::vector<int> temp = temp_all_possible_solutions.front();
        temp_all_possible_solutions.pop_front();

        int last_index_of_temp = temp.back();

        if(CTM[last_index_of_temp].last_point_index_ == last_tspoint_index)
        {
            all_possible_solutions.push_back(temp);
        }

        // We will find all possible subsequent CTMs
        for(auto iter = CTM.begin(); iter != CTM.end(); ++iter)
        {
            if(last_index_of_temp+1 >= iter->first_point_index_ && last_index_of_temp + 1 <= iter->last_point_index_)
            {
                std::vector<int> temptemp = temp;
                temptemp.push_back(iter - CTM.begin());
                temp_all_possible_solutions.push_back(temptemp);
            }
        }

    }    

}
