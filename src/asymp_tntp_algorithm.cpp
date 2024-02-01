#include "asymp_tntp_algorithm.h"

namespace optimal_tntp_algorithm{

void defineStates(std::vector<TaskSpaceWaypoint>& TSpoints, double epsilon_cont, 
    std::vector<ContinuousTrackingMotion>& CTM)
{
	// The index of the next colour to be assigned
	int colorindex = 0;
	CTM.clear();
	TaskSpaceWaypoint frontier_vertex;
	int frontier_vertex_index = -1;
	

	int i = 0; 
	while (i < TSpoints.size() && ros::ok())
	{
		if (TSpoints[i].iks_.size() == 0)
		{
			std::cout << "Error: we should not have an unreachable task-space waypoint. We should set another task-space curve for test. " << std::endl;
			frontier_vertex_index = -1;
			continue;
		}

		if (frontier_vertex_index == -1)
		{
			// Find the first uncoloured IK to be the seed
			for (unsigned int j = 0; j < TSpoints[i].iks_.size(); ++j)
			{
				if (TSpoints[i].color_[j] == -1)
				{
					TSpoints[i].color_[j] = colorindex;
					if (CTM.size() < colorindex + 1)
					{
						CTM.resize(colorindex + 1);
					}
					CTM[colorindex].first_point_index_ = i;

					++colorindex;
				}
			}
			frontier_vertex = TSpoints[i];
			frontier_vertex_index = i;
			++i;
			continue;

		}

		std::vector<std::vector<double> > adj_matrix;

		// Find the nearest row to the seed
		int m = frontier_vertex.iks_.size();
		int n = TSpoints[i].iks_.size();
		std::vector<double> adj_matrix_temp;
		adj_matrix_temp.resize(n);
		adj_matrix.resize(m, adj_matrix_temp);
		for (unsigned int j = 0; j < m; ++j)
		{
			for (unsigned int k = 0; k < n; ++k)
			{
				adj_matrix[j][k] = 0; 
				double temp;
				for (unsigned int l = 0; l < frontier_vertex.iks_[j].size(); ++l)
				{
					temp = fabs(angles::normalize_angle(frontier_vertex.iks_[j][l] - TSpoints[i].iks_[k][l]));
					adj_matrix[j][k] = std::max(adj_matrix[j][k], temp);
				}
			}
		}

		// Match configurations
		int cycle = std::min(m, n);
		while (cycle > 0)
		{
			double mm = adj_matrix[0][0];
			int row = 0;
			int column = 0;
			for (unsigned int j = 0; j < adj_matrix.size(); ++j)
			{
				for (unsigned int k = 0; k < adj_matrix[j].size(); ++k)
				{
					if (adj_matrix[j][k] < mm)
					{
						mm = std::min(mm, adj_matrix[j][k]);
						row = j;
						column = k;
					}
				}
			}
			if (mm >= 998)
				break;

			if (adj_matrix[row][column] < epsilon_cont)
			{
				int color = frontier_vertex.color_[row];
				TSpoints[i].color_[column] = color;
				CTM[color].last_point_index_ = std::max(CTM[color].last_point_index_, i);

				for (unsigned int j = 0; j < adj_matrix[0].size(); ++j)
				{
					adj_matrix[row][j] = 999;
				}
				for (unsigned int j = 0; j < adj_matrix.size(); ++j)
				{
					adj_matrix[j][column] = 999;
				}
			}
			cycle = cycle - 1;
		}



		// Assign left-out configurations
		for (unsigned int j = 0; j < TSpoints[i].iks_.size(); ++j)
		{
			if (TSpoints[i].color_[j] == -1)
			{
				TSpoints[i].color_[j] = colorindex;

				if (CTM.size() < colorindex + 1)
				{
					CTM.resize(colorindex + 1);
				}
				CTM[colorindex].first_point_index_ = i;

				++colorindex;
			}
		}

		// 2022.09.14 YT: we should assign the last point index of the previous CTMs
		auto frontier_color_sequence = TSpoints[i-1].color_;
		// std::cout << "we show frontier_color_sequence" << std::endl;
		// for(auto iter = frontier_color_sequence.begin(); iter != frontier_color_sequence.end(); ++iter)
		// {
		// 	std::cout << *iter << " ";
		// }
		// std::cout <<  std::endl;


		auto new_color_sequence = TSpoints[i].color_;

		// std::cout << "we show new_color_sequence" << std::endl;
		// for(auto iter = new_color_sequence.begin(); iter != new_color_sequence.end(); ++iter)
		// {
		// 	std::cout << *iter << " ";
		// }
		// std::cout <<  std::endl;


		for(unsigned int k = 0; k < frontier_color_sequence.size(); ++k)
		{
			bool continuous = false;
			for(unsigned int j = 0; j < new_color_sequence.size(); ++j)
			{
				if(new_color_sequence[j] == frontier_color_sequence[k])
				{
					continuous = true;
					break;
				}
			}

			if(!continuous)
			{
				CTM[frontier_color_sequence[k]].last_point_index_ = i-1;
				std::cout << "CTM [" << frontier_color_sequence[k] << "].last_point_index = " << i-1 << std::endl;
			}
		}


		frontier_vertex = TSpoints[i];
		++i;
	}
}


OptimalTNTPSolver::OptimalTNTPSolver(std::vector<TaskSpaceWaypoint>* TSpoints, 
        std::vector<ContinuousTrackingMotion>* CTM, 
        optimal_tntp::PRMstar* our_prmstar)
{

    // At the beginning, the manipulator must be in the home configuration
    // With the manipulator moving, the current CTM might change, where then the passed CTMs need not be optimised again. 
    // current_CTM_ = 0;

	// home_configuration_.joint_ = {0.0, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0};

    // We asssume that all task-space points and CTMs have been calculated
    TSpoints_ = TSpoints;
    CTM_ = CTM;

	int last_task_space_point_index = TSpoints->size()-1;
	for(unsigned int i = 0; i < CTM->size(); ++i)
	{
		if((*CTM)[i].last_point_index_ == last_task_space_point_index)
		{
			goal_CTMs_.push_back(i);
		}
	}

    our_prmstarptr_ = our_prmstar;

    // with_all_solved_RMs_ = false;

    rm_count_ = 0;

    rm_cost_map_ = boost::get(boost::edge_weight, ctm_rm_g_);

    initGraphStructure();

}

// void OptimalTNTPSolver::addRMMotion(int source_color, int target_color, const std::vector<optimal_tntp::RobotState>& result_temp)
// {
// 	//// We record the motion into storage
// 	// We find the edge
// 	int loc = -1;
// 	for(unsigned int i = 0; i < rm_count_; ++i)
// 	{
// 		if(all_RMs_[i].source_color_ == source_color && all_RMs_[i].target_color_ == target_color)
// 		{
// 			loc = i;
// 			break;
// 		}
// 	}
// 	if(loc == -1)
// 	{
// 		std::cout << "Error: we should find an Edge. " << std::endl;
// 	}

// 	all_RMs_[loc].latest_construction_time_ = ros::WallTime::now();

// 	all_RMs_[loc].the_motion_.assign(result_temp.begin(), result_temp.end());
// 	all_RMs_[loc].pathLength_ = optimal_tntp::pathLength(result_temp);

// }


void OptimalTNTPSolver::initGraphStructure()
{
    // We construct the graph structure
    all_CTMs_.clear();
	std::cout << "We check the index of the vertices inserted to the graph" << std::endl;
    for(unsigned int i = 0; i < CTM_->size(); ++i)
    {
        all_CTMs_[i] = (*CTM_)[i];
		boost::adjacency_list<>::vertex_descriptor vtemp = boost::add_vertex(ctm_rm_g_);
		std::cout << vtemp << " ";
    }
	std::cout << std::endl;

    all_RMs_.clear();
    for(unsigned int j = 0; j< CTM_->size()-1; ++j)
    {
        for(unsigned int k = j+1; k < CTM_->size(); ++k)
        {
            // We check whether two CTMs are connectable
            int f_j = (*CTM_)[j].first_point_index_;
            int l_j = (*CTM_)[j].last_point_index_;
            int f_k = (*CTM_)[k].first_point_index_;
            int l_k = (*CTM_)[k].last_point_index_;
            if(f_k <= l_j + 1)
            {
                if(l_j < l_k && f_j < f_k)
                {
                    boost::graph_traits<
                        boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, boost::no_property, 
                            boost::property < boost::edge_weight_t, double > > >::edge_descriptor edEdge;

                    bool bInserted;
                    boost::tie(edEdge, bInserted) = boost::add_edge(j, k, ctm_rm_g_);

                    // The initial guessed cost is always 1
                    // rm_cost_map_[edEdge] = 1;
                    rm_cost_map_[edEdge] = 10000;

                    std::cout << "OptimalTNTPSolver: a new edge from " << j << " to " << k << ", cost: " << rm_cost_map_[edEdge] << std::endl;
                    all_RMs_[rm_count_] = ReconfigurationMotion(j, k);
                    all_RMs_[rm_count_].source_last_configuration_index_ = (*CTM_)[j].last_point_index_;
                    all_RMs_[rm_count_].pathLength_ = 10000;
                    all_RMs_[rm_count_].the_motion_.clear();

                    ++rm_count_;

                }
            }
        }
    }
}


// bool OptimalTNTPSolver::getPath(int fromId, int toId, std::vector<boost::graph_traits<
// 	boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, 
// 		boost::no_property, boost::property < boost::edge_weight_t, unsigned long > > >::vertex_descriptor>& vPredecessor, 
// 	std::vector<int>& result)
// {
// 	result.clear();
// 	while (fromId != toId)
// 	{
//         std::cout << "toId = " << toId << ", "; 
// 		result.push_back(toId);
// 		if(toId > vPredecessor.size()-1 || toId == vPredecessor[toId])
// 		{
// 			std::cout << "OptimalTNTPSolver got a dead end. No path found." << std::endl;
// 			return false;
// 		}
// 		toId = vPredecessor[toId];
// 	}
// 	result.push_back(toId);

// 	std::reverse(result.begin(), result.end());

// 	return true;
// }


void OptimalTNTPSolver::eliminateRMs(int prev_CTM_index, int next_CTM_index)
{
	std::vector<bool> active_CTM;
	active_CTM.resize(all_CTMs_.size(), false);
	active_CTM[next_CTM_index] = true;

	std::vector<int> active_RM;
	active_RM.resize(all_RMs_.size(), false);

	bool stable = true;
	while(1)
	{
		for(unsigned int i = 0; i < all_RMs_.size(); ++i)
		{
			int source = all_RMs_[i].source_color_;
			int target = all_RMs_[i].target_color_;
			if(active_CTM[source] == true && active_CTM[target] == false)
			{
				stable = false;
				active_CTM[target] = true;
			}
		}
		if(stable)
			break;
	}

	for(unsigned int i = 0; i < all_RMs_.size(); ++i)
	{
		int source = all_RMs_[i].source_color_;
		int target = all_RMs_[i].target_color_;
		if(active_CTM[source] && active_CTM[target])
		{
			// active_RM[i] = true;
			all_RMs_[i].no_use_ = false;
			all_RMs_[i].pathLength_ = 10000.0;
			// Note that here we should not remove the paths. 
			// Because we just won't optimise them, we may still print them
		}
		else
		{
			all_RMs_[i].no_use_ = true;
		}
	}



}

void OptimalTNTPSolver::updateAllRMs()
{
	for(unsigned int i = 0; i < rm_count_; ++i)
	{
		// We only update usable motions
		if(all_RMs_[i].no_use_)
		{
			all_RMs_[i].pathLength_ = 10000.0;
			continue;
		}

		int source_color = all_RMs_[i].source_color_;
		int target_color = all_RMs_[i].target_color_;

		// TODO: Here we will change it to arbitrary pose after the code has been tested
		int l_j = all_CTMs_[source_color].last_point_index_;
		int s = l_j + 1;

		std::cout << "source_color: " << source_color << ", target_color: " << target_color << std::endl;
		std::cout << "l_j: " << l_j << ", s: " << s << std::endl;

		std::vector<double> last_conf_previous_color;
		std::vector<double> first_conf_next_color;

		// if(source_color == 0)
		// {
		// 	// all_RMs_[i].pathLength_ = 10000.0;
		// 	// all_RMs_[i].the_motion_.clear();
		// 	// continue;
		// 	last_conf_previous_color = home_configuration_.joint_;
		// 	first_conf_next_color = (*TSpoints_)[s-1].getConfiguration(target_color);
		// }
		// else
		// {

			last_conf_previous_color = (*TSpoints_)[l_j].getConfiguration(source_color);
			first_conf_next_color = (*TSpoints_)[s].getConfiguration(target_color);
		// }



		std::vector<optimal_tntp::RobotState> result_temp;
		our_prmstarptr_->findPath(optimal_tntp::RobotState(last_conf_previous_color), 
			optimal_tntp::RobotState(first_conf_next_color), result_temp);

		std::cout << "updateAllRMs: update RM" << all_RMs_[i].source_color_ << "-->" << all_RMs_[i].target_color_ << " from cost " << all_RMs_[i].pathLength_ << " to cost " << optimal_tntp::pathLength(result_temp) << std::endl;
		std::cout << "We show the udpated motion here: " << std::endl;
		for(auto iter = all_RMs_[i].the_motion_.begin(); iter != all_RMs_[i].the_motion_.end(); ++iter)
		{
			for(auto iter2 = iter->joint_.begin(); iter2 != iter->joint_.end(); ++iter2)
			{
				std::cout << *iter2 << " ";
			}
			std::cout << "; " << std::endl;
		}
		std::cout << std::endl;
		std::cout << "The updated motion is shown above." << std::endl;



		// We store the RM into the solver
		// addRMMotion(source_color, target_color, result_temp);
		all_RMs_[i].latest_construction_time_ = ros::WallTime::now();

		all_RMs_[i].the_motion_.assign(result_temp.begin(), result_temp.end());
		all_RMs_[i].pathLength_ = optimal_tntp::pathLength(result_temp);

	}
}

void OptimalTNTPSolver::reportSolutionInSpecificTopology(const std::vector<int>& topo, 
	std::vector<optimal_tntp::RobotState>& path_in_specific_topology, 
	int last_TSpoint_index, int& first_CTM_end_index)
{
	// In this function, we should have guaranteed that, no colour can be skipped. 
	// In other words, l_{i-1} + 1 < f_{i+1}, so colour i is a must. 

	path_in_specific_topology.clear();
	first_CTM_end_index = -1;

	int rm_index;

	for(int i = 1; i < topo.size(); ++i)
	{
		// We locate the index of RM
		for(rm_index = 0; rm_index < rm_count_; ++rm_index)
		{
			if(all_RMs_[rm_index].source_color_ == topo[i-1] && all_RMs_[rm_index].target_color_ == topo[i])
				break;
		}


		// We find the part of CTM that the manipulator should traverse
		int current_CTM_start_index = last_TSpoint_index+1;
		// int current_CTM_end_index = (*CTM_)[topo[i-1]].last_point_index_;
		int current_CTM_end_index = all_RMs_[rm_index].source_last_configuration_index_;


		int j = current_CTM_start_index;
		while(j <= current_CTM_end_index)
		{
			path_in_specific_topology.emplace_back( (*TSpoints_)[j].getConfiguration(topo[i-1]) );
			path_in_specific_topology.back().joint_.emplace_back(topo[i-1]);
			++j;
		}

		// We find the RM from CTM[topo[i-1]] to CTM[topo[i]]
		std::vector<optimal_tntp::RobotState> current_RM = all_RMs_[rm_index].the_motion_;
		for(auto iter = current_RM.begin(); iter != current_RM.end(); ++iter)
		{
			iter->joint_.emplace_back(-1);
		}

		path_in_specific_topology.insert(path_in_specific_topology.end(), current_RM.begin()+1, current_RM.end()-1);

		last_TSpoint_index = current_CTM_end_index;

		if(first_CTM_end_index == -1)
		{
			first_CTM_end_index = current_CTM_end_index;
		}
	}	

	// We insert the last CTM
	int j = all_RMs_[rm_index].source_last_configuration_index_+1;
	while(j < TSpoints_->size())
	{
		path_in_specific_topology.emplace_back( (*TSpoints_)[j].getConfiguration(topo.back()) );
		path_in_specific_topology.back().joint_.emplace_back(topo.back());
		++j;
	}

	std::cout << "The solver is asked to report the current optimal solution in topology:";
	for(auto iter = topo.begin(); iter != topo.end(); ++iter)
	{
		std::cout << *iter << " ";
	}
	std::cout << "with last TSpoint index " << last_TSpoint_index  << std::endl;

	std::cout << "We cout them all: " << std::endl;
	for(auto iter = path_in_specific_topology.begin(); iter != path_in_specific_topology.end(); ++iter)
	{
		for(auto iter2 = iter->joint_.begin(); iter2 != iter->joint_.end(); ++iter2)
		{
			std::cout << *iter2 << " ";
		}
		std::cout << ";" << std::endl;
	}
	std::cout << std::endl;
}


};