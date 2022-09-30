#include "ur5_moveit_perception/optimal_tntp_algorithm.h"
#include <angles/angles.h>

#include "ur5_moveit_perception/definitions.h"


#include <boost/graph/dijkstra_shortest_paths.hpp>


namespace optimal_tntp_algorithm
{

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

void OptimalTNTPSolver::addRMMotion(int source_color, int target_color, const std::vector<optimal_tntp::RobotState>& result_temp)
{
	//// We record the motion into storage
	// We find the edge
	int loc = -1;
	for(unsigned int i = 0; i < rm_count_; ++i)
	{
		if(all_RMs_[i].source_color_ == source_color && all_RMs_[i].target_color_ == target_color)
		{
			loc = i;
			break;
		}
	}
	if(loc == -1)
	{
		std::cout << "Error: we should find an Edge. " << std::endl;
	}

	all_RMs_[loc].latest_construction_time_ = ros::WallTime::now();

	all_RMs_[loc].the_motion_.assign(result_temp.begin(), result_temp.end());
	all_RMs_[loc].pathLength_ = optimal_tntp::pathLength(result_temp);

}


void OptimalTNTPSolver::createInitialGuess()
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
            // Note that the spatial relationship is single-directional
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
                    all_RMs_[rm_count_] = ReconfigurationMotion();
                    all_RMs_[rm_count_].source_color_ = j;
                    all_RMs_[rm_count_].target_color_ = k;
                    all_RMs_[rm_count_].source_last_configuration_index_ = (*CTM_)[j].last_point_index_;
                    all_RMs_[rm_count_].pathLength_ = 1;
                    all_RMs_[rm_count_].the_motion_.clear();

                    ++rm_count_;

                }
            }
        }
    }
}

void OptimalTNTPSolver::findInitialGuess(std::vector<optimal_tntp::RobotState>& resultant_tntp_motion_initial_guess, 
	std::vector<int>& result_motion)
{
    std::vector<boost::graph_traits<
        boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, 
            boost::no_property, boost::property < boost::edge_weight_t, double > > >::vertex_descriptor> vPredecessor(boost::num_vertices(ctm_rm_g_));

    std::vector<double> vDistance(boost::num_vertices(ctm_rm_g_));            

    boost::graph_traits<
        boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, 
            boost::no_property, boost::property < boost::edge_weight_t, double > > >::vertex_descriptor s 
        = boost::vertex(current_CTM_, ctm_rm_g_);

    boost::property_map<
        boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, 
            boost::no_property, boost::property < boost::edge_weight_t, double > >, 
        boost::vertex_index_t>::type pmpIndexmap = boost::get(boost::vertex_index, ctm_rm_g_);
    
    boost::dijkstra_shortest_paths(ctm_rm_g_, s, &vPredecessor[0], &vDistance[0], rm_cost_map_, pmpIndexmap, 
        std::less<double>(), boost::closed_plus<double>(),
        std::numeric_limits<double>::max(), 0, boost::default_dijkstra_visitor());

    // We only need to find one initial guess

    std::cout << "We show possible goal CTMs: " << std::endl;
	for(auto iter = goal_CTMs_.begin(); iter != goal_CTMs_.end(); ++iter)
	{
		std::cout << *iter << " ";
	}
	std::cout << std::endl;


	// std::vector<int> result_motion;
	getOnePath(current_CTM_, goal_CTMs_, vPredecessor, result_motion);

	std::cout << "OptimalTNTPSolver::findInitialGuess: We got one topological solution: " << std::endl;
	for(auto iter = result_motion.begin(); iter != result_motion.end(); ++iter)
	{
		std::cout << *iter << " ";
	}
	std::cout << std::endl;

	// We construct the initial joint-space motion
	resultant_tntp_motion_initial_guess.clear();
	resultant_tntp_motion_initial_guess.push_back(home_configuration_);

	// We only find the reconfiguration motion from the home configuration unitl the finishment of the first tracking motion. 

	std::vector<optimal_tntp::RobotState> the_first_reconfiguration_motion;

	int index_temp = (*CTM_)[result_motion[1]].first_point_index_;
	int color = (*CTM_)[result_motion[1]].color_;
	std::vector<double> first_tracking_joint_angle = (*TSpoints_)[index_temp-1].getConfiguration(color);
	our_prmstarptr_->findPath(home_configuration_, optimal_tntp::RobotState(first_tracking_joint_angle), the_first_reconfiguration_motion);

	// We insert the first reconfiguration motion
	resultant_tntp_motion_initial_guess.insert(resultant_tntp_motion_initial_guess.end(), 
		the_first_reconfiguration_motion.begin()+1, the_first_reconfiguration_motion.end()-1);

	// We append the "color" to the motion
	for(auto iter = resultant_tntp_motion_initial_guess.begin(); iter != resultant_tntp_motion_initial_guess.end(); ++iter)
	{
		iter->joint_.emplace_back(-1.0);
	}

	// We insert the first tracking motion
	int f1 = (*CTM_)[result_motion[1]].first_point_index_ - 1;
	int l1 = (*CTM_)[result_motion[1]].last_point_index_ - 1;

	std::cout << "color, f1, l1 = " << color << ", " << f1 << ", " << l1 << std::endl;

	for(int i = f1; i <= l1; ++i)
	{
		resultant_tntp_motion_initial_guess.push_back((*TSpoints_)[i].getConfiguration(color));
		resultant_tntp_motion_initial_guess.back().joint_.emplace_back(color);
	}
	

	//// We record the motion into storage
	// We find the edge
	int loc = -1;
	for(unsigned int i = 0; i < rm_count_; ++i)
	{
		if(all_RMs_[i].source_color_ == 0 && all_RMs_[i].target_color_ == color)
		{
			loc = i;
			break;
		}
	}
	if(loc == -1)
	{
		std::cout << "Error: we should find an Edge. " << std::endl;
	}

	all_RMs_[loc].source_last_configuration_index_ = 0;

	all_RMs_[loc].latest_construction_time_ = ros::WallTime::now();

	all_RMs_[loc].the_motion_.assign(the_first_reconfiguration_motion.begin(), the_first_reconfiguration_motion.end());
	all_RMs_[loc].pathLength_ = optimal_tntp::pathLength(all_RMs_[loc].the_motion_);



}

void OptimalTNTPSolver::getOnePath(int fromId, std::vector<int> toIds, std::vector<boost::graph_traits<
	boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, 
		boost::no_property, boost::property < boost::edge_weight_t, unsigned long > > >::vertex_descriptor>& vPredecessor, 
	std::vector<int>& shortest_result)
{
	shortest_result.clear();

	int min_length = 100000;
	for(auto iter = toIds.begin(); iter != toIds.end(); ++iter)
	{
		std::cout << "*iter = " << *iter << std::endl;
		std::vector<int> result;
		bool b = getPath(fromId, *iter, vPredecessor, result);
		if(b && result.size() < min_length)
		{
			std::cout << "We find a better solution: toId = " << *iter << std::endl;
			min_length = result.size();
			shortest_result.assign(result.begin(), result.end());
		}
		std::cout << "test 1" << std::endl;
	}
	std::cout << "outside for loop" << std::endl;
}

bool OptimalTNTPSolver::getPath(int fromId, int toId, std::vector<boost::graph_traits<
	boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, 
		boost::no_property, boost::property < boost::edge_weight_t, unsigned long > > >::vertex_descriptor>& vPredecessor, 
	std::vector<int>& result)
{
	result.clear();
	while (fromId != toId)
	{
        std::cout << "toId = " << toId << ", "; 
		result.push_back(toId);
		if(toId > vPredecessor.size()-1 || toId == vPredecessor[toId])
		{
			std::cout << "OptimalTNTPSolver got a dead end. No path found." << std::endl;
			return false;
		}
		toId = vPredecessor[toId];
	}
	result.push_back(toId);

	std::reverse(result.begin(), result.end());

	return true;
}

OptimalTNTPSolver::OptimalTNTPSolver(std::vector<TaskSpaceWaypoint>* TSpoints, 
        std::vector<ContinuousTrackingMotion>* CTM, 
        optimal_tntp::PRMstar* our_prmstar)
{

    // At the beginning, the manipulator must be in the home configuration
    // With the manipulator moving, the current CTM might change, where then the passed CTMs need not be optimised again. 
    current_CTM_ = 0;

	home_configuration_.joint_ = {0.0, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0};

    // We asssume that all task-space points and CTMs have been calculated
    TSpoints_ = TSpoints;
    CTM_ = CTM;

	int last_task_space_point_index = TSpoints->size();
	for(unsigned int i = 0; i < CTM->size(); ++i)
	{
		if((*CTM)[i].last_point_index_ == last_task_space_point_index)
		{
			goal_CTMs_.push_back(i);
		}
	}

    our_prmstarptr_ = our_prmstar;

    with_all_solved_RMs_ = false;

    rm_count_ = 0;

    rm_cost_map_ = boost::get(boost::edge_weight, ctm_rm_g_);

    createInitialGuess();

}

void OptimalTNTPSolver::updateAllRMs()
{
	for(unsigned int i = 0; i < rm_count_; ++i)
	{
		int source_color = all_RMs_[i].source_color_;
		int target_color = all_RMs_[i].target_color_;

		std::vector<double> last_conf_previous_color;
		std::vector<double> first_conf_next_color;

		int l_j = all_CTMs_[source_color].last_point_index_;
		int s = l_j + 1;

		std::cout << "source_color: " << source_color << ", target_color: " << target_color << std::endl;
		std::cout << "l_j: " << l_j << ", s: " << s << std::endl;

		if(source_color == 0)
		{
			// all_RMs_[i].pathLength_ = 10000.0;
			// all_RMs_[i].the_motion_.clear();
			// continue;
			last_conf_previous_color = home_configuration_.joint_;
			first_conf_next_color = (*TSpoints_)[s-1].getConfiguration(target_color);
		}
		else
		{


			last_conf_previous_color = (*TSpoints_)[l_j-1].getConfiguration(source_color);
			first_conf_next_color = (*TSpoints_)[s-1].getConfiguration(target_color);
		}



		std::vector<optimal_tntp::RobotState> result_temp;
		our_prmstarptr_->findPath(optimal_tntp::RobotState(last_conf_previous_color), 
			optimal_tntp::RobotState(first_conf_next_color), result_temp);

		std::cout << "updateAllRMs: update RM from cost " << all_RMs_[i].pathLength_ << " to cost " << optimal_tntp::pathLength(result_temp) << std::endl;
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
		addRMMotion(source_color, target_color, result_temp);

	}
}

void OptimalTNTPSolver::reportSolutionInSpecificTopology(const std::vector<int>& topo, std::vector<optimal_tntp::RobotState>& path_in_specific_topology)
{
	path_in_specific_topology.clear();
	path_in_specific_topology.emplace_back(home_configuration_);
	path_in_specific_topology.back().joint_.emplace_back(-1);

	std::cout << "The solver is asked to report the current optimal solution in topology:";
	for(auto iter = topo.begin(); iter != topo.end(); ++iter)
	{
		std::cout << *iter << " ";
	}
	std::cout << std::endl;

	for(unsigned int i = 0; i < topo.size()-1; ++i)
	{
		// std::cout << "i = " << i << std::endl;
		int source_color = topo[i];
		int target_color = topo[i+1];
		// We find the edge
		bool correct_rm_found = false;
		for(unsigned int j = 0; j < all_RMs_.size(); ++j)
		{
			if(all_RMs_[j].source_color_ == source_color && all_RMs_[j].target_color_ == target_color)
			{
				std::cout << "We find RM " << j << " that connects " << source_color << " and " << target_color << std::endl;
				correct_rm_found = true;
				for(int k = 1; k < all_RMs_[j].the_motion_.size()-1; ++k)
				{
					path_in_specific_topology.emplace_back(all_RMs_[j].the_motion_[k]);
					path_in_specific_topology.back().joint_.emplace_back(-1);
				}
				break;
			}
		}
		if(!correct_rm_found)
		{
			std::cout << "ERROR, we should find some element" << std::endl;
		}

		int l_j = (*CTM_)[source_color].last_point_index_;
		int s = l_j + 1;

		std::cout << "l_j = " << l_j << ", s = " << s << ", (*CTM_)[target_color].last_point_index_-1 = " << (*CTM_)[target_color].last_point_index_-1 << std::endl;

		for(int j = s-1; j <= (*CTM_)[target_color].last_point_index_-1; ++j)
		{
			// std::cout << j << std::endl;
			path_in_specific_topology.emplace_back((*TSpoints_)[j].getConfiguration(target_color));
			path_in_specific_topology.back().joint_.emplace_back(target_color);
		}

	}// For each topo

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