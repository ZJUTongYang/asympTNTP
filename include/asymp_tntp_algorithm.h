#pragma once


#include <boost/graph/adjacency_list.hpp>
#include <iterator>
#include <utility>
#include <algorithm>
#include <vector>

#include "ur5_moveit_perception/definitions.h"

namespace optimal_tntp_algorithm
{

void defineStates(std::vector<TaskSpaceWaypoint>& TSpoints, 
    double epsilon_cont, 
    std::vector<ContinuousTrackingMotion>& continuous_set);



struct OptimalTNTPSolver
{
public: 
    OptimalTNTPSolver(std::vector<TaskSpaceWaypoint>* TSpoints, 
        std::vector<ContinuousTrackingMotion>* CTM, 
        optimal_tntp::PRMstar* our_prmstar);

    void findInitialGuess(std::vector<optimal_tntp::RobotState>& resultant_tntp_motion_initial_guess, 
    	std::vector<int>& topo_motion_initial_guess);

    void showDetails()
    {

        boost::graph_traits<
            boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, 
                boost::property<boost::edge_weight_t, double> > >::vertex_iterator vi, vend;

        int ver_count = 0;        
        for(boost::tie(vi, vend) = vertices(ctm_rm_g_); vi != vend; ++vi)
            ver_count++;

        std::cout << "OptimalTNTPSolver: Number of CTMs created: " << all_CTMs_.size() << std::endl;
        std::cout << "OptimalTNTPSolver: Number of CTMs in PRM: " << ver_count << std::endl;

        boost::graph_traits<
            boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, 
                boost::property<boost::edge_weight_t, double> > >::edge_iterator ei, eend;

        int edge_count = 0;        
        for(boost::tie(ei, eend) = edges(ctm_rm_g_); ei != eend; ++ei)
            edge_count++;

        std::cout << "OptimalTNTPSolver: Number of RMs created: " << all_RMs_.size() << std::endl;
        std::cout << "OptimalTNTPSolver: Number of RMs in optimal_tntp_solver: " << edge_count << std::endl;


    }


    void addRMMotion(int source_color, int target_color, const std::vector<optimal_tntp::RobotState>& result_temp);

    void updateAllRMs();

    void reportSolutionInSpecificTopology(const std::vector<int>& topo, std::vector<optimal_tntp::RobotState>& path_in_specific_topology);

private: 

    int current_CTM_;
    std::vector<int> goal_CTMs_;

    void createInitialGuess();

    void getOnePath(int fromId, std::vector<int> toId, std::vector<boost::graph_traits<
        boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, 
            boost::no_property, boost::property < boost::edge_weight_t, unsigned long > > >::vertex_descriptor>& vPredecessor, 
        std::vector<int>& result);
    
    bool getPath(int fromId, int toId, std::vector<boost::graph_traits<
	boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, 
		boost::no_property, boost::property < boost::edge_weight_t, unsigned long > > >::vertex_descriptor>& vPredecessor, 
	std::vector<int>& result);

    optimal_tntp::RobotState home_configuration_;

    std::vector<TaskSpaceWaypoint>* TSpoints_;

    std::vector<ContinuousTrackingMotion>* CTM_;

    optimal_tntp::PRMstar* our_prmstarptr_;

    bool with_all_solved_RMs_;

    boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, 
        boost::property<boost::edge_weight_t, double> > ctm_rm_g_;

    boost::property_map<
        boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, 
            boost::property<boost::edge_weight_t, double> >, 
        boost::edge_weight_t>::type rm_cost_map_;

    std::unordered_map<int, ContinuousTrackingMotion> all_CTMs_;
    std::unordered_map<int, ReconfigurationMotion> all_RMs_;

    int rm_count_;

};


};
