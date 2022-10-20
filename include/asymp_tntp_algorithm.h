#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <iterator>
#include <utility>
#include <algorithm>
#include <vector>
#include <angles/angles.h>

#include "definitions.h"

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

    void eliminateRMs(int prev_CTM_index, int next_CTM_index);

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

    void reportSolutionInSpecificTopology(const std::vector<int>& topo, std::vector<optimal_tntp::RobotState>& path_in_specific_topology, int last_TSpoint_index, int& first_CTM_end_index);

    void updateAllRMs();


private: 

    std::vector<int> goal_CTMs_;

    void initGraphStructure();

    
    // bool getPath(int fromId, int toId, std::vector<boost::graph_traits<
	// boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, 
	// 	boost::no_property, boost::property < boost::edge_weight_t, unsigned long > > >::vertex_descriptor>& vPredecessor, 
	// std::vector<int>& result);

    std::vector<TaskSpaceWaypoint>* TSpoints_;

    std::vector<ContinuousTrackingMotion>* CTM_;

    optimal_tntp::PRMstar* our_prmstarptr_;

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
