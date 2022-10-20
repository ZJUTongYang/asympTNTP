#pragma once

#include <moveit/planning_scene/planning_scene.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <iterator>
#include <utility>
#include <algorithm>
#include <vector>
#include <time.h>
#include <angles/angles.h>


namespace optimal_tntp{

struct RobotState{  
    int index_;
    std::vector<double> joint_;

    RobotState(){
        index_ = -1;
        joint_.resize(6);
        for(unsigned int i = 0; i < 6; ++i)
            joint_[i] = (rand() % 9999) / 9999.0 * 2*M_PI;
    }

    RobotState(const double* joint){
        index_ = -1;
        joint_.resize(6);
        memcpy(&(joint_[0]), joint, 6*sizeof(double));
    }

    RobotState(const std::vector<double>& joint){
        index_ = -1;
        joint_.resize(6);
        memcpy(&(joint_[0]), &joint[0], 6*sizeof(double));
    }

    void cout(){
        for(auto iter = joint_.begin(); iter != joint_.end(); ++iter)
            std::cout << *iter << " ";
        std::cout << std::endl;
    }
};

double pathLength(const std::vector<RobotState>& the_motion);

struct Edge
{
    int index_;
    int startindex_;
    int endindex_;
    double cost_;
    Edge(){}

    Edge(int index, int startindex, int endindex, double cost){
        index_ = index;
        startindex_ = startindex;
        endindex_ = endindex;
        cost_ = cost;
    }
};

class PRMstar
{
public: 
    PRMstar(std::function<bool(const std::vector<double>*)> collisionChecker)
    {
        g_.clear();
        all_milestones_.clear();
        all_edges_.clear();

        milestone_count_ = 0;
        edge_count_ = 0;

        maxConnectionDistance_ = 5;
        maxCollisionCheckingDistance_ = 0.1;

        collisionChecker_ = collisionChecker;

        edge_weight_map_ = boost::get(boost::edge_weight, g_);
    }

    int add_milestone();
    int add_milestone(const std::vector<double>& temp);
    int add_milestone(const RobotState& temp);

    void add_edge(int new_milestone_index);

    void findPath(const RobotState& source, const RobotState& target, std::vector<RobotState>& result);

    void solve(const RobotState& start, const std::vector<RobotState>& goals, std::vector<std::vector<RobotState> >& motions);

    void dijkstraInPRM(const RobotState& source);

    void refineRoadmap(double t);

    void showDetails()
    {

        boost::graph_traits<
            boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, 
                boost::property<boost::edge_weight_t, double> > >::vertex_iterator vi, vend;

        int ver_count = 0;        
        for(boost::tie(vi, vend) = vertices(g_); vi != vend; ++vi)
            ver_count++;

        std::cout << "Number of milestones created: " << all_milestones_.size() << std::endl;
        std::cout << "Number of milestones in PRM: " << ver_count << std::endl;

        boost::graph_traits<
            boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, 
                boost::property<boost::edge_weight_t, double> > >::edge_iterator ei, eend;

        int edge_count = 0;        
        for(boost::tie(ei, eend) = edges(g_); ei != eend; ++ei)
            edge_count++;

        std::cout << "Number of edges created: " << all_edges_.size() << std::endl;
        std::cout << "Number of edges in PRM: " << edge_count << std::endl;

        // for (auto e : all_milestones_){
        //     for (auto e1 : e.second.joint_){
        //         std::cout << e1 << ";";
        //     }
        //     std::cout << std::endl;
        // }

    }

private:

    bool getPath(int fromId, int toId, std::vector<boost::graph_traits<
        boost::adjacency_list < boost::listS, boost::vecS, boost::directedS, 
            boost::no_property, boost::property < boost::edge_weight_t, unsigned long > > >::vertex_descriptor>& vPredecessor, 
        std::vector<RobotState>& result);

    bool isCollisionFreeMotion(const RobotState& a, const RobotState& b);

    boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, 
        boost::property<boost::edge_weight_t, double> > g_;
    
    boost::property_map<
        boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, 
            boost::property<boost::edge_weight_t, double> >, 
        boost::edge_weight_t>::type edge_weight_map_;
    
    std::unordered_map<int, RobotState> all_milestones_;
    int milestone_count_;
    std::unordered_map<int, Edge> all_edges_;
    int edge_count_;

    std::function<bool(const std::vector<double>*)> collisionChecker_;

    double maxCollisionCheckingDistance_;
    double maxConnectionDistance_;

};


};