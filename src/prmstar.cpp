#include "prmstar.h"

namespace optimal_tntp
{

    double distance(const RobotState &a, const RobotState &b)
    {
        double dis = 0;
        for (unsigned int i = 0; i < 6; ++i)
            dis += fabs(angles::normalize_angle(a.joint_[i] - b.joint_[i]));

        return dis;
    }

    double pathLength(const std::vector<RobotState> &the_motion)
    {
        // This is the 6D path length
        if (the_motion.size() <= 1)
            return 100000.0;

        double L = 0;
        for (auto iter = the_motion.begin(); iter != the_motion.end() - 1; ++iter)
            L += distance(*iter, *std::next(iter));

        return L;
    }

    bool PRMstar::isCollisionFreeMotion(const RobotState &a, const RobotState &b)
    {
        double dis = distance(a, b);
        int count = ceil(dis / maxCollisionCheckingDistance_);

        std::vector<double> d_motion;
        d_motion.resize(6);
        for (unsigned int i = 0; i < 6; ++i)
            d_motion[i] = angles::normalize_angle(b.joint_[i] - a.joint_[i]) / count;

        RobotState temp;
        for (unsigned int i = 0; i <= count; ++i)
        {
            for (unsigned int j = 0; j < 6; ++j)
            {
                temp.joint_[j] = a.joint_[j] + i * d_motion[j];
            }
            bool b = collisionChecker_(&(temp.joint_));
            if (!b)
                return false;
        }

        return true;
    }

    void PRMstar::findPath(const RobotState &source, const RobotState &target, std::vector<RobotState> &result)
    {
        std::cout << "PRMstar is asked to find Path from [";
        for (unsigned int i = 0; i < 6; ++i)
            std::cout << source.joint_[i] << " ";

        std::cout << "] to [";
        for (unsigned int i = 0; i < 6; ++i)
            std::cout << target.joint_[i] << " ";

        std::cout << "]" << std::endl;

        int source_milestone_index = add_milestone(source);
        int target_milestone_index = add_milestone(target);

        std::cout << "source_index: " << source_milestone_index << ", target_index: " << target_milestone_index << std::endl;

        std::vector<boost::graph_traits<
            boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
                                  boost::no_property, boost::property<boost::edge_weight_t, double>>>::vertex_descriptor>
            vPredecessor(boost::num_vertices(g_));

        std::vector<double> vDistance(boost::num_vertices(g_));

        boost::graph_traits<
            boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
                                  boost::no_property, boost::property<boost::edge_weight_t, double>>>::vertex_descriptor s = boost::vertex(source_milestone_index, g_);

        boost::property_map<
            boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
                                  boost::no_property, boost::property<boost::edge_weight_t, double>>,
            boost::vertex_index_t>::type pmpIndexmap = boost::get(boost::vertex_index, g_);

        boost::dijkstra_shortest_paths(g_, s, &vPredecessor[0], &vDistance[0], edge_weight_map_, pmpIndexmap,
                                       std::less<double>(), boost::closed_plus<double>(),
                                       std::numeric_limits<double>::max(), 0, boost::default_dijkstra_visitor());

        std::cout << "We start get Path from " << source_milestone_index << " to " << target_milestone_index << std::endl;

        bool b = getPath(source_milestone_index, target_milestone_index, vPredecessor, result);
        if (b)
            std::cout << "findPath finished. pathLength: " << result.size() << ", joint_space path length: " << pathLength(result) << std::endl;

        else
        {
            for (unsigned int i = 0; i < 20; ++i)
                add_milestone();

            findPath(source, target, result);
        }
    }

    bool PRMstar::getPath(int fromId, int toId, std::vector<boost::graph_traits<boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, unsigned long>>>::vertex_descriptor> &vPredecessor,
                          std::vector<RobotState> &result)
    {
        std::vector<int> vecPath;
        while (fromId != toId)
        {
            vecPath.push_back(toId);
            std::cout << "toId = " << toId << ", vPredecessor.size() = " << vPredecessor.size() << std::endl;
            if (toId > vPredecessor.size() - 1 || toId == vPredecessor[toId])
            {
                std::cout << "PRMstar got a dead end. We need to create more milestones and re-do the pathfinding." << std::endl;
                return false;
            }

            toId = vPredecessor[toId];
            std::cout << "toId = " << toId << ", ";
        }
        vecPath.push_back(toId);

        result.resize(vecPath.size());
        for (unsigned int i = 0; i < vecPath.size(); ++i)
            result[i] = all_milestones_[vecPath[vecPath.size() - 1 - i]];

        return true;
    }

    void PRMstar::dijkstraInPRM(const RobotState &source) {}

    int PRMstar::add_milestone()
    {
        RobotState ran_joint_angles;
        while (1)
        {
            bool b = collisionChecker_(&ran_joint_angles.joint_);
            if (b)
                break;
            ran_joint_angles = RobotState();
        }

        return add_milestone(ran_joint_angles.joint_);
    }

    int PRMstar::add_milestone(const RobotState &temp)
    {
        return add_milestone(temp.joint_);
    }

    int PRMstar::add_milestone(const std::vector<double> &temp)
    {
        // We check whether there has been an existing milestone
        for (unsigned int i = 0; i < all_milestones_.size(); ++i)
        {
            double dis = 0;
            for (unsigned int j = 0; j < all_milestones_[i].joint_.size(); ++j)
                dis += fabs(angles::normalize_angle(all_milestones_[i].joint_[j] - temp[j]));

            if (dis < 0.001)
                return i;
        }

        // We create a new milestone
        RobotState ran_joint_angles(&(temp[0]));
        all_milestones_[milestone_count_] = ran_joint_angles;
        all_milestones_[milestone_count_].index_ = milestone_count_;

        if (milestone_count_ > 1)
            add_edge(milestone_count_);

        return milestone_count_++;
    }

    void PRMstar::add_edge(int new_milestone_index)
    {
        // Here milestone_count_ has been added 1, so it is 1 larger than the index of the latest vertex
        for (unsigned int i = 0; i < milestone_count_; ++i)
        {
            const RobotState &old_milestone = all_milestones_[i];
            double dis = distance(old_milestone, all_milestones_[new_milestone_index]);

            if (dis < maxConnectionDistance_)
            {
                // We need to verify that the motion between milestones is indeed collision-free
                bool b = isCollisionFreeMotion(old_milestone, all_milestones_[new_milestone_index]);
                if (b)
                {
                    // The motion is indeed connectable
                    boost::graph_traits<
                        boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property,
                                              boost::property<boost::edge_weight_t, double>>>::edge_descriptor edEdge;

                    bool bInserted;

                    boost::tie(edEdge, bInserted) = boost::add_edge(i, new_milestone_index, g_);
                    edge_weight_map_[edEdge] = dis;
                    // boost::tie(edEdge, bInserted) = boost::add_edge(new_milestone_index, i, g_);
                    // edge_weight_map_[edEdge] = dis;

                    all_edges_[edge_count_] = Edge(edge_count_, i, new_milestone_index, dis);
                    ++edge_count_;
                }
            }
        }
    }

    void PRMstar::refineRoadmap(double t)
    {
        ros::WallTime t_start = ros::WallTime::now();
        while (1 && ros::ok())
        {
            if ((ros::WallTime::now() - t_start).toSec() < t)
                add_milestone();

            else
                break;
        }
    }
};