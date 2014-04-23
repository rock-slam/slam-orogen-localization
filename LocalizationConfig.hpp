#ifndef GRAPH_SLAM_TASK_CONFIG_HPP_
#define GRAPH_SLAM_TASK_CONFIG_HPP_

#include <string>
#include <base/Time.hpp>
#include <base/Pose.hpp>

namespace localization
{
    
struct PoseProviderUpdate
{
    base::Time time;
    base::Pose body2world;
    base::Pose body2odometry;
    
    PoseProviderUpdate() :time(base::Time::now()) {}
};

struct GICPConfiguration
{
    /** The maximum distance threshold between two correspondent points in source <-> target */
    double max_correspondence_distance;
    
    /** The maximum number of iterations the internal optimization should run for */
    unsigned maximum_iterations;
    
    /** The transformation epsilon (maximum allowable difference between two consecutive transformations) */
    double transformation_epsilon;
    
    /** The maximum allowed Euclidean error between two consecutive steps in the ICP loop */
    double euclidean_fitness_epsilon;
    
    /** The number of neighbors used when selecting a point neighbourhood */
    unsigned correspondence_randomness;
    
    /** Maximum number of iterations at the optimization step */
    unsigned maximum_optimizer_iterations;
    
    /** Set the rotation epsilon (maximum allowable difference between two consecutive rotations) */
    double rotation_epsilon;
    
    /** Maximum count of samples used to align the input cloud */
    unsigned int max_input_sample_count;
    
    /** Maximum mean square error of the final transformation between input cloud and model */
    double max_mean_square_error;
    
    /** Odometry delta in meter, after which a new icp match is performed */
    double icp_match_interval;
    
    /** Time in seconds, after which a new icp match is performed */
    double icp_match_interval_time;

    GICPConfiguration() : max_correspondence_distance(2.5),
			maximum_iterations(50), transformation_epsilon(1e-5),
			euclidean_fitness_epsilon(1.0), correspondence_randomness(20),
			maximum_optimizer_iterations(20), rotation_epsilon(2e-3),
			max_input_sample_count(10000), max_mean_square_error(1.0), 
			icp_match_interval(2.0), icp_match_interval_time(1.0) {}
};

}

#endif
