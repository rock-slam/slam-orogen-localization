#ifndef LOCALIZATION_TASK_CONFIG_HPP_
#define LOCALIZATION_TASK_CONFIG_HPP_

#include <string>
#include <base/Time.hpp>
#include <base/Pose.hpp>

namespace localization
{

enum SubSampling
{
    None,
    VoxelGrid
    //TODO: add MultiLevelSurface based subsampling
};

struct ICPDebugInformation
{
    base::Time time;
    int successful_alignments;
    int failed_alignments;
    double last_fitness_score;
    double icp_alignment_time;
    ICPDebugInformation() : time(base::Time::now()), successful_alignments(0), failed_alignments(0), last_fitness_score(-1.0), icp_alignment_time(0.0) {}
};
    
struct GICPConfiguration
{
    /** The maximum distance threshold between two correspondent points in source <-> target */
    double max_correspondence_distance;
    
    /** The maximum number of iterations the internal optimization should run for */
    unsigned maximum_iterations;
    
    /** The transformation epsilon (maximum allowable difference between two consecutive transformations) */
    double transformation_epsilon;
    
    /** The number of neighbors used when selecting a point neighbourhood */
    unsigned correspondence_randomness;
    
    /** Maximum number of iterations at the optimization step */
    unsigned maximum_optimizer_iterations;
    
    /** Set the rotation epsilon (maximum allowable difference between two consecutive rotations) */
    double rotation_epsilon;
    
    /** Maximum mean square error of the final transformation between input cloud and model */
    double max_mean_square_error;
    
    /** Odometry delta in meter, after which a new icp match is performed */
    double icp_match_interval;
    
    /** Time in seconds, after which a new icp match is performed */
    double icp_match_interval_time;

    GICPConfiguration() : max_correspondence_distance(2.5),
			maximum_iterations(50), transformation_epsilon(1e-5),
			correspondence_randomness(20), maximum_optimizer_iterations(20),
			rotation_epsilon(2e-3), max_mean_square_error(1.0),
			icp_match_interval(2.0), icp_match_interval_time(1.0) {}
};

}

#endif
