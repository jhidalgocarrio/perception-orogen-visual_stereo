#ifndef visual_stereo_TYPES_HPP
#define visual_stereo_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <vector> //std::vector

#include <boost/uuid/uuid.hpp>
#include <base/Time.hpp>
#include <base/Eigen.hpp>

namespace visual_stereo {

    /**  Type for image output **/
    enum IMAGE_OUTPUT_TYPE{INTRA_MATCHES, INTER_KEYPOINTS, INTRA_AND_INTER};

    struct RansacParameters
    {
        double ransac_max_distance; /** Maximum distance from a point to an epipolar line in pixels */
        double ransac_confidence; /** Desirable level of confidence (probability) */
    };

    struct Info
    {
        base::Time time;
        double num_matches;
        double num_inliers;
        double ratio_inliers;
        base::Time compute_time;
    };

    struct Feature
    {
        boost::uuids::uuid index; // Indexes of the points/samples uses to compute the relative measurement
        base::Vector3d point; // Point cloud used for the delta displacement
        base::Matrix3d cov; // Covariance of the points/samples uses to compute the relative measurement
    };

    struct ExteroFeatures
    {
        base::Time time;
        unsigned int img_idx;
        std::vector<Feature> features;
    };



    typedef boost::uuids::uuid image_uuid;
}

#endif

