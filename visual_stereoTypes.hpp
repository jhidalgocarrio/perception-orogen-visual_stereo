#ifndef visual_stereo_TYPES_HPP
#define visual_stereo_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Time.hpp>
#include <base/Eigen.hpp>

namespace visual_stereo {

    /**  Type for image output **/
    enum IMAGE_OUTPUT_TYPE{INTRA_MATCHES, INTER_KEYPOINTS};

    struct Info
    {
        base::Time time;
        double num_matches;
        double num_inliers;
        double ratio_inliers;
        base::Time compute_time;
    };

}

#endif

