/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef VISUAL_STEREO_TASK_TASK_HPP
#define VISUAL_STEREO_TASK_TASK_HPP

#include "visual_stereo/TaskBase.hpp"

/** Opencv for the conversion **/
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv/highgui.h>

/** Rock libraries **/
#include <frame_helper/FrameHelper.h> /** Rock lib for manipulate frames **/
#include <frame_helper/FrameHelperTypes.h> /** Types for FrameHelper **/
#include <frame_helper/Calibration.h> /** Rock type for camera calibration parameters **/
#include <frame_helper/CalibrationCv.h> /** Rock type for camera OpenCv calibration **/

/** Rock Types **/
#include <base/Eigen.hpp>
#include <base/samples/Pointcloud.hpp>

/** Boost **/
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/unordered_map.hpp>
#include <boost/lexical_cast.hpp> //to convert int to string in C++03
#include <boost/math/special_functions/round.hpp> // to round a number in standard C++ < 11
#include <boost/crc.hpp> // CRC-32 of the string UUID

/** Standard **/
#include <cmath> // math functions
#include <vector> //std::vector
#include <algorithm>  // std::fill


namespace visual_stereo {

    struct StereoFeature
    {
        int img_idx; /** Image id */
        cv::KeyPoint keypoint_left; /** left keypoint */
        cv::KeyPoint keypoint_right; /** right keypoint */
        cv::Mat descriptor; /** One descriptor (i.e. from the left) */
        base::Vector3d point; /** 3D point */
        base::Matrix3d cov; /** Covariance matrix of the 3d point */

        StereoFeature(const int _img_idx,
                    const cv::KeyPoint &_keypoint_left,
                    const cv::KeyPoint &_keypoint_right,
                    const cv::Mat &_descriptor)
        {
            img_idx = _img_idx;
            keypoint_left = _keypoint_left;
            keypoint_right = _keypoint_right;
            descriptor = _descriptor;
        }
    };

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

    The corresponding C++ class can be edited in tasks/Task.hpp and
    tasks/Task.cpp, and will be put in the visual_stereo namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','visual_stereo::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        static const int DEFAULT_CIRCULAR_BUFFER_SIZE = 2;

        /**************************/
        /*** Property Variables ***/
        /**************************/
        //Intrinsic and extrinsic parameters for the pinhole camera model
        frame_helper::StereoCalibration cameracalib;

        /** Open Cv calibration for the perspective transformation Matrix Q **/
        frame_helper::StereoCalibrationCv cameracalibCv;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/
        unsigned short computing_counts, left_computing_idx, right_computing_idx; //integer to control the period
        int frame_idx; // incremental stereo pair index
        unsigned int frame_window_hash_size; // number of frame history to keep in the hash
        base::samples::frame::FramePair frame_pair; /** Left and right images **/
        frame_helper::FrameHelper frameHelperLeft, frameHelperRight; /** Frame helper **/
        ::base::samples::frame::Frame left_color_frame;/** coloring point clouds (if selected) */
        ::base::Matrix2d pxleftVar, pxrightVar; /** Error variance of image plane in pixel units **/
        Eigen::Matrix4d Q; /** Re-projection matrix **/
        cv::detail::ImageFeatures fcurrent_left, fcurrent_right, fprevious_left, fprevious_right;
        std::vector< cv::DMatch > intra_matches, inter_matches_left, inter_matches_right;
        cv::detail::ImageFeatures ffinal_left, ffinal_right;

        boost::unordered_map<boost::uuids::uuid, StereoFeature> hash_features; /** current to previous index **/

        /***************************/
        /** Output Port Variables **/
        /***************************/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> intra_frame_out; /** Debug intra frame image **/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> inter_frame_out; /** Debug inter frame image **/

    protected:

        virtual void left_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample);

        virtual void right_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "visual_stereo::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	    ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        void prepareMatches();

        void detectFeatures (const base::samples::frame::Frame &frame_left,
                        const base::samples::frame::Frame &frame_right,
                        cv::detail::ImageFeatures &features_left,
                        cv::detail::ImageFeatures &features_right);

        void interMatches (const cv::detail::ImageFeatures &features_previous,
                        const cv::detail::ImageFeatures &features_current,
                        std::vector< cv::DMatch > &good_matches);

        void drawMatches(const base::samples::frame::Frame &frame1,
                const base::samples::frame::Frame &frame2,
                std::vector<cv::KeyPoint> &keypoints1,
                std::vector<cv::KeyPoint> &keypoints2,
                std::vector<cv::DMatch> &matches);

        void drawKeypoints(const base::samples::frame::Frame &frame2,
                const std::vector<cv::KeyPoint> &keypoints1,
                const std::vector<cv::KeyPoint> &keypoints2,
                const std::vector<cv::DMatch> &matches,
                const boost::unordered_map<boost::uuids::uuid, StereoFeature> &hash);

        void intraMatches(const cv::detail::ImageFeatures &features_left,
                    const cv::detail::ImageFeatures &features_right,
                    const std::vector<cv::DMatch> &matches_left,
                    const std::vector<cv::DMatch> &matches_right,
                    cv::detail::ImageFeatures &final_left,
                    cv::detail::ImageFeatures &final_right,
                    std::vector<cv::DMatch> &intra_matches);

        void hashFeatures (const cv::detail::ImageFeatures &new_features_left,
                        const cv::detail::ImageFeatures &new_features_right,
                        const std::vector< cv::DMatch > &good_matches);

        void cleanHashFeatures (const int current_idx,
                        const int max_number_img,
                        boost::unordered_map<boost::uuids::uuid, StereoFeature> &hash);

        void featuresOut(const int current_image_idx,
                        const boost::unordered_map<boost::uuids::uuid, StereoFeature> &hash);

        int getCRC32(const std::string& my_string)
        {
            boost::crc_32_type result;
            result.process_bytes(my_string.data(), my_string.length());
            return result.checksum();
        };
    };
}

#endif

