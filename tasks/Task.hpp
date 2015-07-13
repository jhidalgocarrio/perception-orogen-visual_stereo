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

/** Boost **/
#include <boost/uuid/uuid.hpp>
#include <boost/unordered_map.hpp>

/** Standard **/
#include <cmath>
#include <vector>
#include <map>


namespace visual_stereo {

    struct Feature
    {
        int img_idx;
        cv::KeyPoint keypoint;
        cv::Mat descriptor;
        base::Vector3d point;
        base::Matrix3d cov; /** Covariance matrix of the 3d point**/
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
        int frame_idx; // incremental stereo pair index
        base::samples::frame::FramePair frame_pair; /** Left and right images **/
        frame_helper::FrameHelper frameHelperLeft, frameHelperRight; /** Frame helper **/
        ::base::Matrix2d pxleftVar, pxrightVar; /** Error variance of image plane in pixel units **/
        Eigen::Matrix4d Q; /** Re-projection matrix **/
        cv::detail::ImageFeatures fcurrent_left, fcurrent_right, fprevious_left, fprevious_right;
        std::vector< cv::DMatch > intra_matches, inter_matches_left, inter_matches_right;
        cv::detail::ImageFeatures ffinal_left, ffinal_right;

        boost::unordered_map<boost::uuids::uuid, Feature> features_hash; /** current to previous index **/

        /***************************/
        /** Output Port Variables **/
        /***************************/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame_out; /** Debug intra frame image **/

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
                std::vector<cv::KeyPoint> &keypoints1,
                std::vector<cv::KeyPoint> &keypoints2,
                std::vector<cv::DMatch> &matches);

        void intraMatches(const cv::detail::ImageFeatures &features_left,
                    const cv::detail::ImageFeatures &features_right,
                    const std::vector<cv::DMatch> &matches_left,
                    const std::vector<cv::DMatch> &matches_right,
                    cv::detail::ImageFeatures &final_left,
                    cv::detail::ImageFeatures &final_right,
                    std::vector<cv::DMatch> &intra_matches);

         void hashFeatures(const cv::Mat &new_descriptors);
    };
}

#endif

