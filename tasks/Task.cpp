/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#define DEBUG_PRINTS 1

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

using namespace visual_stereo;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::left_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout << "[VISO2 LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time.toString()<< std::endl;
    #endif

    frame_previous_pair.first = frame_pair.first;

    /** The image need to be in gray scale and undistorted **/
    frame_pair.first.init(left_frame_sample->size.width, left_frame_sample->size.height, left_frame_sample->getDataDepth(), base::samples::frame::MODE_GRAYSCALE);
    frameHelperLeft.convert (*left_frame_sample, frame_pair.first, 0, 0, _resize_algorithm.value(), true);

    /** If the difference in time is less than half of a period run the odometry **/
    base::Time diffTime = frame_pair.first.time - frame_pair.second.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_left_frame_period/2.0))
    {
        frame_pair.time = frame_pair.first.time;

        #ifdef DEBUG_PRINTS
        std::cout<< "[VISO2 LEFT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

        this->prepareMatches();
        this->detectFeatures(frame_pair.first, frame_pair.second, fcurrent_left, fcurrent_right);

        if (this->frame_idx > 1)
        {
            this->interMatches(fprevious_left, fcurrent_left, inter_matches_left);

            this->interMatches(fprevious_right, fcurrent_right, inter_matches_right);

            this->intraFeatures(fcurrent_left.keypoints, fcurrent_right.keypoints,
                                fcurrent_left.descriptors, fcurrent_right.descriptors,
                                inter_matches_left, inter_matches_right,
                                features_left, features_right, intra_matches);

            /** Draw good matches **/
            if (_output_debug.value())
            {
                if (_image_ouput_type.get() == visual_stereo::INTRA_MATCHES)
                {
                    this->drawMatches(frame_pair.first, frame_pair.second, features_left.keypoints, features_right.keypoints, intra_matches);
                }
                else if (_image_ouput_type.get() == visual_stereo::INTER_KEYPOINTS)
                {
                    this->drawKeypoints(frame_pair.first, fprevious_left.keypoints, fcurrent_left.keypoints, inter_matches);
                }
            }
        }
    }

    return;
}

void Task::right_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<< "[VISO2 RIGHT_FRAME] Frame arrived at: " <<right_frame_sample->time.toString()<<std::endl;
    #endif

    frame_previous_pair.second = frame_pair.second;

    /** Correct distortion in image right **/
    frame_pair.second.init(right_frame_sample->size.width, right_frame_sample->size.height, right_frame_sample->getDataDepth(), base::samples::frame::MODE_GRAYSCALE);
    frameHelperRight.convert (*right_frame_sample, frame_pair.second, 0, 0, _resize_algorithm.value(), true);

    /** Check the time difference **/
    base::Time diffTime = frame_pair.second.time - frame_pair.first.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_right_frame_period/2.0))
    {
        frame_pair.time = frame_pair.second.time;

        #ifdef DEBUG_PRINTS
        std::cout<< "[VISO2 RIGHT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

        this->prepareMatches();
        this->detectFeatures(frame_pair.first, frame_pair.second, fcurrent_left, fcurrent_right);

        if (this->frame_idx > 1)
        {
            this->interMatches(fprevious_left, fcurrent_left, inter_matches_left);

            this->interMatches(fprevious_right, fcurrent_right, inter_matches_right);

            this->intraFeatures(fcurrent_left.keypoints, fcurrent_right.keypoints,
                                fcurrent_left.descriptors, fcurrent_right.descriptors,
                                inter_matches_left, inter_matches_right,
                                features_left, features_right, intra_matches);

            /** Draw good matches **/
            if (_output_debug.value())
            {
                if (_image_ouput_type.get() == visual_stereo::INTRA_MATCHES)
                {
                    this->drawMatches(frame_pair.first, frame_pair.second, features_left.keypoints, features_right.keypoints, intra_matches);
                }
                else if (_image_ouput_type.get() == visual_stereo::INTER_KEYPOINTS)
                {
                    this->drawKeypoints(frame_pair.first, fprevious_left.keypoints, fcurrent_left.keypoints, inter_matches);
                }
            }
        }
    }

    return;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;


    /** Frame index **/
    frame_idx = 0;

    /** Read the camera calibration parameters **/
    this->cameracalib = _calib_parameters.value();

    this->fcurrent_left.img_size.width = this->cameracalib.camLeft.width;
    this->fcurrent_left.img_size.height = this->cameracalib.camLeft.height;
    this->fcurrent_right.img_size.width = this->cameracalib.camRight.width;
    this->fcurrent_right.img_size.height = this->cameracalib.camRight.height;

    /** Re-projection Q matrix **/
    this->cameracalibCv.setCalibration(cameracalib);
    this->cameracalibCv.setImageSize(cv::Size(cameracalib.camLeft.width, cameracalib.camLeft.height));
    this->cameracalibCv.initCv();
    cv::cv2eigen(cameracalibCv.Q, this->Q);

    #ifdef DEBUG_PRINTS
    std::cout<< "[VISO2 CONFIGURATION] Q re-projection matrix:\n "<<Q<<"\n";
    #endif

    this->pxleftVar = cameracalib.camLeft.getPixelCovariance();
    this->pxrightVar = cameracalib.camRight.getPixelCovariance();

    #ifdef DEBUG_PRINTS
    std::cout<< "[VISO2 CONFIGURATION] Left Frame Error matrix:\n "<<pxleftVar<<"\n";
    std::cout<< "[VISO2 CONFIGURATION] Right Frame Error matrix:\n "<<pxrightVar<<"\n";
    #endif

    /** Frame Helper **/
    this->frameHelperLeft.setCalibrationParameter(cameracalib.camLeft);
    this->frameHelperRight.setCalibrationParameter(cameracalib.camRight);

    /** Initialize output frame **/
    ::base::samples::frame::Frame *outframe = new ::base::samples::frame::Frame();

    this->frame_out.reset(outframe);
    outframe = NULL;

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::prepareMatches()
{
    this->fprevious_left = this->fcurrent_left;
    this->fprevious_right = this->fcurrent_right;

    /** Increase the image index**/
    this->fcurrent_left.img_idx = frame_idx;
    this->fcurrent_right.img_idx = frame_idx++;

    this->intra_matches_previous = this->intra_matches_current;

    return;
}

void Task::detectFeatures (const base::samples::frame::Frame &frame_left,
                        const base::samples::frame::Frame &frame_right,
                        cv::detail::ImageFeatures &features_left,
                        cv::detail::ImageFeatures &features_right);
{
    /** Convert Images to opencv **/
    cv::Mat img_l = frameHelperLeft.convertToCvMat(frame_left);
    cv::Mat img_r = frameHelperRight.convertToCvMat(frame_right);

    /** Detect Keypoints **/
    int minHessian = 400;

    cv::SurfFeatureDetector detector(minHessian);
    std::vector<cv::KeyPoint> keypoints_l, keypoints_r;

    detector.detect(img_l, keypoints_l);
    detector.detect(img_r, keypoints_r);

    /** Calculate descriptors (features vectors) **/
    cv::SurfDescriptorExtractor extractor;
    cv::Mat descriptors_l, descriptors_r;

    extractor.compute(img_l, keypoints_l, descriptors_l);
    extractor.compute(img_r, keypoints_r, descriptors_r);

    /** store result sin arguments **/
    features_left.keypoints = keypoints_l;
    features_right.keypoints = keypoints_r;
    features_left.descriptors = descriptors_l;
    features_right.descriptors = descriptors_r;

    return;
}

void Task::interMatches (cv::detail::ImageFeatures &features_previous,
                        cv::detail::ImageFeatures &features_current,
                        std::vector< cv::DMatch > &good_matches)
{

    /** Matching descriptor vectors using flann **/
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match(features_previous.descriptors, features_current.descriptors, matches);

    /** Extract matches points **/
    std::vector<cv::Point2f> points1, points2;
    for (std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it)
    {
        // Get the position of left keypoints
        float x= features_previous.keypoints[it->queryIdx].pt.x;
        float y= features_previous.keypoints[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x,y));

        // Get the position of right keypoints
        x= features_current.keypoints[it->trainIdx].pt.x;
        y= features_current.keypoints[it->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x,y));
    }

    /** Compute F matrix using RANSAC **/
    std::vector<unsigned char> inliers(points1.size(),0);
    std::vector< cv::DMatch > out_matches;
    std::vector< cv::DMatch > ref_matches;
    if (points1.size()>0&&points2.size()>0)
    {
        cv::Mat fundamental= cv::findFundamentalMat(points1, points2, // matching points
                                                    inliers,       // match status (inlier or outlier)
                                                    cv::FM_RANSAC, // RANSAC method
                                                    0.68, 0.99);
        // extract the surviving (inliers) matches
        std::vector<unsigned char>::const_iterator
        itIn= inliers.begin();
        std::vector<cv::DMatch>::const_iterator
        itM= matches.begin();

        /** for all matches **/
        for ( ;itIn!= inliers.end(); ++itIn, ++itM)
        {
            if (*itIn)
            { // it is a valid match
                out_matches.push_back(*itM);
            }
        }

        /** Refinement **/
        points1.clear();
        points2.clear();

        for (std::vector<cv::DMatch>::const_iterator it= out_matches.begin(); it!= out_matches.end(); ++it)
        {
            // Get the position of left keypoints
            float x= features_previous.keypoints[it->queryIdx].pt.x;
            float y= features_previous.keypoints[it->queryIdx].pt.y;
            points1.push_back(cv::Point2f(x,y));

            // Get the position of right keypoints
            x= features_current.keypoints[it->trainIdx].pt.x;
            y= features_current.keypoints[it->trainIdx].pt.y;
            points2.push_back(cv::Point2f(x,y));
        }

        if (points1.size()>0&&points2.size()>0)
        {
            cv::Mat fundamental= cv::findFundamentalMat(points1, points2, // matching points
                                                        inliers,       // match status (inlier or outlier)
                                                        cv::FM_7POINT); // RANSAC method

            // extract the surviving (inliers) matches
            itIn= inliers.begin();
            itM= out_matches.begin();

            /** for all matches **/
            for ( ;itIn!= inliers.end(); ++itIn, ++itM)
            {
                if (*itIn)
                { // it is a valid match
                    ref_matches.push_back(*itM);
                }
            }
        }
    }

    good_matches = ref_matches;

    return;
}


void Task::drawMatches(const base::samples::frame::Frame &frame1,
                const base::samples::frame::Frame &frame2,
                std::vector<cv::KeyPoint> &keypoints1,
                std::vector<cv::KeyPoint> &keypoints2,
                std::vector<cv::DMatch> &matches)
{
    /** Convert Images to opencv **/
    cv::Mat img1 = frameHelperLeft.convertToCvMat(frame1);
    cv::Mat img2 = frameHelperRight.convertToCvMat(frame2);

    ::cv::Mat img_out;
    cv::drawMatches (img1, keypoints1, img2, keypoints2, matches, img_out,
    cv::Scalar::all(-1), cv::Scalar::all(-1), cv::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    ::base::samples::frame::Frame *frame_ptr = this->frame_out.write_access();
    frameHelperLeft.copyMatToFrame(img_out, *frame_ptr);

    frame_ptr->time = this->frame_pair.time;
    this->frame_out.reset(frame_ptr);
    _frame_samples_out.write(this->frame_out);

    return;
}


void Task::drawKeypoints(const base::samples::frame::Frame &frame2,
                std::vector<cv::KeyPoint> &keypoints1,
                std::vector<cv::KeyPoint> &keypoints2,
                std::vector<cv::DMatch> &matches)

{
    /** Convert Images to opencv **/
    cv::Mat img2 = frameHelperLeft.convertToCvMat(frame2);

    ::cv::Mat img_out(img2);
    cv::drawKeypoints (img2, keypoints1, img_out, cv::Scalar(0, 255, 0));//green previous
    for (std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it)
    {
        cv::Point point1 = keypoints1[it->queryIdx].pt;
        cv::Point point2 = keypoints2[it->trainIdx].pt;

        //cv::circle(img_out, point1, 3, cv::Scalar(0, 255, 0), 1);// green previous
        cv::circle(img_out, point2, 3, cv::Scalar(255, 0, 0), 1);// blue current
        cv::line (img_out, point1, point2, cv::Scalar(0, 0, 255)); // red line

    }

    ::base::samples::frame::Frame *frame_ptr = this->frame_out.write_access();
    frameHelperLeft.copyMatToFrame(img_out, *frame_ptr);

    frame_ptr->time = this->frame_pair.time;
    this->frame_out.reset(frame_ptr);
    _frame_samples_out.write(this->frame_out);

    return;
}

void Task::intraFeatures(const std::vector<cv::KeyPoint> &keypoints_left,
                const std::vector<cv::KeyPoint> &keypoints_right,
                const cv::Mat &descriptors_left,
                const cv::Mat &descriptors_right,
                const std::vector<cv::DMatch> &matches_left,
                const std::vector<cv::DMatch> &matches_right,
                cv::detail::ImageFeatures &features_left,
                cv::detail::ImageFeatures &features_right,
                std::vector<cv::DMatch> &intra_matches)
{
    cv::SurfFeatureDetector detector;
    const int length = detector.descriptorSize();

    /** Get good features from left pair matches **/
    cv::detail::ImageFeatures features_left;
    features_left.descriptors = cv::Mat(0, length, CV_32FC1);
    for (std::vector<cv::DMatch>::const_iterator it= matches_left.begin(); it!= matches_left.end(); ++it)
    {
        features_left.keypoints.push_back(keypoints_left[it->trainIdx]);
        features_left.descriptors.push_back(descriptors_left.row(it->trainIdx));
    }


    /** Get good features from right pair matches **/
    cv::detail::ImageFeatures features_right;
    features_right.descriptors = cv::Mat(0, length, CV_32FC1);
    for (std::vector<cv::DMatch>::const_iterator it= matches_right.begin(); it!= matches_right.end(); ++it)
    {
        features_right.keypoints.push_back(keypoints_right[it->trainIdx]);
        features_right.descriptors.push_back(descriptors_right.row(it->trainIdx));
    }


    /** Match features descriptors using flann **/
    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(features_left.descriptors, features_right.descriptors, matches);

    intra_matches.clear();
    for (std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it)
    {
        if (it->distance <= cv::max(std::sqrt(pxleftVar.diagonal()[0]), std::sqrt(pxleftVar.diagonal()[1])))
        {
            float dy_point = std::fabs(features_left.keypoints[it->queryIdx].pt.y - features_right.keypoints[it->trainIdx].pt.y);
            float dy_center = std::fabs(this->cameracalib.camLeft.cy - this->cameracalib.camRight.cy);
            /** Epipolar line **/
            if (dy_point <= 5.0 * dy_center)
            {
                intra_matches.push_back(*it);
            }
        }
    }

    /** Get the good features **/
    cv::detail::ImageFeatures features_l, features_r;
    for (std::vector<cv::DMatch>::const_iterator it= intra_matches.begin(); it!= intra_matches.end(); ++it)
    {
        features_l.keypoints.push_back(features_left.keypoints[it->queryIdx]);
        features_l.descriptors.push_back(features_left.descriptors.row(it->queryIdx));

        features_r.keypoints.push_back(features_right.keypoints[it->trainIdx]);
        features_r.descriptors.push_back(features_right.descriptors.row(it->trainIdx));
    }

    features_left = features_l;
    features_right = features_r;

    return;
}

