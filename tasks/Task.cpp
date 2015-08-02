/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

//#define DEBUG_PRINTS 1

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#include <boost/uuid/uuid_generators.hpp>

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
    std::cout << "[VISUAL_STEREO LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time.toString()<< std::endl;
    #endif

    /** The image need to be in gray scale and undistorted **/
    frame_pair.first.init(left_frame_sample->size.width, left_frame_sample->size.height, left_frame_sample->getDataDepth(), base::samples::frame::MODE_GRAYSCALE);
    frameHelperLeft.convert (*left_frame_sample, frame_pair.first, 0, 0, _resize_algorithm.value(), true);

    /** Left color image **/
    if (_output_debug.get())
    {
        left_color_frame.init(left_frame_sample->size.width, left_frame_sample->size.height, left_frame_sample->getDataDepth(), base::samples::frame::MODE_RGB);
        frameHelperLeft.convert (*left_frame_sample, left_color_frame, 0, 0, _resize_algorithm.value(), true);
    }

    /** Increase th computing index **/
    this->left_computing_idx++;

    /** If the difference in time is less than half of a period run the odometry **/
    base::Time diffTime = frame_pair.first.time - frame_pair.second.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_left_frame_period/2.0) && (this->left_computing_idx == this->computing_counts))
    {
        frame_pair.time = frame_pair.first.time;

        #ifdef DEBUG_PRINTS
        std::cout<< "[VISUAL_STEREO LEFT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

        this->prepareMatches();
        this->detectFeatures(frame_pair.first, frame_pair.second, fcurrent_left, fcurrent_right);

        if (this->frame_idx > 1)
        {
            this->interMatches(fprevious_left, fcurrent_left, inter_matches_left);

            this->interMatches(fprevious_right, fcurrent_right, inter_matches_right);

            this->intraMatches(fcurrent_left, fcurrent_right,
                                inter_matches_left, inter_matches_right,
                                ffinal_left, ffinal_right, intra_matches);

            this->hashFeatures(ffinal_left, ffinal_right, intra_matches);

            this->featuresOut(ffinal_left.img_idx, this->hash_features);

            /** Draw good matches **/
            if (_output_debug.value())
            {
                switch (_image_ouput_type.get())
                {
                case visual_stereo::INTRA_MATCHES:
                    this->drawMatches(frame_pair.first, frame_pair.second, ffinal_left.keypoints, ffinal_right.keypoints, intra_matches);
                    break;
                case visual_stereo::INTER_KEYPOINTS:
                    this->drawKeypoints(frame_pair.first, fprevious_left.keypoints, fcurrent_left.keypoints, inter_matches_left, hash_features);
                    break;
                case visual_stereo::INTRA_AND_INTER:
                    this->drawMatches(frame_pair.first, frame_pair.second, ffinal_left.keypoints, ffinal_right.keypoints, intra_matches);
                    this->drawKeypoints(frame_pair.first, fprevious_left.keypoints, fcurrent_left.keypoints, inter_matches_left, hash_features);
                    break;
                }
            }
        }

        /** Reset computing indices **/
        this->left_computing_idx = this->right_computing_idx = 0;
    }

    return;
}

void Task::right_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<< "[VISUAL_STEREO RIGHT_FRAME] Frame arrived at: " <<right_frame_sample->time.toString()<<std::endl;
    #endif

    /** Correct distortion in image right **/
    frame_pair.second.init(right_frame_sample->size.width, right_frame_sample->size.height, right_frame_sample->getDataDepth(), base::samples::frame::MODE_GRAYSCALE);
    frameHelperRight.convert (*right_frame_sample, frame_pair.second, 0, 0, _resize_algorithm.value(), true);

    /** Increase th computing index **/
    this->right_computing_idx++;

    /** Check the time difference **/
    base::Time diffTime = frame_pair.second.time - frame_pair.first.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_right_frame_period/2.0) && (this->right_computing_idx == this->computing_counts))
    {
        frame_pair.time = frame_pair.second.time;

        #ifdef DEBUG_PRINTS
        std::cout<< "[VISUAL_STEREO RIGHT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

        this->prepareMatches();
        this->detectFeatures(frame_pair.first, frame_pair.second, fcurrent_left, fcurrent_right);

        if (this->frame_idx > 1)
        {
            this->interMatches(fprevious_left, fcurrent_left, inter_matches_left);

            this->interMatches(fprevious_right, fcurrent_right, inter_matches_right);

            this->intraMatches(fcurrent_left, fcurrent_right,
                                inter_matches_left, inter_matches_right,
                                ffinal_left, ffinal_right, intra_matches);

            this->hashFeatures(ffinal_left, ffinal_right, intra_matches);

            this->featuresOut(ffinal_left.img_idx, this->hash_features);

            /** Draw good matches **/
            if (_output_debug.value())
            {
                switch (_image_ouput_type.get())
                {
                case visual_stereo::INTRA_MATCHES:
                    this->drawMatches(frame_pair.first, frame_pair.second, ffinal_left.keypoints, ffinal_right.keypoints, intra_matches);
                    break;
                case visual_stereo::INTER_KEYPOINTS:
                    this->drawKeypoints(frame_pair.first, fprevious_left.keypoints, fcurrent_left.keypoints, inter_matches_left, hash_features);
                    break;
                case visual_stereo::INTRA_AND_INTER:
                    this->drawMatches(frame_pair.first, frame_pair.second, ffinal_left.keypoints, ffinal_right.keypoints, intra_matches);
                    this->drawKeypoints(frame_pair.first, fprevious_left.keypoints, fcurrent_left.keypoints, inter_matches_left, hash_features);
                    break;
                }
           }
        }

        /** Reset computing indices **/
        this->left_computing_idx = this->right_computing_idx = 0;
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
    this->frame_idx = 0;

    /** Frame history to keep in the hash **/
    this->frame_window_hash_size = _frame_window_hash_size.value();

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
    std::cout<< "[VISUAL_STEREO CONFIGURATION] Q re-projection matrix:\n "<<Q<<"\n";
    #endif

    this->pxleftVar = cameracalib.camLeft.getPixelCovariance();
    this->pxrightVar = cameracalib.camRight.getPixelCovariance();

    #ifdef DEBUG_PRINTS
    std::cout<< "[VISUAL_STEREO CONFIGURATION] Left Frame Error matrix:\n "<<pxleftVar<<"\n";
    std::cout<< "[VISUAL_STEREO CONFIGURATION] Right Frame Error matrix:\n "<<pxrightVar<<"\n";
    #endif

    /** Frame Helper **/
    this->frameHelperLeft.setCalibrationParameter(cameracalib.camLeft);
    this->frameHelperRight.setCalibrationParameter(cameracalib.camRight);

    /** Initialize output frame **/
    ::base::samples::frame::Frame *outframe = new ::base::samples::frame::Frame();

    this->intra_frame_out.reset(outframe);
    this->inter_frame_out.reset(outframe);
    outframe = NULL;

    /** Check task property parameters **/
    if (_left_frame_period.value() != _right_frame_period.value())
    {
        throw std::runtime_error("[VISUAL_STEREO] Input port period in Left and Right images must be equal!");
    }

    if (_desired_period.value() < _left_frame_period.value())
    {
        throw std::runtime_error("[VISUAL_STEREO] Desired period cannot be smaller than input ports period!");
    }
    else if (_desired_period.value() == 0.00)
    {
        _desired_period.value() = _left_frame_period.value();
        this->computing_counts = 1;
    }
    else
    {
        this->computing_counts = boost::math::iround(_desired_period.value()/_left_frame_period.value());
        _desired_period.value() = this->computing_counts * _left_frame_period.value();
    }

    RTT::log(RTT::Warning)<<"[VISUAL_STEREO] Actual Computing Period: "<<_desired_period.value()<<" [seconds]"<<RTT::endlog();

    this->left_computing_idx = this->right_computing_idx = 0;

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

    return;
}

void Task::detectFeatures (const base::samples::frame::Frame &frame_left,
                        const base::samples::frame::Frame &frame_right,
                        cv::detail::ImageFeatures &features_left,
                        cv::detail::ImageFeatures &features_right)
{
    /** Convert Images to opencv **/
    cv::Mat img_l = frameHelperLeft.convertToCvMat(frame_left);
    cv::Mat img_r = frameHelperRight.convertToCvMat(frame_right);

    /** Detect Keypoints **/
    int minHessian = _minimum_hessian.get();

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

    #ifdef DEBUG_PRINTS
    std::cout<<"DETECTING...IDX["<<features_left.img_idx<<","<<features_right.img_idx<<"]\n";
    std::cout<<"features_left.keypoints: "<<features_left.keypoints.size()<<"\n";
    std::cout<<"features_left.descriptors: "<<features_left.descriptors.size()<<"\n";
    std::cout<<"features_right.keypoints: "<<features_right.keypoints.size()<<"\n";
    std::cout<<"features_right.descriptors: "<<features_right.descriptors.size()<<"\n";
    std::cout<<"...[OK]\n";
    #endif

    return;
}

void Task::interMatches (const cv::detail::ImageFeatures &features_previous,
                        const cv::detail::ImageFeatures &features_current,
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
                                                    cv::FM_7POINT);

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
                                                        cv::FM_RANSAC, // RANSAC method
                                                        0.68, 0.99);

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
        else
        {
            ref_matches = out_matches;
        }
    }

    good_matches = ref_matches;
    #ifdef DEBUG_PRINTS
    std::cout<<"[INTER_MATCHES] good_matches: "<<good_matches.size()<<"\n";
    #endif

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

    ::base::samples::frame::Frame *frame_ptr = this->intra_frame_out.write_access();
    frameHelperLeft.copyMatToFrame(img_out, *frame_ptr);

    frame_ptr->time = this->frame_pair.time;
    this->intra_frame_out.reset(frame_ptr);
    _intra_frame_samples_out.write(this->intra_frame_out);

    return;
}


void Task::drawKeypoints(const base::samples::frame::Frame &frame2,
                const std::vector<cv::KeyPoint> &keypoints1,
                const std::vector<cv::KeyPoint> &keypoints2,
                const std::vector<cv::DMatch> &matches,
                const boost::unordered_map<boost::uuids::uuid, StereoFeature> &hash)

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

    /** Draw hash features **/
    for (boost::unordered_map<boost::uuids::uuid, StereoFeature>::const_iterator
                    it = hash.begin(); it != hash.end(); ++it)
    {
        float red = 255.0 - (255.0/(this->frame_window_hash_size)*(this->ffinal_left.img_idx-it->second.img_idx));
        cv::circle(img_out, it->second.keypoint_left.pt, 5, cv::Scalar(0, 0, red), 2);
        cv::putText(img_out, boost::lexical_cast<std::string>(it->second.img_idx),
                it->second.keypoint_left.pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(0,0,0), 1.5);
    }

    ::base::samples::frame::Frame *frame_ptr = this->inter_frame_out.write_access();
    frameHelperLeft.copyMatToFrame(img_out, *frame_ptr);

    frame_ptr->time = this->frame_pair.time;
    this->inter_frame_out.reset(frame_ptr);
    _inter_frame_samples_out.write(this->inter_frame_out);

    return;
}

void Task::intraMatches(const cv::detail::ImageFeatures &features_left,
                    const cv::detail::ImageFeatures &features_right,
                    const std::vector<cv::DMatch> &matches_left,
                    const std::vector<cv::DMatch> &matches_right,
                    cv::detail::ImageFeatures &final_left,
                    cv::detail::ImageFeatures &final_right,
                    std::vector<cv::DMatch> &intra_matches)
{
    cv::SurfFeatureDetector detector;
    const int length = detector.descriptorSize();

    /** Subset current features from left pair matches **/
    cv::detail::ImageFeatures subset_left;
    subset_left.img_idx = features_left.img_idx;
    subset_left.descriptors = cv::Mat(0, length, CV_32FC1);
    if (matches_left.size() > 0)
    {
        for (std::vector<cv::DMatch>::const_iterator it= matches_left.begin(); it!= matches_left.end(); ++it)
        {
            subset_left.keypoints.push_back(features_left.keypoints[it->trainIdx]);
            subset_left.descriptors.push_back(features_left.descriptors.row(it->trainIdx));
        }
    }
    else
    {
        subset_left = features_left;
    }

    //std::cout<<"features_left.keypoints: "<<features_left.keypoints.size()<<"\n";
    //std::cout<<"features_left.descriptors: "<<features_left.descriptors.size()<<"\n";
    #ifdef DEBUG_PRINTS
    std::cout<<"subset_left.keypoints: "<<subset_left.keypoints.size()<<"\n";
    std::cout<<"subset_left.descriptors: "<<subset_left.descriptors.size()<<"\n";
    #endif


    /** Subset current features from right pair matches **/
    cv::detail::ImageFeatures subset_right;
    subset_right.img_idx = features_right.img_idx;
    subset_right.descriptors = cv::Mat(0, length, CV_32FC1);
    if (matches_right.size() > 0)
    {
        for (std::vector<cv::DMatch>::const_iterator it= matches_right.begin(); it!= matches_right.end(); ++it)
        {
            subset_right.keypoints.push_back(features_right.keypoints[it->trainIdx]);
            subset_right.descriptors.push_back(features_right.descriptors.row(it->trainIdx));
        }
    }
    else
    {
        subset_right = features_right;
    }

    //std::cout<<"features_right.keypoints: "<<features_right.keypoints.size()<<"\n";
    //std::cout<<"features_right.descriptors: "<<features_right.descriptors.size()<<"\n";
    #ifdef DEBUG_PRINTS
    std::cout<<"subset_right.keypoints: "<<subset_right.keypoints.size()<<"\n";
    std::cout<<"subset_right.descriptors: "<<subset_right.descriptors.size()<<"\n";
    #endif

    /** Match left and right subsets descriptors using flann **/
    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(subset_left.descriptors, subset_right.descriptors, matches);

    intra_matches.clear();
    for (std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it)
    {
        if (it->distance <= cv::max(std::sqrt(pxleftVar.diagonal()[0]), std::sqrt(pxleftVar.diagonal()[1])))
        {
            float dy_point = std::fabs(subset_left.keypoints[it->queryIdx].pt.y - subset_right.keypoints[it->trainIdx].pt.y);
            float dy_center = std::fabs(this->cameracalib.camLeft.cy - this->cameracalib.camRight.cy);

            /** Epipolar line **/
            if (dy_point <= 5.0 * dy_center)
            {
                intra_matches.push_back(*it);
            }
        }
    }

    final_left = subset_left;
    final_right = subset_right;

    #ifdef DEBUG_PRINTS
    std::cout<<"[INTRA_MATCHES] final_left.keypoints: "<<final_left.keypoints.size()<<"\n";
    std::cout<<"[INTRA_MATCHES] final_left.descriptors: "<<final_left.descriptors.size()<<"\n";
    std::cout<<"[INTRA_MATCHES] final_right.keypoints: "<<final_right.keypoints.size()<<"\n";
    std::cout<<"[INTRA_MATCHES] final_right.descriptors: "<<final_right.descriptors.size()<<"\n";
    std::cout<<"[INTRA_MATCHES] intra_matches: "<<intra_matches.size()<<"\n";
    #endif
    return;
}


void Task::hashFeatures (const cv::detail::ImageFeatures &new_features_left,
                        const cv::detail::ImageFeatures &new_features_right,
                        const std::vector< cv::DMatch > &good_matches)
{
    cv::detail::ImageFeatures subset_features_left, subset_features_right;
    cv::SurfFeatureDetector detector;
    const int length = detector.descriptorSize();

    /** Subset the local features with only existing matches **/
    subset_features_left.img_idx = new_features_left.img_idx;
    for (std::vector<cv::DMatch>::const_iterator it= good_matches.begin(); it!= good_matches.end(); ++it)
    {
        /** left[i] matches with right[i] **/
        subset_features_left.keypoints.push_back(new_features_left.keypoints[it->queryIdx]);
        subset_features_left.descriptors.push_back(new_features_left.descriptors.row(it->queryIdx));
        subset_features_right.keypoints.push_back(new_features_right.keypoints[it->trainIdx]);
        subset_features_right.descriptors.push_back(new_features_right.descriptors.row(it->trainIdx));
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[HASH_FEATURES] subset_features_left.keypoints.size(): "<<subset_features_left.keypoints.size()<<"\n";
    std::cout<<"[HASH_FEATURES] subset_features_left.descriptors.size(): "<<subset_features_left.descriptors.size()<<"\n";
    std::cout<<"[HASH_FEATURES] subset_features_right.keypoints.size(): "<<subset_features_right.keypoints.size()<<"\n";
    std::cout<<"[HASH_FEATURES] subset_features_right.descriptors.size(): "<<subset_features_right.descriptors.size()<<"\n";
    #endif

    /** Hash descriptors from the table **/
    std::vector<boost::uuids::uuid> uuid_descriptors;
    cv::Mat hash_descriptors;
    hash_descriptors = cv::Mat(0, length, CV_32FC1);
    for (boost::unordered_map<boost::uuids::uuid, StereoFeature>::const_iterator it = this->hash_features.begin(); it != this->hash_features.end(); ++it)
    {
        uuid_descriptors.push_back(it->first);
        hash_descriptors.push_back(it->second.descriptor);
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[HASH_FEATURES] hash_features.size(): "<<hash_features.size()<<"\n";
    std::cout<<"[HASH_FEATURES] uuid_descriptors.size(): "<<uuid_descriptors.size()<<"\n";
    std::cout<<"[HASH_FEATURES] hash_descriptors.size(): "<<hash_descriptors.size()<<"\n";
    #endif

    /** Match with the current descriptors from left features **/
    bool direct_match = true;
    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;
    std::vector<bool> copy_features(subset_features_left.descriptors.rows, true);
    if (hash_descriptors.rows > subset_features_left.descriptors.rows)
    {
        matcher.match(subset_features_left.descriptors, hash_descriptors, matches);
        direct_match = true;
        copy_features.resize(subset_features_left.descriptors.rows);
        std::fill(copy_features.begin(), copy_features.end(), true);
    }
    else if (hash_descriptors.rows > 0)
    {
        matcher.match(hash_descriptors, subset_features_left.descriptors, matches);
        direct_match = false;
        copy_features.resize(hash_descriptors.rows);
        std::fill(copy_features.begin(), copy_features.end(), true);
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[HASH_FEATURES] copy_features.size(): "<<copy_features.size()<<"\n";
    std::cout<<"[HASH_FEATURES] found "<<matches.size()<<" matches\n";
    #endif

    /** Update hash with matches **/
    register int index_copy = 0;
    for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it)
    {
        boost::unordered_map<boost::uuids::uuid, StereoFeature>::iterator got;
        short int hash_idx, feature_idx;

        if (direct_match)
        {
            hash_idx = it->trainIdx;
            feature_idx = it->queryIdx;
        }
        else
        {
            hash_idx = it->queryIdx;
            feature_idx = it->trainIdx;
        }

        got = this->hash_features.find(uuid_descriptors[hash_idx]);

        /** Update the feature **/
        if (got != this->hash_features.end() && copy_features[feature_idx])
        {
            //std::cout<<"[HASH_FEATURES] uuid:"<<boost::uuids::to_string(got->first)<<"\n";
            //std::cout<<"[HASH_FEATURES] hash_features point: "<<got->second.keypoint.pt<<"\n";
            //std::cout<<"[HASH_FEATURES] new_features_copy point: "<<subset_features_left.keypoints[feature_idx].pt<<"\n";

            got->second.img_idx = subset_features_left.img_idx;
            got->second.keypoint_left = subset_features_left.keypoints[feature_idx];
            got->second.keypoint_right = subset_features_right.keypoints[feature_idx];
            got->second.descriptor = subset_features_left.descriptors.row(feature_idx);
            copy_features[feature_idx] = false;
            index_copy++;
        }
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[HASH_FEATURES] Updated "<<index_copy<<" features into hash\n";
    #endif

    /** Increase hash with features which did not have matches **/
    register int index = 0;
    index_copy = 0;
    for (std::vector<bool>::const_iterator it_copy = copy_features.begin();
            it_copy != copy_features.end(); ++it_copy)
    {
        //std::cout<<"[HASH_FEATURES] copy: "<<(*it_copy)<<"\n";
        if (*it_copy)
        {
            /** Compute the 3d point and covariance of the feature **/

            /** Create the feature **/
            visual_stereo::StereoFeature f (subset_features_left.img_idx,
                                    subset_features_left.keypoints[index],
                                    subset_features_right.keypoints[index],
                                    subset_features_left.descriptors.row(index));

            /** Insert new feature in the hash **/
            this->hash_features.insert(std::make_pair<boost::uuids::uuid, StereoFeature>
                    (boost::uuids::random_generator()(), f));

            #ifdef DEBUG_PRINTS
            index_copy++;
            #endif
        }
        index++;
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[HASH_FEATURES] New "<<index_copy<<" features into hash\n";
    #endif

    this->cleanHashFeatures(new_features_left.img_idx,
                            this->frame_window_hash_size,
                            this->hash_features);


    return;
}

void Task::cleanHashFeatures (const int current_image_idx,
                        const int max_number_img,
                        boost::unordered_map<boost::uuids::uuid, StereoFeature> &hash)
{
    if (current_image_idx > max_number_img)
    {
            const int border_image_idx = (current_image_idx - max_number_img);

            /** All the images with idx <= border_image_idx should be eliminated **/
            for (boost::unordered_map<boost::uuids::uuid, StereoFeature>::const_iterator
                    it = hash.begin(); it != hash.end(); ++it)
            {
                if (it->second.img_idx <= border_image_idx)
                {
                    hash.erase(it);
                }
            }
    }

    return;
}


void Task::featuresOut(const int current_image_idx, const boost::unordered_map<boost::uuids::uuid, StereoFeature> &hash)
{
    ExteroFeatures features_samples;
    base::samples::Pointcloud features_points;
    double const &baseline(cameracalib.extrinsic.tx);
    cv::Mat cv_image1;

    /** Uncertainty in the images (left and right) planes **/
    Eigen::Matrix4d px_var;
    px_var << this->pxleftVar, Eigen::Matrix2d::Zero(),
        Eigen::Matrix2d::Zero(), this->pxrightVar;


    #ifdef DEBUG_PRINTS
    std::cout<<"[FEATURES_OUT] Current hash.size(): "<<hash.size()<<"\n";
    #endif

    /** Parse the hash table of features **/
    for (boost::unordered_map<boost::uuids::uuid, StereoFeature>::const_iterator
                    it = hash.begin(); it != hash.end(); ++it)
    {
        //std::cout<<"[FEATURES_OUT] img_idx: "<<it->second.img_idx<<" current_idx: "<<current_image_idx<<"\n";
        if (it->second.img_idx == current_image_idx)
        {
            Feature feature;

            /** Get the uuid **/
            feature.index = it->first;

            /** Compute the 3d point **/
            cv::Point const &left_pt(it->second.keypoint_left.pt);
            cv::Point const &right_pt(it->second.keypoint_right.pt);
            double disparity = std::min(right_pt.x - left_pt.x, -1);
            base::Vector4d image_point (left_pt.x, left_pt.y, disparity, 1);
            base::Vector4d homogeneous_point = Q * image_point;

            feature.point = base::Vector3d (homogeneous_point(0)/homogeneous_point(3),
                                homogeneous_point(1)/homogeneous_point(3),
                                homogeneous_point(2)/homogeneous_point(3));
            #ifdef DEBUG_PRINTS
            std::cout<<"[FEATURES_OUT] 3D point:\n"<<feature.point<<"\n";
            #endif

            if (_output_debug.value())
            {
                /** 3D point coordinates **/
                features_points.points.push_back(base::Vector3d (
                                homogeneous_point(0)/homogeneous_point(3),
                                homogeneous_point(1)/homogeneous_point(3),
                                homogeneous_point(2)/homogeneous_point(3)));

                /** Color **/
                cv_image1 = frame_helper::FrameHelper::convertToCvMat(this->left_color_frame);
                cv::Vec3b color = cv_image1.at<cv::Vec3b>(left_pt.y, left_pt.x);
                base::Vector4d color4d;
                color4d[0] = color[0]/255.0;//R
                color4d[1] = color[1]/255.0;//G
                color4d[2] = color[2]/255.0;//B
                color4d[3] = 1.0;//Alpha
                features_points.colors.push_back(color4d);
            }

            /** Compute the covariance **/
            Eigen::Matrix<double, 3, 4> noise_jacobian; /** Jacobian Matrix for the triangulation noise model */
            double disparity_power = pow(disparity,2);

            noise_jacobian <<  -(baseline*right_pt.x)/disparity_power, 0.00, (baseline*left_pt.x)/disparity_power, 0.00,
                -(baseline*left_pt.y)/disparity_power, baseline/disparity, (baseline*left_pt.y)/disparity_power, 0.00,
                -(baseline*cameracalib.camLeft.fx)/disparity_power, 0.00,  (baseline*cameracalib.camLeft.fx)/disparity_power, 0.00;

            feature.cov = noise_jacobian * px_var * noise_jacobian.transpose();

            /** Store the feature in the vector **/
            features_samples.features.push_back(feature);
        }
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[FEATURES_OUT] Sent "<<features_samples.features.size()<<" vector of uuids\n";
    #endif

    features_samples.time = this->frame_pair.time;
    features_samples.img_idx = current_image_idx;
    _features_samples_out.write(features_samples);

    if (_output_debug.value())
    {
        features_points.time =  this->frame_pair.time;
        _features_point_samples_out.write(features_points);
    }

    return;
}

