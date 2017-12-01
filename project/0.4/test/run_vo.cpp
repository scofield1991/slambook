// -------------- test the visual odometry -------------
#include <fstream>
#include <iomanip>
#include <string>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<string> &vstrLidar, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";
    string strPrefixLidar = strPathToSequence + "/velodyne_pcd/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    vstrLidar.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
        vstrLidar[i] = strPrefixLidar + ss.str() + ".pcd";
    }
}

void ComputeDepthMap(const cv::Mat &imLeft, const cv::Mat &imRight, cv::Mat &filtered_disp, cv::Mat &filtered_disp_vis)
{
    cv::Mat left_for_matcher, right_for_matcher;
    cv::Mat left_disp, right_disp;
    //cv::Mat filtered_disp_in;
    cv::Mat conf_map = cv::Mat(imLeft.rows, imLeft.cols, CV_8U);
    conf_map = cv::Scalar(255);
    cv::Rect ROI;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
    int wsize = 3;
    double wls_lambda = 8000.0;
    double wls_sigma = 1.5;

    left_for_matcher = imLeft.clone();
    right_for_matcher = imRight.clone();

    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, 160, wsize,  0, 0, 0,
                                                                  0,  0, 0, 0, cv::StereoSGBM::MODE_HH);

    left_matcher->setP1(24*wsize*wsize);
    left_matcher->setP2(96*wsize*wsize);
    left_matcher->setPreFilterCap(63);
    left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);

    cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

    left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
    right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);


    wls_filter->setLambda(wls_lambda);
    wls_filter->setSigmaColor(wls_sigma);
    wls_filter->filter(left_disp, imLeft, filtered_disp, right_disp);

    //std::cout << "Disparity map 1 : " << filtered_disp.size() << std::endl;

    conf_map = wls_filter->getConfidenceMap();
    ROI = wls_filter->getROI();


    //cv::Mat raw_disp_vis, filtered_disp_vis;
    double vis_mult = 1.0;

    //cv::ximgproc::getDisparityVis(left_disp, raw_disp_vis, vis_mult);
    //cv::namedWindow("raw disparity", cv::WINDOW_AUTOSIZE);
    //cv::imshow("raw disparity", raw_disp_vis);

    //cv::ximgproc::getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);
    //cv::namedWindow("filtered disparity", cv::WINDOW_AUTOSIZE);
    //cv::imshow("filtered disparity", raw_disp_vis);
    //cv::waitKey();

}

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    myslam::Config::setParameterFile ( argv[1] );
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<string> vstrLidar;
    vector<double> vTimestamps;
    LoadImages(dataset_dir, vstrImageLeft, vstrImageRight, vstrLidar, vTimestamps);

    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }

    myslam::Camera::Ptr camera ( new myslam::Camera );

    // visualization
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );

    cout<<"read total "<<vstrImageLeft.size() <<" entries"<<endl;
    for ( int i=0; i<vstrImageLeft.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat imLeft = cv::imread(vstrImageLeft[i],CV_LOAD_IMAGE_UNCHANGED);
        Mat imRight = cv::imread(vstrImageRight[i],CV_LOAD_IMAGE_UNCHANGED);
        Mat filtered_disp_vis;
        Mat depth;

        //Mat color = cv::imread ( rgb_files[i] );
        ComputeDepthMap(imLeft, imRight, depth, filtered_disp_vis);
        
        //Mat depth = cv::imread ( depth_files[i], -1 );
        if ( imLeft.data==nullptr || depth.data==nullptr )
            break;
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = imLeft;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST )
            break;
        SE3 Twc = pFrame->T_c_w_.inverse();

        // show the map and the camera pose
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

        Mat img_show = imLeft.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        cout<<endl;
    }

    return 0;
}
