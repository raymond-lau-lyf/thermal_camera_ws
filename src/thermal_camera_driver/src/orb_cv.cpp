#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace cv;

cv::Mat AutoCanny(cv::Mat& mImEH)
{
    Mat SmoothedImage;
    GaussianBlur( mImEH, SmoothedImage, Size( 3, 3 ), 1.8, 1.8 );

    /// Detect edges using canny (auto th)
    Mat dx, dy, dxM, dyM, histImg, hist;
    float th1;

    Sobel(SmoothedImage, dx, CV_16SC1, 1, 0, 3, 1, 0, BORDER_REPLICATE);
    Sobel(SmoothedImage, dy, CV_16SC1, 0, 1, 3, 1, 0, BORDER_REPLICATE);
    double maxMagGf = 0;
    cv::resize(dx, dxM,Size(0,0),0.2,0.2, INTER_NEAREST);
    cv::resize(dy, dyM,Size(0,0),0.2,0.2, INTER_NEAREST);
    add(abs(dxM),abs(dyM),SmoothedImage);
    SmoothedImage.convertTo(histImg, CV_32FC1);
    minMaxIdx(histImg, NULL, &maxMagGf);

    float bin = 256;
    calcHist(vector<Mat>{histImg},vector<int>{0},noArray()
             ,hist,vector<int>{bin},vector<float>{0,maxMagGf});
    int sum = 0, total = SmoothedImage.cols*SmoothedImage.rows*0.9;
    float ii = 0;
    for(; ii<bin; ii++)
    {
        sum+=hist.at<float>(ii);
        if(sum>total)
            break;
    }
    if(ii == bin)
        ii--;
    th1 = (ii+1)/bin*maxMagGf;
    cv::Mat mEdgeImage;
    Canny( dx, dy, mEdgeImage, th1, 0.9 * th1 ); //OpenCV3

    return mEdgeImage;
}

void imageDataHandler(const sensor_msgs::Image::ConstPtr &imageData)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(imageData, "mono8");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img_1 = cv_ptr->image;
    cv::Mat img_2;
    cv::GaussianBlur(img_1, img_2, Size(3, 3), 0.7, 0.7);
    cv::equalizeHist(img_2, img_2);
    img_2 = img_2 * 0.5 + img_1 * 0.5;

    // //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // //-- 第一步:检测 Oriented FAST 角点位置
    // detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // //-- 第二步:根据角点位置计算 BRIEF 描述子
    // descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    Mat outimg1, outimg2,imText1,imText2;
    // cvtColor(img_1, outimg1, CV_GRAY2RGB);
    cvtColor(img_2, outimg2, CV_GRAY2RGB);
    int r = 5;
    //int n = keypoints_1.size();
    // for (int i = 0; i < n; i++)
    // {
    //     cv::Point2f pt1, pt2;
    //     pt1.x = keypoints_1[i].pt.x - r;
    //     pt1.y = keypoints_1[i].pt.y - r;
    //     pt2.x = keypoints_1[i].pt.x + r;
    //     pt2.y = keypoints_1[i].pt.y + r;

    //     cv::rectangle(outimg1, pt1, pt2, cv::Scalar(0, 255, 0));
    //     cv::circle(outimg1, keypoints_1[i].pt, 2, cv::Scalar(0, 255, 0), -1);
    // }
    stringstream s1,s2;
    // s1<<"Origin ORB features num : "<<n;
    // int baseline1 = 0;
    // cv::Size textSize1 = cv::getTextSize(s1.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline1);

    // imText1 = cv::Mat(outimg1.rows + textSize1.height + 10, outimg1.cols, outimg1.type());
    // outimg1.copyTo(imText1.rowRange(0, outimg1.rows).colRange(0, outimg1.cols));
    // imText1.rowRange(outimg1.rows, imText1.rows) = cv::Mat::zeros(textSize1.height + 10, outimg1.cols, outimg1.type());
    // cv::putText(imText1, s1.str(), cv::Point(5, imText1.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);

    int n = keypoints_2.size();
    for (int i = 0; i < n; i++)
    {
        cv::Point2f pt1, pt2;
        pt1.x = keypoints_2[i].pt.x - r;
        pt1.y = keypoints_2[i].pt.y - r;
        pt2.x = keypoints_2[i].pt.x + r;
        pt2.y = keypoints_2[i].pt.y + r;

        cv::rectangle(outimg2, pt1, pt2, cv::Scalar(0, 255, 0));
        cv::circle(outimg2, keypoints_2[i].pt, 2, cv::Scalar(0, 255, 0), -1);
    }
    s2<<"EH ORB features num : "<<n;
    int baseline2 = 0;
    cv::Size textSize2 = cv::getTextSize(s2.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline2);

    imText2 = cv::Mat(outimg2.rows + textSize2.height + 10, outimg2.cols, outimg2.type());
    outimg2.copyTo(imText2.rowRange(0, outimg2.rows).colRange(0, outimg2.cols));
    imText2.rowRange(outimg2.rows, imText2.rows) = cv::Mat::zeros(textSize2.height + 10, outimg2.cols, outimg2.type());
    cv::putText(imText2, s2.str(), cv::Point(5, imText2.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);

    // drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // drawKeypoints(img_2, keypoints_2, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    cv::Mat edge_image = AutoCanny(img_1);
    // imshow("img1", imText1);
    imshow("img2", imText2);
    imshow("edge_image", edge_image);
    // cout<<keypoints_2.size()-keypoints_1.size()<<endl;

    // //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    // vector<DMatch> matches;
    // t1 = chrono::steady_clock::now();
    // matcher->match(descriptors_1, descriptors_2, matches);
    // t2 = chrono::steady_clock::now();
    // time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

    // //-- 第四步:匹配点对筛选
    // // 计算最小距离和最大距离
    // auto min_max = minmax_element(matches.begin(), matches.end(),
    //                               [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    // double min_dist = min_max.first->distance;
    // double max_dist = min_max.second->distance;

    // printf("-- Max dist : %f \n", max_dist);
    // printf("-- Min dist : %f \n", min_dist);

    // //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    // std::vector<DMatch> good_matches;
    // for (int i = 0; i < descriptors_1.rows; i++)
    // {
    //     if (matches[i].distance <= max(2 * min_dist, 30.0))
    //     {
    //         good_matches.push_back(matches[i]);
    //     }
    // }

    // //-- 第五步:绘制匹配结果
    // Mat img_match;
    // Mat img_goodmatch;
    // drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    // drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    // imshow("all matches", img_match);
    // imshow("good matches", img_goodmatch);
    waitKey(10);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "orb_cv");
    ros::NodeHandle nh("~");

    ros::Subscriber imageDataSub = nh.subscribe<sensor_msgs::Image>("/camera/image_raw", 1, imageDataHandler); //hik_cam_node/hik_camera /camera/image_raw

    ros::Rate rate(200);
    while (nh.ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}