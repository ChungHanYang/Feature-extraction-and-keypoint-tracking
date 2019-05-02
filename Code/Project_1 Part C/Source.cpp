#include <stdio.h> //include all libraries we need
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/video/tracking.hpp>
#include "opencv2/videoio.hpp"
#include <ctype.h>
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;


int main(int argc, char** argv)
{

	Mat frame3, old_frame3, image3, gray3, old_gray3; //define all the matrix to do SURF descriptor
	Mat frame4, old_frame4, image4, gray4, old_gray4; //define all the matrix to do ORB descriptor
	Mat descriptors_3, descriptors_4, descriptors_3_2, descriptors_4_2; //define the descriptors of SURF and ORB

																		// default parameters of ORB
	int nfeatures = 500;
	float scaleFactor = 1.2f;
	int nlevels = 8;
	int edgeThreshold = 15;
	int firstLevel = 0;
	int WTA_K = 2;
	int scoreType = ORB::HARRIS_SCORE;
	int patchSize = 31;
	int fastThreshold = 20;


	VideoCapture video3("car3.avi"); //capture the video for SURF descriptor
	if (!video3.isOpened()) //check if we succeed
	{
		return -1;
	}

	VideoCapture video4("car4.avi"); //capture the video for SURF descriptor
	if (!video4.isOpened()) //check if we succeed
	{
		return -1;
	}

	video3 >> old_frame3; //catch the first frame for SURF descriptor
	cvtColor(old_frame3, old_gray3, COLOR_BGR2GRAY); // convert the frame for SURF descriptor to the grayscale

	video4 >> old_frame4; //catch the first frame for ORB descriptor
	cvtColor(old_frame4, old_gray4, COLOR_BGR2GRAY); // convert the frame for ORB descriptor to the grayscale

	Ptr<ORB> detector2 = ORB::create(
		nfeatures,
		scaleFactor,
		nlevels,
		edgeThreshold,
		firstLevel,
		WTA_K,
		scoreType,
		patchSize,
		fastThreshold); //create ORB detector with the parameters we just defined

	int minHessian = 7000; //define minimum value to decide if a pixel in an image is a SURF keypoint
	Ptr<SURF> detector = SURF::create(minHessian); //create SURF detector with minHessian we just defined
	vector<KeyPoint> keypoints3, keypoints4, keypoints3_2, keypoints4_2; //KeyPoint type vector to store keypoints for descriptors 

	BFMatcher matcher(NORM_L2, true); //define the matcher for SURF
	BFMatcher matcher2(NORM_HAMMING, true); //define the matcher for ORB
	vector<DMatch> matches_3, matches_4, good_matches_3, good_matches_4; //define vector to store the index of matched keypoints

	detector->detect(old_gray3, keypoints3); //detect the SURF keypoints
	detector->compute(old_gray3, keypoints3, descriptors_3); //compute the SURF descriptor    

	detector2->detect(old_gray4, keypoints4); //detect the ORB keypoints
	detector2->compute(old_gray4, keypoints4, descriptors_4); //compute the ORB descriptor    

	Mat mask3, mask4; // create mask for drawing lines
	old_frame3.copyTo(mask3);
	old_frame4.copyTo(mask4);

	Size videoSize = Size((int)video3.get(CV_CAP_PROP_FRAME_WIDTH), (int)video3.get(CV_CAP_PROP_FRAME_HEIGHT)); //determine the video size
	VideoWriter writer; //write the outcome as file
	writer.open("SURF_descriptor.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize); //define the file name of the SURF outcome

	Size videoSize2 = Size((int)video3.get(CV_CAP_PROP_FRAME_WIDTH), (int)video4.get(CV_CAP_PROP_FRAME_HEIGHT)); //determine the video size
	VideoWriter writer2; //write the outcome as file
	writer2.open("ORB_descriptor.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize2); //define the file name of the SURF outcome


	for (;;) //computing optical flow of SURF descriptor
	{

		video3 >> frame3; //catch the video frame
		if (frame3.empty()) //check if we succeed
		{
			break;
		}
		cvtColor(frame3, gray3, COLOR_BGR2GRAY); //convert to grayscale

		detector->detect(gray3, keypoints3_2); //detect the SURF keypoints
		detector->compute(gray3, keypoints3_2, descriptors_3_2); //compute the SURF keypoints descriptor

		matcher.match(descriptors_3, descriptors_3_2, matches_3); //match two descriptors


		size_t i;
		for (i = 0; i<matches_3.size(); i++) //start drawing lines
		{

			Point2f point_old = keypoints3[matches_3[i].queryIdx].pt; //get the coordinate of matched keypoints
			Point2f point_new = keypoints3_2[matches_3[i].trainIdx].pt; //get the coordinate of matched keypoints
			line(mask3, point_new, point_old, Scalar(255, 0, 0), 1); //draw the lines at mask we just created
			circle(frame3, point_new, 3, Scalar(0, 255, 0), -1, 8); //circle the keypoints at current frame
		}

		addWeighted(mask3, 0.3, frame3, 0.7, 0.0, image3); //combine frame and mask
		writer.write(image3); //write the outcome into file
		imshow("SURF_matches", image3); //show the matched picture
		waitKey(30); //set the amount of frame to be displayed in a second

	}


	for (;;) //computing optical flow of ORB descriptor
	{

		video4 >> frame4; //catch the video frame
		if (frame4.empty()) //check if we succeed
		{
			break;
		}
		cvtColor(frame4, gray4, COLOR_BGR2GRAY); //convert to grayscale

		detector2->detect(gray4, keypoints4_2); //detect the ORB keypoints
		detector2->compute(gray4, keypoints4_2, descriptors_4_2); //compute the ORB keypoints descriptor

		matcher2.match(descriptors_4, descriptors_4_2, matches_4); //match two descriptors


		size_t k;
		for (k = 0; k<matches_4.size(); k++) //start drawing lines
		{

			Point2f point_old2 = keypoints4[matches_4[k].queryIdx].pt; //get the coordinate of matched keypoints
			Point2f point_new2 = keypoints4_2[matches_4[k].trainIdx].pt; //get the coordinate of matched keypoints
			line(mask4, point_new2, point_old2, Scalar(255, 0, 0), 1); //draw the lines at mask we just created
			circle(frame4, point_new2, 3, Scalar(0, 255, 0), -1, 8); //circle the keypoints at current frame
		}

		addWeighted(mask4, 0.3, frame4, 0.7, 0.0, image4); //combine frame and mask
		writer2.write(image4); //write the outcome into file
		imshow("ORB_matches", image4); //show the matched picture
		waitKey(30); //set the amount of frame to be displayed in a second   

	}


	return 0;
}
