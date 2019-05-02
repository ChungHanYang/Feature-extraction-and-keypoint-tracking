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
	Mat frame, old_frame, image, gray, old_gray; //define all the matrix to do SURF
	Mat frame2, old_frame2, image2, gray2, old_gray2; ///define all the matrix to do  ORB
	vector<Point2f> keypoints_1[2]; //Point2f type vector to store keypoint after doing SURF
	vector<Point2f> keypoints_2[2]; //Point2f type vector to store keypoint after doing ORB
	Size  winSize(15, 15); //Set the window size of calcalating optical flow;
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03); ////Set the terminal criteria of calcalating optical flow;

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

	VideoCapture video("car.avi"); //capture the video for SURF
	if (!video.isOpened()) //check if we succeed
	{
		return -1;
	}

	VideoCapture video2("car2.avi"); //capture the video for ORB
	if (!video2.isOpened()) //check if we succeed
	{
		return -1;
	}

	video >> old_frame; //catch the first frame for SURF
	video2 >> old_frame2; //catch the first frame for SURF
	cvtColor(old_frame, old_gray, COLOR_BGR2GRAY); // convert the first frame for SURF to the grayscale
	cvtColor(old_frame2, old_gray2, COLOR_BGR2GRAY); // convert the first frame for ORB to the grayscale


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

	int minHessian = 1500; //define minimum value to decide if a pixel in an image is a SURF keypoint
	Ptr<SURF> detector = SURF::create(minHessian); //create SURF detector with minHessian we just defined
	vector<KeyPoint> keypoints, keypoints2; //KeyPoint type vector to store keypoint after doing SURF and ORB

	detector->detect(old_gray, keypoints); //detect the SURF keypoints 
	detector2->detect(old_gray2, keypoints2); //detect the ORB keypoints 

	KeyPoint::convert(keypoints, keypoints_1[0]); //change the type of keypoints from KeyPoint to Point2f to fit the function of calculating opticl flow
	KeyPoint::convert(keypoints2, keypoints_2[0]); ////change the type of keypoints from KeyPoint to Point2f to fit the function of calculating opticl flow

	Mat mask, mask2; // create mask for drawing lines
	old_frame.copyTo(mask); //initiate the mask
	old_frame2.copyTo(mask2); //initiate the mask2

	Size videoSize = Size((int)video.get(CV_CAP_PROP_FRAME_WIDTH), (int)video.get(CV_CAP_PROP_FRAME_HEIGHT)); //determine the video size
	VideoWriter writer; //write the outcome as file
	writer.open("SURF.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize); //define the file name of the SURF outcome

	Size videoSize2 = Size((int)video2.get(CV_CAP_PROP_FRAME_WIDTH), (int)video2.get(CV_CAP_PROP_FRAME_HEIGHT)); //determine the video size
	VideoWriter writer2; //write the outcome as file
	writer2.open("ORB.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize2); //define the file name of the SURF outcome


	for (;;) //computing optical flow of SURF keypoints
	{
		video >> frame; //catch the frame of video for SURF
		if (frame.empty()) //check if we succeed
		{
			break;
		}
		cvtColor(frame, gray, COLOR_BGR2GRAY); // convert the frame for SURF to the grayscale

		vector<uchar> status; //define the status parameter for calculating optical flow
		vector<float> err; //define the error parameter for calculating optical flow

		calcOpticalFlowPyrLK(old_gray, gray, keypoints_1[0], keypoints_1[1], status, err, winSize, 3, termcrit, 0, 0.001); //calculating optical flow


		size_t i;
		for (i = 0; i < keypoints_1[1].size(); i++) //drawing lines between keypoints and circle the keypoints
		{

			line(mask, keypoints_1[1][i], keypoints_1[0][i], Scalar(255, 0, 0), 1); //drawing lines between keypoint at mask
			circle(frame, keypoints_1[1][i], 3, Scalar(0, 255, 0), -1, 8); //circle the keypoints at frame

		}


		addWeighted(mask, 0.2, frame, 0.8, 0.0, image); //combine frame and mask
		writer.write(image); //write the outcome into file
		imshow("SURF", image); //show the video
		waitKey(30); //set the amount of frame to be displayed in a second
		swap(keypoints_1[0], keypoints_1[1]); //update the previous keypoints
		swap(old_gray, gray); //update the previous frame 
	}



	for (;;) //computing optical flow of ORB keypoints
	{
		video2 >> frame2; //catch the frame of video for ORB
		if (frame2.empty()) //check if we succeed
		{
			break;
		}
		cvtColor(frame2, gray2, COLOR_BGR2GRAY); // convert the frame for SURF to the grayscale

		vector<uchar> status2; //define the status parameter for calculating optical flow
		vector<float> err2; //define the error parameter for calculating optical flow

		calcOpticalFlowPyrLK(old_gray2, gray2, keypoints_2[0], keypoints_2[1], status2, err2, winSize, 3, termcrit, 0, 0.001); //calculating optical flow


		size_t k;
		for (k = 0; k < keypoints_2[1].size(); k++) //drawing lines between keypoints and circle the keypoint
		{

			line(mask2, keypoints_2[1][k], keypoints_2[0][k], Scalar(255, 0, 0), 1); //drawing lines between keypoint at mask
			circle(frame2, keypoints_2[1][k], 3, Scalar(0, 255, 0), -1, 8); //circle the keypoints at frame

		}


		addWeighted(mask2, 0.2, frame2, 0.8, 0.0, image2); //combine frame and mask
		writer2.write(image2); //write the outcome into file
		imshow("ORB", image2); //show the video
		waitKey(30); //set the amount of frame to be displayed in a second
		swap(keypoints_2[0], keypoints_2[1]); //update the previous keypoints
		swap(old_gray2, gray2); //update the previous frame 
	}

	return 0;
}
