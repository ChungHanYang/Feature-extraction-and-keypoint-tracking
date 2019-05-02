Richard Aguilar

Yang Chung Han

Project 1 
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

The following program uses OpenCV on Visual Studio 2015 on a Windows computer. In order to avoid memory breakpoints, we have created one project for Part B and another for Part C. Both will be executed for results. 

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
DESCRIPTION: 

This program was created to make use of the SURF and ORB feature descriptors/detectors in conjunction with Optical Flow (KLT). It is necessary to use one of these feature detectors in order to determine the keypoints 
in our video so that Optical Flow use them for tracking purposes. This program will visually show this process in two windows one at a time.The first window will play through the video and feature the SURF functionality
and then another window will appear and feature ORB. 

SURF code citation: detector->detect(old_gray, keypoints); //detect the SURF keypoints 

			
ORB code citation: detector2->detect(old_gray2, keypoints2); //detect the ORB keypoints

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

FOLDER LISTING:

-Code : Contains the source code C++ file with comments as well as the Visual Studio project file for executable code. 
-Input : Contains all input data such as the two videos used to demonstrate the feature detectors. 
-Output : Contains screenshots of all results of running our code. 
-Report : Contains details on all project goals and results
-Grading Sheet: Contains requirements of project for grading.  

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

RUNNING THE PROGRAMS:

1. Navigate to Han_Aguilar_Project\Code\Project_1 Part B 

2. Open Project_1.vcxproj. (Both videos (car.avi, car2.avi) that are necessary for code simulation are found in Han_Aguilar_Project\Input\Part B) 

3. Click Local Windows Debugger at the top middle of the Visual Studio's panel.

4. You will now observe the SURF functionality throughout the length of the video, followed by the ORB functionality in a new window. 

5. Navigate to Han_Aguilar_Project\Code\Project_1 Part C

6.Open Project_1_Part_C.vcxproj. (Both videos (car3.avi, car4.avi) that are necessary for code simulation are found in Han_Aguilar_Project\Input\Part C) 

7. Repeat Step 3

8. You will now observe the SURF functionality throughout the length of the video, followed by the ORB functionality in a new window. 

 

