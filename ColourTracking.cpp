/*
 * ColourTracking.cpp
 * 
 * Copyright 2015 Carl Ivask
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/core.hpp" // already defined in header

#include "ColourTracking.hpp"

#include <vector>
#include <cmath>
#include <iostream>
#include <string>
// <chrono> also included with header

using namespace cv;
using namespace std::chrono;


/*
int ColourTracking::InitCam(unsigned int vidnr, unsigned int height, unsigned int width)
{
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cout << "Problem loading camera. Exiting..\n";
        return -1;
    }
    cap.set(cv::CV_CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CV_CAP_PROP_FRAME_HEIGHT, height);
    std::cout << "Camera frame HEIGHT:" cap.get(cv::CV_CAP_PROP_FRAME_HEIGHT) << " WIDTH:" << cap.get(CV_CAP_PROP_FRAME_WIDTH);
    
    return 1;
}
*/

 // constructor that gives default values to all parameters
ColourTracking::ColourTracking()
{
	iCount = ENABLED;
	iMorphLevel = DISABLED;
	iShowOriginal = DISABLED;
	iShowThresh = DISABLED;
	bGUI = ENABLED;
	
	int buffer[6] = {LHUE, HHUE, LSAT, HSAT, LVAL, HVAL};
	setHSV(buffer);
	
	uiDelay = MAX_CYCLE_T - MIN_CYCLE_T;
    iDebugLevel = DISABLED;
	uiCaptureHeight = CAP_HEIGHT;
	uiCaptureWidth = CAP_WIDTH;
	
	ObjectMinsize = OBJ_MINSIZE;
	ObjectMaxsize = OBJ_MAXSIZE;
	
//	comm_pass = COMM_PASS;
	comm_port = COMM_PORT;
}


void ColourTracking::Process(int msize)
{
    std::vector<cv::Point> locs;
    std::vector<float> areas;
    
    ThresholdImage(imgOriginal, imgHSV, imgThresh, iHSV, true);
    MorphImage(iMorphLevel, msize, imgThresh, imgThresh);
    CorrectAmount(FindObjects(imgThresh, locs, areas, ObjectMinsize, ObjectMaxsize), 20, 0.25);
    DrawCircles(imgOriginal, imgCircles, locs, areas);
}

void ColourTracking::ThresholdImage(cv::Mat src, cv::Mat& buf, cv::Mat& dst, int hsv[], bool blur)
{
	// RGB -> HSV		
	cv::cvtColor(src, buf, cv::COLOR_BGR2HSV);
	if (blur) cv::GaussianBlur(buf, buf, cv::Size(5,5), 0,0);
			
	// HSV -> binary (black&white)
	cv::inRange(buf, cv::Scalar(hsv[0], hsv[2], hsv[4]), cv::Scalar(hsv[1], hsv[3], hsv[5]), dst);

}
    

void ColourTracking::MorphImage(unsigned int morph, int size, cv::Mat src, cv::Mat& dst)
{
	// buffer Mat on which to use erode and dilate
	cv::Mat buf = src;
	
	if (morph > 0)	cv::erode(buf, buf, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size)));
	if (morph > 1) cv::dilate(buf, buf, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size))); 	
	if (morph > 1) cv::dilate(buf, buf, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size)));
	if (morph > 0) cv::erode(buf, buf, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size)));
	
	dst = buf;
}
    
    // originally named CountContours
int ColourTracking::FindObjects(cv::Mat src, std::vector<cv::Point>& centers, std::vector<float>& areas, float minsize, float maxsize)
{
	cv::Mat imgBuffer8u;
			
	src.convertTo(imgBuffer8u, CV_8U);
			
	std::vector<std::vector<cv::Point> > contours;
	
	cv::findContours(imgBuffer8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	
	
	std::vector<cv::Moments> mv; //temporary moment vector
	std::vector<float> sv; //temporary area vector
	std::vector<cv::Point> mc; //temporary mass center vector (location)
	
	
	// get moments of objects
	for (unsigned int i = 0; i < contours.size(); i++){

		sv.push_back (contourArea(contours[i]));

		if (sv.back() >= minsize && sv.back() <= maxsize){
			mv.push_back (cv::moments(contours[i], false));
		}
		else sv.pop_back();
		
	}
	areas = sv;
	
	
	// get mass centers
	for (unsigned int i = 0; i < sv.size(); i++){
		mc.push_back (cv::Point((int) (mv[i].m10/mv[i].m00), (int) (mv[i].m01/mv[i].m00)));
	}
	centers = mc;
	
	
	// returns number of mass centers (aka objects)
	return centers.size();
}
    
// correction of the results of FindObjects
// interval - number of cycles after which new amount is checked
// dif - difference between new amount and average of previous X amounts (+-)
int ColourTracking::CorrectAmount(int amount, int interval, float dif)
{
	static int k = 0;
	static int prev = 0;
	static double total = 0;
	
	k++;
	total += amount;
		
	if (k >= interval)
	{	

		if (amount != prev && amount < (total/(k+1))+dif && amount >= (total/(k+1))-dif){
			prev = amount;
			k = 0;
			total = amount;
			return amount;
		} 
		else {
			k = 0;
			total = amount;
			return prev;
		}
	}
	
	return prev;
}
    
    // draw circles around found objects
void ColourTracking::DrawCircles(cv::Mat src, cv::Mat& dst, std::vector<cv::Point> coords, std::vector<float> area)
{
	dst = cv::Mat::zeros(src.size(), src.type());
	float rad;
	
	for (unsigned int i=0; i<coords.size(); i++){

		if (iDebugLevel >= 3) std::cout /*<< Timestamp() << " " */<< (i+1) << \
		".OBJECT X:" << coords[i].x << " Y:" << coords[i].y << " AREA: " << area[i] << std::endl;
		 
		// circle area = pi * radius^2
		rad = sqrt(area[i]/PI_VALUE);
		cv::circle(dst,coords[i],rad, cv::Scalar(0,0,255), 2, 8, 0);
	}
	
}

void ColourTracking::nogui()
{
	bGUI = false;
}

void ColourTracking::Display()
{
    if (bGUI && iShowThresh == ENABLED) imshow("Thresholded Image", imgThresh); //show the thresholded image	
            else destroyWindow("Thresholded Image");
    if (bGUI && iShowOriginal == ENABLED) imshow("Original", imgOriginal); //show the original image
            else destroyWindow("Original");
}
    
    // acquire hsv values from around clicked area
int ColourTracking::ClickHSV(cv::Mat src, int x, int y, int edge)
{
	cv::Vec3b values;
	cv::Mat square;
	int avgH = 0, avgS = 0, avgV = 0;
	int lowestS, lowestV;
	int highestS, highestV;
	
	int i,j;
		
	square = src(cv::Rect(x-(edge/2), y-(edge/2), edge, edge));
	cv::blur(square, square, cv::Size(3,3), cv::Point(-1,-1));
		
	for (i=0; i<edge; i++){
		for (j=0; j<edge; j++){
			
			values = square.at<cv::Vec3b>(i,j);
			if (i == 0 && j == 0){
				lowestS = 127;
				highestS = 128;
				lowestV = 127;
				highestV = 127;
			}
				
			// calculate hue by iterating through each pixel and finding average
			avgH += values.val[0];
				
			// calculate saturation & value (intensity) by finding highest and lowest values
			if (values.val[1] <= lowestS) lowestS = values.val[1];
			if (values.val[1] > highestS) highestS = values.val[1];
				
			if (values.val[2] <= lowestV) lowestV = values.val[2];
			if (values.val[2] > highestV) highestV = values.val[2];
		}
	}
	avgH = avgH / (edge * edge);
	avgS = (highestS + lowestS)/2;
	avgV = (highestV + lowestV)/2;

	
	if (avgH > 10 && avgH <= ORANGE){
		iHSV[0] = 0;
		iHSV[1] = ORANGE;
	}
	else if (avgH > ORANGE && avgH <= YELLOW){
		iHSV[0] = ORANGE;
		iHSV[1] = YELLOW;
	} 
	else if (avgH > YELLOW && avgH <= GREEN){
		iHSV[0] = YELLOW;
		iHSV[1] = GREEN;
	}
	else if (avgH > GREEN && avgH <= BLUE){
		iHSV[0] = GREEN;
		iHSV[1] = BLUE;
	}
	else if (avgH > BLUE && avgH <= VIOLET){
		iHSV[0] = BLUE;
		iHSV[1] = VIOLET;
	}
	else if (avgH > VIOLET && avgH <= RED){
		iHSV[0] = VIOLET;
		iHSV[1] = RED;
	}    

	iHSV[2] = (int) avgS - iSatAdjust;
	iHSV[3] = highestS + 50;//(int) avgS + 200;
	iHSV[4] = (int) avgV - iValAdjust;
	iHSV[5] = highestV + 50;//(int) avgV + 200;
	
	return 1;
}


void ColourTracking::setHSV (int user[])
{
	for (int i = 0; i < 6; i++){
        iHSV[i] = user[i];
    }
}


int* ColourTracking::hsv()
{
    return iHSV;
}

int ColourTracking::val(unsigned int k)
{
    return iHSV[k];
}

unsigned int ColourTracking::height()
{
	return uiCaptureHeight;
}

unsigned int ColourTracking::width()
{
	return uiCaptureWidth;
}


void ColourTracking::t_start()
{
	start_time = high_resolution_clock::now();
}

void ColourTracking::t_end()
{
	end_time = high_resolution_clock::now();
	
	//
	time_dif += (duration_cast<milliseconds> (end_time - start_time).count());
}

unsigned int ColourTracking::delay()
{
	static unsigned int k = 0; //counter
	static unsigned int result = uiDelay;
	
	if (k >= DEF_INTERVAL){
		
		if (iDebugLevel >= 2){
			std::cout << " [avg duration per " << k+1 << " cycles]: " << ((int) time_dif/(k+1)) << "ms\n";
		}
		
		result = (int) time_dif/(k+1); 
				
		if (result < MIN_CYCLE_T) result = MIN_CYCLE_T; //minimum cycle time
		if (result > MAX_CYCLE_T) result = MAX_CYCLE_T; //max
		
		if (iDebugLevel >= 2){				
			std::cout << " Setting cycle time to: " << result << "ms\n";
		}
				
		k = 0;
		time_dif = 0;
	}
	
	k++;
	
	return result;
}


bool ColourTracking::CreateControlWindow()
{
	namedWindow("Control", CV_WINDOW_NORMAL);
		
	cvCreateTrackbar("HUE min", "Control", &iHSV[0], 179);
	cvCreateTrackbar("HUE max", "Control", &iHSV[1], 179);
	cvCreateTrackbar("SAT min", "Control", &iHSV[2], 255);
	cvCreateTrackbar("SAT max", "Control", &iHSV[3], 255);
	cvCreateTrackbar("VAL min", "Control", &iHSV[4], 255);
	cvCreateTrackbar("VAL max", "Control", &iHSV[5], 255);
	cvCreateTrackbar("SAT adj", "Control", &iSatAdjust, 150);
	cvCreateTrackbar("VAL adj", "Control", &iValAdjust, 150);
	cvCreateTrackbar("Original", "Control", &iShowOriginal, 1);
	cvCreateTrackbar("Thresh", "Control", &iShowThresh, 1);
	cvCreateTrackbar("Count", "Control", &iCount, 1);
	cvCreateTrackbar("Morph", "Control", &iMorphLevel, 2);
	cvCreateTrackbar("Debug", "Control", &iDebugLevel, 4);
	
	return true;
}


int ColourTracking::CmdParameters(int argc, char** argv)
{
	if (argc > 1){
		for (int j=1; j<argc; j++){
			if (!std::strcmp(argv[j], "-help")){
				std::cout << "List of arguments:\n-framesize  height width\n-objsize min max (0..300000)\n";
				std::cout << "-nogui (Disables graphical user interface)\n";
				std::cout << "-hue min max (0..179)\n-sat min max (0..255)\n-val min max (0.255)\n";
				std::cout << "-morph # (0..2)\n-nocount (Not recommended for commandline)\n";
				std::cout << "-udppass [string]  (passphrase that udp client needs to provide)\n";
				std::cout << "-udpport [port nr]  (port nr for udp communication, 2000..65535)\n";
				return -1;
			}
			else if (!std::strcmp(argv[j],"-framesize")){
					uiCaptureHeight = std::atoi(argv[j+1]);
					uiCaptureWidth = std::atoi(argv[j+2]);
					j += 2;
					if (uiCaptureHeight < 64 || uiCaptureHeight > 512 || uiCaptureWidth < 64 || uiCaptureWidth > 512){
						std::cout << "Height and width can be set between 64..512.\n";
						return -1;
					}
			} 
			else if (!std::strcmp(argv[j],"-nogui")){
					nogui();
			}
			else if (!std::strcmp(argv[j],"-hue")){
				iHSV[0] = std::atoi(argv[j+1]);
					iHSV[1] = std::atoi(argv[j+2]);
					if (iHSV[0] < 0 || iHSV[0] > 179 || iHSV[1] < 0 || iHSV[1] > 179){
						std::cout << "Hue values can be set between 0..179.\n";
						return -1;
					}
					j += 2;
			}
			else if (!std::strcmp(argv[j],"-sat")){
				iHSV[2] = std::atoi(argv[j+1]);
					iHSV[3] = std::atoi(argv[j+2]);
					if (iHSV[2] < 0 || iHSV[2] > 255 || iHSV[3] < 0 || iHSV[3] > 255){
						std::cout << "Saturation can be set between 0..255.\n";
						return -1;
					}
					j += 2;
			}
			else if (!std::strcmp(argv[j],"-val")){
				iHSV[4] = std::atoi(argv[j+1]);
					iHSV[5] = std::atoi(argv[j+2]);
					if (iHSV[4] < 0 || iHSV[4] > 255 || iHSV[5] < 0 || iHSV[5] > 255){
						std::cout << "Light intensity can be set between 0..255.\n";
						return -1;
					}
					j += 2;
			}
			else if (!std::strcmp(argv[j],"-debug")){
				iDebugLevel = std::atoi(argv[j+1]);
				j++;
					if (iDebugLevel < 0 || iDebugLevel > 4){
						std::cout << "Debug level can be set from 0 to 4.\n";
						return -1;
					}
			}
			else if (!std::strcmp(argv[j],"-morph")){
				iMorphLevel = std::atoi(argv[j+1]);
				j++;
					if (iMorphLevel < 0 || iMorphLevel > 2){
						std::cout << "Morph level can be set from 0 to 2.\n0 - no morph, 1 - only erode, 2 - erode & dilate\n";
						return -1;
					}
			}
			else if (!std::strcmp(argv[j],"-nocount")){
				iCount = 0;
			}
			else if (!std::strcmp(argv[j],"-udppass")){
				std::strcpy(argv[j+1],comm_pass);
				j++;
			}
			else if (!std::strcmp(argv[j],"-udpport")){
				comm_port = std::atoi(argv[j+1]);
				if (comm_port < 2000 || comm_port > 65535){
					std::cout << "Communication port must be between 2000 and 65535.\n";
					return -1;
				}
				j++;
			}
			else if (!std::strcmp(argv[j],"-objsize")){
				ObjectMinsize = std::atoi(argv[j+1]);
				ObjectMaxsize = std::atoi(argv[j+2]);
				if (ObjectMaxsize > 300000 || ObjectMinsize < 0){
					std::cout << "Object area must be above 0 and below 300000 (512x512 == 262144)\n";
					return -1;
				}
				j += 2;
			}

		}
	}
	
	return 1;
}

std::string ColourTracking::timestamp()
{
	std::string stamp = "[";
	
	
	
	return stamp;
}
