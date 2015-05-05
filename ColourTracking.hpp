/*
 * ColourTracking.hpp
 * 
 * Copyright 2015 Carl Ivask <carl@vaarikakook>
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


#ifndef _ColourTracking_HPP_
#define _ColourTracking_HPP_

#define COMM_PORT 12015
#define COMM_PROTOCOL 0
#define COMM_PASS "objectcount"

#define CAP_HEIGHT 256
#define CAP_WIDTH 256
#define RECT_EDGE 9
#define ENABLED 1
#define DISABLED 0
#define MAX_CYCLE_T 200
#define MIN_CYCLE_T 50
#define LHUE 0
#define LSAT 0
#define LVAL 0
#define HHUE 179
#define HSAT 255
#define HVAL 255
#define OBJ_MINSIZE 250.00 // 50x50 large enough (example: cups from 2m away)
#define OBJ_MAXSIZE 10000.00 //100x100
#define ESCAPE 27
#define DEF_INTERVAL 100
#define PI_VALUE 3.1415926535
#define DEFAULT 3

// approximate high hues of colours
#define ORANGE 22
#define YELLOW 38
#define GREEN 75
#define BLUE 130
#define VIOLET 160
#define RED 179

#include "opencv2/core/core.hpp"
#include <chrono>

class ColourTracking
{
	private:
	/******************** Private access variables ********************/
	/***************** (e.g. configuration parameters) ****************/
	
	// Mats for different image stages
    cv::Mat imgHSV;
    cv::Mat imgThresh;
    cv::Mat imgCircles;

	// do counting; show unaltered image; show thresholded image; GUI
	int iCount;
	int iShowOriginal;
	int iShowThresh;
	bool bGUI;
	
	// hue/saturation/light intensity values; morph level; debug level
	int iHSV[6];
	int iMorphLevel;
	int iDebugLevel;
	
	// sat and val adjustment after ClickHSV
	int iSatAdjust;
	int iValAdjust;
	
	// main loop delay; captured frame height; captured frame width
	unsigned int uiDelay;
	unsigned int uiCaptureHeight;
	unsigned int uiCaptureWidth;
	
	// limits for counting objects
	float ObjectMinsize;
	float ObjectMaxsize;
	
	// parameters for use in UDP communication
	char comm_pass[256];
	unsigned int comm_port;
	
	// chrono time measuring variables
	std::chrono::high_resolution_clock::time_point start_time;
	std::chrono::high_resolution_clock::time_point end_time;
	unsigned int time_dif;
	
	/******************************************************************/
	
	
    
    /******************** OpenCV-related and other ********************/
    /******************** private access functions ********************/
    
    // threshold image with user defined parameters
    void ThresholdImage(cv::Mat, cv::Mat&, cv::Mat&, int [], bool);
    
    // erode & dilate binary image
    void MorphImage(unsigned int, int, cv::Mat, cv::Mat&);
    
    // create vectors for moments, areas and mass centers
    int FindObjects(cv::Mat, std::vector<cv::Point>&, std::vector<float>&, float, float); 
    
    // correction of the results of FindObjects
    int CorrectAmount(int, int, float);
    
    // draw circles around found objects
    void DrawCircles(cv::Mat, cv::Mat&, std::vector<cv::Point>, std::vector<float>);
    
    // acquire hsv values from around clicked area
    int ClickHSV(cv::Mat, int, int, int);
        
    // returns timestamp closed with brackets (used in std::cout)
    // STILL NOT IMPLEMENTED AGAIN
    std::string timestamp();
    
    // TODO
    // SETMOUSECALLBACK!!!!!!!!!!!!!!!!!!!!!!!!
    
    /******************************************************************/
    
    
    
    public:
    /******************** Public access variables *********************/
    /******************************************************************/
    
	cv::Mat imgOriginal; /* this Mat is public because it's used in main */
	
	void setHSV (int *); /* set hue, saturation and light intensity*/
    int* hsv(); /* return array of hsv values */
    int val(unsigned int); /* return single value from hsv array */
    
    unsigned int height(); /* return captured frame height */
	unsigned int width();
	
    void nogui(); /* set GUI parameter to false */
    
    /******************************************************************/
    
    
    
	/********************* Public functions ***************************/
	/******************************************************************/
	
	//constructor that sets default values for all parameters
	ColourTracking();
	
//   int InitCam(unsigned int);
//   int CapFrame(cv::Mat&);
    
    // applies all the main functions (described above) on captured frame
    void Process(int);
    
    // display original and thresholded images
    void Display();
	
	// run-time control panel with highgui trackbars
	bool CreateControlWindow();

	
	void t_start(); /* set starting timepoint */
	void t_end();   /* set end timepoint and calculate time_dif*/
	
	// calculate delay for the waitKey function in main.cpp
    unsigned int delay();
	
	// parse command line arguments
	int CmdParameters(int, char**);
	
	/******************************************************************/
};




#endif

