/*
 * main.cpp
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

#include "ColourTracking.hpp"

//#include "opencv2/core/core.hpp" included with header
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
//#include <chrono> included with header

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ColourTracking capture;	
	
	capture.CmdParameters(argc, argv);
//	img.setHSV(capture.hsv());
	
//	img.InitCam();
	
	VideoCapture cap(0); /* initialize camera & video capturing */
	
	
    if (!cap.isOpened())
    {
         cout << " Problem loading the camera. Exiting..\n";
         return -1;
    }

	cap.set(CV_CAP_PROP_FRAME_WIDTH, capture.width());   /* set width and */ 
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, capture.height()); /* height of captured frame */
	
	cout << " Camera frame HEIGHT:" << cap.get(CV_CAP_PROP_FRAME_HEIGHT) \
	<< " WIDTH:" << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl; /* print current frame size */
	
	capture.CreateControlWindow(); /* create control panel with trackbars */
	
	while (true)
	{
		capture.t_start(); /* starting point for time measurement */
		
		bool bSuccess = cap.read(capture.imgOriginal);
		if (!bSuccess){
			cout << " Problem reading from camera.\n";
			return -1;
		}
		
		capture.Process(3); /* the argument is the kernel size */
		
		capture.Display(); /* display original and/or thresholded frame */
		
		capture.t_end(); /* end point for time measurement */
		
		if (waitKey(capture.delay()) == ESCAPE)
		{
			cout << "ESC key pressed by user. Exiting..\n";
			return -1;
		}
		
	}
	
	
	return 0;
}

