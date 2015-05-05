/*
 * CT_main.cpp
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

//#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ColourTracking capture;	
	
	capture.CmdParameters(argc, argv);
//	img.setHSV(capture.hsv());
	
//	img.InitCam();
	
	// initialize camera & video capturing
	VideoCapture cap(0);
	
	
    if (!cap.isOpened())
    {
         cout << " Problem loading the camera. Exiting..\n";
         return -1;
    }

	// set height & width of videocapture
	cap.set(CV_CAP_PROP_FRAME_WIDTH, capture.width());
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, capture.height());
	
	cout << " Camera frame HEIGHT:" << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << " WIDTH:" << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
	
	capture.CreateControlWindow();
	
	chrono::high_resolution_clock::time_point start_time, end_time;
	unsigned int time_dif;
	
	while (true)
	{
		start_time = chrono::high_resolution_clock::now();
		
		bool bSuccess = cap.read(capture.imgOriginal);
		if (!bSuccess){
			cout << " Problem reading from camera.\n";
			return -1;
		}
		
		//the argument is the kernel size
		capture.Process(3);
		
		// Display images
		capture.Display();
		
		// stop time measurement
		end_time = chrono::high_resolution_clock::now();
		time_dif = time_dif + (chrono::duration_cast<chrono::milliseconds> (end_time - start_time).count());
		
		if (waitKey(capture.delay(time_dif)) == ESCAPE)
		{
			cout << "ESC key pressed by user. Exiting..\n";
			return -1;
		}
		
	}
	
	
	return 0;
}

