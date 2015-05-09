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

#include "opencv2/highgui/highgui.hpp"

#include <iostream>


using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ColourTracking ct;
		
	if (ct.CmdParameters(argc, argv) < 0) return -1; /* parse command line arguments */
	
	
	VideoCapture cap(0); /* initialize camera & video capturing */
	
	
    if (!cap.isOpened()) /* if camera failed to initialize, exit program */
    {
         cout << ct.ts() << " Problem loading the camera. Exiting..\n";
         return -1;
    }

	cap.set(CV_CAP_PROP_FRAME_WIDTH, ct.width());   /* set width and */ 
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, ct.height()); /* height of captured frame */

	cout << ct.ts() << " Camera frame HEIGHT:" << cap.get(CV_CAP_PROP_FRAME_HEIGHT) \
	<< " WIDTH:" << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl; /* print current frame size */
	
	
	ct.CreateControlWindow(); /* create control panel with trackbars */
	
	
	while (true)
	{

		ct.t_start(); /* starting point for time measurement */
		
		bool bSuccess = cap.read(ct.imgOriginal);
		if (!bSuccess){
			cout << ct.ts() << " Problem reading from camera to Mat.\n";
			return -1;
		}
		
		ct.Process(3); /* the argument is the kernel size for morph*/

		ct.Display(); /* display original and/or thresholded frame */	
		
		ct.t_end(); /* end point for time measurement, calculation of time_dif */
		
		if (waitKey(ct.delay()) == ESCAPE) /* if specified key (ESC) is pressed, exit program */
		{
			cout << ct.ts() << " ESC key pressed by user. Exiting..\n";
			return -1;
		}
		
	}
		
	return 0;
}

