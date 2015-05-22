/*
 * File name: main.cpp
 * File description: Main program.
 * Author: Carl-Martin Ivask
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

    cout << ct.ts() << "Camera frame ";   /* print current frame size */
    cout << " HEIGHT:" << cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << " WIDTH:" << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl; 
    
    
    ct.CreateControlWindow(); /* create control panel with trackbars */
    
    while (true)
    {

        ct.t_start(); /* starting point for time measurement */
        
        bool bSuccess = cap.read(ct.imgOriginal);
        if (!bSuccess){
            cout << ct.ts() << " Problem reading from camera to Mat.\n";
            return -1;
        }
        
        ct.Process(); /* runs all the required functions for image manipulation and object storage */

        ct.Display(); /* display original and/or thresholded frame */   
        
        ct.t_end(); /* end point for time measurement, calculation of time_dif */
        
        if (ct.getGUI()) {
            if (waitKey(ct.delay()) == ESCAPE) /* if specified key (ESC) is pressed, exit program */
            {
                cout << ct.ts() << " ESC key pressed by user. Exiting..\n";
                return -1;
            }
        }
    }
        
    return 0;
}

