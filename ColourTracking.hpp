/*
 * File name: ColourTracking.hpp
 * File description: Class header.
 * Author: Carl-Martin Ivask
 * 
 */

#ifndef _ColourTracking_HPP_
#define _ColourTracking_HPP_

#define COMM_PORT 12015
#define COMM_PROTOCOL 0 // UDP
#define COMM_PASS "objectcount"

#define CAP_HEIGHT 320
#define CAP_WIDTH 360
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
#define ESCAPE 27
#define DEF_INTERVAL 100
#define PI_VALUE 3.1415926535
#define DEFAULT 3
#define DEF_DEBUG 1
#define MORPH_KERNEL_SIZE 3

// approximate high hues of colours
#define ORANGE 22
#define YELLOW 38
#define GREEN 75
#define BLUE 130
#define VIOLET 160
#define RED 179

#include "opencv2/core/core.hpp"
#include <chrono>

#include <netinet/in.h>

class ColourTracking
{
    private:
    /******************** Private access variables ********************/
    /***************** (e.g. configuration parameters) ****************/
    
    // Image pixel arrays
    cv::Mat imgThresh;
    cv::Mat imgCircles;

    // do counting; show unaltered image; show thresholded image; GUI; blur when thresh
    int iCount;
    int iShowOriginal;
    int iShowThresh;
    bool bGUI;
    bool bThreshBlur;

    // hue/saturation/light intensity values; morph level; debug level
    int iHSV[6];
    int iMorphLevel;
    int iDebugLevel;
    
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
    //int objectamount;
    
    // chrono time measuring variables
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point end_time;
    unsigned int time_dif;  
    
    // udp communication variables
    int sockfd; /* socket file descriptor */
    struct sockaddr_in server_addr, client_addr; /* server & client address */
    socklen_t clientlen; /* length of client address */
    char CommPassBuffer[64]; /* message received from client */
    char CommSendBuffer[2048]; /* message sent to client */

    
    // struct for object member variables
    struct Object
    {
        unsigned int index; // object index
        int x; // x coord
        int y; // y coord
        int area; // area value
        unsigned int rm_counter; // if this reaches 0, the object is removed
        unsigned int exist_counter; // amount of cycles the object has existed
        float cm; // coordinate margin

        
        Object(unsigned int newindex, int newx, int newy, int newarea, int rmdef) 
        { 
            index = newindex;
            x = newx;
            y = newy;
            area = newarea;
            rm_counter = rmdef;
            exist_counter = 0;
            cm = (sqrt(area))/2;
            
        } 

    };
    
    unsigned int IDcounter; // will have a new ID for each object
    unsigned int rm_default;
    unsigned int MinLife;
    int iObjMove;
    
    std::vector<Object> vecExistingObjects;
    std::vector<Object> vecFoundObjects;


    /******************************************************************/
    
    
    
    /******************** OpenCV-related and other ********************/
    /******************** private access functions ********************/
    
    // threshold image with user defined parameters
    void ThresholdImage(cv::Mat, cv::Mat&, int [], bool);
    
    // erode & dilate binary image
    void MorphImage(unsigned int, int, cv::Mat, cv::Mat&);
    
    // create vectors for moments, areas and mass centers
    int FindObjects(cv::Mat, float, float, std::vector<Object>&); 
    
    // work with object vectors
    unsigned int AddNewObjects(std::vector<Object> found, std::vector<Object>& exist);
    void ExistentialObjects(std::vector<Object> found, std::vector<Object>& exist);
    unsigned int CleanupObjects(std::vector<Object>& exist);
    bool FitMargin(int ax, int ay, int bx, int by, float cm);
    
    // draw circles around found objects
    void DrawCircles(cv::Mat, cv::Mat&, std::vector<Object>);
    
    // Information transmission via UDP
    void SetupSocket();/* bind socket */
    void RecvSend(char*, char*); /* receive and send information back (if correct pass) */
    void WriteSendBuffer(std::vector<Object>, char*); /* write useful information to buffer */ 
    
    void disablegui(); /* set GUI parameter to false */

    /******************************************************************/
    
    
    
    public:
    /******************** Public access variables *********************/
    /******************************************************************/
    cv::Mat imgOriginal; /* this Mat is public because it's used in main */

    void setHSV (int *); /* set hue, saturation and light intensity*/
 
    int* hsv() { return iHSV; } /* return HSV array */

    int val(unsigned int k) { return iHSV[k]; } /* return single value from hsv array */ 

    unsigned int height() { return uiCaptureHeight; } /* return captured frame height */ 

    unsigned int width() { return uiCaptureWidth; } /* return array of hsv values */

    /******************************************************************/
    
    
    
    /********************* Public functions ***************************/
    /******************************************************************/
        
    ColourTracking() /* assign default values */
    {
        iCount = ENABLED;
        iMorphLevel = DISABLED;
        iShowOriginal = DISABLED;
        iShowThresh = DISABLED;
        bGUI = ENABLED;
        bThreshBlur = ENABLED;
        
        int buffer[6] = {LHUE, HHUE, LSAT, HSAT, LVAL, HVAL};
        setHSV(buffer);
        
        uiDelay = MAX_CYCLE_T - MIN_CYCLE_T;
        iDebugLevel = DEF_DEBUG;
        uiCaptureHeight = CAP_HEIGHT;
        uiCaptureWidth = CAP_WIDTH;
        
        ObjectMinsize = (uiCaptureHeight * uiCaptureWidth) / 100;
        ObjectMaxsize = (uiCaptureHeight * uiCaptureWidth) / 4;
        
        strncpy(comm_pass, COMM_PASS, sizeof(COMM_PASS));
        comm_port = COMM_PORT;
        
        rm_default = 5;
        MinLife = rm_default * 2; // default is always higher than removal counter
        iObjMove = ENABLED;
        
        SetupSocket();
    }
        
    void Process();
    
    // display original and thresholded images
    void Display();
    
    // run-time control panel with highgui trackbars
    bool CreateControlWindow();
    
    // calculate delay for the waitKey function in main.cpp
    unsigned int delay();
    void t_start(); /* set starting timepoint */
    void t_end();   /* set end timepoint and calculate time_dif*/
    
    // parse command line arguments
    int CmdParameters(int, char**); 
    
    // returns timestamp "[HH:MM:SS]"
    std::string ts();
    
    /******************************************************************/
};




#endif

