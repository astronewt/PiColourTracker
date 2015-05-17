/*
 * File name: ColourTracking.cpp
 * File description: Implementation of ColourTracking class.
 * Author: Carl-Martin Ivask
 * 
 */


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ColourTracking.hpp"

#include <vector>
#include <iostream>

/** Includes for socket communication **/
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
/** ** ** **/

using namespace cv;
using namespace std::chrono;

void ColourTracking::Process() // main process
{   
    ThresholdImage(imgOriginal, imgThresh, iHSV, bThreshBlur);
    
    MorphImage(iMorphLevel, MORPH_KERNEL_SIZE, imgThresh, imgThresh);
    
    if (iCount > 0){
        
        FindObjects(imgThresh, ObjectMinsize, ObjectMaxsize, vecFoundObjects);
        
        AddNewObjects(vecFoundObjects, vecExistingObjects);
        ExistentialObjects(vecFoundObjects, vecExistingObjects);
        
        CleanupObjects(vecExistingObjects);
        
        WriteSendBuffer(vecExistingObjects, CommSendBuffer); /* write useful info to buffer */

    } 
    else {
        IDcounter = 0;  /* reset ID counter */
        if (!vecExistingObjects.empty()) vecExistingObjects.clear();
    }
    
    RecvSend(CommPassBuffer, CommSendBuffer); /* transmit buffer via UDP */
    
    DrawCircles(imgOriginal, imgCircles, vecExistingObjects);
}

/******** Functions regarding detection and storage of objects ********/
int ColourTracking::FindObjects(cv::Mat src, float minsize, float maxsize, std::vector<Object>& found)
{
    cv::Mat imgBuffer8u;
            
    src.convertTo(imgBuffer8u, CV_8U);
            
    std::vector<std::vector<cv::Point> > contours;
    
    cv::findContours(imgBuffer8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    found.clear(); // clear vector to make room for new objects
    
    std::vector<cv::Moments> mv; // temporary moment vector 
    std::vector<float> sv; // temporary area vector 
    std::vector<cv::Point> mc; // temporary mass center vector (location) 
    
    
    for (unsigned int i = 0; i < contours.size(); i++) { // get moments of objects 

        sv.push_back (contourArea(contours[i]));


        if (sv.back() >= minsize && sv.back() <= maxsize) {
            mv.push_back (cv::moments(contours[i], false));
        }
        else sv.pop_back();
        
    }
    
    // get mass centers and create new objects in one loop
    for (unsigned int i = 0; i < sv.size(); i++) {
        
        mc.push_back (cv::Point((int) (mv[i].m10/mv[i].m00), (int) (mv[i].m01/mv[i].m00)));
        
        // Object arguments: new index, x, y, area, coordinate margin, remove counter
        found.push_back (Object(i, (int) mc[i].x, (int) mc[i].y, sv[i], rm_default));
    }
    
    // returns number of mass centers (aka objects)
    return found.size();
}

unsigned int ColourTracking::AddNewObjects(std::vector<Object> found, std::vector<Object>& exist)
{
    bool isnew = true;
    
    if (!exist.empty()) {         
        if (!found.empty()) {
            
            // add currently found objects to vecExistingObjects
            for (unsigned int i = 0; i < found.size(); i++) {
                
                isnew = true;
                
                for (unsigned int j = 0; j < exist.size(); j++) {

                    if (found[i].x >= (exist[j].x - exist[j].cm) && found[i].x <= (exist[j].x + exist[j].cm)) {
                        if (found[i].y >= (exist[j].y - exist[j].cm) && found[i].y <= (exist[j].y + exist[j].cm)) {
                    
                            isnew = false;
                            if (iObjMove == ENABLED) {
                                exist[j].x = found[i].x; 
                                exist[j].y = found[i].y; 
                            }

                            break;
                        }
                    }
                }

                // is a new object, add to existing objects
                if (isnew){ 
                    
                    IDcounter++;
                    found[i].index = IDcounter;
                    
                    exist.push_back (found[i]);
                }
            }
        }
    }
    else {
        if (!found.empty()) exist = found; 
    }

    // return size of vecExistingObjects
    return exist.size();
}

void ColourTracking::ExistentialObjects(std::vector<Object> found, std::vector<Object>& exist)
{
    bool addrm;
    unsigned int i, j;
    
    // if object has not been detected for too long, start decreasing rm_counter
    // when it reaches zero or below, the object will be deleted during cleanup
    if (!exist.empty()){
        
        if (!found.empty()){
            for (i = 0; i < exist.size(); i++){
                
                addrm = false;
                
                for (j = 0; j < found.size(); j++){
                    if (FitMargin(exist[i].x, exist[i].y, found[j].x, found[j].y, found[j].cm)){
                        exist[i].exist_counter++; // increase exist_counter (checked when drawing circles) 
                        break;
                    }
                    
                }

                if (j == found.size() && !FitMargin(exist[i].x, exist[i].y, found[j].x, found[j].y, found[j].cm))
                    addrm = true;
                    
                if (addrm) exist[i].rm_counter--;
            }
        }
        else {
            for (i = 0; i < exist.size(); i++){  // if no objects are found at all
                exist[i].rm_counter--;
            }
        }
    }
}

bool ColourTracking::FitMargin(int ax, int ay, int bx, int by, float cm)
{
//  float r = sqrt(rad); // instead of coordm, use sqrt of second objects area (so objects won't overlap, for example?)
    
    if (ax >= bx - cm && ax <= bx + cm && ay >= by - cm && ay <= by + cm){
            return true;
    }
    else return false;
}

unsigned int ColourTracking::CleanupObjects(std::vector<Object>& exist)
{   
    if (!exist.empty()) {
        // remove objects that no longer exist
        exist.erase(std::remove_if(exist.begin(), exist.end(),
                     [](const Object & o) { return o.rm_counter == 0; } ), exist.end());
        
        if (iDebugLevel == 4) {
            std::cout << ts() << " Amount: " << exist.size() << std::endl;
            for (unsigned int i = 0; i < exist.size(); i++){
                std::cout << ts() << " Ind:" << exist[i].index << " x:" << exist[i].x << " y:" << exist[i].y << " rm:" << exist[i].rm_counter << " life:" << exist[i].exist_counter << std::endl;
            }
        }
    }
    
    // return size of vecExistingObjects
    return exist.size();
}
/**********************************************************************/


/*** Functions regarding information transmission via socket | UDP ****/
void ColourTracking::SetupSocket()
{
    sockfd = socket(AF_INET, SOCK_DGRAM, COMM_PROTOCOL);
    
    bzero((char *) &server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(comm_port);
    bind(sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr));
}

void ColourTracking::WriteSendBuffer(std::vector<Object> obj, char* send)
{
    unsigned int k = 0;

    for (unsigned int i = 0; i < obj.size(); i++) {

        if (obj[i].exist_counter >= MinLife) k++; // real object amount 
    }

    bzero(send, 2048); /* flush send buffer */
    
    if (iCount != 0){
        std::string amt = std::to_string(k);
        std::string time = ts();

        strcat(send, "<time>"); /* append timestamp to sent message */
        strncat(send, time.c_str(), time.size());  
        
        strcat(send, "<nr>"); 
        strncat(send, amt.c_str(), amt.size()); /* append object amount */
        
        strcat(send, "\n");
        
        for (unsigned int i = 0; i < obj.size(); i++){
            
            if (obj[i].exist_counter >= MinLife) {

                std::string ind = std::to_string(obj[i].index); /* convert object index to string */
                std::string x = std::to_string(obj[i].x); /* convert x coord to string */
                std::string y = std::to_string(obj[i].y); /* convert y coordinate to string */
                std::string s = std::to_string((int) obj[i].area); /* convert area to string */
                
                strcat(send, "<i>");
                strncat(send, ind.c_str(), ind.size());  /* append object index */
                
                strcat(send, "<x>");
                strncat(send, x.c_str(), x.size());   /* append x coordinate of object */
                
                strcat(send, "<y>");
                strncat(send, y.c_str(), y.size());  /* append y coordinate of object */
                
                strcat(send, "<S>");
                strncat(send, s.c_str(), s.size());  /* append area value of object */
                
                strcat(send, "\n"); /* append endline for each object */
            }
        
        }
        strcat(send, "\0");
    }
    else strcpy(send, "<start>NOT_COUNTING<end>\n");
    
    if (iDebugLevel == 7){
        std::cout << ts() << " Sending:\n" << send;
        std::cout << "\n" << ts() << "send length: " << strlen(send) << std::endl;
    }
}
    
void ColourTracking::RecvSend(char* pass, char* send)
{       
    bzero(pass,64); /* flush pass buffer */
    clientlen = sizeof(client_addr);
    
    recvfrom(sockfd, pass, 64, MSG_DONTWAIT, (struct sockaddr *)&client_addr, &clientlen);

    if (!strcmp(pass,comm_pass)){

        sendto(sockfd, send, strlen(send), 0, (struct sockaddr *) &client_addr, sizeof(client_addr));
    }
} 
/**********************************************************************/

/*** Functions regarding the time measurement of various operations ***/
void ColourTracking::t_start()
{
    start_time = high_resolution_clock::now();
}

void ColourTracking::t_end()
{
    end_time = high_resolution_clock::now();
    time_dif += (duration_cast<milliseconds> (end_time - start_time).count());
//    if (iDebugLevel == 2) std::cout << "Execution time this cycle: " << (duration_cast<milliseconds> (end_time - start_time).count()) << "ms\n";
}

unsigned int ColourTracking::delay()
{
    static unsigned int k = 0; //counter
    static unsigned int result = uiDelay;
    
    if (k >= DEF_INTERVAL){
        
        if (iDebugLevel >= 2){
            std::cout << ts() << " Average duration per " << k << " cycles: " << ((int) time_dif/(k)) << "ms\n";
        }
        
        result = (int) time_dif/(k); 
                
        if (result < MIN_CYCLE_T) result = MIN_CYCLE_T; //minimum cycle time
        if (result > MAX_CYCLE_T) result = MAX_CYCLE_T; //max
        
        if (iDebugLevel >= 2){              
            std::cout << ts() << " Setting cycle time to: " << result << "ms\n";
        }
                
        k = 0;
        time_dif = 0;
    }
    
    k++;
    
    return result;
}
/**********************************************************************/


/****** OpenCV-based functions (using functionality of imgproc) *******/
void ColourTracking::ThresholdImage(cv::Mat src, cv::Mat& dst, int hsv[], bool blur)
{
    // container for HSV image
    cv::Mat buf;

    // RGB -> HSV       
    cv::cvtColor(src, buf, cv::COLOR_BGR2HSV);
    if (blur) cv::GaussianBlur(buf, buf, cv::Size(5,5), 0,0);
    
    // HSV -> binary (black&white)
    if (hsv[0] <= hsv[1]) {
        cv::inRange(buf, cv::Scalar(hsv[0], hsv[2], hsv[4]), cv::Scalar(hsv[1], hsv[3], hsv[5]), dst);
    } 
    // apply circular thresholding (e.g. Hue ranges from 130 (low red) to 22(high orange))
    else {
        cv::Mat higher, lower;
        cv::inRange(buf, cv::Scalar(hsv[0], hsv[2], hsv[4]), cv::Scalar(HHUE, hsv[3], hsv[5]), higher);
        cv::inRange(buf, cv::Scalar(LHUE, hsv[2], hsv[4]), cv::Scalar(hsv[1], hsv[3], hsv[5]), lower);
        dst = higher + lower;
    }
}    

void ColourTracking::MorphImage(unsigned int morph, int size, cv::Mat src, cv::Mat& dst)
{
    cv::Mat buf = src; /* buffer Mat on which to use erode and dilate */
    
    if (morph > 0) cv::erode(buf, buf, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size)));
    if (morph > 1) cv::dilate(buf, buf, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size)));    
    if (morph > 1) cv::dilate(buf, buf, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size)));
    if (morph > 0) cv::erode(buf, buf, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size)));
    
    dst = buf;
}    
   
void ColourTracking::DrawCircles(cv::Mat src, cv::Mat& dst, std::vector<Object> obj)
{
    dst = cv::Mat::zeros(src.size(), src.type());
    float rad;
    
    if (!obj.empty()){
        for (unsigned int i=0; i<obj.size(); i++){
             
             if (obj[i].exist_counter >= MinLife){
                 
                // circle area = pi * radius^2
                rad = sqrt(obj[i].area/PI_VALUE);
                cv::circle(dst,cv::Point(obj[i].x,obj[i].y), rad, cv::Scalar(0,0,255), 2, 8, 0);
                if (iDebugLevel > 0) cv::circle(dst,cv::Point(obj[i].x,obj[i].y), 3, cv::Scalar(0,255,0), 2, 8, 0); // draw mass center
            }
        }
    }
    else if (iDebugLevel == 6) std::cout << ts() << " No existing objects. (DrawCircles)\n";
}
/**********************************************************************/

void ColourTracking::disablegui()
{
    bGUI = false;
}

void ColourTracking::Display()
{
    if (bGUI && iShowThresh == ENABLED){
        
        imshow("Thresholded Image", imgThresh); /* show the thresholded image */
        
    }else destroyWindow("Thresholded Image");
    
    if (bGUI && iShowOriginal == ENABLED){
        
        if (imgOriginal.size() == imgCircles.size()){
            imgOriginal = imgOriginal + imgCircles;       /* add drawn circles to original */
        }
        imshow("Original", imgOriginal); /* show the original image */
        
    }else destroyWindow("Original");
}

std::string ColourTracking::ts()
{
    time_t currenttime;
    time(&currenttime);
    tm *curtime = localtime(&currenttime);
    
    std::string stamp = "["; /* start with a bracket for enclosure */
    
    if (curtime->tm_hour < 10) stamp.append("0"); /* append current hours */
    stamp.append(std::to_string(curtime->tm_hour));
    stamp.append(":");
    
    if (curtime->tm_min < 10) stamp.append("0"); /* append current minutes */
    stamp.append(std::to_string(curtime->tm_min));
    stamp.append(":");
    
    if (curtime->tm_sec < 10) stamp.append("0"); /* append current seconds */
    stamp.append(std::to_string(curtime->tm_sec));
    
    stamp.append("]"); /* finish string up with a bracket */
    
    return stamp; /* return "[HH:MM:SS]" */
}

void ColourTracking::setHSV (int user[])
{
    for (int i = 0; i < 6; i++){
        iHSV[i] = user[i];
    }
}

bool ColourTracking::CreateControlWindow()
{
    if (bGUI){
        namedWindow("Control", CV_WINDOW_NORMAL);
        
        cvCreateTrackbar("HUE min", "Control", &iHSV[0], 179);
        cvCreateTrackbar("HUE max", "Control", &iHSV[1], 179);
        cvCreateTrackbar("SAT min", "Control", &iHSV[2], 255);
        cvCreateTrackbar("SAT max", "Control", &iHSV[3], 255);
        cvCreateTrackbar("VAL min", "Control", &iHSV[4], 255);
        cvCreateTrackbar("VAL max", "Control", &iHSV[5], 255);
        cvCreateTrackbar("Original", "Control", &iShowOriginal, 1);
        cvCreateTrackbar("Thresh", "Control", &iShowThresh, 1);
        cvCreateTrackbar("Count", "Control", &iCount, 1);
        cvCreateTrackbar("Morph", "Control", &iMorphLevel, 2);
        cvCreateTrackbar("Debug", "Control", &iDebugLevel, 10);
        cvCreateTrackbar("Move", "Control", &iObjMove, 1);
    }

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
                std::cout << "-rmstart [5..50]  defines how many cycles before object is dropped\n";
                std::cout << "-drawmin [0..500] (Default is 30) Defines how many cycles an object must exist, before it is marked on the original frame.\n";
                std::cout << "-noblur   Disables blurring before thresholding the HSV image.\n";
                return -1;
            }
            else if (!std::strcmp(argv[j],"-framesize")){
                    uiCaptureHeight = std::atoi(argv[j+1]);
                    uiCaptureWidth = std::atoi(argv[j+2]);
                    ObjectMinsize = (uiCaptureHeight * uiCaptureWidth) / 100;
                    ObjectMaxsize = (uiCaptureHeight * uiCaptureWidth) / 4;
                    j += 2;
                    if (uiCaptureHeight < 64 || uiCaptureHeight > 512 || uiCaptureWidth < 64 || uiCaptureWidth > 512){
                        std::cout << "Height and width can be set between 64..512.\n";
                        return -1;
                    }
            } 
            else if (!std::strcmp(argv[j],"-nogui")){
                    disablegui();
            }
            else if (!std::strcmp(argv[j],"-noblur")){
                    bThreshBlur = DISABLED;
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
                std::cout << ts() << " Object minimum size: " << ObjectMinsize << "px2\n";
                std::cout << ts() << " Object maximum size: " << ObjectMaxsize << "px2\n"; 
                j += 2;
            }
            else if (!std::strcmp(argv[j],"-rmstart")){
                rm_default = std::atoi(argv[j+1]);
                if (rm_default > 50 || rm_default < 5){
                    std::cout << "If removal counter is below 5, objects get removed too fast. If it's too high, objects get removed too slowly. Try 5..50\n";
                    return -1;
                }
                j++;
            }
            else if (!std::strcmp(argv[j],"-drawmin")){
                MinLife = std::atoi(argv[j+1]);
                if (MinLife > 500){
                    std::cout << "Aiming a bit too high there, aren't we.. Try something more reasonable, between 10..100.\n";
                    return -1;
                }
                j++;
            }
        }
    }
    
    return 1;
}
