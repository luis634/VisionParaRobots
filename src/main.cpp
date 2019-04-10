#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <thread>
#include "BebopController/BebopDrone.hpp"
#include "Joystick/JoystickInterface.hpp"
#include <chrono>
#include <ctime>
#include <iostream>
#include <string.h>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "segmentation.h"
#include "color.h"
using namespace cv;
using namespace std;
using namespace std::chrono;

//Variables drone
const int KEY_DELAY_MS = 50;
const int DRONE_OUTPUT = 70;

const char KEY_STOP_PROGRAM = 'x';
const char KEY_TAKEOFF = 'o';
const char KEY_LAND = 'p';
const char KEY_MOVE_FORWARD = 'w';
const char KEY_MOVE_BACK = 's';
const char KEY_MOVE_LEFT = 'a';
const char KEY_MOVE_RIGHT = 'd';
const char KEY_TURN_LEFT = 'q';
const char KEY_TURN_RIGHT = 'e';
const char KEY_EMERGENCY_STOP = 'f';
const char KEY_HOVER = 'r';
const char KEY_SEQUENCE = 'k';

const String WINDOW_ORIGINAL_NAME = "Original";
const String WINDOW_FLIPPED_NAME = "Flipped";
//Variables drone

int main(int argc, char **argv)
{

  BebopDrone &drone = BebopDrone::getInstance();
  namedWindow(WINDOW_ORIGINAL_NAME);
  high_resolution_clock::time_point currentTime = high_resolution_clock::now();
  high_resolution_clock::time_point lastKeyPress = high_resolution_clock::now();

	srand(time(NULL));
  VideoCapture camera = VideoCapture(0);
  bool isCameraAvailable = camera.isOpened();

  Mat currentImage;
  Mat YIQimage;
  Mat aux,aux1;
  Mat sepYIQ, sepImage;
  Mat YIQrest;

  cout << "\033[2J\033[1;1H";
  cout << "Basic Show Image \t|\tUse 'x' or 'Esc' to terminate execution\nUse space to freeze\n";

  namedWindow("Original");
  namedWindow("YIQ");

  setMouseCallback("Original", mouseCoordinatesExampleCallback, &currentImage);
  setMouseCallback("YIQ",mouseCallbackYIQ, &YIQimage);

  bool update = true; //enable camera update


  while (true) {

    if (isCameraAvailable && update)
     {
      camera.read(currentImage);
     }
    if (currentImage.size().width <= 0 && currentImage.size().height <= 0) {
      cout << "ERROR: Camera returned blank image, check connection\n";
      break;
    }


   // while (true)
   // {
   //
   //  if (update)
   //  {
   //    currentTime = high_resolution_clock::now();
   //    currentImage = drone.getFrameAsMat();
   //    imshow(WINDOW_ORIGINAL_NAME, currentImage);
   //
   //  }

    // Clear the console
    //cout << "Battery Level: " << drone.getBatteryLevel() << endl;

    RGBtoYIQ(currentImage,YIQimage);//Transforma la imagén a YIQ
    binarizeChannel(YIQimage,YIQ[0]-dsv,YIQ[0]+dsv,YIQ[1]-dsv,YIQ[1]+dsv,YIQ[2]-dsv,YIQ[2]+dsv,sepYIQ);//Binarisa sobre los valores en los que se dio click YIQ
    imshow("Original", currentImage);
    imshow("YIQ",YIQimage);

		for(int i=0; i<1000;i++)
	  {
  		for(int j=0; j<1000;j++)
  		{
  			esta[i][j]= true;
  		}
	  }

  	Mat binaryImage;
  	Mat auximage;
  	cvtColor( sepYIQ, auximage, CV_BGR2GRAY);
  	for ( int i = 1; i < 13; i = i + 4 ){ medianBlur ( auximage, auximage, i );}
  	threshold( auximage, binaryImage, 120, 255,THRESH_BINARY );
  	Mat detectionimage = Mat(binaryImage.rows,binaryImage.cols,currentImage.type());
  	detectionimage.setTo(Scalar(0,0,0));
  	detectobject(binaryImage,detectionimage);
  	imshow("detection",detectionimage);


    char key = waitKey(3);
    if(key == 'x' || key == 27 )
    { // 27 = ESC
      break;
    } else if(key == ' ')
    { // if 32=Space is pressed, freeze image
      update = !update;
    }
  }
}
