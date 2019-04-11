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
#include "object.h"
#include "segmentation.h"
#include "color.h"

using namespace cv;
using namespace std;
using namespace std::chrono;
//Constantes Figuras
const long double arrPhi1[4]={0.3288777,0.2407369,0.2054008,0.1659098};
const long double arrPhi2[4]={0.074311,0.0107755,0.0010336,0.0006762};
const long double stdPhi1[4]={0.004461183,0.004096511,0.001597802,0.000245853};
const long double stdPhi2[4]={0.002389544,0.001390904,0.001208792,0.000099343};

bool izqDer = false, arrAba = false, adeAtr = false;
int subBaj = 0;
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
const char KEY_UP = 'u';
const char KEY_DOWN = 'j';

const String WINDOW_ORIGINAL_NAME = "Original";
const String WINDOW_FLIPPED_NAME = "Flipped";
//Variables drone

Object objetos[100];
long double phi1Test,phi2Test,thetaTest;

// void funcion_prueba(BebopDrone &drone) {
//   cout << "funcion prueba" << endl;
//   // Despegue
//   drone.takeoff();
//   usleep(3500000);
//   cout << "takeoff" << endl;
//
//   drone.hover();
//   usleep(2000000);
//   cout << "hover" << endl;
//
//   drone.setYaw(50);
//   usleep(1000000);
//   cout << "yaw" << endl;
//
//   drone.hover();
//   usleep(2000000);
//   cout << "hover2" << endl;
//
//   drone.setPitch(100);
//   usleep(1250000);
//   cout << "pitch" << endl;
//
//   drone.hover();
// }
void funcion_prueba(BebopDrone &drone) {
  bool atras,derecha,baja = false;
  atras = adeAtr;
  derecha = izqDer;
  baja = subBaj;
  cout << "funcion prueba" << endl;
  drone.hover();
  usleep(2000000);
  cout << "hover" << endl;

  if (derecha){
  drone.setRoll(50);
  usleep(500000);
  cout << "roll" << endl;
  }else{
  drone.setRoll(-50);
  usleep(500000);
  cout << "roll" << endl;
  }

  drone.hover();
  usleep(2000000);
  cout << "hover2" << endl;

  if (atras){
  drone.setPitch(-100);
  usleep(500000);
  cout << "pitch" << endl;
  }else{
  drone.setPitch(100);
  usleep(500000);
  cout << "pitch" << endl;
  }

  drone.hover();
  usleep(2000000);
  cout << "hover2" << endl;

  if(baja){
    drone.setVerticalSpeed(-20);
    usleep(500000);
    cout << "vertical" << endl;
  }else{
    drone.setVerticalSpeed(20);
    usleep(500000);
    cout << "vertical" << endl;
  }
  drone.hover();
}

void dibujaPhis(Mat &destinationImage, Object object[]){
  Point a;
  for (int i = 0; i < 10; i++){
    phi1Test=objetos[i].getPhi1();
    phi2Test=objetos[i].getPhi2();
    a = Point(67+phi1Test*(1900),379-(phi2Test*4225));
    circle(destinationImage, a ,5, Scalar(0,128,0),-1);
  }
}

int main(int argc, char **argv)
{


  BebopDrone &drone = BebopDrone::getInstance();
  namedWindow(WINDOW_ORIGINAL_NAME);
  high_resolution_clock::time_point currentTime = high_resolution_clock::now();
  high_resolution_clock::time_point lastKeyPress = high_resolution_clock::now();
  JoystickInterface joystick(1);
  int var;

	srand(time(NULL));
  VideoCapture camera = VideoCapture(0);
  bool isCameraAvailable = camera.isOpened();

  Mat currentImage;
  Mat momentos;
  Mat YIQimage;
  Mat aux,aux1;
  Mat sepYIQ, sepImage;
  Mat YIQrest;

  namedWindow("Original");
  namedWindow("YIQ");
  //setMouseCallback("Original", mouseCoordinatesExampleCallback, &currentImage);
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
  // bool stop = false;
  // while (!stop) {
  //   currentTime = high_resolution_clock::now();
  //
  //   currentImage = drone.getFrameAsMat();
  //   imshow(WINDOW_ORIGINAL_NAME, currentImage);
  //
  //   char key = tolower(cv::waitKey(16));
  //
  //   if (key == KEY_STOP_PROGRAM) {
  //     break;
  //   }
  //   if (key != -1) {
  //     lastKeyPress = high_resolution_clock::now();
  //   }
  //
  //   if (joystick.isConnected()) {
  //     int yawOutput = DRONE_OUTPUT * joystick.getAxisValue(3);
  //     int rollOutput = DRONE_OUTPUT * joystick.getAxisValue(0);
  //     int pitchOutput = DRONE_OUTPUT * joystick.getAxisValue(1) * -1;
  //     int verticalSpeedOutput = DRONE_OUTPUT * joystick.getAxisValue(4) * -1;
  //
  //     if (((yawOutput == 0 && rollOutput == 0) && pitchOutput == 0)) {
  //       drone.hover();
  //     } else {
  //       drone.setYawRollPitchVSpeed(yawOutput, rollOutput, pitchOutput,
  //                                   verticalSpeedOutput);
  //     }
  //
  //     if (joystick.getButtonState(0)) {
  //       drone.takeoff();
  //     } else if (joystick.getButtonState(1)) {
  //       drone.land();
  //     } else if (joystick.getButtonState(3)) {
  //       drone.emergencyStop();
  //     } else if (joystick.getButtonState(2)) {
  //       funcion_prueba(drone);
  //     }
  //
  //   } else {
  //     // Keyboard control
  //     switch (key) {
  //       case KEY_TAKEOFF:
  //         drone.takeoff();
  //         break;
  //       case KEY_LAND:
  //         drone.land();
  //         break;
  //       case KEY_MOVE_FORWARD:
  //         drone.setPitch(DRONE_OUTPUT);
  //         break;
  //       case KEY_MOVE_BACK:
  //         drone.setPitch(-DRONE_OUTPUT);
  //         break;
  //       case KEY_MOVE_LEFT:
  //         drone.setRoll(-DRONE_OUTPUT);
  //         break;
  //       case KEY_MOVE_RIGHT:
  //         drone.setRoll(DRONE_OUTPUT);
  //         break;
  //       case KEY_TURN_LEFT:
  //         drone.setYaw(-DRONE_OUTPUT);
  //         break;
  //       case KEY_TURN_RIGHT:
  //         drone.setYaw(DRONE_OUTPUT);
  //         break;
  //       case KEY_EMERGENCY_STOP:
  //         drone.emergencyStop();
  //         break;
  //       case KEY_HOVER:
  //         drone.hover();
  //         break;
  //       case KEY_UP:
  //         drone.setVerticalSpeed(DRONE_OUTPUT);
  //       case KEY_DOWN:
  //         drone.setVerticalSpeed(-DRONE_OUTPUT);
  //       case KEY_SEQUENCE:
  //         funcion_prueba(drone);
  //         break;
  //       default:
  //         // If key delay has passed and no new keys have been pressed stop
  //         // drone
  //         duration<double, std::milli> time_span = currentTime - lastKeyPress;
  //         if (time_span.count() > KEY_DELAY_MS) {
  //           drone.hover();
  //         }
  //         break;
  //     }
  //   }


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
  	detectobject(binaryImage,detectionimage,objetos);
  	imshow("detection",detectionimage);
    momentos = imread("Momentos.jpg",CV_LOAD_IMAGE_COLOR);
    dibujaPhis(momentos,objetos);
    imshow("Momentos",momentos);
    for (int j = 0; j<10;j++){
      phi1Test=objetos[j].getPhi1();
      phi2Test=objetos[j].getPhi2();
      thetaTest=objetos[j].getTheta();


      if( phi1Test < arrPhi1[0]+3*stdPhi1[0] && phi1Test > arrPhi1[0]-3*stdPhi1[0] && phi2Test < arrPhi2[0]+3*stdPhi2[0] && phi2Test > arrPhi2[0]-3*stdPhi2[0]){
        //objeto1 Espada
        cout<<"Espadin Derecha"<<endl;
        izqDer = true; //Movimiento a la derecha
        if(thetaTest > 0){
          cout<<"Sube"<< thetaTest<<endl;
          subBaj = 20;
        }else
        {
          cout<<"Baja"<< thetaTest<<endl;
          subBaj = -20;
        }
      }
      else if(phi1Test < arrPhi1[1]+3*stdPhi1[1] && phi1Test > arrPhi1[1]-3*stdPhi1[1] && phi2Test < arrPhi2[1]+3*stdPhi2[1] && phi2Test > arrPhi2[1]-3*stdPhi2[1]){
        //objeto2 Casco
        cout<<"Casquin Delante"<<endl;
        adeAtr = false; //Movimiento hacia adelante
      }
      else if(phi1Test < arrPhi1[2]+3*stdPhi1[2] && phi1Test > arrPhi1[2]-3*stdPhi1[2] && phi2Test < arrPhi2[2]+3*stdPhi2[2] && phi2Test > arrPhi2[2]-3*stdPhi2[2]){
        //objeto3 Hacha
        cout<<"Hachin Izquierda"<<endl;
        izqDer = false; //Movimiento a la Izquierda
        if(thetaTest > 0){
          cout<<"Sube"<< thetaTest<<endl;
          subBaj = 20;
        }else
        {
          cout<<"Baja"<< thetaTest<<endl;
          subBaj = -20;
        }
      }
      else if(phi1Test < arrPhi1[3]+3*stdPhi1[3] && phi1Test > arrPhi1[3]-3*stdPhi1[3] && phi2Test < arrPhi2[3]+3*stdPhi2[3] && phi2Test > arrPhi2[3]-3*stdPhi2[3]){
        //objeto4 Escudo
        cout<<"Escudin Atras"<<endl;
        adeAtr = true; //Movimiento hacia atras
      }
    }

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
