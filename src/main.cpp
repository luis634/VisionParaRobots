#include <chrono>
#include <ctime>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <thread>
#include <cmath>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "BebopController/BebopDrone.hpp"
#include "Joystick/JoystickInterface.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;

//Variables Parrot
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

//Variables gota de aceite

const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;
int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;
int thresh = 100;
int max_thresh = 255;
int areas[1000];
bool esta[1000][1000];

//#############################NOTAAAAAAAAAAAAAAA
//##############################NOTAAAAAAAAAAAAAAAAAA
// DEBE SER "Y" LUEGO "X" PARA PINTAR

////////////#########################LINKED LIST##########################
//###################LINKED LIST#############################
struct Nodo {
   int x1;
   int y1;
   Nodo *next;
};

class list
{
		private:
		Nodo *head, *tail;
		public:
		list()
		{
			head=NULL;
			tail=NULL;
		}
		Nodo* gethead()
		{
			return head;
		}
		void insert(int value,int value2)
		{
			Nodo *temp=new Nodo;
			temp->x1=value;
			temp-> y1=value2;
			temp->next=NULL;
			if(head==NULL)
			{
				head=temp;
				tail=temp;
				temp=NULL;
			}
			else
			{
				tail->next=temp;
				tail=temp;
			}
		}
    void cleanLinkedList()
		{
      while(head != NULL){
        Nodo *temp = head;
        head = head -> next;
        delete temp;
      }
		}
};
//////////////////###############END LINKED LIST#####################
list obj;
void detectobject(Mat &sourceImage,Mat &destinationImage)
{
	int i,j;
	int color[3]={30,30,30};
	int aux4=0;
	//DATOS PARA LA ACCESAR A LAS IMAGENES
	uint8_t* pixelPtr = (uint8_t*)sourceImage.data;
	int cn = sourceImage.channels();
	Scalar_<uint8_t> bgrPixel;
	uint8_t* pixelPtr2 = (uint8_t*)destinationImage.data;
	int cn2 = destinationImage.channels();
	Scalar_<uint8_t> bgrPixel2;

	//RECORRES LA IMAGEN
		for (int x = 1; x<sourceImage.rows;x=x+100)
		{
			for(int y = 1; y<sourceImage.cols;y=y+100)
			{
				//Input image PARA QUE ME DE EL VALOR DEL PIXEL
				bgrPixel.val[0] = pixelPtr[x*sourceImage.cols*cn + y*cn + 0];
				bgrPixel.val[1] = pixelPtr[x*sourceImage.cols*cn + y*cn + 1];
				bgrPixel.val[2] = pixelPtr[x*sourceImage.cols*cn + y*cn + 2];
				//OutputImage
				bgrPixel2.val[0] = pixelPtr2[x*destinationImage.cols*cn2 + y*cn2 + 0];
				bgrPixel2.val[1] = pixelPtr2[x*destinationImage.cols*cn2 + y*cn2 + 1];
				bgrPixel2.val[2] = pixelPtr2[x*destinationImage.cols*cn2 + y*cn2 + 2];

				if (bgrPixel[0] == 255 && bgrPixel[1] == 255  && bgrPixel[2] == 255  && bgrPixel2.val[0] == 0 && bgrPixel2.val[1] == 0 && bgrPixel2.val[2] == 0)
				{
					//PONES EN FALSO PARA QUE NO ENTRE SI YA ESTA PINTADO EL INDEX
					esta[x][y]=false;
					obj.insert(x,y);
					struct Nodo* ptr;
   					ptr = obj.gethead();
					//HASTA QUE LA LISTA SE VACIE
  					while (ptr != NULL)
					{
						i= ptr-> x1;
						j = ptr-> y1;
						destinationImage.at<Vec3b>(Point(j, i))[0] = color[0];
						destinationImage.at<Vec3b>(Point(j, i))[1] = color[1];
						destinationImage.at<Vec3b>(Point(j, i))[2] = color[2];
						if(i>0 && j>0 && i<(sourceImage.rows-1)&& j<(sourceImage.cols-1)){
						//Input image i-1
						bgrPixel.val[0] = pixelPtr[(i-1)*sourceImage.cols*cn + j*cn + 0];
						bgrPixel.val[1] = pixelPtr[(i-1)*sourceImage.cols*cn + j*cn + 1];
						bgrPixel.val[2] = pixelPtr[(i-1)*sourceImage.cols*cn + j*cn + 2];
						//OutputImage
						bgrPixel2.val[0] = pixelPtr2[(i-1)*destinationImage.cols*cn2 + j*cn2 + 0];
						bgrPixel2.val[1] = pixelPtr2[(i-1)*destinationImage.cols*cn2 + j*cn2 + 1];
						bgrPixel2.val[2] = pixelPtr2[(i-1)*destinationImage.cols*cn2 + j*cn2 + 2];
						if (bgrPixel[0] == 255 && bgrPixel[1] == 255  && bgrPixel[2] == 255  && bgrPixel2.val[0] == 0 && bgrPixel2.val[1] == 0 && bgrPixel2.val[2] == 0 && esta[i-1][j])
						{
							obj.insert(i-1,j);
							esta[i-1][j] = false;
						}
						//Input image i-1
						bgrPixel.val[0] = pixelPtr[i*sourceImage.cols*cn + (j+1)*cn + 0];
						bgrPixel.val[1] = pixelPtr[i*sourceImage.cols*cn + (j+1)*cn + 1];
						bgrPixel.val[2] = pixelPtr[i*sourceImage.cols*cn + (j+1)*cn + 2];
						//OutputImage
						bgrPixel2.val[0] = pixelPtr2[i*destinationImage.cols*cn2 + (j+1)*cn2 + 0];
						bgrPixel2.val[1] = pixelPtr2[i*destinationImage.cols*cn2 + (j+1)*cn2 + 1];
						bgrPixel2.val[2] = pixelPtr2[i*destinationImage.cols*cn2 + (j+1)*cn2 + 2];
						if (bgrPixel[0] == 255 && bgrPixel[1] == 255  && bgrPixel[2] == 255  && bgrPixel2.val[0] == 0 && bgrPixel2.val[1] == 0 && bgrPixel2.val[2] == 0 && esta[i][j+1])
						{
							obj.insert(i,j+1);
							esta[i][j+1] = false;
						}
						//Input image i-1
						bgrPixel.val[0] = pixelPtr[(i+1)*sourceImage.cols*cn + j*cn + 0];
						bgrPixel.val[1] = pixelPtr[(i+1)*sourceImage.cols*cn + j*cn + 1];
						bgrPixel.val[2] = pixelPtr[(i+1)*sourceImage.cols*cn + j*cn + 2];
						//OutputImage
						bgrPixel2.val[0] = pixelPtr2[(i+1)*destinationImage.cols*cn2 + j*cn2 + 0];
						bgrPixel2.val[1] = pixelPtr2[(i+1)*destinationImage.cols*cn2 + j*cn2 + 1];
						bgrPixel2.val[2] = pixelPtr2[(i+1)*destinationImage.cols*cn2 + j*cn2 + 2];
						if (bgrPixel[0] == 255 && bgrPixel[1] == 255  && bgrPixel[2] == 255  && bgrPixel2.val[0] == 0 && bgrPixel2.val[1] == 0 && bgrPixel2.val[2] == 0 && esta[i+1][j])
						{
							obj.insert(i+1,j);
							esta[i+1][j] = false;
						}
						bgrPixel.val[0] = pixelPtr[i*sourceImage.cols*cn + (j-1)*cn + 0];
						bgrPixel.val[1] = pixelPtr[i*sourceImage.cols*cn + (j-1)*cn + 1];
						bgrPixel.val[2] = pixelPtr[i*sourceImage.cols*cn + (j-1)*cn + 2];
						//OutputImage
						bgrPixel2.val[0] = pixelPtr2[i*destinationImage.cols*cn2 + (j-1)*cn2 + 0];
						bgrPixel2.val[1] = pixelPtr2[i*destinationImage.cols*cn2 + (j-1)*cn2 + 1];
						bgrPixel2.val[2] = pixelPtr2[i*destinationImage.cols*cn2 + (j-1)*cn2 + 2];
                       if (bgrPixel[0] == 255 && bgrPixel[1] == 255  && bgrPixel[2] == 255  && bgrPixel2.val[0] == 0 && bgrPixel2.val[1] == 0 && bgrPixel2.val[2] == 0 && esta[i][j-1])
						{
							obj.insert(i,j-1);
							esta[i][j-1] = false;
						}

						}
						ptr = ptr->next;
						areas[aux4]++;

					}

					aux4++;
					color[0] = color[0] + 40;
					color[1] = color[1] + 40;
					color[2] = color[2] + 40;
				}
			}
		}
		cout<<"numero de objetos = "<<aux4<<endl;
		for(int i=0; i<aux4;i++)
		{
			cout<<"objeto "<<i+1<<" = "<<areas[i]<<endl;
      areas[i] = 0;
		}
    obj.cleanLinkedList();
}

void flipImageBasic(const Mat &sourceImage, Mat &destinationImage);

void funcion_prueba(BebopDrone &drone) {
  cout << "funcion prueba" << endl;
  // Despegue
  drone.takeoff();
  usleep(3500000);
  cout << "takeoff" << endl;

  drone.hover();
  usleep(2000000);
  cout << "hover" << endl;

  drone.setYaw(50);
  usleep(1000000);
  cout << "yaw" << endl;

  drone.hover();
  usleep(2000000);
  cout << "hover2" << endl;

  drone.setPitch(100);
  usleep(1750000);
  cout << "pitch" << endl;

  drone.hover();
}

int main() {
  BebopDrone &drone = BebopDrone::getInstance();
  namedWindow(WINDOW_ORIGINAL_NAME);

  high_resolution_clock::time_point currentTime = high_resolution_clock::now();
  high_resolution_clock::time_point lastKeyPress = high_resolution_clock::now();
  Mat currentImage, flippedImage;

  JoystickInterface joystick(1);
	int var;

  bool stop = false;
  while (!stop) {
    currentTime = high_resolution_clock::now();

    currentImage = drone.getFrameAsMat();
    imshow(WINDOW_ORIGINAL_NAME, currentImage);

    // Clear the console
    cout << ("\033[2J\033[1;1H");

    cout << "===================== Parrot Basic Example "
            "=====================\n\n";
    cout << "Takeoff with '" << KEY_TAKEOFF << "', land with '" << KEY_LAND
         << "', move with '" << KEY_MOVE_FORWARD << "','" << KEY_MOVE_LEFT
         << "','" << KEY_MOVE_BACK << "','" << KEY_MOVE_RIGHT
         << "' and "
            "' "
         << KEY_STOP_PROGRAM << " ' to end the program" << endl;
    cout << "Battery Level: " << drone.getBatteryLevel() << endl;

    char key = tolower(cv::waitKey(16));

    if (key == KEY_STOP_PROGRAM) {
      break;
    }
    if (key != -1) {
      lastKeyPress = high_resolution_clock::now();
    }

    if (joystick.isConnected()) {
      int yawOutput = DRONE_OUTPUT * joystick.getAxisValue(3);
      int rollOutput = DRONE_OUTPUT * joystick.getAxisValue(0);
      int pitchOutput = DRONE_OUTPUT * joystick.getAxisValue(1) * -1;
      int verticalSpeedOutput = DRONE_OUTPUT * joystick.getAxisValue(4) * -1;

      if (((yawOutput == 0 && rollOutput == 0) && pitchOutput == 0)) {
        drone.hover();
      } else {
        drone.setYawRollPitchVSpeed(yawOutput, rollOutput, pitchOutput,
                                    verticalSpeedOutput);
      }

      if (joystick.getButtonState(0)) {
        drone.takeoff();
      } else if (joystick.getButtonState(1)) {
        drone.land();
      } else if (joystick.getButtonState(3)) {
        drone.emergencyStop();
      } else if (joystick.getButtonState(2)) {
        funcion_prueba(drone);
      }

    } else {
      // Keyboard control
      switch (key) {
        case KEY_TAKEOFF:
          drone.takeoff();
          break;
        case KEY_LAND:
          drone.land();
          break;
        case KEY_MOVE_FORWARD:
          drone.setPitch(DRONE_OUTPUT);
          break;
        case KEY_MOVE_BACK:
          drone.setPitch(-DRONE_OUTPUT);
          break;
        case KEY_MOVE_LEFT:
          drone.setRoll(-DRONE_OUTPUT);
          break;
        case KEY_MOVE_RIGHT:
          drone.setRoll(DRONE_OUTPUT);
          break;
        case KEY_TURN_LEFT:
          drone.setYaw(-DRONE_OUTPUT);
          break;
        case KEY_TURN_RIGHT:
          drone.setYaw(DRONE_OUTPUT);
          break;
        case KEY_EMERGENCY_STOP:
          drone.emergencyStop();
          break;
        case KEY_HOVER:
          drone.hover();
          break;
        case KEY_SEQUENCE:
          funcion_prueba(drone);
          break;
        default:
          // If key delay has passed and no new keys have been pressed stop
          // drone
          duration<double, std::milli> time_span = currentTime - lastKeyPress;
          if (time_span.count() > KEY_DELAY_MS) {
            drone.hover();
          }
          break;
      }
    }
    ///INICIALIZA MATRIZ PARA SABER SI YA ESTA PINTANDO EL OBJETO
  	for(int i=0; i<1000;i++)
  	{
  		for(int j=0; j<1000;j++)
  		{
  			esta[i][j]= true;
  		}
  	}
    Mat binaryImage;
    Mat auximage;
    cvtColor( currentImage, auximage, CV_BGR2GRAY );
    for ( int i = 1; i < 25; i = i + 8 ){ medianBlur ( auximage, auximage, i );}

    threshold( auximage, binaryImage, 120, 255,THRESH_BINARY );
    Mat detectionimage = Mat(binaryImage.rows,binaryImage.cols,currentImage.type());
    detectionimage.setTo(Scalar(0,0,0));

    detectobject(binaryImage,detectionimage);

      // Display images
    imshow("original",auximage);
    imshow("binaryimage",binaryImage);
    imshow("detection",detectionimage);
  }

  return EXIT_SUCCESS;
}
