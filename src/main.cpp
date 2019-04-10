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
int areas[100];
double corrimientoX,corrimientoY;
long double sumaX[100],sumaY[100];
long double sumaXcuadrada[100], sumaYcuadrada[100];
long double miu20[100], miu02[100], miu11[100];
long double centrox[100], centroy[100];
long double phi1[100],phi2[100];
long double sumaXY[100];
long double niu20[100],niu02[100],niu11[100];
long double theta[100];

bool esta[1000][1000];
void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param);
void mouseCallbackYIQ(int event, int x, int y, int flags, void* param);
void RGBtoYIQ(const Mat &sourceImage, Mat &destinationImage);
void restore(const Mat &sourceImage,const Mat &binImage, Mat &destinationImage);
void binarizeChannel(const Mat &sourceImage, int blowValue, int bhighValue,int glowValue, int ghighValue,int rlowValue, int rhighValue, Mat &destinationImage);
Mat display2(const Mat &mat_1, const Mat &mat_2);
Mat display2v(const Mat &mat_1, const Mat &mat_2);


vector<Point> points;//Se guardan los puntos donde se hace click
int RGB[3];//Se guardan los valores BGR del punto donde se da click
int YIQ[3];//Se guardan los valores YIQ del punto donde se da click
const int dsv = 35;

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
			Nodo *temp = new Nodo;
			temp->x1=value;
			temp-> y1=value2;
			temp->next=NULL;
			if(head==NULL)
			{
				head=temp;
				tail=temp;
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
	int aux=0;
	//DATOS PARA LA ACCESAR A LAS IMAGENES
	uint8_t* pixelPtr = (uint8_t*)sourceImage.data;
	int cn = sourceImage.channels();
	Scalar_<uint8_t> bgrPixel;
	uint8_t* pixelPtr2 = (uint8_t*)destinationImage.data;
	int cn2 = destinationImage.channels();
	Scalar_<uint8_t> bgrPixel2;
  struct Nodo* ptr;
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
   				ptr = obj.gethead();
					//HASTA QUE LA LISTA SE VACIE
  				while (ptr != NULL)
					{
						i = ptr-> x1;
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

  					areas[aux]++;
  					sumaX[aux] = sumaX[aux] + j;
            sumaY[aux] = sumaY[aux] + i;

  					sumaXY[aux] += j * i;
  					sumaXcuadrada[aux] = sumaXcuadrada[aux] + j * j ;
  					sumaYcuadrada[aux] = sumaYcuadrada[aux] + i * i;
					}
					obj.cleanLinkedList();

					centrox[aux] = sumaX[aux]/areas[aux];
					centroy[aux] = sumaY[aux]/areas[aux];
					miu20[aux] = sumaXcuadrada[aux] - centrox[aux] * sumaX[aux];
					miu02[aux] = sumaYcuadrada[aux] - centroy[aux] * sumaY[aux];
					miu11[aux] = sumaXY[aux] - centroy[aux] * centrox[aux] * areas[aux];

					niu20[aux] = miu20[aux]/pow(areas[aux],2);
					niu02[aux] = miu02[aux]/pow(areas[aux],2);
					niu11[aux] = miu11[aux]/pow(areas[aux],2);

					phi1[aux] = niu20[aux] + niu02[aux];
					phi2[aux] = pow(niu20[aux] - niu02[aux],2) + 4 *pow(niu11[aux],2);

         // theta[aux] = 0.5 * atan((2*(niu11[aux]/areas[aux]))/(niu20[aux]/areas[aux]-niu02[aux]/areas[aux]));

         theta[aux] = 0.5 * atan2(2 * niu11[aux], niu20[aux] - niu02[aux]);
					aux++;

					color[0] = color[0] + 10;
					color[1] = color[1] + 40;
					color[2] = color[2] + 60;
				}
			}
		}
		cout<<"numero de objetos = "<<aux<<endl;

		for(int i=0; i<aux;i++)
		{

			cout<<"objeto "<<i+1<<" = "<<areas[i]<<endl;
			cout << "X: " <<i+1 << " = "<< sumaX[i] <<endl;
			cout << "Y: " <<i+1 << " = "<< sumaY[i] <<endl;
      cout<<"x testada "<<i+1<<" = "<<sumaX[i]/areas[i]<<endl;
      cout<<"y testada "<<i+1<<" = "<<sumaY[i]/areas[i]<<endl;
			cout<<"miu20 " <<i+1 <<" = "<< miu20[i]<<endl;
			cout<<"miu02 " <<i+1 <<" = "<< miu02[i]<<endl;
			cout<<"miu11 " <<i+1 <<" = "<< miu11[i]<<endl;
			cout <<"phi1 " <<i+1 << " = " << phi1[i] <<endl;
			cout <<"phi2 " <<i+1 << " = " << phi2[i] <<endl;
			cout << "miu20 " << i+1 << " = "<< miu20[i] <<endl;
      cout << "theta " << i+1 << " = "<< theta[i] * 180 / 3.14159265 <<endl;

			Point p(centrox[i],centroy[i]);

      corrimientoX = sqrt(areas[i])*cos(theta[i]);
      corrimientoY = sqrt(areas[i])*sin(theta[i]);

      Point a(centrox[i]+corrimientoX,centroy[i]+corrimientoY);

			circle(destinationImage, p ,5, Scalar(128,0,0),-1);
      circle(destinationImage, a ,5, Scalar(0,128,0),-1);
      line(destinationImage,p,a,Scalar(128,128,128),1,1);

      areas[i] = 0;
      sumaX[i] = 0;
      sumaY[i] = 0;
			centrox[i] = 0;
			centroy[i] = 0;
			sumaXcuadrada[i] = 0;
			sumaYcuadrada[i] = 0;
			sumaXY[i] = 0;
			miu20[i] = 0;
    	miu02[i] = 0;
			miu11[i] = 0;
			phi1[i] = 0;
			phi2[i] = 0;
      theta[i] = 0;
		}
    obj.cleanLinkedList();
}
int main(int argc, char **argv)
{

  BebopDrone &drone = BebopDrone::getInstance();
  namedWindow(WINDOW_ORIGINAL_NAME);
  high_resolution_clock::time_point currentTime = high_resolution_clock::now();
  high_resolution_clock::time_point lastKeyPress = high_resolution_clock::now();

	srand(time(NULL)); 
  //VideoCapture camera = VideoCapture(0);
  //bool isCameraAvailable = camera.isOpened();

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
 
  /*
  while (true) {
    
    if (isCameraAvailable && update)
     {
      camera.read(currentImage);
     } 
    if (currentImage.size().width <= 0 && currentImage.size().height <= 0) {
      cout << "ERROR: Camera returned blank image, check connection\n";
      break;
    }
    */

   while (true) 
   {

    if (update)
    {
      currentTime = high_resolution_clock::now();
      currentImage = drone.getFrameAsMat();
      imshow(WINDOW_ORIGINAL_NAME, currentImage);

    }

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
void RGBtoYIQ(const Mat &sourceImage, Mat &destinationImage)
//Convierte de RGB a YIQ
{
  int R,G,B;
  float Y,I,Q;

  if (destinationImage.empty())
		destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type());

	for (int y = 0; y < sourceImage.rows; ++y)
		for (int x = 0; x < sourceImage.cols; ++x){
      //Extrae los componentes de cada color
      B = sourceImage.at<Vec3b>(y, x)[0];
      G = sourceImage.at<Vec3b>(y, x)[1];
      R = sourceImage.at<Vec3b>(y, x)[2];
      //Conversion a YIQ
      Y = (0.299*R + 0.587*G + 0.114*B);
      I = (0.596*R - 0.275*G - 0.321*B)+128;
      Q = (0.212*R - 0.523*G + 0.311*B)+128;
      //Guardar los valores en la nueva matriz
      destinationImage.at<Vec3b>(y, x)[0] = Y;
      destinationImage.at<Vec3b>(y, x)[1] = I;
      destinationImage.at<Vec3b>(y, x)[2] = Q;
    }
}

void binarizeChannel(const Mat &sourceImage, int blowValue, int bhighValue,int glowValue, int ghighValue,int rlowValue, int rhighValue, Mat &destinationImage)
//Binariza una imagen en 3 canales, recibiendo los valores altos y bajos para binarizar
{
  if (destinationImage.empty())
    destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type());
  for(int y = 0; y < sourceImage.rows; ++y){
    for(int x = 0; x < sourceImage.cols; ++x){
      if (sourceImage.at<Vec3b>(y,x)[0]>blowValue && sourceImage.at<Vec3b>(y,x)[0]<bhighValue && sourceImage.at<Vec3b>(y,x)[1]>glowValue && sourceImage.at<Vec3b>(y,x)[1]<ghighValue && sourceImage.at<Vec3b>(y,x)[2]>rlowValue && sourceImage.at<Vec3b>(y,x)[2]<rhighValue)
        destinationImage.at<Vec3b>(y,x) = sourceImage.at<Vec3b>(y,x);
      else{
        destinationImage.at<Vec3b>(y,x)[0] = 0;
        destinationImage.at<Vec3b>(y,x)[1] = 0;
        destinationImage.at<Vec3b>(y,x)[2] = 0;
      }
    }
  }
}


void restore(const Mat &sourceImage, const Mat &binImage, Mat &destinationImage)
//Crea una imagen con los colores originales RGB de la sección binarizada
{
  if (destinationImage.empty())
    destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type());
  for(int y = 0; y < sourceImage.rows; ++y){
    for(int x = 0; x < sourceImage.cols; ++x){
      if(binImage.at<Vec3b>(y,x)[0])
        destinationImage.at<Vec3b>(y,x) = sourceImage.at<Vec3b>(y,x);
      else{
        destinationImage.at<Vec3b>(y,x)[0] = 0;
        destinationImage.at<Vec3b>(y,x)[1] = 0;
        destinationImage.at<Vec3b>(y,x)[2] = 0;
      }
    }
  }
}
void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param)
//Regresa los valores al arreglo de RGB y los muestra en consola
{
    Mat &currentImage = *(Mat*)param;
    int R = currentImage.at<Vec3b>(y,x)[2];
    int G = currentImage.at<Vec3b>(y,x)[1];
    int B = currentImage.at<Vec3b>(y,x)[0];
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            cout << "  Mouse X, Y: " << x << ", " << y << "   ";
            cout << "R: " << R<< " G: " << G<< " B: " << B<< endl;
            /*  Draw a point */
            points.push_back(Point(x, y));
            RGB[2] = R;
            RGB[1] = G;
            RGB[0] = B;
            break;
        case CV_EVENT_RBUTTONDOWN:
            points.clear();
            break;
    }
}
void mouseCallbackYIQ(int event, int x, int y, int flags, void* param)
//Regresa los valores al arreglo de YIQ y los muestra en consola
{
  Mat &currentImage = *(Mat*)param;
  int Q = currentImage.at<Vec3b>(y,x)[2];
  int I = currentImage.at<Vec3b>(y,x)[1];
  int Y = currentImage.at<Vec3b>(y,x)[0];
  switch (event)
  {
      case CV_EVENT_LBUTTONDOWN:
          cout << "  Mouse X, Y: " << x << ", " << y << "   ";
          cout << "Y: " << Y<< " I: " << I<< " Q: " << Q<< endl;
          YIQ[0] = Y;
          YIQ[1] = I;
          YIQ[2] = Q;
          break;
  }
}

/*
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
*/

Mat display2(const Mat &mat_1, const Mat &mat_2)
//Une dos imagenes del mismo tamaño horizontalmente
{
  Mat win_mat(mat_1.rows,mat_1.cols * 2,mat_1.type());

  mat_1.copyTo(win_mat(Rect(  0, 0, mat_1.cols, mat_1.rows)));
  mat_2.copyTo(win_mat(Rect(mat_1.cols, 0, mat_1.cols, mat_1.rows)));

  return win_mat;
}

Mat display2v(const Mat &mat_1, const Mat &mat_2)
//Une dos imagenes del mismo tamaño verticalmente
{
  Mat win_mat(mat_1.rows * 2,mat_1.cols,mat_1.type());

  mat_1.copyTo(win_mat(Rect(  0, 0, mat_1.cols, mat_1.rows)));
  mat_2.copyTo(win_mat(Rect(0, mat_1.rows, mat_1.cols, mat_1.rows)));

  return win_mat;
}
