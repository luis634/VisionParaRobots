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
      Point b(centrox[i]-corrimientoX,centroy[i]-corrimientoY);
			circle(destinationImage, p ,5, Scalar(128,0,0),-1);
      circle(destinationImage, a ,5, Scalar(0,128,0),-1);
      circle(destinationImage, b ,5, Scalar(0,0,128),-1);
      line(destinationImage,p,a,Scalar(128,128,128),1,1);
      line(destinationImage,p,b,Scalar(128,128,128),1,1);

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
