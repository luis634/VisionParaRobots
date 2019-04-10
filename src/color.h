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
