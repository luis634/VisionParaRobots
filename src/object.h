class Object {
   private:
   long double phi1, phi2, theta;
  public:
    Object(){
      this->phi1=0;
      this->phi2=0;
      this->theta=0;
    }
   void setPhi1(long double phi1){
     this -> phi1 = phi1;
   }
   void setPhi2(long double phi2){
     this -> phi2 = phi2;
   }
   void setTheta(long double theta){
     this -> theta = theta;
   }
   long double getPhi1(){
     return this->phi1;
   }
   long double getPhi2(){
     return this->phi2;
   }
   long double getTheta(){
     return this->theta;
   }
};
