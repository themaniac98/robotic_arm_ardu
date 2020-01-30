//#include "Arduino.h"
//#include "AX12A.h"
//using namespace std; 
//#define DirectionPin   (10u)
//// speed about 0.111rpm.
//#define BaudRate      (1000000ul)
//// joints
//#define c1 (18u)
//#define c2 (14u)
//#define c3 (1u)
//// move in range [0,1024] where 1 unit = 0.293 
//#define maxPos (1000u)
//#define minPos (20u)
//#define servoUnit (0.293)
//#define motorSpeedFast (300)
//#define motorSpeedSlow (50)
//
//
//// MGD
//double l1 = 9.4, l2=14.9; // cm
//double T01[4][4], T02[4][4], T0E[4][4];
//double pozC1[2], pozC2[2],pozE[2],thetaE;
//// animatie linie dreapta
//double q1init, q2init, q1fin,q2fin,q3;
//double qN ;//nr of intermediate angular positions
//bool toolIsUp = true;
//
//
//// we want 0 degree to have a horizontal line 
//// so we can set servo in degree from 0 to 240
//// if we want the arm to be between 30 and 90 degree in the system when 0 degree is vertical
//// we have to put -0 to -60  
//int getPosFromDegreeFirst(int degree) {
//  // if 0 degree have a vertical line we have +30 degree of invalid range
//  // if we want our 0 degree to start from a horizontal line
//  // we have to add 90-30=60 degree
//   int pos = round((degree + 60 )/servoUnit);
////  if (pos <= minPos){
////    return minPos;
////   } 
////   if (pos >= maxPos){
////     return maxPos;
////    }
//      return pos;
//}
//
//int getPosFromDegreeSecond(int degree, int degree1) {
//  // if 0 degree have a vertical line we have +30 degree of invalid range
//  // if we want our 0 degree to start from a horizontal line
//  // we have to add 90-30=60 degree
//   int pos = round((degree + 60 + 90 - degree1)/servoUnit);
////  if (pos <= minPos){
////    return minPos;
////   } 
////   if (pos >= maxPos){
////     return maxPos;
////    }
////      return pos + q1;
//
//
//      return pos;
//}
//
//int getDegreeFromPosFirst(int pos) {
//  // we need to subtract the +60 degree added to have horizontal line 0 degree
//  int correction = 60;
// return round(pos * servoUnit - correction);
//}
//
//int getDegreeFromPosSecond(int pos, int degree1) {
//  // we need to subtract the +60 degree added to have horizontal line 0 degree
//  int correction = 60 + 90 - degree1;
// return round(pos * servoUnit - correction );
//}
//
//double getRadiansFromDegree(double degree) {
//  return degree*PI/180;
//}
//double getDegreeFromRadians(double rad){
//  return rad*180/PI;
//  }
//void toolDown() {
//  toolIsUp = false;
//  ax12a.move(c3,getPosFromDegreeFirst(91));
//  }
//
//void toolUp() {
//  toolIsUp = true;
//  ax12a.move(c3,getPosFromDegreeFirst(60));
//  }
//  
//void resetInitPos() {
//  toolUp();
//  delay(1000); 
//  ax12a.moveSpeed(c1,getPosFromDegreeFirst(0),motorSpeedFast);
//  q1init = 0;
//  delay(1000);
//  ax12a.moveSpeed(c2,getPosFromDegreeSecond(0,q1init),motorSpeedFast);
//  q2init = 0;
//  delay(1000);
//}
//
//
////int getQ2MinPos() {
////  return getPosFromDegreeSecond(-45);
////}
////
////int getQ2MaxPos(){
////  return getPosFromDegreeSecond(225); 
////}
////
////int getQ1MinPos() {
////  return getPosFromDegreeFirst(-50);
////}
////
////int getQ1MaxPos() {
////  return getPosFromDegreeFirst(220);
////}
//// sequence of angles we have to calculate inverse kinematics
//double** getIntermediateRotations(double q1final, double q2final, int nrSubValori){
//  double** rots = new double* [2];
//for (int i = 0; i < 2; ++i)
//{
//   rots[i] = new double[nrSubValori+1];
//}
//  rots[0][0]=q1init;
//  rots[1][0]=q2init;
//    for (int j=1 ; j<=nrSubValori; j++) {
//          rots[0][j-1] += j * (q1final - q1init)/nrSubValori;
//          rots[1][j-1] += j* (q2final - q1init)/nrSubValori;   
//      }
//      return rots;
//} 
//void calculateMatrixesInFrame0(double theT12[4][4], double theT2E[4][4]) {
//   int i, j, k; 
//   // calculate T02
//    for (i = 0; i < 4; i++) 
//    { 
//        for (j = 0; j < 4; j++) 
//        { 
//            T02[i][j] = 0; 
//            for (k = 0; k < 4; k++) 
//                T02[i][j] += T01[i][k] *  
//                             theT12[k][j]; 
//        } 
//    } 
//  // calculate T0E
//      for (i = 0; i < 4; i++) 
//    { 
//        for (j = 0; j < 4; j++) 
//        { 
//            T0E[i][j] = 0; 
//            for (k = 0; k < 4; k++) 
//                T0E[i][j] += T02[i][k] *  
//                             theT2E[k][j]; 
//        } 
//    } 
//  }   
//void calculateGMD(double q1, double q2)
//{
//  // convert degree to radians
//  double angle1 = getRadiansFromDegree(q1);
//  // we need to get the angle that starts from first q1
//  double angle2 = getRadiansFromDegree(q2-q1);
////  if (q2 > q1) {
////  angle2 = getRadiansFromDegree(q2-q1);
////    } else if (q2<=q1) {
////      angle2 = getRadiansFromDegree(q1-q2);
////      }
//  // construct homogenius matrixes
// for(int i=0;i<4;i++){
//  for(int j=0;j<4;j++){
//    if (i==0 && j==0) {
//      T01[i][j]=cos(angle1);
//      } else if (i==0 && j==1){
//         T01[i][j]=-sin(angle1);
//        } else if (i==1 && j==0) {
//           T01[i][j]=sin(angle1);
//          } else if (i==1 && j==1){
//             T01[i][j]=cos(angle1);
//            }else if (i==2 && j==2) {
//               T01[i][j]=1.0;
//              } else if (i==3 && j==3){
//                 T01[i][j]=1.0;
//                 } else {
//                   T01[i][j]=0.0;
//                  }
//    }
//  }
//// T01[4][4] = { {cos(angle1),-sin(angle1),0.0,0.0},
////          {sin(angle1),cos(angle1),0.0,0.0},
////          {0.0,0.0,1.0,0.0},
////          {0.0,0.0,0.0,1.0}
////        };
// 
//  double T12[4][4] = {  {cos(angle2),-sin(angle2),0.0,l1},
//                        {sin(angle2),cos(angle2),0.0,0.0},
//                        {0.0,0.0,1.0,0.0},
//                        {0.0,0.0,0.0,1.0}
//                     };
//  double T2E[4][4] = {  {1.0,0.0,0.0,l2},
//                        {0.0,1.0,0.0,0.0},
//                        {0.0,0.0,1.0,0.0},
//                        {0.0,0.0,0.0,1.0}
//                     };
//   calculateMatrixesInFrame0(T12,T2E);
//  
//  }
//  double *getC1Coord(){
//    return new double[2] {T01[0][3],T01[1][3]} ;
//    }
//      double *getC2Coord(){
//    return new double[2] {T02[0][3],T02[1][3]} ;
//    }
//      double *getECoord(){
//    return new double[2] {T0E[0][3],T0E[1][3]} ;
//    }
//
//void calculateInverseKinematics(double x, double y) {
//    double L = sqrt(x*x+y*y);
//  q1fin = atan(x/y) - acos((L*L-l1*l1-l2*l2)/(-2*l1*L));
//  q2fin = acos((L*L-l1*l1-l2*l2)/(2*l1*l2)); 
//
//  q1fin= getDegreeFromRadians(q1fin);
//  q2fin= getDegreeFromRadians(q2fin)+q1fin;
//  
////  if (q1fin < 0) {
////    q1fin = round(360 +q1fin);
////    }
////    if (q2fin < 0) {
////    q2fin =round( 360 + q2fin);
////    }
//      }
//
//
//void goToPositionFast (double x, double y){
//  calculateInverseKinematics(x,y);
//  delay(100);
//  ax12a.moveSpeed(c1,getPosFromDegreeFirst(round(q1fin)),motorSpeedFast);
//  delay(100);
//  ax12a.moveSpeed(c2,getPosFromDegreeSecond(round(q2fin),round(q1fin)),motorSpeedFast);
//}
//
//void goToPositionSlow (double x, double y){
//  calculateInverseKinematics(x,y);
//  delay(100);
//  ax12a.moveSpeed(c1,getPosFromDegreeFirst(round(q1fin)),motorSpeedSlow);
//  delay(100);
//  ax12a.moveSpeed(c2,getPosFromDegreeSecond(round(q2fin),round(q1fin)),motorSpeedSlow);
//}
//
//
//void drawLineBetween (double x1,double y1,double x2, double y2) {
//  double xi, yi, a, b, L, d;
//  a = x2 - x1;
//  b = y2 - y1;
//  L = sqrt(a*a + b*b);
//  d = L/20;
//
//  toolUp();
//  delay(100);
//  goToPositionFast(x1,y1);
//  delay(500);
//  toolDown();
//  
//  for (int i = 0; i < 10; i++){
//    xi = x1 + d/L*a;
//    yi = y1 + d/L*b;
//    goToPositionSlow(xi,yi);
//  }
//  
//  goToPositionSlow(x2,y2);
//}
//
//void continueLineBetween (double x1,double y1,double x2, double y2) {
//  double xi, yi, a, b, L, d;
//  a = x2 - x1;
//  b = y2 - y1;
//  L = sqrt(a*a + b*b);
//  d = L/20;
//  
//  goToPositionSlow(x1,y1);
//  for (int i = 0; i < 10; i++){
//    xi = x1 + d/L*a;
//    yi = y1 + d/L*b;
//    goToPositionSlow(xi,yi);
//  }
//  goToPositionSlow(x2,y2);
//}
//      
//void setup() {
//  // put your setup code here, to run once:
//  Serial.begin(1000000);
//  ax12a.begin(BaudRate, DirectionPin, &Serial);
//  delay(1000); 
////  resetInitPos();
////  delay(1000);
////  drawLineBetween(13, 9, 16, 7);
//
//
////    delay(1000);
////    toolUp();
////    delay(1000);
////    goToPositionFast(15,18);
////    delay(1000);
////    toolDown();
////    delay(1000);
//
//
////  int pos1 = ax12a.readPosition(c1);
////  int pos2 = ax12a.readPosition(c2);
////  double deg1 = getDegreeFromPosFirst(pos1);
////  double deg2 = getDegreeFromPosSecond(pos2,deg1);
////  calculateGMD(deg1,deg2);
////  double *c1Coord = getC1Coord();
////  double *c2Coord = getC2Coord();
////  double *ePosCoord = getECoord();
////  Serial.println("/n");
////  Serial.println("Manipulator:");
////   Serial.println(ePosCoord[0]);
////    Serial.println(ePosCoord[1]);
////  Serial.println("Articulatie 1:");
////   Serial.println(c1Coord[0]);
////    Serial.println(c1Coord[1]);
////  Serial.println("Articulatie 2:");
////   Serial.println(c2Coord[0]);
////    Serial.println(c2Coord[1]);
//
////  goToPosition(ePosCoord[0],ePosCoord[1]);
//
//
//}
//
//void loop() {
//  delay(2000);
//  drawLineBetween(13, 9, 16, 7);
//  continueLineBetween(16, 7, 15, 9);
//  continueLineBetween(15, 9, 12, 10);
//  
////  continueLineBetween(12, 10, 13, 9);
//  
//  
//
//
//
//  
//
//}
