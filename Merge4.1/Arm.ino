#include "Arduino.h"
#include "AX12A.h"
using namespace std;
#define DirectionPin   (10u)
// speed about 0.111rpm.
#define BaudRate      (1000000ul)
// joints
#define c1 (18u)
#define c2 (14u)
#define c3 (1u)
// move in range [0,1024] where 1 unit = 0.293
#define maxPos (1000u)
#define minPos (20u)
#define servoUnit (0.293)
#define motorSpeedFast (300)
#define motorSpeedSlow (50)

// MGD
double l1 = 9.4, l2 = 15.5; // cm
double T01[4][4], T12[4][4], T02[4][4];
/// MC
double** jacob;

//// we want 0 degree to have a horizontal line
//// so we can set servo in degree from 0 to 240
//// if we want the arm to be between 30 and 90 degree in the system when 0 degree is vertical
//// we have to put -0 to -60
int getPosFromDegreeFirst(int degree) {
  // if 0 degree have a vertical line we have +30 degree of invalid range
  // if we want our 0 degree to start from a horizontal line
  // we have to add 90-30=60 degree
  int pos = round((degree + 60 ) / servoUnit);
  return pos;
}

int getPosFromDegreeSecond(int degree) {
  // if 0 degree have a vertical line we have +30 degree of invalid range
  // if we want our 0 degree to start from a horizontal line
  // we have to add 90-30=60 degree
  int pos = round((degree + 60 + 90) / servoUnit);
  return pos;
}

int getDegreeFromPosFirst(int pos) {
  // we need to subtract the +60 degree added to have horizontal line 0 degree
  int correction = 60;
  return round(pos * servoUnit - correction);
}

int getDegreeFromPosSecond(int pos) {
  // q2 0 degree is continuous to q1
  int correction = 60 + 90;
  return round(pos * servoUnit - correction );
}
double getRadiansFromDegree(double degree) {
  return degree * PI / 180;
}
double getDegreeFromRadians(double rad) {
  return rad * 180 / PI;
}
void toolDown() {
  ax12a.move(c3, getPosFromDegreeFirst(91));
  delay(1000);
}

void toolUp() {
  ax12a.move(c3, getPosFromDegreeFirst(60));
  delay(1000);
}

void resetInitPos() {
  toolUp();
  delay(1000);
  ax12a.moveSpeed(c1, getPosFromDegreeFirst(0), motorSpeedFast);
  delay(1000);
  ax12a.moveSpeed(c2, getPosFromDegreeSecond(0), motorSpeedFast);
  delay(1000);
}
void calculateGMD(double q1, double q2, int isRadians)
{ double angle1, angle2;
  if (isRadians != 1 ) {
    // convert degree to radians
    angle1 = getRadiansFromDegree(q1);
    angle2 = getRadiansFromDegree(q2);
  } else {
    angle1 = q1;
    angle2 = q2;
  }


  // construct homogenius matrixes
  T01[0][0] = cos(angle1);
  T01[0][1] = -sin(angle1);
  T01[0][2] = 0.0;
  T01[0][3] = l1 * cos(angle1);
  T01[1][0] = sin(angle1);
  T01[1][1] = cos(angle1);
  T01[1][2] = 0.0;
  T01[1][3] = l1 * sin(angle1);
  T01[2][0] = 0.0;
  T01[2][1] = 0.0;
  T01[2][2] = 1.0;
  T01[2][3] = 0.0;
  T01[3][0] = 0.0;
  T01[3][1] = 0.0;
  T01[3][2] = 0.0;
  T01[3][3] = 1.0;

  T12[0][0] = cos(angle2);
  T12[0][1] = -sin(angle2);
  T12[0][2] = 0.0;
  T12[0][3] = l2 * cos(angle2);
  T12[1][0] = sin(angle2);
  T12[1][1] = cos(angle2);
  T12[1][2] = 0.0;
  T12[1][3] = l2 * sin(angle2);
  T12[2][0] = 0.0;
  T12[2][1] = 0.0;
  T12[2][2] = 1.0;
  T12[2][3] = 0.0;
  T12[3][0] = 0.0;
  T12[3][1] = 0.0;
  T12[3][2] = 0.0;
  T12[3][3] = 1.0;

  // calculate T02
  int i, j, k;
  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 4; j++)
    {
      T02[i][j] = 0;
      for (k = 0; k < 4; k++)
        T02[i][j] += T01[i][k] *
                     T12[k][j];
    }
  }

}
double safeAcos (double x)
{
  if (x < -1.0) x = -1.0 ;
  else if (x > 1.0) x = 1.0 ;
  return acos(x) ;
}
double* calculateGMI(double x, double y) {
  double* qs = new double[2];
  if (x == 0.0) {
    x = 0.0001;
  }
  if (y == 0.0) {
    y = 0.0001;
  }
  double L = sqrt(x * x + y * y);
  qs[1] = safeAcos((L * L - l1 * l1 - l2 * l2) / (2 * l1 * l2));
  qs[0] = atan(y / x) - atan(l2 * sin(qs[1]) / (l1 + l2 * cos(qs[1])));
  qs[0] = getDegreeFromRadians(qs[0]);
  qs[1] = getDegreeFromRadians(qs[1]);
  return qs;
}
double estimateErr(double xEstimat, double xReal, double yEstimat, double yReal) {
  // Standard Error of the Estimate
  return sqrt((xReal - xEstimat) * (xReal - xEstimat) + (yReal - yEstimat) * (yReal - yEstimat)) / 2;
}
void drawLineGMI (double x1, double y1, double x2, double y2) {
  double xi, yi, a, b, L, d;
  a = x2 - x1;
  b = y2 - y1;
  L = sqrt(a * a + b * b);
  d = L / 100;
  xi = x1 + d / L * a;
  yi = y1 + d / L * b;
  double err = estimateErr(xi, x2, yi, y2);
  double* qes = calculateGMI(xi, yi);
  while (err > 0.001) {

    xi += d / L * a;
    yi += d / L * b;
    err = estimateErr(xi, x2, yi, y2);
    double* qes = calculateGMI(xi, yi);
    ax12a.moveSpeed(c1, getPosFromDegreeFirst(round(qes[0])), motorSpeedSlow);
    delay(100);
    ax12a.moveSpeed(c2, getPosFromDegreeSecond(round(qes[1])), motorSpeedSlow);
    delay(100);
  }
  toolUp();
}
// time final tf is in seconds
double getThetaDotFn(double tf, double qInit, double qFin, double qDotInit, double qDotFin, double t) {

  double q0 = getRadiansFromDegree(qInit), q1 = getRadiansFromDegree(qFin);
  double a0 = q0, a1 = qDotInit, a2 = (3.0 * (q1 - q0) - tf * (2 * qDotInit + qDotFin)) / tf * tf;
  double a3 = (2.0 * (q0 - q1) + tf * (qDotFin + qDotInit)) / tf * tf * tf;

  return a1 + 2.0 * a2 * t + 3.0 * a3 * t * t;
}
//Vector<double> getThetaDot(double tf, double qInit, double qFin, double qDotInit, double qDotFin) {
//  Vector<double> theta;
//  Vector<double> thetaDot;
//  double tDotut = 0.0;
//  double q0 = getRadiansFromDegree(qInit), q1 = getRadiansFromDegree(qFin);
//  double a0 = q0, a1 = qDotInit, a2 = (3.0 * (q1 - q0) - tf * (2 * qDotInit + qDotFin)) / tf * tf;
//  double a3 = (2.0 * (q0 - q1) + tf * (qDotFin + qDotInit)) / tf * tf * tf;
//  double t = 0.0;
//  double err = abs(qInit - qFin);
//  for (int i = 0; err > 0.2; i++) {
//    theta.push_back( getDegreeFromRadians(a0 + a1 * t + a2 * t * t + a3 * t * t * t));
//    tDotut = getThetaDotFn(tf, theta.at(i), qFin, qDotInit, qDotFin, t);
//    thetaDot.push_back(tDotut);
//    //            cout<<"theta at T = "<<i<<" --> "<<theta.at(i)<<endl;
//    //              cout<<"thetaDot at T = "<<i<<" --> "<<thetaDot.at(i)/0.01<<endl;
//    err = abs(theta.at(i) - qFin);
//    t += 0.001;
//  }
//  return theta;
//}
double* getCrossProduct(double* a, double* b) {
  double* prod = new double[3];
  prod[0] = a[1] * b[2] - a[2] * b[1];
  //maybe not * (-1)
  prod[1] = -1 * (a[0] * b[2] - a[2] * b[0]);
  prod[2] = a[0] * b[1] - a[1] * b[0];
  return prod;
}
void calculateJacob(double qi, double qj) {
  // parameters can be readed from the servos
  calculateGMD(qi, qj, 1);

  jacob[0][2] = 0.0;
  jacob[1][2] = 0.0;
  jacob[2][2] = 0.0;
  jacob[3][2] = 0.0;
  jacob[4][2] = 0.0;
  jacob[5][2] = 1.0;
  double* v = new double[3];
  v[0] = T01[0][2];
  v[1] = T01[1][2];
  v[2] = T01[2][2];
  double* vb = new double[3];
  vb[0] = T02[0][3] - T01[0][3];
  vb[1] = T02[1][3] - T01[1][3];
  vb[2] = T02[2][3] - T01[2][3];
  double* prod = getCrossProduct(v, vb);
  for (int i = 0; i < 6; i++) {
    if (i < 3) {
      jacob[i][0] = prod[i];
    } else {
      jacob[i][0] = v[i - 3];
    }
  }
  v[0] = T02[0][2];
  v[1] = T02[1][2];
  v[2] = T02[2][2];
  vb[0] = T02[0][3] - T12[0][3];
  vb[1] = T02[1][3] - T12[1][3];
  vb[2] = T02[2][3] - T12[2][3];
  prod = getCrossProduct(v, vb);
  for (int i = 0; i < 6; i++) {
    if (i < 3) {
      jacob[i][1] = prod[i];
    } else {
      jacob[i][1] = v[i - 3];
    }
  }
}
//void drawLineCMI(double x1, double y1, double x2, double y2) {
//  double* qesI = calculateGMI(x1, y1); //degree
//  double* qesF = calculateGMI(x2, y2);
//  Vector<double> thetaDot1 = getThetaDot(0.5, qesI[0], qesF[0], 0.0, 0.0);
//  Vector<double> thetaDot2 = getThetaDot(0.5, qesI[1], qesF[1], 0.0, 0.0);
//
//  double** jInv = new double* [2];
//  jInv[0] = new double [2];
//  jInv[1] = new double [2];
//  double increment1 = 0.0, increment2 = 0.0, jDet;
//  double q111 = getRadiansFromDegree(qesI[0]), q222 = getRadiansFromDegree(qesI[1]);
//
//
//  for (int i = 0; i < thetaDot1.size(); i++ ) {
//    //      cout<<"C1 ="<<getDegreeFromRadians(q111)<<endl;
//    //      cout<<"C2 ="<<getDegreeFromRadians(q222)<<endl;
//   // calculateGMD(q111, q222, 1);
//    //      cout<<"X ="<<T02[0][3];
//    //      cout<<"Y ="<<T02[1][3]<<endl;
//    //      cout<<"Incr1 ="<<getDegreeFromRadians(increment1)<<endl;
//    //      cout<<"Incr2 ="<<getDegreeFromRadians(increment2)<<endl;
//    ax12a.moveSpeed(c1, getPosFromDegreeFirst(round(getDegreeFromRadians(q111))), motorSpeedFast);
//    delay(100);
//    ax12a.moveSpeed(c2, getPosFromDegreeSecond(round(getDegreeFromRadians(q222))), motorSpeedFast);
//    delay(1000);
//    calculateJacob(q111, q222);
//    jDet = 1.0 / (jacob[0][0] * jacob[1][1] - jacob[0][1] * jacob[1][0]);
//
//    jInv[0][0] = jDet * jacob[1][1];
//    jInv[0][1] = jDet * (-jacob[0][1]);
//    jInv[1][0] = jDet * (-jacob[1][0]);
//    jInv[1][1] = jDet * (jacob[0][0]);
//
//    increment1 =  (jInv[0][0] * thetaDot1.at(i) + jInv[0][1] * thetaDot2.at(i)) / 100.0;
//    increment2 =  (jInv[1][0] * thetaDot1.at(i) + jInv[1][1] * thetaDot2.at(i)) / 100.0;
//    q111 +=  increment1;
//    q222 += increment2;
//  }
//}
double* getVelocity(double x1, double y1, double x2, double y2, double t) {
  double* vit = new double[2];
  double T = 1.0;
  double x = x2 - x1, y = y2 - y1, a, b;
  double module = sqrt(x * x + y * y);
  double vMax = 2 * module / T;
  double beta;

  if (t < T / 2) {
    a = 2 * vMax / T;
    b = 0;
    beta = a * t + b;
  } else if (t > T / 2) {
    a = -2 * vMax / T;
    b = 2 * vMax;
    beta = a * t  + b;
  } else if (t == T / 2) {
    beta = vMax;
  }

  vit[0] = x / module * beta;
  vit[1] = y / module * beta;
  return vit;
}
double** linearInterpolation(double x0, double y0, double x1, double y1) {
  double** points = new double* [11];
    for (int i = 0; i <= 15; ++i) {
      points[i] = new double[2];
    double t = (double)i / 15;
    points[i][0] = (1.0 - t) * x0 + t * x1;
    points[i][1] = (1.0 - t) * y0 + t * y1;
  }
  return points;
  }
void drawLineCMI(double x1, double y1, double x2, double y2) {
  double** jInv = new double* [2];
  jInv[0] = new double [2];
  jInv[1] = new double [2];
  double* vit = new double [2];
  double increment1 = 0.0, increment2 = 0.0, jDet;
  double err = estimateErr(x1, x2, y1, y2);
  double pastErr = err;
  double t = 0.1;
  double** points = linearInterpolation( x1, y1,  x2,  y2);
  //vit = getVelocity(x1, y1, x2, y2, t);
  double* qes = calculateGMI(x1, y1); //degree
  double q111 = getRadiansFromDegree(qes[0]), q222 = getRadiansFromDegree(qes[1]);
  for(int i=1;i<=15;i++){
    t = 0.1;
  while (err > 0.1) {
    
   // t = (double)i / 10;
    vit = getVelocity(points[i-1][0], points[i-1][1], points[i][0], points[i][1], t);
    calculateJacob(q111, q222);
    jDet = 1.0 / (jacob[0][0] * jacob[1][1] - jacob[0][1] * jacob[1][0]);
    jInv[0][0] = jDet * jacob[1][1];
    jInv[0][1] = jDet * (-jacob[0][1]);
    jInv[1][0] = jDet * (-jacob[1][0]);
    jInv[1][1] = jDet * (jacob[0][0]);

    increment1 =  (jInv[0][0] * vit[0] + jInv[0][1] * vit[1]);
    increment2 =  (jInv[1][0] * vit[0] + jInv[1][1] * vit[1]);
    q111 +=  increment1;
    q222 += increment2;
    calculateGMD(q111, q222, 1);
    err = estimateErr(T02[0][3], x2, T02[1][3], y2);
  
    ax12a.moveSpeed(c1, getPosFromDegreeFirst(round(getDegreeFromRadians(q111))), motorSpeedFast);
    
    ax12a.moveSpeed(c2, getPosFromDegreeSecond(round(getDegreeFromRadians(q222))), motorSpeedFast);
 
      if (pastErr > err) {
      pastErr = err;
    } else if (pastErr < err) {
      break;
    }
    t += 0.1;
   // vit = getVelocity(T02[0][3], T02[1][3], x2, y2, t);
    Serial.print("Q1 = ");
    Serial.println(getDegreeFromRadians(q111));
    Serial.print("Q2 = ");
    Serial.println(getDegreeFromRadians(q222));
    Serial.print("Vit X = ");
    Serial.println(vit[0]);
    Serial.print("Vit Y = ");
    Serial.println(vit[1]);
    Serial.print("X = ");
    Serial.println(T02[0][3]);
    Serial.print("Y = ");
    Serial.println(T02[1][3]);
    }
 }
  delay(1000);
}
void drawLineMCI(double x1, double y1, double x2, double y2) {
  double** jInv = new double* [2];
  jInv[0] = new double [2];
  jInv[1] = new double [2];
  double* qesI = calculateGMI(x1, y1); //degree
  double* qesF = calculateGMI(x2, y2);
  double xi, yi , xDot, yDot;
  double b = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  double v = 2 * b / 1.0;
  double xa = v * (x2 - x1);
  double ya = v * (y2 - y1);

  double err = estimateErr(x1, x2, y1, y2);
  double pastErr = err;
  double increment1 = 0.0, increment2 = 0.0;
  double q111 = getRadiansFromDegree(qesI[0]), q222 = getRadiansFromDegree(qesI[1]);
  double jDet = 1.0;
  double t = 0.0;
  for (int i = 0; err > 0.02; i++) {
    xi =  (xa / b) * t + x1;
    yi =  (ya / b) * t + y1;
    if (i == 0) {
      xDot = 0.01;
      yDot = 0.01;
    } else {
      xDot = (v * (xi - x1)) / sqrt((xi - x1) * (xi - x1) + (yi - y1) * (yi - y1));
      yDot = (v * (yi - y1)) / sqrt((xi - x1) * (xi - x1) + (yi - y1) * (yi - y1));
    }
    b = sqrt((x2 - xi) * (x2 - xi) + (y2 - yi) * (y2 - yi));
    v = 2 * b / 1.0;
    xa = v * (xi - x1);
    ya = v * (yi - y1);
    t += 0.0195;
    calculateGMD(q111, q222, 1);
    Serial.print("X = ");
    Serial.println(T02[0][3]);
    Serial.print("Y = ");
    Serial.println(T02[1][3]);
    calculateJacob(q111, q222);
    jDet = 1.0 / (jacob[0][0] * jacob[1][1] - jacob[0][1] * jacob[1][0]);

    jInv[0][0] = jDet * jacob[1][1];
    jInv[0][1] = jDet * (-jacob[0][1]);
    jInv[1][0] = jDet * (-jacob[1][0]);
    jInv[1][1] = jDet * (jacob[0][0]);

    increment1 =  (jInv[0][0] * xDot + jInv[0][1] * yDot);
    increment2 =  (jInv[1][0] * xDot + jInv[1][1] * yDot);
    q111 +=  increment1;
    q222 += increment2;
    err = estimateErr((T02[0][3]) , (x2), (T02[1][3]), (y2));
    //     if (pastErr > err) {
    //      pastErr = err;
    //    } else if (pastErr < err) {
    //      break;
    //    }
    Serial.print("Err = ");
    Serial.println(err);

  }
}
void setup() {
  jacob = new double* [6];
  for (int i = 0; i < 6; ++i)
  {
    jacob[i] = new double[3];
  }
  Serial.begin(1000000);
  ax12a.begin(BaudRate, DirectionPin, &Serial);
  delay(10);
  resetInitPos();
  double* qs = calculateGMI(0.467, 24.43);
  ax12a.moveSpeed(c1, getPosFromDegreeFirst(round(qs[0])), motorSpeedFast);
  delay(100);
  ax12a.moveSpeed(c2, getPosFromDegreeSecond(round(qs[1])), motorSpeedFast);
  delay(1000);
  toolDown();
  //  //  drawLineGMI(0.467, 24.43, 9.394, 15.501);
  //  drawLineMCI(0.467, 24.43, 0.467, 10.501);
  drawLineCMI(0.467, 24.43, 9.394, 15.501);
 resetInitPos();
}
void loop() {}
