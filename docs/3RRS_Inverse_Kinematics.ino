#include <InverseKinematics.h>
#include <math.h>

const double Rb = 2;
const double Rp = 3.125;
const double Llow = 1.75;
const double Lup = 3.669291339;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 3; i++) {
    Serial.println((String) "Theta #"+i+": "+MtrAng(i, 4.25, 0.5, 0.1));
  }
}

void loop() {
  
}

// Inverse kinematics function. mtrNum is the motor number, which may either be 0, 1, or 2.
double MtrAng(int mtrNum, double h, double n_x, double n_y) {

  double theta = 0;    // Initialize theta to be 0 (radians)
  double x, y, z, len; // Declare intermediate values

  double n_z = pow(1 - n_x * n_x - n_y * n_y, 0.5);  // This line is unneccessary, but I haven't verified that yet

  // Find the normal vector
  double nlen = sqrt(1 + n_y * n_y + n_x * n_x);
  n_z = pow(nlen,-1);
  n_y /= nlen;
  n_x /= nlen;

  // Find the corresponding motor angle based on inverse kinematics calculations.
  if (mtrNum == 0) {
    y = Rb + (Rp / 2) * (1 - (n_x * n_x + 3 * n_z * n_z + 3 * n_z) / (n_z + 1 - n_x * n_x + (pow(n_x, 4) - 3 * n_x * n_x * n_y * n_y) / ((n_z + 1) * (n_z + 1 - n_x * n_x))));
    z = Rp * n_y + h;
    len = sqrt(z * z + y * y);
    theta = acos((len * len + Llow * Llow - Lup * Lup) / (2 * len * Llow)) + acos(y / len);
  }
  else if (mtrNum == 1) {
    x = (sqrt(3) / 2) * (Rp * (1 - (n_x * n_x + sqrt(3) * n_x * n_y) / (n_z + 1)) - Rb);
    y = x / sqrt(3);
    z = h - (Rp / 2) * (sqrt(3) * n_x + n_y);
    len = sqrt(z * z + y * y + x * x);
    theta = acos((len * len + Llow * Llow - Lup * Lup) / (2 * len * Llow)) + acos((sqrt(3) * x + y) / (-2 * len));
  }
  else if (mtrNum == 2) {
    x = (sqrt(3) / 2) * (Rb - Rp * (1 - (n_x * n_x - sqrt(3) * n_x * n_y) / (n_z + 1)));
    y = -x / sqrt(3);
    z = h + (Rp / 2) * (sqrt(3) * n_x - n_y);
    len = sqrt(z * z + y * y + x * x);
    theta = acos((len * len + Llow * Llow - Lup * Lup) / (2 * len * Llow)) + acos((sqrt(3) * x - y) / (2 * len));
  }

  return theta * 180 / PI; // Return angle that motor needs to be at (in degrees)

}