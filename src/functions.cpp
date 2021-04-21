#include "vex.h"
#include "cmath"
using namespace vex;

void turn_right_PID(float target) {
  Motor1.resetRotation();
  Motor2.resetRotation();
  Motor3.resetRotation();
  Motor4.resetRotation();

  target *= 2.95;

  float curPosition = (Motor1.rotation(rotationUnits::deg) + Motor2.rotation(rotationUnits::deg) -
  Motor3.rotation(rotationUnits::deg) - Motor4.rotation(rotationUnits::deg)) / 4;

  float error = target - curPosition;
  float Kp = 0.3;
  float Ki = 0.1;
  float Kd = 0.01;

  float integral = 0;
  float derivative = 0;
  float lastError = error;

  while(error > 0) {
    curPosition = (Motor1.rotation(rotationUnits::deg) + Motor2.rotation(rotationUnits::deg) -
    Motor3.rotation(rotationUnits::deg) - Motor4.rotation(rotationUnits::deg)) / 4;
    
    error = target - curPosition;
    
    integral += error;
    derivative = error - lastError;
    // Left side
    Motor3.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor3.spin(forward);
    Motor4.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor4.spin(forward);
    // Right side
    Motor1.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor1.spin(reverse);
    Motor2.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor2.spin(reverse);

    lastError = error;

    vex::task::sleep(20);
  }
  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  Motor4.stop();
}

void turn_left_PID(float target) {
  target *= 2.95;

  Motor1.resetRotation();
  Motor2.resetRotation();
  Motor3.resetRotation();
  Motor4.resetRotation();

  float curPosition = ( - Motor1.rotation(rotationUnits::deg) - Motor2.rotation(rotationUnits::deg) +
  Motor3.rotation(rotationUnits::deg) + Motor4.rotation(rotationUnits::deg)) / 4;

  float error = target - curPosition;
  float Kp = 0.3;
  float Ki = 0.1;
  float Kd = 0.01;

  float integral = 0;
  float derivative = 0;
  float lastError = error;

  while(error > 0) {
    curPosition = ( - Motor1.rotation(rotationUnits::deg) - Motor2.rotation(rotationUnits::deg) +
    Motor3.rotation(rotationUnits::deg) + Motor4.rotation(rotationUnits::deg)) / 4;
    
    error = target - curPosition;
    
    integral += error;
    derivative = error - lastError;
    // Left side
    Motor3.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor3.spin(reverse);
    Motor4.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor4.spin(reverse);
    // Right side
    Motor1.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor1.spin(forward);
    Motor2.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor2.spin(forward);

    lastError = error;

    vex::task::sleep(20);
  }
  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  Motor4.stop();
}

void drive_PID(float target) {
  Motor1.resetRotation();
  Motor2.resetRotation();
  Motor3.resetRotation();
  Motor4.resetRotation();

  float curPosition = (Motor1.rotation(rotationUnits::deg) + Motor2.rotation(rotationUnits::deg) + 
  Motor3.rotation(rotationUnits::deg) + Motor4.rotation(rotationUnits::deg)) / 4;

  float error = target - curPosition;
  float Kp = 0.25;
  float Ki = 0.1;
  float Kd = 1;

  float integral = 0;
  float derivative = 0;
  float lastError = error;

  while(error > 0) {
    curPosition = (Motor1.rotation(rotationUnits::deg) + Motor2.rotation(rotationUnits::deg) + 
    Motor3.rotation(rotationUnits::deg) + Motor4.rotation(rotationUnits::deg)) / 4;
    
    error = target - curPosition;
    derivative = error - lastError;
    // Right side
    Motor1.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor1.spin(reverse);
    Motor2.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor2.spin(reverse);
    // Left side
    Motor3.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor3.spin(reverse);
    Motor4.setVelocity((error * Kp) + (integral * Ki) + (derivative * Kd), rpm);
    Motor4.spin(reverse);

    lastError = error;

    vex::task::sleep(10);
  }
  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  Motor4.stop();
}
