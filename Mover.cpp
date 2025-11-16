#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
motor XMotor = motor(PORT3, true);
motor YMotor = motor(PORT6, false);
motor MMotor = motor(PORT2, false);
bumper YCalSwitch = bumper(PORT4);
controller Controller = controller();
optical XCalOptical = optical(PORT1);
touchled TouchLED = touchled(PORT5);


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}

// Converts a color to a string
const char* convertColorToString(color col) {
  if (col == colorType::red) return "red";
  else if (col == colorType::green) return "green";
  else if (col == colorType::blue) return "blue";
  else if (col == colorType::white) return "white";
  else if (col == colorType::yellow) return "yellow";
  else if (col == colorType::orange) return "orange";
  else if (col == colorType::purple) return "purple";
  else if (col == colorType::cyan) return "cyan";
  else if (col == colorType::black) return "black";
  else if (col == colorType::transparent) return "transparent";
  else if (col == colorType::red_violet) return "red_violet";
  else if (col == colorType::violet) return "violet";
  else if (col == colorType::blue_violet) return "blue_violet";
  else if (col == colorType::blue_green) return "blue_green";
  else if (col == colorType::yellow_green) return "yellow_green";
  else if (col == colorType::yellow_orange) return "yellow_orange";
  else if (col == colorType::red_orange) return "red_orange";
  else if (col == colorType::none) return "none";
  else return "unknown";
}


// Convert colorType to string
const char* convertColorToString(colorType col) {
  if (col == colorType::red) return "red";
  else if (col == colorType::green) return "green";
  else if (col == colorType::blue) return "blue";
  else if (col == colorType::white) return "white";
  else if (col == colorType::yellow) return "yellow";
  else if (col == colorType::orange) return "orange";
  else if (col == colorType::purple) return "purple";
  else if (col == colorType::cyan) return "cyan";
  else if (col == colorType::black) return "black";
  else if (col == colorType::transparent) return "transparent";
  else if (col == colorType::red_violet) return "red_violet";
  else if (col == colorType::violet) return "violet";
  else if (col == colorType::blue_violet) return "blue_violet";
  else if (col == colorType::blue_green) return "blue_green";
  else if (col == colorType::yellow_green) return "yellow_green";
  else if (col == colorType::yellow_orange) return "yellow_orange";
  else if (col == colorType::red_orange) return "red_orange";
  else if (col == colorType::none) return "none";
  else return "unknown";
}


void vexcodeInit() {

  // Initializing random seed.
  initializeRandomSeed(); 
}


// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

//----------------------------------------------------------------------------
//                                                                            
//    Module:       main.cpp                                                  
//    Author:       {author}                                                  
//    Created:      {date}                                                    
//    Description:  IQ project                                                
//                                                                            
//----------------------------------------------------------------------------

// Include the IQ Library
#include "iq_cpp.h"

// Allows for easier use of the VEX Library
using namespace vex;

class Mover {
  private:
    bool XTargetSet, YTargetSet, MTargetSet;
    float XTarget, YTarget, MTarget;
    float DXSpeed, DYSpeed, DMSpeed,
        UXSpeed, UYSpeed, UMSpeed;

    bool paused, stopped;

    // In mm/degree
    const float X_MM_PER_DEG = 0.25;
    const float Y_MM_PER_DEG = 0.05;
    const float M_MM_PER_DEG = 1.8;

    // In degrees/s
    const float MAX_MOTOR_SPEED_DEG_PER_S = 90.0;
    const float MIN_MOTOR_SPEED_PERCENT = 0.03;
    // In mm/s
    const float MAX_X_SPEED = X_MM_PER_DEG*MAX_MOTOR_SPEED_DEG_PER_S;
    const float MAX_Y_SPEED = Y_MM_PER_DEG*MAX_MOTOR_SPEED_DEG_PER_S;
    const float MAX_M_SPEED = M_MM_PER_DEG*MAX_MOTOR_SPEED_DEG_PER_S;
    const float PERCENT = 100.0;

    // In mm
    const float MAX_TARGET_TOLERANCE = 0.8;
    const float MAX_SPEED_AT_TARGET = 0.2;

    // K-values for PID
    const float K_P = 0.1;
    const float K_I = 0;
    const float K_D = 0;

    const float X_MIN = 0.0;
    const float Y_MIN = 0.0;
    const float M_MIN = 0.0;

    const float X_MAX = 11.0*25.4;
    const float Y_MAX = 8.5*25.4;
    const float M_MAX = 20.0;
  
    int getXAPos () {
      return X_MM_PER_DEG*XMotor.position(degrees);
    }
    int getYAPos() {
      return Y_MM_PER_DEG*YMotor.position(degrees);
    }
    int getMAPos() {
      return M_MM_PER_DEG*MMotor.position(degrees);
    }

    void setXASpeed(float aXSpeed) {
      if (abs(aXSpeed/MAX_X_SPEED*PERCENT) > MIN_MOTOR_SPEED_PERCENT) {
        XMotor.setVelocity(aXSpeed/MAX_X_SPEED*PERCENT, percent);
        XMotor.spin(forward);
      } else {
        XMotor.stop();
      }
    }
    void setYASpeed(float aYSpeed) {
      if (abs(aYSpeed/MAX_Y_SPEED*PERCENT) > MIN_MOTOR_SPEED_PERCENT) {
        YMotor.setVelocity(aYSpeed/MAX_Y_SPEED*PERCENT, percent);
        YMotor.spin(forward);
      } else {
        YMotor.stop();
      }
    }
    void setMASpeed(float aMSpeed) {
      if (abs(aMSpeed/MAX_M_SPEED*PERCENT) > MIN_MOTOR_SPEED_PERCENT) {
        MMotor.setVelocity(aMSpeed/MAX_M_SPEED*PERCENT, percent);
        MMotor.spin(forward);
      } else {
        MMotor.stop();
      }
    }

    void pauseXMotor() {
      if (UXSpeed != 0.0) {
        setXASpeed(0.0);
        UXSpeed = 0.0;
      }
    }
    bool updateXSpeed() {
      if (DXSpeed < 0 && getXAPos()<=X_MIN) {
        // Below lower bound
        pauseXMotor();
      }
      if (DXSpeed > 0 && getXAPos()>=X_MAX) {
        // Above upper bound
        pauseXMotor();
      }
      if (DXSpeed != UXSpeed) {
        setXASpeed(DXSpeed);
        UXSpeed = DXSpeed;
        return true;
      }
      return false;
    }

    void pauseYMotor() {
      if (UYSpeed != 0.0) {
        setYASpeed(0.0);
        UYSpeed = 0.0;
      }
    }
    bool updateYSpeed() {
      if (DYSpeed < 0 && getYAPos()<=Y_MIN) {
        // Below lower bound
        pauseYMotor();
      }
      if (DYSpeed > 0 && getYAPos()>=Y_MAX) {
        // Above upper bound
        pauseYMotor();
      }
      if (DYSpeed != UYSpeed) {
        setYASpeed(DYSpeed);
        UYSpeed = DYSpeed;
        return true;
      }
      return false;
    }

    void pauseMMotor() {
      if (UMSpeed != 0.0) {
        setMASpeed(0.0);
        UMSpeed = 0.0;
      }
    }
    bool updateMSpeed() {
      if (DMSpeed < 0 && getMAPos()<=M_MIN) {
        // Below lower bound
        pauseMMotor();
      }
      if (DMSpeed > 0 && getMAPos()>=M_MAX) {
        // Above upper bound
        pauseMMotor();
      }
      if (DMSpeed != UMSpeed) {
        setMASpeed(DMSpeed);
        UMSpeed = DMSpeed;
        return true;
      }
      return false;
    }

    void ifXTargetSetPIDControl() {
      if (XTargetSet) {
        float toCover = XTarget - getXAPos();
        if (abs(toCover) < MAX_TARGET_TOLERANCE && abs(UXSpeed) < MAX_SPEED_AT_TARGET) {
          XTargetSet = false;
          pauseXMotor();
        } else {
          DXSpeed = K_P*toCover;
        }
      }
    }
  
  public:
    Mover() {
      XTargetSet = false;
      YTargetSet = false;
      MTargetSet = false;
      
      // In mm
      XTarget = 0.0;
      YTarget = 0.0;
      MTarget = 0.0;

      // Desired Speed In mm/s
      DXSpeed = 0.0;
      DYSpeed = 0.0;
      DMSpeed = 0.0;

      UXSpeed = 0.0;
      UYSpeed = 0.0;
      UMSpeed = 0.0;

      paused = false;
      stopped = false;
    }

    float getXSpeed() {
      return UXSpeed;
    }
    float getYSpeed() {
      return UYSpeed;
    }
    float getMSpeed() {
      return UMSpeed;
    }

    bool setXSpeed(float nXSpeed) {
      if (nXSpeed != DXSpeed) {
        if (nXSpeed < -MAX_X_SPEED) {
          DXSpeed = -MAX_X_SPEED;
        } else if (nXSpeed > MAX_X_SPEED) {
          DXSpeed = MAX_X_SPEED;
        } else {
          DXSpeed = nXSpeed;
        }
        return true;
      }
      return false;
    }
    bool setYSpeed(float nYSpeed) {
      if (nYSpeed != DYSpeed) {
        if (nYSpeed < -MAX_Y_SPEED) {
          DYSpeed = -MAX_Y_SPEED;
        } else if (nYSpeed > MAX_Y_SPEED) {
          DYSpeed = MAX_Y_SPEED;
        } else {
          DYSpeed = nYSpeed;
        }
        return true;
      }
      return false;
    }
    bool setMSpeed(float nMSpeed) {
      if (nMSpeed != DMSpeed) {
        if (nMSpeed < -MAX_M_SPEED) {
          DMSpeed = -MAX_M_SPEED;
        } else if (nMSpeed > MAX_M_SPEED) {
          DMSpeed = MAX_M_SPEED;
        } else {
        DMSpeed = nMSpeed;
        }
        return true;
      }
      return false;
    }

    bool isXTargetSet() {
      return XTargetSet;
    }
    bool setXTarget(float nXTarget) {
      if (nXTarget < X_MIN || nXTarget > X_MAX) {
        return false;
      }
      XTarget = nXTarget;
      XTargetSet = true;
      return true;
    }
    bool reachedXTarget() {
      return (abs(XTarget-getXAPos()) < MAX_TARGET_TOLERANCE);
    }

    // Return whether now paused
    bool changePause() {
      paused = !paused;
      return paused;
    }
    // Return whether change occured
    bool pause() {
      if (!paused) {
        paused = true;
        return true;
      }
      return false;
    }
    bool unPause() {
      if (paused) {
        paused = false;
        return true;
      }
      return false;
    }

    bool tick() {
      if (paused) {
        pauseXMotor();
        pauseYMotor();
        pauseMMotor();
      }
      else {
        // Target processing
        ifXTargetSetPIDControl();
        
        // Speed updating
        if (updateXSpeed() || updateYSpeed() || updateMSpeed()) {
          // Moved successfully
        }
        else {
          // Attempting to move to unreachable location - Stuck
          return false;
        }
      }
      return true;
    }
};

void calibrateYaxis()
{
  const int SAMPLES = 3;
  Const double APPROCH_SPEED = 10.0; //towards sensor
  const double RETRACT_SPEED = 20; //away from sensor

  double samples[samples];

  for (int i = 0; i < samples; i++) //move away until not prressed
  {
    YMotor.setVelocity(RETRACT_SPEED, percent);
    YMotor.spin(forward);
    while (!YCalSwitch.pressing())
    {}
    while (YCalSwitch.pressing()) 
    {
    wait(10,msec)  
    }
    YMotor.stop(brake)


  }


}

void manualControlOverride(float maxX, float maxY)
{
  XMotor.spin(forward);
  YMotor.spin(forward);
  while (!Controller.ButtonFDown.pressing())
  {
    if (XCalOptical.color() != black)
    {
      XMotor.setVelocity(Controller.AxisB.position(), percent);
    }
    else
    {
      XMotor.setVelocity(0, percent);
    }
    YMotor.setVelocity(Controller.AxisA.position(), percent);
  }
  XMotor.stop(brake);
  YMotor.stop(brake);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  XMotor.setPosition(0, degrees);
  YMotor.setPosition(0, degrees);
  MMotor.setPosition(0, degrees);

  // Begin project code

  Mover m;

  m.setXSpeed(3);

  for (int i=0; i<500000; i++) {
    m.tick();
    if (i==250000) {
      m.setXSpeed(-3);
    }
  }

  Brain.programStop();
}
