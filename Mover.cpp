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
motor YMotor = motor(PORT6, true);
motor MMotor = motor(PORT2, false);
bumper YCalSwitch = bumper(PORT4);
controller Controller = controller();
touchled TouchLED = touchled(PORT5);
distance XCalDistance = distance(PORT1);


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

const int MAX_DEGREES_X = 500;
const int MAX_DEGREES_Y = 500;

// presents a color on touch led based on a certain state of robot
void KeepUserInformed(char C);

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
  
    float getXAPos () {
      return X_MM_PER_DEG*XMotor.position(degrees);
    }
    float getYAPos() {
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
      // Check lower bound (at origin, position = 0)
      if (DXSpeed < 0 && getXAPos()<=X_MIN) {
        // Below lower bound
        pauseXMotor();
        return false;
      }
      // Check upper bound
      if (DXSpeed > 0 && getXAPos()>=X_MAX) {
        // Above upper bound
        pauseXMotor();
        return false;
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
      // Check lower bound (at origin, position = 0)
      if (DYSpeed < 0 && getYAPos()<=Y_MIN) {
        // Below lower bound
        pauseYMotor();
        return false;
      }
      // Check upper bound
      if (DYSpeed > 0 && getYAPos()>=Y_MAX) {
        // Above upper bound
        pauseYMotor();
        return false;
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

void calibrateAllAxes()
{
  const int SAMPLES = 3;
  const double APPROACH_SPEED = 10.0; //towards sensor
  const double RETRACT_SPEED = 20; //away from sensor


  double samplesY[SAMPLES];

  for (int i = 0; i < SAMPLES; i++) //move away until not pressed
  {
    YMotor.setVelocity(RETRACT_SPEED, percent);
    YMotor.spin(forward);
    while (YCalSwitch.pressing()) 
    {
    }
    YMotor.stop(brake);
    wait(200,msec);//pause to settle

    //move slowly toards sernsor unitl pressed

    YMotor.setVelocity(APPROACH_SPEED*3, percent);
    YMotor.spin(reverse);
    while (!YCalSwitch.pressing()) 
    {
    }
    YMotor.stop(brake);
    //records position
    samplesY[i] = YMotor.position(degrees);

    //pause to settle 
    wait(2000,msec);
    //move backwards more 
    YMotor.setVelocity(RETRACT_SPEED*2, percent);
    YMotor.spin(forward);
    wait(3000, msec);
    YMotor.stop(brake);

  }
  //average contact position calculation
  double avgDeg = (samplesY[0] + samplesY[1] + samplesY[2]) / SAMPLES;
  //shift encoder so the position becomes 0
  double currentDeg = YMotor.position(degrees);
  YMotor.setPosition(currentDeg - avgDeg, degrees);

    //x axis calibration
    double samplesX[SAMPLES];

    for (int i = 0; i < SAMPLES; i++) //move away until not pressed
    {
      XMotor.setVelocity(RETRACT_SPEED, percent);
      XMotor.spin(forward);
      while (XCalDistance.objectDistance(mm) < 60) 
      {
      }
      XMotor.stop(brake);
      wait(200, msec); //pause to settle

      //mmove slowly towards sensor until 
      XMotor.setVelocity(APPROACH_SPEED*2, percent);
      XMotor.spin(reverse);
      while (XCalDistance.objectDistance(mm) > 40) 
      {
      }
      XMotor.stop(brake);
      //records position
      samplesX[i] = XMotor.position(degrees);
    }

    //average contact position calculation
    double avgDegX = (samplesX[0] + samplesX[1] + samplesX[2]) / SAMPLES;
    double currentDegX = XMotor.position(degrees);
    XMotor.setPosition(currentDegX - avgDegX, degrees);
}

void manualControlOverride()
{
  bool markerDown = true;
  bool markerInProgress = false;
  bool xSpinningForward = true;
  bool ySpinningForward = true;
  
  while (!Controller.ButtonFDown.pressing())
  {
    //moves along the x-axis
    double xPos = XMotor.position(degrees);
    int xAxisInput = Controller.AxisB.position();
    

    //check boundaries: stop if at limit and trying to go further
    if ((xPos <= 0 && xAxisInput < 0) || (xPos >= MAX_DEGREES_X && xAxisInput > 0))
    {
      //at boundary and trying to go further will stop motor completely
      XMotor.stop(brake);
    }
    else if (xAxisInput > 0)
    {
      //moving away from origin (forward direction)
      if (!xSpinningForward)
      {
        XMotor.stop();
        xSpinningForward = true;
      }
      XMotor.setVelocity(xAxisInput/3, percent);
      XMotor.spin(forward);
    }
    else if (xAxisInput < 0)
    {
      //moving towards origin (reverse direction)
      if (xSpinningForward)
      {
        XMotor.stop();
        xSpinningForward = false;
      }
      XMotor.setVelocity(-xAxisInput/3, percent);
      XMotor.spin(reverse);
    }
    else
    {
      //no input will make it stop
      XMotor.stop(brake);
    }

    //moves along the y-axis
    double yPos = YMotor.position(degrees);
    int yAxisInput = Controller.AxisA.position();
    
    // Check boundaries: stop if at limit and trying to go further
    if ((yPos <= 0 && yAxisInput < 0) || (yPos >= MAX_DEGREES_Y && yAxisInput > 0))
    {
      //at boundary and trying to go further will stop motor completely
      YMotor.stop(brake);
    }
    else if (yAxisInput > 0)
    {
      //moving away from origin (forward direction)
      if (!ySpinningForward)
      {
        YMotor.stop();
        ySpinningForward = true;
      }
      YMotor.setVelocity(yAxisInput, percent);
      YMotor.spin(forward);
    }
    else if (yAxisInput < 0)
    {
      //moving towards origin (reverse direction)
      if (ySpinningForward)
      {
        YMotor.stop();
        ySpinningForward = false;
      }
      YMotor.setVelocity(-yAxisInput, percent);
      YMotor.spin(reverse);
    }
    else
    {
      //no input will make it stop
      YMotor.stop(brake);
    }
    //moves marker down if pressed and marker not already down
    if (Controller.ButtonRDown.pressing() && markerDown == false && markerInProgress == false)
    {
      markerInProgress = true;
      markerDown = true;
      // Set low torque for constant gentle pressure
      MMotor.setMaxTorque(10, percent);
      MMotor.setVelocity(100, percent);
      MMotor.spin(forward);
      markerInProgress = false;
    }
    //moves marker up if pressed and marker already down
    if (Controller.ButtonRUp.pressing() && markerDown == true && markerInProgress == false)
    {
      markerInProgress = true;
      markerDown = false;
      //stop applying pressure and lift marker
      MMotor.stop(brake);
      //reset to full torque for lifting
      MMotor.setMaxTorque(100, percent);
      MMotor.setVelocity(100, percent);
      MMotor.spin(reverse);
      wait(200, msec);
      MMotor.stop(brake);
      markerInProgress = false;
    }
    
    // Keep marker pressing down with low torque when in down position
    if (markerDown && !markerInProgress)
    {
      MMotor.setMaxTorque(10, percent);
      MMotor.setVelocity(100, percent);
      MMotor.spin(forward);
    }
    // Display coordinates in degrees (using positions already read above)
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.clearLine();
    Brain.Screen.print("X: %.1f deg  AxisB: %d", xPos, xAxisInput);
    
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.clearLine();
    Brain.Screen.print("Y: %.1f deg  AxisA: %d", yPos, yAxisInput);

  }
  XMotor.stop(brake);
  YMotor.stop(brake);
}

void markerSwitch()
{
  //percent speed of the marker motors
  const int MARKER_RETRACT_SPEED = 20;
  const int MARKER_INSERTION_SPEED = 20;

  //time in seconds the marker motors will spin for
  const int MARKER_RETRACT_TIME = 2;
  const int MARKER_INSERTION_TIME = 2; 
    
    MMotor.setVelocity(MARKER_RETRACT_SPEED, percent);
    MMotor.spin(reverse);

    Brain.Screen.clearLine(1);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Removing Marker...");

    wait(MARKER_RETRACT_TIME, seconds);
    MMotor.stop(brake);

    Brain.Screen.clearLine(1);
    Brain.Screen.print("Marker ready to be inserted");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Press TouchLED when inserted");

    while(!TouchLED.pressing())
    {
    }

    Brain.Screen.clearLine(1);
    Brain.Screen.clearLine(2);

    MMotor.setVelocity(MARKER_INSERTION_SPEED, percent);
    MMotor.spin(forward);

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Inserting Marker...");

    wait(MARKER_INSERTION_TIME, seconds);
    MMotor.stop(brake);

    Brain.Screen.clearLine(1);
    Brain.Screen.print("Marker switch complete!");

    wait(2, seconds);
}

void drawMenu(int selectedOption) {
  const int NUM_OPTIONS = 5;
  const char* menuOptions[] = {
    "Manual Control",
    "Automator",
    "Switch Marker",
    "Recalibration",
    "Shutdown"
  };
  
  // colors for each menu option
  color optionColors[] = {blue, green, purple, yellow, orange};
  
  // Clear screen with the selected option's color
  Brain.Screen.clearScreen(optionColors[selectedOption]);
  
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFont(mono15);
  
  // Draw menu options
  for (int i = 0; i < NUM_OPTIONS; i++) {
    Brain.Screen.setCursor(i + 1, 1);
    
    // Highlight selected option with arrow and bright color
    if (i == selectedOption) {
      Brain.Screen.setPenColor(yellow);
      Brain.Screen.print("> %s <", menuOptions[i]);
    } else {
      Brain.Screen.setPenColor(white);
      Brain.Screen.print("  %s", menuOptions[i]);
    }
  }
  
  Brain.Screen.setPenColor(white);
  
  // Navigation instructions
  Brain.Screen.setPenColor(white);
  Brain.Screen.setCursor(7, 1);
  Brain.Screen.setFont(mono12);
  Brain.Screen.print("Left/Right: Navigate");
  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Check: Select");
}

int mainMenu ()
{
  int selectedOption = 0;
  const int NUM_OPTIONS = 5;
  
  bool buttonPressed = false;
  bool needsRedraw = true;
  
  while (true) {
    // Only redraw when selection changes
    if (needsRedraw) {
      drawMenu(selectedOption);
      needsRedraw = false;
    }
    
    // Check for buttonLeft (navigate up)
    if (Brain.buttonLeft.pressing()) {
      if (!buttonPressed) {
        buttonPressed = true;
        selectedOption--;
        if (selectedOption < 0) {
          selectedOption = NUM_OPTIONS - 1; // Wrap to bottom
        }
        needsRedraw = true;
        wait(200, msec); // Debounce
      }
    }
    // Check for buttonRight (navigate down)
    else if (Brain.buttonRight.pressing()) {
      if (!buttonPressed) {
        buttonPressed = true;
        selectedOption++;
        if (selectedOption >= NUM_OPTIONS) {
          selectedOption = 0; // Wrap to top
        }
        needsRedraw = true;
        wait(200, msec); // Debounce
      }
    }
    // Check for buttonCheck (select option)
    else if (Brain.buttonCheck.pressing()) {
      if (!buttonPressed) {
        buttonPressed = true;
        wait(200, msec); // Show selection briefly
        return selectedOption;
      }
    }
    else {
      buttonPressed = false;
    }
    
    wait(50, msec);
  }
  
  return selectedOption;
}

// presents a color on touch led based on a certain state of robot
void keepUserInformed(char C)
{
  // while homing all axies
  if (C == 'H')
  {
    TouchLED.setColor(yellow);
  }

  // startup done and completed
  else if (C == 'S')
  {
    TouchLED.setColor(green);
  }

  // autodraw
  else if (C == 'A')
  {
    TouchLED.setColor(orange);
  }

  // controller
  else if (C == 'C')
  {
    TouchLED.setColor(blue);
  }

  // marker switcher
  else if (C == 'M')
  {
    TouchLED.setColor(purple);
  }

  // limit triggered unexpectedly 
  else if (C == 'T')
  {
    // use return to main menu function
  }

  // paper is removed
  else if (C == 'R')
  {
    TouchLED.setColor(red);
    while (!TouchLED.pressing())
    {
      wait(0.5, seconds);
      TouchLED.setFade(fast);
      wait(0.5, seconds);
      TouchLED.setBrightness(100);
    }
    TouchLED.off(); 
  }

  // shutdown procedure
  else if (C == 'P')
  {
    TouchLED.setColor(red);
  }

} 

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  XMotor.setPosition(0, degrees);
  YMotor.setPosition(0, degrees);
  MMotor.setPosition(0, degrees);

  // Begin project code
  
  // Initial calibration
  //calibrateAllAxes();
  
  // Main menu loop
  while (true) {
    int selection = mainMenu();

    switch (selection) {
      case 0: // Manual Control
        //Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Starting Manual Mode");
        wait(500, msec);
        manualControlOverride();
        break;
        
      case 1: // Automator
        //Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Automator Mode");
        keepUserInformed('A');
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Not implemented yet");
        wait(2, seconds);
        break;
        
      case 2: // Switch Marker
        //Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Starting Marker Switch");
        keepUserInformed('M');
        wait(500, msec);
        markerSwitch();
        break;
        
      case 3: // Recalibration
        //Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Recalibrating...");
        calibrateAllAxes();
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Done!");
        wait(1, seconds);
        break;
        
      case 4: // Shutdown
        //Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Shutting down...");
        keepUserInformed('P');
        wait(1, seconds);
        Brain.programStop();
        return 0;
    }
  }

  Brain.programStop();
}
