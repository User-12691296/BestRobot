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
#include "cmath"

// Allows for easier use of the VEX Library
using namespace vex;

const float GEAR_RATIO_X = 137.84;
const float GEAR_RATIO_Y = 480;

const int MAX_DEGREES_X = 1000;
const int MAX_DEGREES_Y = 500;

void calibrateAllAxes();
void moveTo(float x, float y);
void markerDown();
void markerUp();
void manualControlOverride();
void markerSwitch();
void drawMenu(int selectedOption);
void penPressure(bool applyPressure);
void keepUserInformed(char C);
void exitRobot();
int mainMenu();

void calibrateAllAxes()
{
  const int XSAMPLES = 3;
  const int YSAMPLES = 2;

  const double APPROACH_SPEED = 10.0; //towards sensor
  const double RETRACT_SPEED = 20; //away from sensor

  TouchLED.setBlink(yellow, 0.5, 0.5);

  double samplesY[YSAMPLES];

  for (int i = 0; i < YSAMPLES; i++) //move away until not pressed
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
  double avgDeg = (samplesY[0] + samplesY[1]) / YSAMPLES;
  //shift encoder so the position becomes 0
  double currentDeg = YMotor.position(degrees);
  YMotor.setPosition(currentDeg - avgDeg, degrees);

    //x axis calibration
    double samplesX[XSAMPLES];

    for (int i = 0; i < XSAMPLES; i++) //move away until not pressed
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
    double avgDegX = (samplesX[0] + samplesX[1] + samplesX[2]) / XSAMPLES;
    double currentDegX = XMotor.position(degrees);
    XMotor.setPosition(currentDegX - avgDegX, degrees);

    wait(2000, msec);

    moveTo(0,0);
}

void moveTo(float x, float y)
{
  float xLocation = -1;
  float yLocation = -1;

  xLocation = (static_cast<float>(XMotor.position(degrees)))/GEAR_RATIO_X;
  yLocation = (static_cast<float>(YMotor.position(degrees)))/GEAR_RATIO_Y;

  float xDistanceOff = -1;
  float yDistanceOff = -1;
  const float ERROR_MARGIN = 0.1;
  xDistanceOff = x - xLocation;
  yDistanceOff = y - yLocation;

  bool xReached = false;
  bool yReached = false;

  float D_LEN = pow((xDistanceOff*xDistanceOff)+(yDistanceOff*yDistanceOff), 0.5);

  const int X_VELOCITY = abs(10 * (xDistanceOff/D_LEN));
  const int Y_VELOCITY = abs(10 * ((GEAR_RATIO_Y / GEAR_RATIO_X) * (yDistanceOff/D_LEN)));

  XMotor.setVelocity(X_VELOCITY, percent);
  YMotor.setVelocity(Y_VELOCITY, percent);

  XMotor.spin(forward);
  YMotor.spin(forward);

  int tick=0;

  while(xReached == false || yReached == false)
  {

    xLocation = (static_cast<float>(XMotor.position(degrees)))/GEAR_RATIO_X;
    yLocation = (static_cast<float>(YMotor.position(degrees)))/GEAR_RATIO_Y;
    xDistanceOff = x - xLocation;
    yDistanceOff = y - yLocation;

    Brain.Screen.setCursor(1,1);
    Brain.Screen.clearLine(1);
    Brain.Screen.print("X: %f", xLocation);
    Brain.Screen.setCursor(2,1);
    Brain.Screen.clearLine(2);
    Brain.Screen.print("Y: %f", yLocation);
    Brain.Screen.setCursor(3,1);
    Brain.Screen.clearLine(3);
    Brain.Screen.print("XDO: %f", xDistanceOff);
    Brain.Screen.setCursor(4,1);
    Brain.Screen.clearLine(4);
    Brain.Screen.print("YDO: %f", yDistanceOff);
    Brain.Screen.setCursor(5,1);
    Brain.Screen.clearLine(5);
    Brain.Screen.print("XL: %f", xLocation);
    Brain.Screen.setCursor(6,1);
    Brain.Screen.clearLine(6);
    Brain.Screen.print("XL: %f", yLocation);

    if (xLocation < x && abs(xDistanceOff) > ERROR_MARGIN)
    {
      XMotor.setVelocity(X_VELOCITY, percent);
    }
    else if (xLocation > x && abs(xDistanceOff) > ERROR_MARGIN)
    {
      XMotor.setVelocity(-X_VELOCITY, percent);
    }
    else if (abs(xDistanceOff) <= ERROR_MARGIN)
    {
      XMotor.stop(brake);
      xReached = true;
    }

    if (yLocation < y && abs(yDistanceOff) > ERROR_MARGIN)
    {
      YMotor.setVelocity(Y_VELOCITY, percent);
    }
    else if (yLocation > y && abs(yDistanceOff) > ERROR_MARGIN)
    {
      YMotor.setVelocity(-Y_VELOCITY, percent);
    }
    else if (abs(yDistanceOff) <= ERROR_MARGIN)
    {
      YMotor.stop(brake);
      yReached = true;
    }

    if (tick%5 == 0) {
      MMotor.setMaxTorque(8, percent);
      MMotor.setVelocity(10, percent);
      wait(200, msec);
      MMotor.stop();
    }
    tick++;
  }
}

void markerDown()
{
  MMotor.setMaxTorque(2, percent);
  MMotor.setVelocity(30, percent);
  MMotor.spin(forward);
  wait(500, msec);
  MMotor.stop();
}

void markerUp()
{
  MMotor.setMaxTorque(100, percent);
  MMotor.setVelocity(20, percent);
  MMotor.spin(reverse);
  wait(200, msec);
  MMotor.stop(brake);
}

void manualControlOverride()
{
  bool markerDown = true;
  bool markerInProgress = false;
  bool xSpinningForward = true;
  bool ySpinningForward = true;

  TouchLED.setBlink(blue, 0.5, 0.5);

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
      MMotor.setMaxTorque(5, percent);
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
      MMotor.setMaxTorque(5, percent);
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
  MMotor.stop(brake);
  if (markerDown)
    markerUp();
  calibrateAllAxes();
}

void markerSwitch()
{
  //percent speed of the marker motors
  const int MARKER_RETRACT_SPEED = 20;
  const int MARKER_INSERTION_SPEED = 20;

  //time in seconds the marker motors will spin for
  const int MARKER_RETRACT_TIME = 2;
  const int MARKER_INSERTION_TIME = 2;

  TouchLED.setBlink(purple, 0.5, 0.5);

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

void penPressure(bool applyPressure)
{
  if (applyPressure == true) {
      MMotor.setMaxTorque(1, percent);
      //MMotor.setVelocity(20, percent);
      MMotor.spin(forward);
  }
  else {
    MMotor.stop(brake);
    MMotor.setMaxTorque(1, percent);
    //MMotor.setVelocity(50, percent);
    MMotor.spin(reverse);
  }
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

const float SEGMENTS_P1[][2][2] = {
	{{1.0, 1}, {1.0, 10}},
	{{2.5, 10.0}, {6, 10.0}},
	{{7.5, 1}, {7.5, 10}},
	{{2.5, 1.0}, {6, 1.0}},
	{{1.25, 1}, {1.25, 10}},
	{{2.5, 9.75}, {6, 9.75}},
	{{7.25, 1}, {7.25, 10}},
	{{2.5, 1.25}, {6, 1.25}},
	{{1.5, 1}, {1.5, 10}},
	{{2.5, 9.5}, {6, 9.5}},
	{{7.0, 1}, {7.0, 10}},
	{{2.5, 1.5}, {6, 1.5}},
	{{1.75, 1}, {1.75, 10}},
	{{2.5, 9.25}, {6, 9.25}},
	{{6.75, 1}, {6.75, 10}},
	{{2.5, 1.75}, {6, 1.75}},
	{{2.0, 1}, {2.0, 10}},
	{{2.5, 9.0}, {6, 9.0}},
	{{6.5, 1}, {6.5, 10}},
	{{2.5, 2.0}, {6, 2.0}},
	{{4.25, 5.5}, {4.283333333333333, 5.5}},
	{{4.283333333333333, 5.5}, {4.303172261921682, 5.540213865849686}},
	{{4.303172261921682, 5.540213865849686}, {4.2772280247040575, 5.596221799352929}},
	{{4.2772280247040575, 5.596221799352929}, {4.201566502603603, 5.624225497011406}},
	{{4.201566502603603, 5.624225497011406}, {4.108045510976159, 5.5873309843283465}},
	{{4.108045510976159, 5.5873309843283465}, {4.050920760326564, 5.4808308494959554}},
	{{4.050920760326564, 5.4808308494959554}, {4.078244013005396, 5.342062273740712}},
	{{4.078244013005396, 5.342062273740712}, {4.202319409302307, 5.2376306565510555}},
	{{4.202319409302307, 5.2376306565510555}, {4.385263262206829, 5.232224254464362}},
	{{4.385263262206829, 5.232224254464362}, {4.549342525253539, 5.353353610048028}},
	{{4.549342525253539, 5.353353610048028}, {4.609929967264144, 5.569963298304733}},
	{{4.609929967264144, 5.569963298304733}, {4.5171330221884185, 5.797724618492461}},
	{{4.5171330221884185, 5.797724618492461}, {4.286260398777337, 5.931813572341452}},
	{{4.286260398777337, 5.931813572341452}, {4.000635570284747, 5.894455522170199}},
	{{4.000635570284747, 5.894455522170199}, {3.7819702321305133, 5.675920824202351}},
	{{3.7819702321305133, 5.675920824202351}, {3.7386294747369635, 5.348525149491241}},
	{{3.7386294747369635, 5.348525149491241}, {3.913729297591236, 5.043893514831649}},
	{{3.913729297591236, 5.043893514831649}, {4.257330200501149, 4.900044778203728}},
	{{4.257330200501149, 4.900044778203728}, {4.6381744673135525, 4.999568492162063}},
	{{4.6381744673135525, 4.999568492162063}, {4.89364842058615, 5.326330327571568}},
	{{4.89364842058615, 5.326330327571568}, {4.8990286981967515, 5.762224615391139}},
	{{4.8990286981967515, 5.762224615391139}, {4.6265969181759905, 6.129247597530673}},
	{{4.6265969181759905, 6.129247597530673}, {4.167200493958716, 6.2621824057121085}},
	{{4.167200493958716, 6.2621824057121085}, {3.701345317658099, 6.082218206125768}},
	{{3.701345317658099, 6.082218206125768}, {3.428336635774579, 5.638973955596751}},
	{{3.428336635774579, 5.638973955596751}, {3.4812572779122077, 5.099817868529637}},
	{{3.4812572779122077, 5.099817868529637}, {3.863958379937169, 4.68699823642303}},
	{{3.863958379937169, 4.68699823642303}, {4.439268142332746, 4.586058710086352}},
	{{4.439268142332746, 4.586058710086352}, {4.9773344119325325, 4.863266855218607}},
	{{4.9773344119325325, 4.863266855218607}, {5.247440782930944, 5.428502555667316}},
	{{5.247440782930944, 5.428502555667316}, {5.116626253543606, 6.062793669515527}},
	{{5.116626253543606, 6.062793669515527}, {4.61307152022725, 6.502974002144448}},
	{{4.61307152022725, 6.502974002144448}, {3.9247211449515484, 6.550806198334578}},
	{{3.9247211449515484, 6.550806198334578}, {3.3296386695226188, 6.1613467062036}},
	{{3.3296386695226188, 6.1613467062036}, {3.0836815952552863, 5.4714957921552365}},
	{{3.0836815952552863, 5.4714957921552365}, {3.310870211777102, 4.752982436034858}},
	{{3.310870211777102, 4.752982436034858}, {3.9432824724966817, 4.305414101274687}},
	{{3.9432824724966817, 4.305414101274687}, {4.738814320277469, 4.331451753355421}},
	{{4.738814320277469, 4.331451753355421}, {5.373557242539816, 4.846074069380694}},
	{{5.373557242539816, 4.846074069380694}, {5.573676455905927, 5.66018245177953}},
	{{5.573676455905927, 5.66018245177953}, {5.233097727129489, 6.449366439627298}},
	{{5.233097727129489, 6.449366439627298}, {4.466594540819253, 6.8831438120771455}},
	{{4.466594540819253, 6.8831438120771455}, {3.57267723702102, 6.763201614626487}},
	{{3.57267723702102, 6.763201614626487}, {2.9175213396274753, 6.11287170824158}},
	{{2.9175213396274753, 6.11287170824158}, {2.784991582498748, 5.177897009249405}},
	{{2.784991582498748, 5.177897009249405}, {3.2541796592270518, 4.334044186079952}},
	{{3.2541796592270518, 4.334044186079952}, {4.1570866813976615, 3.9360909362527674}},
	{{4.1570866813976615, 3.9360909362527674}, {5.1377510839168, 4.1688734045912295}},
	{{5.1377510839168, 4.1688734045912295}, {5.7924814047212205, 4.962851143660317}},
	{{5.7924814047212205, 4.962851143660317}, {5.835990673132396, 6.012261029666349}},
	{{5.835990673132396, 6.012261029666349}, {5.225079941796779, 6.892558475291286}},
	{{5.225079941796779, 6.892558475291286}, {4.186484238241969, 7.232169215883005}},
	{{4.186484238241969, 7.232169215883005}, {3.1334148986254293, 6.869068596710717}},
	{{3.1334148986254293, 6.869068596710717}, {2.5012115840042366, 5.926308662918112}},
	{{2.5012115840042366, 5.926308662918112}, {2.567449852912702, 4.771895533836233}},
	{{2.567449852912702, 4.771895533836233}, {3.3308093544735318, 3.8753360342455183}},
	{{3.3308093544735318, 3.8753360342455183}, {4.5012871414885245, 3.6166904735220697}},
	{{4.5012871414885245, 3.6166904735220697}, {5.609896425433646, 4.1257876838455525}},
	{{5.609896425433646, 4.1257876838455525}, {6.19655992078532, 5.219496430379168}},
	{{6.19655992078532, 5.219496430379168}, {6.000929054000031, 6.466564766510013}}
};

const float SEG_TEST[][2][2] = {
	{{0, 0}, {5, 0}},
	{{5, 0}, {2.5, 4}},
	{{2.5, 4}, {0, 0}}
};

const float CUBE[][2][2] = {
  // Front face (bottom square)
  {{1, 1}, {4, 1}},      // bottom edge
  {{4, 1}, {4, 4}},      // right edge
  {{4, 4}, {1, 4}},      // top edge
  {{1, 4}, {1, 1}},      // left edge
 
  // Back face
  {{2, 2}, {5, 2}},      // bottom edge
  {{5, 2}, {5, 5}},      // right edge
  {{5, 5}, {2, 5}},      // top edge
  {{2, 5}, {2, 2}},      // left edge
 
  // Connecting edges (front to back)
  {{1, 1}, {2, 2}},      // bottom-left
  {{4, 1}, {5, 2}},      // bottom-right
  {{4, 4}, {5, 5}},      // top-right
  {{1, 4}, {2, 5}}       // top-left
};

const float AUTOMATED_TOLERANCE = 0.05;

void automatedDrawing(const float segments[][2][2], const int segs) {
	float xloc = 0; float yloc = 0;
	
	bool markerDownYet = false;
	
	for (int cseg=0; cseg<segs; cseg++) {
		// TODO: convert to inches
		xloc = (static_cast<float>(XMotor.position(degrees)))/GEAR_RATIO_X;
		yloc = (static_cast<float>(YMotor.position(degrees)))/GEAR_RATIO_Y;
		
		// If farther than 0.05 on either axis from the start of next line, pick up marker and move there
		if (abs(xloc)-abs(segments[cseg][0][0]) > AUTOMATED_TOLERANCE || abs(yloc)-abs(segments[cseg][0][1]) > AUTOMATED_TOLERANCE) {
			markerUp();
			moveTo(segments[cseg][0][0], segments[cseg][0][1]);
			markerDown();
			markerDownYet = true;
		}
		// Move to end of line segment with marker down
		if (!markerDownYet) {
			markerDown();
			markerDownYet = true;
		}
		moveTo(segments[cseg][1][0], segments[cseg][1][1]);
	}
}

void exitRobot()
{
  moveTo(0,0);
  markerUp();
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  XMotor.setPosition(0, degrees);
  YMotor.setPosition(0, degrees);
  MMotor.setPosition(0, degrees);

  // Begin project code

  // Initial calibration
  calibrateAllAxes();
  if (XMotor.isSpinning() == false || YMotor.isSpinning() == false)
  {
    keepUserInformed('S');
  }

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

        if (XMotor.isSpinning() == false || YMotor.isSpinning() == false)
        {
          keepUserInformed('S');
        }
        
        break;

      case 1: // Automator
        //Brain.Screen.clearScreen();
        //automatedDrawing(CUBE, 12);

		TouchLED.setBlink(orange, 0.5, 0.5);

        markerUp();
        moveTo(0, 0);
        markerDown();
        moveTo(4, 0);
        moveTo(4, 4);
        moveTo(0, 4);
        moveTo(0, 0);
        markerUp();

        //automatedDrawing(SEG_TEST, 3);
        wait(2, seconds);

        if (XMotor.isSpinning() == false || YMotor.isSpinning() == false)
        {
          keepUserInformed('S');
        }

        break;

      case 2: // Switch Marker
        //Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Starting Marker Switch");
        keepUserInformed('M');
        wait(500, msec);
        markerSwitch();
        while (TouchLED.pressing())
        {

        }
        while (!TouchLED.pressing())
        {
          keepUserInformed('S');
        }
        break;

      case 3: // Recalibration
        //Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Recalibrating...");
        calibrateAllAxes();
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Done!");
        wait(1, seconds);
        keepUserInformed('S'); 
        break;

      case 4: // Shutdown
        //Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Shutting down...");
        keepUserInformed('P');
        wait(1, seconds);
        TouchLED.setFade(slow);
        exitRobot();
        return 0;
    }
  }

  Brain.programStop();
}