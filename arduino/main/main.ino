#include <Tic.h>

struct MoveData
{
  float azimuth, elevation;

  float accelerometer_x, accelerometer_y, accelerometer_z;
  float magnetometer_x, magnetometer_y, magnetometer_z;
  float gyro_x, gyro_y, gyro_z;
  float latitude, longitude;

  /*
  Returns false if the data string fails to parse each float.
  Format: "FLOAT FLOAT FLOAT ... FLOAT"
  */
  void read(String data);
};

TicI2C azimuth_motor;
TicI2C elevation_motor;

MoveData data;

void setup()
{
  Serial.begin(9600);

  // Set up I2C.
  Wire.begin();

  // Give the Tic some time to start up.
  delay(20);

  // Set the Tic's current position to 0, so that when we command
  // it to move later, it will move a predictable amount.
  azimuth_motor.haltAndSetPosition(0);
  elevation_motor.haltAndSetPosition(0);

  // Tells the Tic that it is OK to start driving the motor.  The
  // Tic's safe-start feature helps avoid unexpected, accidental
  // movement of the motor: if an error happens, the Tic will not
  // drive the motor again until it receives the Exit Safe Start
  // command.  The safe-start feature can be disbled in the Tic
  // Control Center.
  azimuth_motor.exitSafeStart();
  elevation_motor.exitSafeStart();

  calibrateAzimuth(azimuth_motor);
  calibrateElevation(elevation_motor);
}


bool active = true;

void loop()
{
    if (active)
    {
      if (Serial.available() > 0)
      {
        data.read(Serial.readString());

        moveToAngle(azimuth_motor, data.azimuth);
        moveToAngle(elevation_motor, data.elevation);
      }
    }
}

void MoveData::read(String input_data)
{
  String val;

  int i = 0;

  float* data_arr[13] = { 
    &azimuth, &elevation,
    &accelerometer_x, &accelerometer_y, &accelerometer_z,
    &magnetometer_x, &magnetometer_y, &magnetometer_z,
    &gyro_x, &gyro_y, &gyro_z,
    &latitude, &longitude
  };

  for (char c : input_data)
  {
    if (c != ' ')
    {
      val += c;
    }
    else
    {
      *data_arr[i] = val.toFloat();
      val = "";
      i++;
    }
  } 
}

// Sends a "Reset command timeout" command to the Tic.  We must
// call this at least once per second, or else a command timeout
// error will happen.  The Tic's default command timeout period
// is 1000 ms, but it can be changed or disabled in the Tic
// Control Center.
void resetCommandTimeout()
{
  tic.resetCommandTimeout();
}

// Delays for the specified number of milliseconds while
// resetting the Tic's command timeout so that its movement does
// not get interrupted by errors.
void delayWhileResettingCommandTimeout(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}

// Polls the Tic, waiting for it to reach the specified target
// position.  Note that if the Tic detects an error, the Tic will
// probably go into safe-start mode and never reach its target
// position, so this function will loop infinitely.  If that
// happens, you will need to reset your Arduino.
void waitForPosition(TicI2C& tic, int32_t targetPosition)
{
  do
  {
    resetCommandTimeout();
  } while (tic.getCurrentPosition() != targetPosition);
}

void moveAndDelay(TicI2C& tic, int32_t step_position)
{
    tic.exitSafeStart();
    tic.setTargetPosition(step_position);
    waitForPosition(tic, step_position);
}

float stepsToDegrees(int steps)
{
    return float(steps) / 128.0;
}

int degreesToSteps(float degrees)
{
    int steps = int(degrees * 128.0);
    steps -= steps/575;

    return steps;
}


void moveToAngle(TicI2C& tic, float degrees)
{
    int steps = degreeToSteps(degrees);

    float current_deg = stepsToDegrees(tic.getCurrentPosition());

    float forward_turn_error = 360.0 + degrees - current_deg;
    float backward_turn_error = current_deg - degrees;

    if (forward_turn_error < backward_turn_error)
    {
      int new_steps = degreesToSteps(360.0 + degrees);

      moveAndDelay(tic, new_steps);
      tic.haltAndSetPosition(stepsToDegrees(current_deg - degrees));
    }
    else
    {
      moveAndDelay(steps);
    }

    //Error will still accumulate for many, small increments.
    //Backsteps are only calculate for the given degrees.
}

void calibrateAzimuth(TicI2C& azimuth_motor, MoveData data, float mag_declination_east_degrees)
{
    float mag_north_deg_2d = 180.0 * atan2(data.magnetometer_y, data.magnetometer_x) / 3.14159265359;
    float true_north = mag_north_deg_2d + mag_declination_east_degrees;

    moveToAngle(true_north);
    azimuth_motor.haltAndSetPosition(0);
}

void calibrateElevation(TicI2C& elevation_motor, MoveData data)
{
    moveToAngle(180.0 * data.gyro_y / 3.14159265359);
    elevation_motor.haltAndSetPosition(0);
}