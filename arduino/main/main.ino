#include <Tic.h>

class MoveData
{
  public:

  float azimuth, elevation;

  float accelerometer_x, accelerometer_y, accelerometer_z;
  float magnetometer_x, magnetometer_y, magnetometer_z;
  float gyro_x, gyro_y, gyro_z;

  /*
  Returns false if the data string fails to parse each floatt.
  Format: "FLOAT FLOAT FLOAT ... FLOAT"
  */
  void read(String data);
};

TicI2C tic;

void setup()
{
  Serial.begin(9600);

  // Set up I2C.
  Wire.begin();

  // Give the Tic some time to start up.
  delay(20);

  // Set the Tic's current position to 0, so that when we command
  // it to move later, it will move a predictable amount.
  tic.haltAndSetPosition(0);

  // Tells the Tic that it is OK to start driving the motor.  The
  // Tic's safe-start feature helps avoid unexpected, accidental
  // movement of the motor: if an error happens, the Tic will not
  // drive the motor again until it receives the Exit Safe Start
  // command.  The safe-start feature can be disbled in the Tic
  // Control Center.
  tic.exitSafeStart();
}


bool active = true;

void loop()
{
    if (active)
    {
      int input = 0;

      if (Serial.available() > 0)
      {

        input = Serial.readString().toInt();

        Serial.print("Input angle (deg): ");
        Serial.println(input);

        moveToAngle(input);

        Serial.print("Current pos: ");
        Serial.println(tic.getCurrentPosition());

      }
    }
}

void MoveData::read(String input_data)
{
  String val;

  int i = 0;

  float* data_arr[11] = { 
    &azimuth, &elevation,
    &accelerometer_x, &accelerometer_y, &accelerometer_z,
    &magnetometer_x, &magnetometer_y, &magnetometer_z,
    &gyro_x, &gyro_y, &gyro_z
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
void waitForPosition(int32_t targetPosition)
{
  do
  {
    resetCommandTimeout();
  } while (tic.getCurrentPosition() != targetPosition);
}

void moveAndDelay(int32_t step_position)
{
    tic.exitSafeStart();
    tic.setTargetPosition(step_position);
    waitForPosition(step_position);
}

void moveToAngle(float degrees)
{
    int steps = int(degrees * 128.0);

    steps -= steps/575;

    float current_deg = float(tic.getCurrentPosition()) / 128.0;

    float forward_turn_error = 360.0 + degrees - current_deg;
    float backward_turn_error = current_deg - degrees;

    if (forward_turn_error < backward_turn_error)
    {
      int new_steps = int((360.0 + degrees) * 128.0);
      new_steps -= new_steps/575;

      moveAndDelay(new_steps);
      tic.haltAndSetPosition(int((current_deg - degrees) * 128.0));
    }
    else
    {
      moveAndDelay(steps);
    }

    //Error will still accumulate for many, small increments.
    //Backsteps are only calculate for the given degrees.
}