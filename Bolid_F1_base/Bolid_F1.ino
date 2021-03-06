/*
  Author : Simeon Simeonov
  Name : Bolid  Arduino project
  Ver: 1.0.0
*/

#include <Arduino.h>
#include <stdio.h> // for function sprintf
#include <PWM.h>

#include "pin_definitions.h"
#include "pitches.h"


//-------------------------- Sensors and position -------------------------
#define SENSORS_NR  6
const unsigned int sensors[SENSORS_NR] = { A7, A6, A5, A4, A3, A2 };  //left-right

#define LINE_TRESHOLD     40    // Ниво под което се смята че няма линия под сензора
#define SENSOR_TRESHOLD   5    // Филтриране на шумове от сензорите
#define ACQUIRE_SPEED     35

//---------------- Състояние ---------
#define FORWARD    1
#define LEFT       2
#define RIGHT      3
#define BACK       4


///////////////////////////////////////////////////////////////////////////
//----------------------------------- PID ---------------------------------
///////////////////////////////////////////////////////////////////////////
#define PWM_FREQ      50000


/////////////////////////////////////////////////////////////////////////
//------- Global Variables -----
signed long position;
int dir;
unsigned int sensors_sum;
signed int error;

//=================  CALIBRATE ==========================
unsigned int sensor_values[SENSORS_NR];
unsigned int sensor_calmin[SENSORS_NR];
unsigned int sensor_calmax[SENSORS_NR];
unsigned int sensor_denom[SENSORS_NR];

//--------- Common ------------
char tmp_str[64];

//--------------------------------
int melody[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };


//////////////////////////////////////////////////////////////////////////////
//
//                          SETUP
//////////////////////////////////////////////////////////////////////////////

//     Първоначална инициализация на всички входове, изходи и променливи
//
void setup() {

  pinMode(RIGHT_EDGE, INPUT);

  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // Управление на оптроните на 3 секции х 2
  pinMode( OPT_ENABLE_ONE , OUTPUT);
  pinMode( OPT_ENABLE_TWO , OUTPUT);
  pinMode( OPT_ENABLE_THREE , OUTPUT);

  // --- Изходи  ---
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BEEP, OUTPUT);

  // --- Входове ---
  pinMode(BUTT1, INPUT);
  pinMode(IR_PIN, INPUT_PULLUP);

  //---- PWM инициализация за управление на моторите
  InitTimersSafe();
  if ( SetPinFrequencySafe(LEFT_PWM, PWM_FREQ))
  {
    pinMode(LEFT_PWM, OUTPUT);
  }
  if (SetPinFrequencySafe(RIGHT_PWM, PWM_FREQ))
  {
    pinMode(RIGHT_PWM, OUTPUT);
  }

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  tone(BEEP, 2400, 400);
  delay(2000);
  noTone(BEEP);
  // Изчакване натискането на бутон
  while (digitalRead (BUTT1) == HIGH);

  callibrate();

  // ---  Изпълняване на мелодията ---
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    // to calculate the note duration, take one second
    // divided by the note type. //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BEEP, melody[thisNote], noteDuration);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BEEP);
  }

  // Изчакване натискането на бутон или команда от старт модул
  while ((digitalRead (BUTT1) == HIGH) && (digitalRead(IR_PIN) == LOW)) ;

}


//============================================================================
//
// the loop routine runs over and over again forever:
//
//============================================================================
void loop() {;

  //---- Прочитане на текущата позиция и пресмятане на грешката ----
  position = read_position();
  error = position - 350;


  delay(10);
}



//=======================================================================================
//=======================================================================================
//=======================================================================================

/* -----------------------------------------------------------------------------------
    Two way PWM speed control. If speed is >0 then motor runs forward.
    Else if speed is < 0 then motor runs backward
  ------------------------------------------------------------------------------------ */
void left_motor_speed(signed int motor_speed) {
  if (motor_speed == 0)
    motor_speed = 1;

  if (motor_speed > 0) {
    digitalWrite(LEFT_DIR, HIGH);
  }
  else {
    digitalWrite(LEFT_DIR, LOW);
  }
  motor_speed = abs(motor_speed);
  if (motor_speed > 255)
    motor_speed = 255;

  pwmWrite(LEFT_PWM, motor_speed);
}

//---------------------------------------
void right_motor_speed(signed int motor_speed) {
  if (motor_speed == 0)
    motor_speed = 1;

  if (motor_speed > 0) {
    digitalWrite(RIGHT_DIR, HIGH);
  }
  else {
    digitalWrite(RIGHT_DIR, LOW);
  }
  motor_speed = abs(motor_speed);
  if (motor_speed > 255)
    motor_speed = 255;

  pwmWrite(RIGHT_PWM, motor_speed);
}

//===============================  CALIBRATE ======================================
//-------------------
void callibrate(void) {
  int i, sens;
  int center_pos;
  unsigned int tmp_value;

  //--- Preset the arrays ---
  for (i = 0; i < SENSORS_NR; i++)
    (sensor_calmin)[i] = 4095;    // Maximum ADC value

  for (i = 0; i < SENSORS_NR; i++) {
    (sensor_calmax)[i] = 0;     // Minimum ADC value
    tmp_value = analogRead(sensors[i]);
  }

  //--- Turn right ---
  left_motor_speed(ACQUIRE_SPEED);    // forward
  right_motor_speed(-ACQUIRE_SPEED);    // backward

  for (i = 0; i < 600; i++) {

    //----- Find minimum and maximum values for all sensors -----
    for (sens = 0; sens < SENSORS_NR; sens++) {
      if (( sens == 0) || ( sens == 3)) {
        digitalWrite( OPT_ENABLE_ONE , HIGH);
        digitalWrite( OPT_ENABLE_THREE , LOW);
      } else if ((sens == 1) || (sens == 4)) {
        digitalWrite( OPT_ENABLE_ONE , LOW);
        digitalWrite( OPT_ENABLE_TWO , HIGH);
      } else {
        digitalWrite( OPT_ENABLE_TWO , LOW);
        digitalWrite( OPT_ENABLE_THREE , HIGH);
      }
      delayMicroseconds(300);

      tmp_value = analogRead(sensors[sens]) / 2;
      if (tmp_value < sensor_calmin[sens])
        sensor_calmin[sens] = tmp_value;
      if (tmp_value > sensor_calmax[sens])
        sensor_calmax[sens] = tmp_value;
    }

    if (i == 200) {   // --- turn left  ---
      left_motor_speed(-ACQUIRE_SPEED);
      right_motor_speed(ACQUIRE_SPEED);
    }
    //delay(1);
  }

  digitalWrite( OPT_ENABLE_ONE , HIGH);
  digitalWrite( OPT_ENABLE_THREE , LOW);

  //-------   Calculate calibration  denom --------
  for (sens = 0; sens < SENSORS_NR; sens++) {
    sensor_denom[sens] = (sensor_calmax[sens] - sensor_calmin[sens]) / 10;

    sprintf(tmp_str, " min %d - max %d", sensor_calmin[sens], sensor_calmax[sens]);
    Serial.println(tmp_str);
  }

  //---------- Go back to the line ----------
  left_motor_speed(ACQUIRE_SPEED);
  right_motor_speed(-ACQUIRE_SPEED);
  do {
    tmp_value = analogRead(sensors[3]) / 2;
    center_pos = ((tmp_value - sensor_calmin[3]) * 10) / sensor_denom[3];// Center sonsor position in the array = 2
    delay(10);
  }
  while (center_pos < 80);

  // --- Strom the motors ---
  left_motor_speed(0);
  right_motor_speed(1);

  digitalWrite( OPT_ENABLE_ONE , LOW);
  digitalWrite( OPT_ENABLE_TWO , LOW);
  digitalWrite( OPT_ENABLE_THREE , LOW);
}


//============================= Read sensors  and scale =====================
unsigned int read_position(void) {
  unsigned char on_line;
  static unsigned int last_pos;
  unsigned char sens;
  unsigned int tmp_value;
  signed long pos;

  pos = 0;
  sensors_sum = 0;
  on_line = 0;

  //-------------- Read sensors ------------
  for (sens = 0; sens < 3; sens++) {
    if ( sens == 0) {
      digitalWrite( OPT_ENABLE_ONE , HIGH);
    } else if (sens == 1) {
      digitalWrite( OPT_ENABLE_ONE , LOW);
      digitalWrite( OPT_ENABLE_TWO , HIGH);
    } else {
      digitalWrite( OPT_ENABLE_TWO , LOW);
      digitalWrite( OPT_ENABLE_THREE , HIGH);
    }
    delayMicroseconds(320);    // Wait for lighting
    tmp_value = analogRead(sensors[sens]) / 2;

    //--------- Validate ----------
    if (tmp_value < sensor_calmin[sens])
      tmp_value = sensor_calmin[sens];
    if (tmp_value > sensor_calmax[sens])
      tmp_value = sensor_calmax[sens];

    //-------- Calibrate ----------
    sensor_values[sens] = ((tmp_value - sensor_calmin[sens]) * 10)
                          / sensor_denom[sens];

    //----------- Noise filtering ----------
    if (sensor_values[sens]  < SENSOR_TRESHOLD)
      sensor_values[sens] = 0;

    // The estimate position is made using a weighted average of the sensor values
    // multiplied by 100,  The formula is:
    //
    //    100*value0 + 200*value1 + 300*value2 + ...
    //   --------------------------------------------
    //         value0  +  value1  +  value2 + ...

    pos += sensor_values[sens] * ((sens + 1) * 100);
    sensors_sum += sensor_values[sens];

    //--- line presens check and count ---
    if (sensor_values[sens] > LINE_TRESHOLD) {
      on_line += 1;
    }

    //================================================
    tmp_value = analogRead(sensors[sens + 3]) / 2;

    //--------- Validate ----------
    if (tmp_value < sensor_calmin[sens + 3])
      tmp_value = sensor_calmin[sens + 3];
    if (tmp_value > sensor_calmax[sens + 3])
      tmp_value = sensor_calmax[sens + 3];

    //-------- Calibrate ----------
    sensor_values[sens + 3] = ((tmp_value - sensor_calmin[sens + 3]) * 10)
                              / sensor_denom[sens + 3];

    //----------- Noise filtering ----------
    if (sensor_values[sens + 3]  < SENSOR_TRESHOLD)
      sensor_values[sens + 3] = 0;

    // The estimate position is made using a weighted average of the sensor values
    // multiplied by 100,  The formula is:
    //
    //    100*value0 + 200*value1 + 300*value2 + ...
    //   --------------------------------------------
    //         value0  +  value1  +  value2 + ...

    pos += sensor_values[sens + 3] * ((sens + 1 + 3) * 100);
    sensors_sum += sensor_values[sens + 3];

    //--- line presens check ---
    if (sensor_values[sens + 3] > LINE_TRESHOLD) {
      on_line += 1;
    }
  }
  digitalWrite( OPT_ENABLE_THREE , LOW);

  if (!on_line) {
    // If it last read to the left of center, return 0.
    //if (last_pos < (SENSORS_NR - 1) * 100 / 2) {
    if (last_pos < 250) {
      last_pos = 90;
      // If it last read to the right of center, return the max.
    }
    else if (last_pos > 450) {
      last_pos = (SENSORS_NR * 100) + 10;
    } else {
      last_pos = 350;   // center pos
    }
  }
  else {
    if (on_line > 2) {
      tmp_value = sensor_values[0] + sensor_values[1];
      if (  tmp_value > 90 )
        last_pos = 100;
      tmp_value = sensor_values[SENSORS_NR - 2] + sensor_values[SENSORS_NR - 1];
      if  ( tmp_value > 90 )
        last_pos = (SENSORS_NR * 100);

    } else
      last_pos = pos / sensors_sum;
  }

  return last_pos;
}

