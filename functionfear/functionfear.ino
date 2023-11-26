bool gbSenserConnectState = false;
int threshold = 16700;
int mood = 0;
uint8_t sad = 0;
bool hasBeenFlippedOnce = false;
bool hasBeenRotated = false;
bool hasBeenLifted = false;
bool isDark = false;
bool hasSwitchedMood = false;

long lightTimer = 0;


#include <Waveshare_10Dof-D.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include "Adafruit_Soundboard.h"


// Choose any two pins that can be used with SoftwareSerial to RX & TX
#define SFX_TX 9
#define SFX_RX 8

// Connect to the RST pin on the Sound Board
#define SFX_RST 7

// You can also monitor the ACT pin for when audio is playing!

// we'll be using software serial
SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);

// pass the software serial to Adafruit_soundboard, the second
// argument is the debug port (not used really) and the third
// arg is the reset pin
Adafruit_Soundboard sfx = Adafruit_Soundboard(&ss, NULL, SFX_RST);
// can also try hardware serial with
// Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial1, NULL, SFX_RST);


LiquidCrystal lcd(12, 11, 5, 4, 3, 2);


void setup() {
  // pinMode (Analog_Input, INPUT);
  // pinMode (Digital_Input, INPUT);
  //bool bRet;
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  //Used to configure the sensitivity to the 8g range (not currently using it due to how)
  //I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG, REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_8g | REG_VAL_BIT_ACCEL_DLPF);

  Serial.begin(115200);
  ss.begin(9600);

  lcd.begin(16, 2);
  lcd.clear();

  imuInit(&enMotionSensorType, &enPressureType);
  if (IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType) {
    Serial.println("Motion sersor is ICM-20948");
  } else {
    Serial.println("Motion sersor NULL");
  }
  if (IMU_EN_SENSOR_TYPE_BMP280 == enPressureType) {
    Serial.println("Pressure sersor is BMP280");
  } else {
    Serial.println("Pressure sersor NULL");
  }
  delay(200);
  Serial.print("\nPlaying track #");
  Serial.println(sad);
  if (!sfx.playTrack((uint8_t)sad)) {
    Serial.println("Failed to play track?");
  }
}

void loop() {
  //Cases can be reduced down if too similar or too complicated
  switch (mood) {
    case 1:
      //Unease
      Serial.println("I am now uneasy!");
      lcdTextDisplay(mood - 1);
      //Output code goes here
      //Could add some code that resets the mood back to 0 if left alone long enough
      delay(500);
      imuDetection();
      ldrDetection();

      break;
    case 2:
      //Nervous
      //Output code goes here
      Serial.println("I am now nervous!");
      lcdTextDisplay(mood - 1);
      delay(500);
      imuDetection();
      ldrDetection();


      break;
    case 3:
      //Anxiety
      //Output code goes here
      Serial.println("I am now anxious!");
      lcdTextDisplay(mood - 1);

      //sfx.playTrack("00");
      delay(500);
      imuDetection();
      ldrDetection();


      break;
    case 4:
      //Disturbed
      //Output code goes here
      Serial.println("I am now disturbed!");
      lcdTextDisplay(mood - 1);

      delay(500);
      imuDetection();
      ldrDetection();
      break;
    case 5:
      //Fear
      Serial.println("I am now fearful!");
      lcdTextDisplay(mood - 1);

      // Output code goes here
      //   Once we're at the worst mood possible you cannot return to default break; default : imuDetection(); ldrDetection();

      delay(500);

      break;

    default:
      imuDetection();
      ldrDetection();

      delay(500);

      break;
  }
}

void imuDetection() {

  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

  int zVal = stAccelRawData.s16Z;
  int rollVal = stAngles.fRoll;

  if (hasBeenLifted == false) {
    if (zVal >= threshold) {
      do {
        hasBeenLifted = true;
        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        zVal = stAccelRawData.s16Z;
        Serial.println("I have been lifted!");

        Serial.println("    Acceleration: Z : ");
        Serial.print(stAccelRawData.s16Z);
      } while (zVal >= threshold);
    }
  } else if (hasBeenLifted == true) {
    hasBeenLifted = false;
    if (mood < 5) {
      mood++;
    }
  }
  // if (hasBeenRotated == false) {
  //   if (rollVal >= 90 || rollVal <= -90) {
  //     do
  //     {
  //       hasBeenRotated = true;
  //       imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  //     rollVal = stAngles.fRoll;
  //       Serial.println("I am upside down!");
  //       Serial.println("Roll : "); Serial.print(stAngles.fRoll);
  //     } while (rollVal >= 90 || rollVal <= -90);
  //   }
  // } else if (hasBeenRotated == true) {
  //   hasBeenRotated = false;
  //   if(mood < 5) {
  //     mood++;
  //   }
  // }


  // Serial.println();
  // Serial.println("/-------------------------------------------------------------/");
  // Serial.print("Roll : "); Serial.print(stAngles.fRoll);
  // Serial.print("    Pitch : "); Serial.print(stAngles.fPitch);
  // Serial.print("    Yaw : "); Serial.print(stAngles.fYaw);
  // Serial.println();
  // Serial.print("Acceleration: X : "); Serial.print(stAccelRawData.s16X);
  // Serial.print("    Acceleration: Y : "); Serial.print(stAccelRawData.s16Y);
  // Serial.print("    Acceleration: Z : "); Serial.print(stAccelRawData.s16Z);
  // Serial.println();
  // Serial.print("Gyroscope: X : "); Serial.print(stGyroRawData.s16X);
  // Serial.print("       Gyroscope: Y : "); Serial.print(stGyroRawData.s16Y);
  // Serial.print("       Gyroscope: Z : "); Serial.print(stGyroRawData.s16Z);
  // Serial.println();
}

void ldrDetection() {
  int ldrSensorValue = analogRead(A0);
  Serial.print("LDR Sensor Value: ");
  Serial.println(ldrSensorValue);
  // LDR Sensor needs to be adjusted too be less sensitive to light
  // Arbitrary value set for the if statement
  // Most I could get out of the reading was 20 - 28 with current config

  if (ldrSensorValue >= 900) {
    isDark = true;
    Serial.println("It's too dark!");
  } else {
    lightTimer = millis();
  }
  while (isDark == true) {
    Serial.print("light Timer = ");
    Serial.println(lightTimer);
    if (millis() - lightTimer > 5000) {
      Serial.println("IN DARK FOR TOO LONG");
      isDark = false;
      lightTimer = millis();
      if (mood < 5) {
        mood++;
        hasSwitchedMood = false;
      }
    } else {
      isDark = false;
      break;
    }
  }
}
void micDetection() {
  int Analog_Input = A5;
  int Digital_Input = 11;
}


void lcdTextDisplay(int num) {
  char *moods[] = { "I am now uneasy!",
                    "I am now nervous!",
                    "I am now anxious!",
                    "I am now disturbed!",
                    "I am now fearful!" };
  if (!hasSwitchedMood) {
    lcd.clear();
    lcd.print(moods[num]);
    hasSwitchedMood = true;
  }
}
