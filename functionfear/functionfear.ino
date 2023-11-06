#include <Waveshare_10Dof-D.h>
bool gbSenserConnectState = false;
int threshold = 16700;
int mood = 0;
bool hasBeenRotated = false; bool hasBeenLifted = false; bool hasBeenBlinded = false; 

void setup() {
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  //Used to configure the sensitivity to the 8g range (not currently using it due to how)
  //I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG, REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_8g | REG_VAL_BIT_ACCEL_DLPF);

  Serial.begin(115200);

  imuInit(&enMotionSensorType, &enPressureType);
  if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
  {
    Serial.println("Motion sersor is ICM-20948");
  }
  else
  {
    Serial.println("Motion sersor NULL");
  }
  if(IMU_EN_SENSOR_TYPE_BMP280 == enPressureType)
  {
    Serial.println("Pressure sersor is BMP280");
  }
  else
  {
    Serial.println("Pressure sersor NULL");
  }
  delay(200);
}

void loop() {
  //Cases can be reduced down if too similar or too complicated
  switch(mood) {
    case 1:
        //Unease
        Serial.println("I am now uneasy!");
        //Output code goes here
        //Could add some code that resets the mood back to 0 if left alone long enough
        delay(500);
        detection();

        break;
    case 2:
        //Nervous
        //Output code goes here
        Serial.println("I am now nervous!");
        delay(500);
        detection();

        break;
    case 3:
        //Anxiety
        //Output code goes here
        Serial.println("I am now anxious!");
        delay(500);
        detection();

        break;
    case 4:
        //Disturbed
        //Output code goes here
        Serial.println("I am now disturbed!");
        delay(500);
        detection();

        break;
    case 5:
        //Fear
        //Serial.println("I am now fearful!");
        //Output code goes here
        //Once we're at the worst mood possible you cannot return to default
        break;
    default:
        detection();
        delay(500);

        break;
  }
}

void detection() {
  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;
  
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

  int zVal = stAccelRawData.s16Z;
  int rollVal = stAngles.fRoll;
  //int ldrSensorValue = analogRead(A0);

  if (hasBeenLifted == false) {
    if (zVal >= threshold) {
      do
      {
        hasBeenLifted = true;
        imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        zVal = stAccelRawData.s16Z;
        Serial.println("I have been lifted!");
        Serial.println("    Acceleration: Z : "); Serial.print(stAccelRawData.s16Z);
      } while (zVal >= threshold);
    }
  } else if (hasBeenLifted == true) {
    hasBeenLifted = false;
    if(mood < 5) {
      mood++;
    }
  }
  
  //LDR Sensor needs to be adjusted too be less sensitive to light
  //Arbitrary value set for the if statement
  //Most I could get out of the reading was 20 - 28 with current config
  // if (hasBeenBlinded == false) {
  //   if (ldrSensorValue >= 600) {
  //     do
  //     {
  //       hasBeenBlided = true
  //       ldrSensorValue = analogRead(A0);
  //       Serial.println("It's too dark!");
  //     } while (ldrSensorValue >= 600); 
  //   }
  // } else if (hasBeenBlinded == true) {
  //   hasBeenBlinded = false;
  //   if(mood < 5) {
  //     mood++;
  //   }
  // }

  if (hasBeenRotated == false) {
    if (rollVal >= 90 || rollVal <= -90) {
      do
      {
        hasBeenRotated = true;
        imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        rollVal = stAngles.fRoll;
        Serial.println("I am upside down!");
        Serial.println("Roll : "); Serial.print(stAngles.fRoll);
      } while (rollVal >= 90 || rollVal <= -90);
    }
  } else if (hasBeenRotated == true) {
    hasBeenRotated = false;
    if(mood < 5) {
      mood++;
    }
  }

  Serial.println();
  Serial.println("/-------------------------------------------------------------/");
  Serial.print("Roll : "); Serial.print(stAngles.fRoll);
  Serial.print("    Pitch : "); Serial.print(stAngles.fPitch);
  Serial.print("    Yaw : "); Serial.print(stAngles.fYaw);
  Serial.println();
  Serial.print("Acceleration: X : "); Serial.print(stAccelRawData.s16X);
  Serial.print("    Acceleration: Y : "); Serial.print(stAccelRawData.s16Y);
  Serial.print("    Acceleration: Z : "); Serial.print(stAccelRawData.s16Z);
  Serial.println();
  Serial.print("Gyroscope: X : "); Serial.print(stGyroRawData.s16X);
  Serial.print("       Gyroscope: Y : "); Serial.print(stGyroRawData.s16Y);
  Serial.print("       Gyroscope: Z : "); Serial.print(stGyroRawData.s16Z);
  Serial.println();
}
