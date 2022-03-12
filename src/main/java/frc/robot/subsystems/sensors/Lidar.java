// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



// https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf
package frc.robot.subsystems.sensors;

import org.ejml.interfaces.decomposition.DecompositionSparseInterface;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LIDAR;
import frc.robot.Constants.CONVERSIONS;

public class Lidar extends SubsystemBase {
  /** Creates a new Lidar. */
  private short distance = 0;
  private I2C i2c;

  enum ENABLE {OFF, ON};
  enum STATE {IDLE, START_MEASUREMENT, WAIT, GET_MEASUREMENT};
  ENABLE enable = ENABLE.ON;
  STATE state = STATE.START_MEASUREMENT;
  byte buffer[] = new byte[2];

  public Lidar() {
     i2c = new I2C(I2C.Port.kMXP, LIDAR.I2C_ADDRESS);


     // write default settings?
     // i2c.write(0x02,0x80); 
     // i2c.write(0x04,0x08); 
     // i2c.write(0x1c,0x00); 
  }

  @Override
  public void periodic() {
    switch (state) {
      case IDLE:
        if (ENABLE.ON == enable) {
          state = STATE.START_MEASUREMENT;
        } else {
          state = STATE.IDLE;
        }
        break;
      case START_MEASUREMENT:
        i2c.write(LIDAR.ACQ_COMMAND_REG, LIDAR.MEASURE_COMMAND);
        state = STATE.WAIT;
        break;
      case WAIT:
        i2c.read(LIDAR.STATUS_REG, 1, buffer);
        if ((buffer[0] & 0x01) == 0x00) {
          // wait for LSB to go low
          state = STATE.GET_MEASUREMENT;
        }
        else {
          state = STATE.WAIT;
        }
        break;
      case GET_MEASUREMENT:
        i2c.read(LIDAR.READ_REG, 2, buffer);
        distance =  (short) (((int) buffer[0] << 0x8) | (int) buffer[1]);
        state = STATE.START_MEASUREMENT;
        break;
    }
  }
  public int getDistanceInCentimeters() {
    return distance;
  }

  public double getDistanceInInches() {
    return distance * CONVERSIONS.CENTIMETER_TO_INCHES;
  }

  public void enable() {
    enable = ENABLE.ON;
  }
  public void disable() {
    enable = ENABLE.OFF;
  }
}
