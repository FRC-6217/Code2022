// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LIDAR;

public class Lidar extends SubsystemBase {
  /** Creates a new Lidar. */
  private short distance = 0;
  private I2C i2c;

  private byte BYTE_COUNT = 1;

  enum ENABLE {OFF, ON};
  enum STATE {IDLE, START_MEASUREMENT, WAIT, GET_MEASUREMENT};
  ENABLE enable = ENABLE.ON;
  STATE state = STATE.START_MEASUREMENT;
  byte buffer[] = new byte[2];
  public Lidar() {
     i2c = new I2C(I2C.Port.kOnboard, LIDAR.I2C_ADDRESS);
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
        i2c.read(LIDAR.FULL_DELAY_HIGH_REG, BYTE_COUNT, buffer);
        short high = buffer[0];
        i2c.read(LIDAR.FULL_DELAY_LOW_REG, BYTE_COUNT, buffer);
        byte low = buffer[0];
        distance =  (short) (((int) high << 0x8) | (int) low);
        state = STATE.START_MEASUREMENT;
        System.out.println("lidar: " + distance);
        break;
    }
    // This method will be called once per scheduler run
  }
  public short getDistanceCM() {
    return distance;
  }

  public void enable() {
    enable = ENABLE.ON;
  }
  public void disable() {
    enable = ENABLE.OFF;
  }
}
