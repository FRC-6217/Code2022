// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {


  enum MODE {OFF, RED, BLUE, WHITE}; //Add more 
  MODE current_mode = MODE.OFF;
  SerialPort serialPort;
  private byte[] write_buffer = new byte[1];

  /** Creates a new LEDController. */
  public LEDController() {
    serialPort = new SerialPort(9600, SerialPort.Port.kMXP);
    CommandScheduler.getInstance().registerSubsystem(this);
    SmartDashboard.putString("ledColor", "OFF");
  }

  @Override
  public void periodic() {
    String string = SmartDashboard.getString("ledColor ", "OFF");
    MODE new_mode = MODE.valueOf(string); // todo try catch
    if (new_mode != current_mode) {
      set(new_mode);
      current_mode = new_mode;
    }

    // This method will be called once per scheduler run
  }

  public void set(Alliance alliance) {
    switch (alliance) {
      case Red:
        current_mode = LEDController.MODE.RED;
        break;
      case Blue:
        current_mode = LEDController.MODE.BLUE;
        break;
      case Invalid:
        current_mode = LEDController.MODE.OFF;
        break;
    }

    set(current_mode);
  }


  public void set(MODE mode) {
    write_buffer[0] = (byte) mode.ordinal();
    serialPort.write(write_buffer, 1);
  }
}
