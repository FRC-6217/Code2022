// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {

// MODE needs to match case statement on arduino
  enum MODE {OFF, RED, BLUE, WHITE}; //Add more 
  MODE current_mode = MODE.OFF;

  SerialPort serialPort;
  private byte[] write_buffer = new byte[1];
  private final int baud_rate = 9600;
  private final int TRANSFER_SIZE = 1;
  String dashboardName = "ledColor";
  String dashboardDefault = "OFF";

  /** Creates a new LEDController. */
  public LEDController() {
    serialPort = new SerialPort(baud_rate, SerialPort.Port.kMXP);
    CommandScheduler.getInstance().registerSubsystem(this);
    SmartDashboard.putString(dashboardName, dashboardDefault);
  }

  @Override
  public void periodic() {
    String string = SmartDashboard.getString(dashboardName, dashboardDefault);
    MODE new_mode = MODE.valueOf(string); // todo try catch
    if (new_mode != current_mode) {
      set(new_mode);
      current_mode = new_mode;
    }
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
    serialPort.write(write_buffer, TRANSFER_SIZE);
  }
}
