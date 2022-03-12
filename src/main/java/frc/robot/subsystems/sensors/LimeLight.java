// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {

    NetworkTable table;
  double x, y, area;
  /** Creates a new LimeLight. */
  public LimeLight() {

  table = NetworkTableInstance.getDefault().getTable("limelight");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = table.getEntry("tx").getDouble(0.0);
    y = table.getEntry("ty").getDouble(0.0);
    area = table.getEntry("ta").getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  // todo ..return distance

  double getX() {
    return x;
  }
  
  double getY() {
    return y;
  }

}
