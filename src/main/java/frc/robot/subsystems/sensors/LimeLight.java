// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {

  NetworkTable table;
  NetworkTableEntry  x, y, area, valid;

  public enum PiplineID{
    RedBall,
    BlueBall,
    Range
  }
  /** Creates a new LimeLight. */
  public LimeLight(String name) {
    table = NetworkTableInstance.getDefault().getTable("limelight-" + name);
    x = table.getEntry("tx");
    y = table.getEntry("ty");
    area = table.getEntry("ta");
    valid = table.getEntry("tv");
    if(DriverStation.getAlliance() == Alliance.Red){
      setPipline(LimeLight.PiplineID.RedBall);
    }
    else{
      setPipline(LimeLight.PiplineID.BlueBall);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("LimelightX", x.getDouble(0));
    SmartDashboard.putNumber("LimelightY", y.getDouble(0));
    SmartDashboard.putNumber("LimelightArea", area.getDouble(0));
    SmartDashboard.putNumber("LimelightValid", valid.getDouble(0));
  }

  // todo ..return distance

  public double getX() {
    return x.getDouble(0);
  }
  
  public double getY() {
    return y.getDouble(0);
  }

  public boolean getValid() {
    return valid.getDouble(0) == 1;
  }



  public void setPipline(PiplineID piplineID){
    switch(piplineID){
      case BlueBall:
        table.getEntry("pipline").setNumber(0);
      case RedBall:
        table.getEntry("pipline").setNumber(1);
      case Range:
        table.getEntry("pipline").setNumber(0);
    }
  }

}
