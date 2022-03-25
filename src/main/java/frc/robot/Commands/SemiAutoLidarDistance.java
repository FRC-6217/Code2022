// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.sensors.Lidar;

public class SemiAutoLidarDistance extends CommandBase {
  /** Creates a new SemiAutoLidarDistance. */
  private Lidar lidar;
  private DriveTrain driveTrain;
  private double distanceInches;
  public SemiAutoLidarDistance(DriveTrain driveTrain, Lidar lidar, double distanceInches) {
    this.driveTrain = driveTrain;
    this.lidar = lidar;
    this.distanceInches = distanceInches;
    addRequirements(driveTrain);
    addRequirements(lidar);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(lidar.getDistanceInInches() - distanceInches > 1){
      driveTrain.drive(.3,0);
    }
    else if (lidar.getDistanceInInches() - distanceInches < -1){
      driveTrain.drive(-0.3, 0);
    }
    else{
      driveTrain.drive(0,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
