// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveAngle extends CommandBase {
  /** Creates a new AutoDriveAngle. */
  private DriveTrain driveTrain;
  private Pose2d desiredPose;
  private boolean isFinished = false;

  public AutoDriveAngle(DriveTrain driveTrain, Pose2d desiredPose) {
    this.driveTrain = driveTrain;
    this.desiredPose = desiredPose;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentX = this.driveTrain.getPose().getX();
    double currentY = this.driveTrain.getPose().getY();
    double currentRot = this.driveTrain.getPose().getRotation().getDegrees();

    if (Math.abs(currentRot - desiredPose.getRotation().getDegrees()) > 5){
      if(Math.abs(toAngle(currentRot, desiredPose.getRotation().getDegrees())) > 5){
        driveTrain.drive(0, -((toAngle(currentRot, desiredPose.getRotation().getDegrees()))/180) - (0.1 * toAngle(currentRot, desiredPose.getRotation().getDegrees()))/(Math.abs(toAngle(currentRot, desiredPose.getRotation().getDegrees()))));
      }
      else{
        driveTrain.drive(0,0);
      }
    }
    else{
      isFinished = true;
    }
  }

  private double toAngle(double angleBounded, double angleTarget){
    double ccwDifference;
    double cwDifference;
    if(angleBounded < angleTarget){
      ccwDifference = angleTarget - angleBounded;
      cwDifference = ccwDifference - 360;
    }
    else{
      cwDifference = angleTarget - angleBounded;
      ccwDifference = cwDifference - 360;
    }
    
    if(Math.abs(cwDifference) > Math.abs(ccwDifference)){
      return ccwDifference;
    }
    else{
      return cwDifference;
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
    return isFinished;
  }
}
