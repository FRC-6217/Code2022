// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveDistance extends CommandBase {
  /** Creates a new AutoDriveDistance. */
  private DriveTrain driveTrain;
  private Pose2d desiredPose;
  private double tangentAngle;
  private boolean isFinished = false;
  public AutoDriveDistance(DriveTrain driveTrain, Pose2d pose2d) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.desiredPose = pose2d;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentX = this.driveTrain.getPose().getX();
    double currentY = this.driveTrain.getPose().getY();

    tangentAngle = Math.toDegrees(Math.atan2(desiredPose.getY() - currentY, desiredPose.getX() - currentX));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentX = this.driveTrain.getPose().getX();
    double currentY = this.driveTrain.getPose().getY();
    double currentRot = this.driveTrain.getPose().getRotation().getDegrees();
    
    SmartDashboard.putNumber("tangAngle", tangentAngle);
    SmartDashboard.putNumber("AngleDifference", toAngle(currentRot, tangentAngle));

    if (Math.abs(currentY - desiredPose.getY()) > 0.1 || Math.abs(currentX - desiredPose.getX()) > 0.1){
      if(Math.abs(toAngle(currentRot, tangentAngle)) > 5){
        driveTrain.drive(0, -((toAngle(currentRot, tangentAngle))/180) - (0.1 * toAngle(currentRot, tangentAngle))/(Math.abs(toAngle(currentRot, tangentAngle))));
      }
      else{
        driveTrain.drive(-.3, 0);
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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
