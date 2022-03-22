// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VelocityPID;

public class AutoDriveForwardDistance extends CommandBase {
  /** Creates a new AutoDriveWeekZero. */
  private DriveTrain driveTrain;
  private double distance;
  private Intake intake;
  private double currentEncoder;

  public AutoDriveForwardDistance(DriveTrain driveTrain, Intake intake, VelocityPID spinner, double distance) {
    this.driveTrain = driveTrain;
    this.distance = distance;
    this.intake = intake;
    addRequirements(driveTrain);
    addRequirements(intake);
    addRequirements(spinner);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentEncoder = driveTrain.getLeftEncoderPosition();
    // driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setForward();
    driveTrain.drive(-.4, 0);
    SmartDashboard.putNumber("DriveTrain", driveTrain.getLeftEncoderPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setOff();
    driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.getLeftEncoderPosition() > currentEncoder + distance;
  }
}
