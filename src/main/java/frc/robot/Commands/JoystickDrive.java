// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.sensors.Lidar;

public class JoystickDrive extends CommandBase {
  /** Creates a new JoystickDrive. */
  private DriveTrain driveTrain;
  private Joystick joy;
  private Lidar lidar;

  

  public JoystickDrive(DriveTrain driveTrain, Joystick joy, Lidar lidar) {
    this.joy = joy;
    this.driveTrain = driveTrain;
    this.lidar = lidar;
    addRequirements(this.driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gov = (1-joy.getRawAxis(3))/2;
    double xSpeed = (Math.abs(joy.getRawAxis(1)) < 0.3) ? 0.0 : (joy.getRawAxis(1) * gov);
    double rot = (Math.abs(joy.getRawAxis(2)) < 0.3) ? 0.0 : (joy.getRawAxis(2) * gov);

    driveTrain.drive(xSpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
