// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.CustomPID;
import frc.robot.subsystem.CustomPIDPosition;

public class PIDCommand extends CommandBase {
  /** Creates a new PIDCommand. */
  private CustomPIDPosition shooterSubsystem;
  private XboxController xboxController;

  public PIDCommand(CustomPIDPosition shooterSubsystem, XboxController xboxController) {
    this.shooterSubsystem = shooterSubsystem;
    this.xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (xboxController.getAButton()) {
      shooterSubsystem.turnOn();
    }
    else {
      shooterSubsystem.turnOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
