// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HANGER;
import frc.robot.subsystems.PositionPID;

public class extenderPIDCommand extends CommandBase {
  private PositionPID p;
  private Joystick j;
  /** Creates a new extenderPIDCommand. */
  public extenderPIDCommand(PositionPID extender, Joystick j) {
    addRequirements(extender);
    this.p = extender;
    this.j = j;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (j.getRawButton(HANGER.EXTEND_BUTTON)) {
      p.turnOn();
    }
    else {
      p.turnOff();
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
