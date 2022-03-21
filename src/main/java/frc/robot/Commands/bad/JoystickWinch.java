// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.bad;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SingleMotorControl;

public class JoystickWinch extends CommandBase {
  /** Creates a new JoystickWinch. */
  private SingleMotorControl winch;
  private XboxController xbox;

  public JoystickWinch(SingleMotorControl winch, XboxController xbox) {
    this.winch = winch;
    this.xbox = xbox;
    addRequirements(this.winch);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xbox.getRightY() < -0.5){
      winch.turnOnForward();
    }
    else if(xbox.getRightY() > 0.5){
      winch.turnOnReverse();
    }
    else {
      winch.turnOff();
    }
    SmartDashboard.putNumber("Winch Encoder position", winch.getPostion());
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
