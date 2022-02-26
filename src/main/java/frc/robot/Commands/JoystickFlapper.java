// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flapper;

public class JoystickFlapper extends CommandBase {
  /** Creates a new JoystickFlapper. */
  private Flapper leftFlapper;
  private XboxController xbox;
  public JoystickFlapper(Flapper leftFlapper, XboxController xbox) {
    this.leftFlapper = leftFlapper;
    this.xbox = xbox;
    addRequirements(leftFlapper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("joy y", xbox.getLeftY());
    if(xbox.getLeftY() < -0.5){
      leftFlapper.setForward();
    }
    else if (xbox.getLeftY() > 0.5){
      leftFlapper.setReverse();
    }
    else{
      leftFlapper.setOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftFlapper.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
