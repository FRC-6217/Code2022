// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class JoystickIntake extends CommandBase {
  /** Creates a new JoystickIntake. */
  private Intake intake;
  private XboxController xbox;
  private boolean isOn = false;

  public JoystickIntake(Intake intake, XboxController xbox) {
    this.intake = intake;
    this.xbox = xbox;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //If the button is press, toggle the state
    if(xbox.getRightBumperPressed()){
      isOn = !isOn;
    }

    //If the xbox trigger is press, reverse
    if (xbox.getRightTriggerAxis() > 0.5){
      intake.setBackward();
      isOn = false;//TODO Figuare this out
    }
    //If isOn then forward
    else if(isOn){
      intake.setForward();
    }
    //Else off
    else {
      intake.setOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
