// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TeleClimb extends CommandBase {
  /** Creates a new TeleClimb. */
  private Climber climber;
  private XboxController xbox;
  public TeleClimb(Climber climber, XboxController xbox) {
    this.climber = climber;
    this.xbox = xbox;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xbox.getLeftTriggerAxis() > 0.5){
      climber.winchUpUp();
    }
    else if (xbox.getLeftBumperPressed()){
      climber.winchUpDown();
    }
    else{
      climber.winchUpOff();
    }
    if(xbox.getRightTriggerAxis() > 0.5){
      climber.winchDownUp();
    }
    else if (xbox.getRightBumperPressed()){
      climber.winchDownDown();
    }
    else{
      climber.winchDownOff();
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
