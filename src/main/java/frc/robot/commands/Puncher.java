// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Piston;

public class Puncher extends CommandBase { 
  private Piston puncher;
  private int state;
  /** Creates a new puncher. */
  public Puncher(Piston p, int state) {
    this.puncher = p;
    this.state = state;
    addRequirements(this.puncher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state == 1){
      puncher.forwardPiston1();
    } 
    else if (state == 2){
      puncher.reversePiston1();
    }
    else {
      puncher.offPiston1();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
