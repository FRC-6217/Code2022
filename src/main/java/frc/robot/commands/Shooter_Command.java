// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter_2;

public class Shooter_Command extends CommandBase {
  /** Creates a new Shooter_Command. */
  Shooter_2 shooter;
  XboxController joystick;
  public Shooter_Command(Shooter_2 shooter, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(shooter);
      this.shooter = shooter;
      this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getAButton()){
      shooter.on();
    }
    else{
      shooter.off();
    }
    if (joystick.getXButtonPressed()){
      shooter.speedup();
    } else if (joystick.getYButtonPressed()){
      shooter.speeddown();
    } else if (joystick.getStartButtonPressed()){
      shooter.reverse();
    }
    SmartDashboard.putNumber("Speed", shooter.getSpeed());
    SmartDashboard.putBoolean("On", shooter.getS());
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
