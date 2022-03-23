// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.crypto.dsig.SignedInfo;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SingleMotorControl;
import frc.robot.subsystems.VelocityPID;

public class AutoFlapper extends CommandBase {
  public enum States{
    FIRE_LEFT,
    FIRE_RIGHT,
    IDLE
  }

  /** Creates a new AutoFlapper. */
  private SingleMotorControl leftFlapper, rightFlapper;
  private VelocityPID spinner;
  private Intake intake;
  private boolean fireLeft = false;
  private boolean fireRight = false;
  private States state;
  private double timeSinceFire = Double.POSITIVE_INFINITY;

  public AutoFlapper(SingleMotorControl leftFlapper, SingleMotorControl rightFlapper, VelocityPID spinner, Intake intake) {
    this.intake = intake;
    this.leftFlapper = leftFlapper;
    this.rightFlapper = rightFlapper;
    this.spinner = spinner;

    this.state = States.IDLE;
    
    addRequirements(intake);
    addRequirements(leftFlapper);
    addRequirements(rightFlapper);
    addRequirements(spinner);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spinner.turnOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state){
      case FIRE_LEFT:
        leftFlapper.turnOnForward();
        intake.setForward();
        
        if(!spinner.isAtSetpoint() || Timer.getFPGATimestamp() - timeSinceFire > 1.0){
          fireLeft = true;
          state = States.IDLE;
        }
        break;
      case FIRE_RIGHT:
        rightFlapper.turnOnForward();
        intake.setForward();

        if(!spinner.isAtSetpoint() || Timer.getFPGATimestamp() - timeSinceFire > 1.0){
          fireRight = true;
          state = States.IDLE;
        }
        break;
      case IDLE:
        leftFlapper.turnOff();
        rightFlapper.turnOff();
        intake.setOff();
        if(!fireLeft && spinner.isAtSetpoint()){
          timeSinceFire = Timer.getFPGATimestamp();
          state = States.FIRE_LEFT;
        }
        else if (!fireRight && spinner.isAtSetpoint()){
          timeSinceFire = Timer.getFPGATimestamp();
          state = States.FIRE_RIGHT;
        }
        break;
    }

    SmartDashboard.putString("State of Shooter", state.toString());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinner.turnOff();
    leftFlapper.turnOff();
    rightFlapper.turnOff();
    intake.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fireRight && fireLeft;
  }
}
