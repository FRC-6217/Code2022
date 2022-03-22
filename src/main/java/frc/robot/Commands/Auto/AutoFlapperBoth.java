// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SingleMotorControl;
import frc.robot.subsystems.VelocityPID;

public class AutoFlapperBoth extends CommandBase {
  /** Creates a new AutoFlapperBoth. */
  private SingleMotorControl leftFlapper, rightFlapper;
  private Intake intake;
  private VelocityPID spinner;

  private boolean firstShot;
  private boolean secondShot;
  private double timeStampStartFirst;
  private double timeStampEndFirst;
  private double timeStampStartSecond;

  private boolean isDone;

  public AutoFlapperBoth(SingleMotorControl leftFlapper, SingleMotorControl rightFlapper, Intake intake, VelocityPID spinner) {
    addRequirements(leftFlapper);
    addRequirements(rightFlapper);
    addRequirements(spinner);
    addRequirements(intake);

    this.leftFlapper = leftFlapper;
    this.rightFlapper = rightFlapper;
    this.spinner = spinner;
    this.intake = intake;

    firstShot = true;
    secondShot = false;
    timeStampStartFirst = Double.POSITIVE_INFINITY;
    timeStampEndFirst = Double.POSITIVE_INFINITY;
    timeStampStartSecond = Double.POSITIVE_INFINITY;

    isDone = false;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(spinner.isAtSetpoint()){
      if(firstShot){
        leftFlapper.turnOnForward();
        intake.setSlow();
        firstShot = false;
        timeStampStartFirst = Timer.getFPGATimestamp();
      }
      else if(Timer.getFPGATimestamp() - timeStampStartFirst > 1 && timeStampStartFirst != Double.POSITIVE_INFINITY){
        intake.setOff();
        leftFlapper.turnOff();
        timeStampEndFirst = Timer.getFPGATimestamp();
        secondShot = true;
      }
      else if (secondShot && Timer.getFPGATimestamp() - timeStampEndFirst > 1 && timeStampEndFirst != Double.POSITIVE_INFINITY){
        rightFlapper.turnOnForward();
        intake.setSlow();
        secondShot = false;
        timeStampStartSecond = Timer.getFPGATimestamp();
      }
      else if(Timer.getFPGATimestamp() - timeStampStartSecond > 1 && timeStampStartSecond != Double.POSITIVE_INFINITY){
        intake.setOff();
        rightFlapper.turnOff();
        isDone = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setOff();
    rightFlapper.turnOff();
    leftFlapper.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
