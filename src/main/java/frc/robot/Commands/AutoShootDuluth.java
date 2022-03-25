// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SingleMotorControl;
import frc.robot.subsystems.VelocityPID;

public class AutoShootDuluth extends CommandBase {
  /** Creates a new AutoShootDuluth. */
  private enum AutoStates{
    SPEEDING,
    SHOOTING,
    DRIVING
  }

  private AutoStates prevState;
  private AutoStates currentState;
  private AutoStates nextState;
  private DriveTrain driveTrain;
  private Intake intake;
  private SingleMotorControl leftFlapper;
  private SingleMotorControl rightFlapper;
  private VelocityPID spinner;
  private double startTime;
  private double flapperTime;
  public AutoShootDuluth(DriveTrain driveTrain, VelocityPID spinner, Intake intake, SingleMotorControl leftFlapper, SingleMotorControl rightFlapper) {
    this.intake = intake;
    this.leftFlapper = leftFlapper;
    this.rightFlapper = rightFlapper; 
    this.driveTrain = driveTrain;
    this.spinner = spinner;

    this.flapperTime = 0;


    addRequirements(driveTrain);
    addRequirements(intake);
    addRequirements(leftFlapper);
    addRequirements(rightFlapper);
    addRequirements(spinner);
    driveTrain.resetEncoders();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    this.prevState = AutoStates.SPEEDING;
    this.currentState = AutoStates.SPEEDING;
    this.nextState = AutoStates.SPEEDING;
    spinner.setSetpoint(1700);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(currentState){
      case DRIVING:
        driveTrain.drive(.4, 0);
        spinner.turnOff();
        nextState = AutoStates.DRIVING;
        break;
      case SHOOTING:
        if (Math.abs(Timer.getFPGATimestamp() - flapperTime) > 2){
          leftFlapper.turnOff();
          rightFlapper.turnOff();
          intake.setOff();
          nextState = AutoStates.DRIVING;
          driveTrain.resetEncoders();
        }
        else{
          leftFlapper.turnOnForward();
          rightFlapper.turnOnForward();
          intake.setSlow();
          nextState = AutoStates.SHOOTING;
        }
        break;
      case SPEEDING:
        spinner.turnOn();
        if(spinner.isAtSetpoint() && Math.abs(Timer.getFPGATimestamp() - startTime) > 4){
          nextState = AutoStates.SHOOTING;
          flapperTime = Timer.getFPGATimestamp();
        }

        else if (Math.abs(Timer.getFPGATimestamp() - startTime) > 10){
          spinner.turnOff();
          nextState = AutoStates.DRIVING;
        }
        else {
          nextState = AutoStates.SPEEDING;
        }
        break;
      default:
      //cry
        break;

    }
    prevState = currentState;
    currentState = nextState;
    SmartDashboard.putNumber("FlapperTime",flapperTime);
    SmartDashboard.putNumber("time",Timer.getFPGATimestamp());
    SmartDashboard.putString("PrevStateAuto", prevState.name());
    SmartDashboard.putString("CurrStateAuto", currentState.name());
    SmartDashboard.putString("NextStateAuto", nextState.name());
    SmartDashboard.putNumber("DriveTrain", driveTrain.getLeftEncoderPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
    driveTrain.resetEncoders();
    spinner.turnOff();
    leftFlapper.turnOff();
    rightFlapper.turnOff();
    intake.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return -driveTrain.getLeftEncoderPosition() > 2;

  }
}