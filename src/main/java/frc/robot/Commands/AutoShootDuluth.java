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
  private Timer timer;
  public AutoShootDuluth(DriveTrain driveTrain, VelocityPID spinner, Intake intake, SingleMotorControl leftFlapper, SingleMotorControl rightFlapper) {
    this.intake = intake;
    this.leftFlapper = leftFlapper;
    this.rightFlapper = rightFlapper; 
    this.driveTrain = driveTrain;
    this.spinner = spinner;

    this.timer = new Timer();
    this.startTime = timer.get();
    this.flapperTime = 0;

    this.prevState = AutoStates.SPEEDING;
    this.currentState = AutoStates.SPEEDING;
    this.nextState = AutoStates.SPEEDING;
    addRequirements(driveTrain);
    addRequirements(intake);
    addRequirements(leftFlapper);
    addRequirements(rightFlapper);
    addRequirements(spinner);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(currentState){
      case DRIVING:
        driveTrain.drive(.4, 0);
        nextState = AutoStates.DRIVING;
        break;
      case SHOOTING:
        if (Math.abs(timer.get() - startTime) > 3){
          leftFlapper.turnOff();
          rightFlapper.turnOff();
          intake.setOff();
          nextState = AutoStates.DRIVING;
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

        if(spinner.isAtSetpoint()){
          nextState = AutoStates.SHOOTING;
        }

        else if (Math.abs(timer.get() - startTime) > 10){
          spinner.turnOff();
          nextState = AutoStates.DRIVING;
          flapperTime = timer.get();
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
    SmartDashboard.putString("PrevStateAuto", prevState.name());
    SmartDashboard.putString("CurrStateAuto", currentState.name());
    SmartDashboard.putString("NextStateAuto", nextState.name());
    SmartDashboard.putNumber("DriveTrain", driveTrain.getLeftEncoderPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return -driveTrain.getLeftEncoderPosition() > 1.5;
  }
}