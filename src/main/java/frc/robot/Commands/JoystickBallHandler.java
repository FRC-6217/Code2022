// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PositionPID;
import frc.robot.subsystems.SingleMotorControl;
import frc.robot.subsystems.VelocityPID;

public class JoystickBallHandler extends CommandBase {
  /** Creates a new JoystickFLapperPID. */
  private SingleMotorControl leftFlapper,rightFlapper;
  private VelocityPID spinner;
  private Intake intaker;
  private XboxController xbox;
  boolean isIntakeOn = false;
  public JoystickBallHandler(SingleMotorControl leftFlapper, SingleMotorControl rightFlapper,VelocityPID spinner, Intake intaker, XboxController xbox){
    this.leftFlapper = leftFlapper;
    this.xbox = xbox;
    this.intaker = intaker;
    this.rightFlapper = rightFlapper;
    this.spinner = spinner;
    addRequirements(this.leftFlapper);
    addRequirements(this.rightFlapper);
    addRequirements(this.spinner);
    addRequirements(this.intaker);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void execute() {
    boolean isSpinnerUpToSpeed = spinner.isAtSetpoint();


    boolean isUserToggledSpinner = xbox.getYButtonPressed();
    boolean isUserFlappingLeft = xbox.getXButton() || xbox.getAButton();
    boolean isUserFlappingRight = xbox.getBButton() || xbox.getAButton();
    boolean isUserToggleIntakerForward = xbox.getRightBumperPressed();
    boolean isUserIntakingReverse = xbox.getRightTriggerAxis() > 0.5;

    //decide inputs priorities

    // if (!isSpinnerUpToSpeed){
    //   isUserFlappingLeft = false;
    //   isUserFlappingRight = false;
    // }

      SmartDashboard.putBoolean("Spinner", isSpinnerUpToSpeed);
    //Do motor stuff
    //Spinner motor stuff
    if (isUserToggledSpinner){
      if(spinner.getMotorState()){
        spinner.turnOff();
      }
      else {
        spinner.turnOn();
      }
    }

    //Intaker motor stuff
    if (isUserFlappingRight || isUserFlappingLeft){
      intaker.setSlow();
    }
    else if (intaker.getMotorState() == Intake.MotorState.SLOW){
      intaker.setOff();
    }
    else if (isUserIntakingReverse){
      intaker.setReverse();
    }
    else if(intaker.getMotorState() == Intake.MotorState.REVERSE){
      intaker.setOff();
    }
    else if (isUserToggleIntakerForward){
      if(intaker.getMotorState() == Intake.MotorState.FAST){
        intaker.setOff();
      }
      else{
        intaker.setForward();
      }
    }

    //Flapper motor stuff
    if(isUserFlappingRight){
      rightFlapper.turnOnForward();
    }
    else{
      rightFlapper.turnOff();
    }

    if(isUserFlappingLeft){
      leftFlapper.turnOnForward();
    }
    else{
      leftFlapper.turnOff();
    }

    if(xbox.getLeftBumperPressed()){
      spinner.increaseSetpoint();
    }
    else if (xbox.getStartButtonPressed()){
      spinner.decreaseSetpoint();
    }
    else if (xbox.getBackButton()){
      spinner.clearSpinnerOffset();
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
