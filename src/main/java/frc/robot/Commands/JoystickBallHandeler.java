// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PositionPID;
import frc.robot.subsystems.VelocityPID;

public class JoystickBallHandeler extends CommandBase {
  /** Creates a new JoystickFLapperPID. */
  private PositionPID leftFlapper,rightFlapper;
  private VelocityPID spinner;
  private Intake intaker;
  private XboxController xbox;
  boolean isIntakeOn = false;
  public JoystickBallHandeler(PositionPID leftFlapper,PositionPID rightFlapper,VelocityPID spinner, Intake intaker, XboxController xbox){
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

  // Called every time the scheduler runs while the command is scheduled.
  private void executeIntaker(){
    if(xbox.getRightBumperPressed()){
      isIntakeOn = !isIntakeOn;
    }

    //If the xbox trigger is press, reverse
    if (xbox.getRightTriggerAxis() > 0.5){
      intaker.setBackward();
      isIntakeOn = false;//TODO Figuare this out
    }
    //If isOn then forward
    else if(isIntakeOn){
      intaker.setForward();
    }
    //Else off
    else {
      intaker.setOff();
    }
  }
  public void execute() {
    boolean isSpinnerUpToSpeed = false;


    boolean isUserToggledSpinner = xbox.getYButtonPressed();
    boolean isUserFlappingLeft = xbox.getXButton() || xbox.getAButton();
    boolean isUserFlappingRight = xbox.getBButton() || xbox.getAButton();
    boolean isUserToggleIntakerForward = xbox.getRightBumperPressed();
    boolean isUserIntakingReverse = xbox.getRightTriggerAxis() > 0.5;

    //decide inputs priorities

    if (!isSpinnerUpToSpeed){
      isUserFlappingLeft = false;
      isUserFlappingRight = false;
    }
    if (isUserFlappingRight || isUserFlappingLeft){
      isUserIntakingReverse = false;
    }


    //Do motor stuff

    if (isUserToggledSpinner){
      if(spinner.getMotorState()){
        spinner.turnOff();
      }
      else {
        spinner.turnOn();
      }
    }
    

    executeIntaker();
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
