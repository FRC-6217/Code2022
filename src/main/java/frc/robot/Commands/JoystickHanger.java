// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SingleMotorControl;

public class JoystickHanger extends CommandBase {
  /** Creates a new JoystickHanger. */

  //TODO explain what each state is. Sucks to be you future me
  public enum HangerState{
    TELE,
    DEEXTENDED,
    EXTENDING,
    DEEXTENDING,
    STALLEDEX,
    EXTENDED,
    HOOKING,
    STALLEDHOOK,
    HOOKED,
    HANGING,
    DEHANGING,
    STALLEDHANG,
    HANGED
  }

  private HangerState currentState;
  private Joystick joystick;
  private SingleMotorControl extender;
  private SingleMotorControl winch;

  public JoystickHanger(SingleMotorControl extender, SingleMotorControl winch, Joystick joystick) {
    currentState = HangerState.TELE;
    this.joystick = joystick;
    this.extender = extender;
    this.winch = winch;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isUserInit = joystick.getRawButton(Constants.HANGER.INIT_BUTTON_1) && joystick.getRawButton(Constants.HANGER.INIT_BUTTON_2);
    boolean isUserExtending = joystick.getRawButton(Constants.HANGER.EXTEND_BUTTON);
    boolean isUserDeextending = joystick.getRawButton(Constants.HANGER.DEEXTEND_BUTTON);
    boolean isUserHooking = joystick.getRawButton(Constants.HANGER.HOOKING_BUTTON);
    boolean isUserHanging = joystick.getRawButton(Constants.HANGER.HANG_BUTTON);
    boolean isUserDehanging = joystick.getRawButton(Constants.HANGER.DEHANG_BUTTON);

    switch(currentState){
      case TELE:
        if(isUserInit){
          currentState = HangerState.DEEXTENDED;
        }
        break;

      case DEEXTENDED:
        if(isUserExtending){
          currentState = HangerState.EXTENDING;
        }
        break;

      case EXTENDING:
        if(!isUserExtending){
          extender.turnOff();
          currentState = HangerState.STALLEDEX;
        }
        else if(extender.getPostion() < Constants.HANGER.EXTENDER_MAX_POSITION){
          extender.turnOnForward();
          currentState = HangerState.EXTENDING;
        }
        else{
          extender.turnOff();
          currentState = HangerState.EXTENDED;
        }
        break;

      case DEEXTENDING:
        if(!isUserDeextending){
          extender.turnOff();
          currentState = HangerState.STALLEDEX;
        }
        else if(extender.getPostion() > Constants.HANGER.EXTENDER_MIN_POSITION){
          extender.turnOnReverse();
          currentState = HangerState.DEEXTENDING;
        }
        else{
          extender.turnOff();
          currentState = HangerState.DEEXTENDED;
        }
        break;

      case STALLEDEX:
        if(isUserExtending){
          currentState = HangerState.EXTENDING;
        }
        else if (isUserDeextending){
          currentState = HangerState.DEEXTENDING;
        }
        break;

      case EXTENDED:
        if(isUserHanging){
          currentState = HangerState.HOOKING;
        }
        else if (isUserDeextending){
          currentState = HangerState.DEEXTENDING;
        }
        break;
      case HOOKING:
        if(!isUserHooking){
          extender.turnOff();
          currentState = HangerState.STALLEDHOOK;
        }
        else if(extender.getPostion() > Constants.HANGER.EXTENDER_HOOK_POSITION){
          extender.turnOnReverse();
          currentState = HangerState.HOOKING;
        }
        else{
          extender.turnOff();
          currentState = HangerState.HOOKED;
        }
        break;
      case STALLEDHOOK:
        if(isUserHooking){
          currentState = HangerState.HOOKING;
        }
        break;
      case HOOKED:
        if(isUserHanging){
          currentState = HangerState.HANGING;
        }
        break;

      case HANGING:
        if(!isUserHanging){
          winch.turnOff();
          currentState = HangerState.STALLEDHANG;
        }
        else if(winch.getPostion() < Constants.HANGER.WINCH_MAX_POSITION){
          extender.turnOnForward();
          currentState = HangerState.HANGING;
        }
        else{
          extender.turnOff();
          currentState = HangerState.HANGED;
        }
        break;

      case DEHANGING:
        if(!isUserDehanging){
          winch.turnOff();
          currentState = HangerState.STALLEDHANG;
        }
        else if(winch.getPostion() > Constants.HANGER.WINCH_MIN_POSITION){
          extender.turnOnReverse();
          currentState = HangerState.DEHANGING;
        }
        else{
          extender.turnOff();
          currentState = HangerState.HOOKED;
        }
        break;

      case STALLEDHANG:
        if(isUserHanging){
          currentState = HangerState.HANGING;
        }
        else if (isUserDehanging){
          currentState = HangerState.DEHANGING;
        }
        break;

      case HANGED:
        if (isUserDehanging){
          currentState = HangerState.DEHANGING;
        }
        break;

      default:
        //cry
        break;
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
