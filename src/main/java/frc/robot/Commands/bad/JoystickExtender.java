// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.bad;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SingleMotorControl;

public class JoystickExtender extends CommandBase {
  /** Creates a new JoystickExtender. */
  private SingleMotorControl extender;
  private Joystick joystick;
  private double maxEncoder;
  private double minEncoder;
  public JoystickExtender(SingleMotorControl extender, Joystick joystick) {
    this.extender = extender;
    this.joystick = joystick;
    addRequirements(this.extender);
    SmartDashboard.putNumber("Max Extender Encoder", 10);
    SmartDashboard.putNumber("Min Extender Encoder", 0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    maxEncoder = SmartDashboard.getNumber("Max Extender Encoder", 10);
    minEncoder = SmartDashboard.getNumber("Min Extender Encoder", 0);
    if(joystick.getRawButton(Constants.HANGER.EXTEND_INDEPENDENT_UP_BUTTON)){// && -extender.getPostion() < maxEncoder){
      extender.turnOnForward();
    }
    else if (joystick.getRawButton(Constants.HANGER.EXTEND_INDEPENDENT_DOWN_BUTTON)){ //&& -extender.getPostion() > minEncoder){
      extender.turnOnReverse();
    }
    else {
      extender.turnOff();
    }
    SmartDashboard.putBoolean("Button", joystick.getRawButton(Constants.HANGER.EXTEND_INDEPENDENT_UP_BUTTON));
    SmartDashboard.putNumber("Extender Encoder", extender.getPostion());
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
