// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.bad;

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
  public JoystickExtender(SingleMotorControl extender, Joystick joystick) {
    this.extender = extender;
    this.joystick = joystick;
    addRequirements(this.extender);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getRawButton(Constants.HANGER.EXTEND_INDEPENDENT_UP_BUTTON)){
      extender.turnOnForward();
    }
    else if (joystick.getRawButton(Constants.HANGER.EXTEND_INDEPENDENT_DOWN_BUTTON)){
      extender.turnOnReverse();
    }
    else {
      extender.turnOff();
    }
    
    SmartDashboard.putNumber("Extender Encoder position", extender.getPostion());
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
