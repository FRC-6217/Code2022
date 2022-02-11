// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.PIDCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystem.CustomPID;
import frc.robot.subsystem.CustomPIDPosition;
import frc.robot.subsystem.PIDMotorControl;
import frc.robot.subsystem.SingleMotorControl;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   /* 
    SingleMotorControl intake = new SingleMotorControl(28, MotorType.kBrushless, 1, .05);
    SingleMotorControl shooter = new SingleMotorControl(29, MotorType.kBrushless, 1, .05);
    */
    XboxController xboxController = new XboxController(0);
    CustomPID shooter = new CustomPID("practice", 29);
    // Configure the button bindings
    CommandScheduler.getInstance().setDefaultCommand(shooter, new PIDCommand(shooter, xboxController));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command m_autoCommand = null;
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
