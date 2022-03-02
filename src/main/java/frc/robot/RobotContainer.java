// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.VelocityPID;
import frc.robot.subsystems.PositionPID;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flapper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SingleMotorControl;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.bad.JoystickFlapper;
import frc.robot.commands.bad.JoystickIntake;
import frc.robot.commands.bad.JoystickShooter;
import frc.robot.commands.JoystickBallHandeler;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  //private final Flapper left_flapper = new Flapper(23);
  private final PositionPID leftFlapper = new PositionPID("leftFlapper", 23, 0, 0, 0);
  private final PositionPID rightFlapper = new PositionPID("rightFlapper", 22, 0, 0, 0);
  private final Joystick driveStick = new Joystick(0);
  private final XboxController xbox = new XboxController(1);
  private final VelocityPID spinner = new VelocityPID("spinner", 20);
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, new JoystickDrive(driveTrain, driveStick));
    CommandScheduler.getInstance().setDefaultCommand(spinner, new JoystickBallHandeler(leftFlapper, rightFlapper, spinner, intake, xbox));
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
