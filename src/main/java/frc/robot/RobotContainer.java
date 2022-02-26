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
import frc.robot.commands.JoystickFlapper;
import frc.robot.commands.JoystickIntake;
import frc.robot.commands.JoystickShooter;
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
  private final Flapper left_flapper = new Flapper(22);
  
  private final Joystick driveStick = new Joystick(0);
  private final XboxController xboxStick = new XboxController(1);
  private final VelocityPID shooter = new VelocityPID("practice", 20);
  public RobotContainer() {
   /* 
    SingleMotorControl intake = new SingleMotorControl(28, MotorType.kBrushless, 1, .05);
    SingleMotorControl shooter = new SingleMotorControl(29, MotorType.kBrushless, 1, .05);
    */

    // Configure the button bindings
    configureButtonBindings();
    
    CommandScheduler.getInstance().setDefaultCommand(shooter, new JoystickShooter(shooter, xboxStick));
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, new JoystickDrive(driveTrain, driveStick));
    CommandScheduler.getInstance().setDefaultCommand(intake , new JoystickIntake(intake, xboxStick));
    CommandScheduler.getInstance().setDefaultCommand(left_flapper , new JoystickFlapper(left_flapper, xboxStick));
    
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
