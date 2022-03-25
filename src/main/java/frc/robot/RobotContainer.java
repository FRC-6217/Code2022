// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.VelocityPID;
import frc.robot.subsystems.sensors.Lidar;
import frc.robot.subsystems.sensors.LimeLight;
import frc.robot.subsystems.PositionPID;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flapper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.SingleMotorControl;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.JoystickHanger;
import frc.robot.commands.SemiAutoBallAlign;
import frc.robot.commands.SemiAutoLidarDistance;
import frc.robot.commands.extenderPIDCommand;
import frc.robot.commands.bad.JoystickExtender;
import frc.robot.commands.bad.JoystickShooter;
import frc.robot.commands.bad.JoystickWinch;
import frc.robot.commands.AutoBallIntake;
import frc.robot.commands.AutoCedarFallsTwoBall;
import frc.robot.commands.AutoDriveDistance;
import frc.robot.commands.AutoDrivePose;
import frc.robot.commands.AutoDriveWeekZero;
import frc.robot.commands.AutoDriveWeekZeroBack;
import frc.robot.commands.AutoFlapper;
import frc.robot.commands.AutoGrabber;
import frc.robot.commands.AutoOneBallBackup;
import frc.robot.commands.AutoShootDuluth;
import frc.robot.commands.JoystickBallHandler;
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

  // private final Lidar lidar = new Lidar();
  //private final Flapper left_flapper = new Flapper(23);

  private final SingleMotorControl rightFlapper = new SingleMotorControl(23, MotorType.kBrushless, true, 0.1, 0.2);
  private final SingleMotorControl leftFlapper = new SingleMotorControl(22, MotorType.kBrushless, false, 0.1, 0.4);
  private final VelocityPID spinner = new VelocityPID("spinner", 20);
  private final SingleMotorControl extender = new SingleMotorControl(Constants.HANGER.EXTENDER_ID, MotorType.kBrushless, false, 0.1, .5);
  private final SingleMotorControl winch = new SingleMotorControl(10, MotorType.kBrushless, false, 0.1, 1);
 // private final PositionPID extenderPID = new PositionPID("extender", Constants.HANGER.EXTENDER_ID, 0, 0, 0);
  private final Joystick driveStick = new Joystick(0);

  private final XboxController xbox = new XboxController(1);

  private final LimeLight ballLimeLight = new LimeLight("ball");

  private final LEDController ledController = new LEDController();

  SendableChooser<Command> aChooser = new SendableChooser<>();

  public RobotContainer() {
    // AutoCedarFallsTwoBall twoball = new AutoCedarFallsTwoBall(driveTrain, intake, spinner, ballLimeLight, leftFlapper, rightFlapper);
    AutoShootDuluth oneballNoBack = new AutoShootDuluth(driveTrain, spinner, intake, leftFlapper, rightFlapper, 1800);
    AutoOneBallBackup oneBallBackup = new AutoOneBallBackup(driveTrain, spinner, intake, leftFlapper, rightFlapper);
    // aChooser.setDefaultOption("Two Ball Auto", twoball);
    aChooser.setDefaultOption("One Ball Auto No Back", oneballNoBack);
    aChooser.addOption("One Ball Auto Back Up", oneBallBackup);
    aChooser.addOption("Null", null); 
    SmartDashboard.putData(aChooser);


    ledController.set(DriverStation.getAlliance());
    // Configure the button bindings
    configureButtonBindings();

    CommandScheduler.getInstance().setDefaultCommand(driveTrain, new JoystickDrive(driveTrain, driveStick));//, lidar, ballLimeLight));

    CommandScheduler.getInstance().setDefaultCommand(spinner, new JoystickBallHandler(leftFlapper, rightFlapper, spinner, intake, xbox));
    CommandScheduler.getInstance().setDefaultCommand(extender, new JoystickExtender(extender, xbox));
   // CommandScheduler.getInstance().setDefaultCommand(extenderPID, new extenderPIDCommand(extenderPID, driveStick));
    CommandScheduler.getInstance().setDefaultCommand(winch, new JoystickWinch(winch, xbox));
  
    // CommandScheduler.getInstance().setDefaultCommand(extender, new JoystickHanger(extender, winch, driveStick));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveStick, 1).whileHeld(new SemiAutoBallAlign(driveTrain, ballLimeLight, driveStick));
    new JoystickButton(driveStick, 2).whenPressed(new AutoDriveWeekZeroBack(driveTrain, .455));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    
    //new AutoCedarFallsTwoBall(driveTrain, intake, spinner, ballLimeLight);//new AutoGrabber(driveTrain, intake, spinner, ballLimeLight);//new AutoDrivePose(driveTrain, new Pose2d(0, 0, new Rotation2d(Math.PI))); // new AutoShootDuluth(driveTrain,spinner,intake,leftFlapper,rightFlapper);
    // An ExampleCommand will run in autonomous
    return aChooser.getSelected();//new AutoShootDuluth(driveTrain, spinner, intake, leftFlapper, rightFlapper);//aChooser.getSelected();//new AutoShootDuluth(driveTrain, spinner, intake, leftFlapper, rightFlapper);//aChooser.getSelected();//new AutoShootDuluth(driveTrain, spinner, intake, leftFlapper, rightFlapper);
  }
}
