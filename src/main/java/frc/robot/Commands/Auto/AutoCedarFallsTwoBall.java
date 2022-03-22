// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VelocityPID;
import frc.robot.subsystems.sensors.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCedarFallsTwoBall extends SequentialCommandGroup {
  /** Creates a new AutoCedarFallsTwoBall. */
  public AutoCedarFallsTwoBall(DriveTrain driveTrain, Intake intake, VelocityPID spinner, LimeLight ballLimeLight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoDriveForwardDistance(driveTrain, intake, spinner, 1.3), new AutoGrabber(driveTrain, intake, spinner, ballLimeLight), new AutoShooterToggle(spinner, true), new AutoDrivePose(driveTrain, new Pose2d(0, 0, new Rotation2d(Math.PI))), new AutoShooterToggle(spinner, false));
  }
}
