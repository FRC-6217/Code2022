// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SingleMotorControl;
import frc.robot.subsystems.VelocityPID;
import frc.robot.subsystems.sensors.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCedarFallsTwoBall extends SequentialCommandGroup {
  /** Creates a new AutoCedarFallsTwoBall. */
  public AutoCedarFallsTwoBall(DriveTrain driveTrain, Intake intake, VelocityPID spinner, LimeLight ballLimeLight, SingleMotorControl leftFlapper, SingleMotorControl rightFlapper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetPose(driveTrain), new AutoGrabber(driveTrain, intake, spinner, ballLimeLight), new AutoDrivePose(driveTrain, intake, spinner, true, false, new Pose2d(0, 0, new Rotation2d(Math.PI))), new AutoFlapper(leftFlapper, rightFlapper, spinner, intake));
  }
}