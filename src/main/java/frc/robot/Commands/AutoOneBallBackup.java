// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SingleMotorControl;
import frc.robot.subsystems.VelocityPID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoOneBallBackup extends SequentialCommandGroup {
  /** Creates a new AutoOneBallBackup. */
  public AutoOneBallBackup(DriveTrain driveTrain, VelocityPID spinner, Intake intake, SingleMotorControl leftFlapper, SingleMotorControl rightFlapper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoDriveWeekZeroBack(driveTrain, .305), new AutoShootDuluth(driveTrain, spinner, intake, leftFlapper, rightFlapper));
  }
}
