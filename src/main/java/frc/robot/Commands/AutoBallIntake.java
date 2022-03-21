// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VelocityPID;
import frc.robot.subsystems.sensors.LimeLight;

public class AutoBallIntake extends CommandBase {
  /** Creates a new AutoBallIntake. */
  enum BallIntakeState{
    Searching,
    Found,
    Chasing,
    Aquired
  }

  private BallIntakeState state;

  private DriveTrain driveTrain;
  private LimeLight ballLimeLight;
  private Intake intake;

  public AutoBallIntake(DriveTrain driveTrain, Intake intake, VelocityPID spinner, LimeLight ballLimeLight, LimeLight.PiplineID color) {
    addRequirements(driveTrain);
    addRequirements(ballLimeLight);
    addRequirements(intake);
    addRequirements(spinner);

    this.driveTrain = driveTrain;
    this.ballLimeLight = ballLimeLight;
    this.intake = intake;
    this.ballLimeLight.setPipline(color);
    this.state = BallIntakeState.Searching;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.state = BallIntakeState.Searching;
    SmartDashboard.putString("Limelight State", state.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -0.3;
    
    switch(state){
      case Searching:
        driveTrain.drive(0, 0.3);
        if(ballLimeLight.getValid()){
          state = BallIntakeState.Chasing;
        }
        break;
      case Chasing:
        intake.setForward();
        double rotationError = ballLimeLight.getX() / 75; // Set to be between -0.66 0.66
        SmartDashboard.putNumber("Rot Error", rotationError);
        if(Math.abs(ballLimeLight.getX()) < 3){
          driveTrain.drive(xSpeed, 0);
        }
        else if(Math.abs(ballLimeLight.getX())< 10){
          driveTrain.drive(xSpeed, rotationError);
        }
        else{
          driveTrain.drive(0, rotationError);
        }

        if(!ballLimeLight.getValid()){
          state = BallIntakeState.Aquired;
        }
        break;
      case Aquired:
        break;
    }
    SmartDashboard.putString("Limelight State", state.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Interrupted", interrupted);
    intake.setOff();
    driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == BallIntakeState.Aquired;
  }
}
