// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.sensors.LimeLight;

public class SemiAutoBallAlign extends CommandBase {
  /** Creates a new SemiAutoBallAlign. */  /** Creates a new AutoBallIntake. */
  enum BallIntakeState{
    Searching,
    Found,
    Chasing,
    Aquired
  }

  private BallIntakeState state;

  private DriveTrain driveTrain;
  private LimeLight ballLimeLight;
  private Joystick joy;

  public SemiAutoBallAlign(DriveTrain driveTrain, LimeLight ballLimeLight, LimeLight.PiplineID color, Joystick joy) {
    addRequirements(driveTrain);
    addRequirements(ballLimeLight);

    this.driveTrain = driveTrain;
    this.ballLimeLight = ballLimeLight;
    this.ballLimeLight.setPipline(color);
    this.state = BallIntakeState.Searching;
    this.joy = joy;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.state = BallIntakeState.Searching;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gov = (1-joy.getRawAxis(3))/2;
    double xSpeed = (Math.abs(joy.getRawAxis(1)) < 0.3) ? 0.0 : (joy.getRawAxis(1) * gov);
    double rot = (Math.abs(joy.getRawAxis(2)) < 0.3) ? 0.0 : (joy.getRawAxis(2) * gov);
    
    switch(state){
      case Searching:
        driveTrain.drive(xSpeed, rot);
        if(ballLimeLight.getValid()){
          state = BallIntakeState.Chasing;
        }
        break;
      case Chasing:
        double rotationError = ballLimeLight.getX()/75; // Set to be between -0.66 0.66
        if(Math.abs(ballLimeLight.getX()) < 3){
          driveTrain.drive(xSpeed, 0);
        }
        else if(Math.abs(ballLimeLight.getX())< 10){
          driveTrain.drive(xSpeed, rotationError);
        }
        else{
          driveTrain.drive(xSpeed,rotationError);
        }

        if(!ballLimeLight.getValid()){
          state = BallIntakeState.Searching;
        }
        break;
    }
    SmartDashboard.putString("Limelight State", state.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == BallIntakeState.Aquired;
  }
}