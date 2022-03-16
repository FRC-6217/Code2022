// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVE_TRAIN;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax dt_LeftMotor_L;
  private CANSparkMax dt_LeftMotor_F;
  private CANSparkMax dt_RightMotor_L;
  private CANSparkMax dt_RightMotor_F;

  private MotorControllerGroup dt_LeftMotors;
  private MotorControllerGroup dt_RightMotors;

  //private final DifferentialDrive dt_Drive;


  private RelativeEncoder dt_LeftMotor_Encoder_L;
  private RelativeEncoder dt_LeftMotor_Encoder_F;
  private RelativeEncoder dt_RightMotor_Encoder_L;
  private RelativeEncoder dt_RightMotor_Encoder_F;


  private DifferentialDriveKinematics dt_kinematics;
  private DifferentialDriveOdometry dt_odometry;

  private Pigeon2 gyro;




  /** Creates a new DriveTrain. */
  public DriveTrain() {

    //Declaration of Motors
    dt_LeftMotor_L = new CANSparkMax(DRIVE_TRAIN.LEFT_L_MOTOR_ID, MotorType.kBrushless);
    dt_LeftMotor_L.setIdleMode(IdleMode.kBrake);
    dt_LeftMotor_F = new CANSparkMax(DRIVE_TRAIN.LEFT_F_MOTOR_ID, MotorType.kBrushless);
    dt_LeftMotor_F.setIdleMode(IdleMode.kBrake);
    dt_RightMotor_L = new CANSparkMax(DRIVE_TRAIN.RIGHT_L_MOTOR_ID, MotorType.kBrushless);
    dt_RightMotor_L.setIdleMode(IdleMode.kBrake);
    dt_RightMotor_F = new CANSparkMax(43, MotorType.kBrushless);
    dt_RightMotor_F.setIdleMode(IdleMode.kBrake);

    dt_LeftMotors = new MotorControllerGroup(dt_LeftMotor_L, dt_LeftMotor_F);
    dt_RightMotors = new MotorControllerGroup(dt_RightMotor_L, dt_RightMotor_F);
    
    //To run the motors in the same direction
    dt_LeftMotors.setInverted(true);

    //dt_Drive = new DifferentialDrive(dt_LeftMotors, dt_RightMotors);

    //Set the meter per pulse for the encoder on the Motors
    dt_LeftMotor_Encoder_L = dt_LeftMotor_F.getEncoder();
    dt_LeftMotor_Encoder_L.setPositionConversionFactor(DRIVE_TRAIN.ENCODER_DISTANCE_CONVERSION_FACTOR);
    dt_LeftMotor_Encoder_L.setVelocityConversionFactor(DRIVE_TRAIN.ENCODER_DISTANCE_CONVERSION_FACTOR);
    
    dt_LeftMotor_Encoder_F = dt_LeftMotor_F.getEncoder();
    dt_LeftMotor_Encoder_F.setPositionConversionFactor(DRIVE_TRAIN.ENCODER_DISTANCE_CONVERSION_FACTOR);
    dt_LeftMotor_Encoder_F.setVelocityConversionFactor(DRIVE_TRAIN.ENCODER_DISTANCE_CONVERSION_FACTOR);

    dt_RightMotor_Encoder_L = dt_RightMotor_F.getEncoder();
    dt_RightMotor_Encoder_L.setPositionConversionFactor(DRIVE_TRAIN.ENCODER_DISTANCE_CONVERSION_FACTOR);
    dt_RightMotor_Encoder_L.setVelocityConversionFactor(DRIVE_TRAIN.ENCODER_DISTANCE_CONVERSION_FACTOR);

    dt_RightMotor_Encoder_F = dt_RightMotor_F.getEncoder();
    dt_RightMotor_Encoder_F.setPositionConversionFactor(DRIVE_TRAIN.ENCODER_DISTANCE_CONVERSION_FACTOR);
    dt_RightMotor_Encoder_F.setVelocityConversionFactor(DRIVE_TRAIN.ENCODER_DISTANCE_CONVERSION_FACTOR);
    
    //Setup the Gyro
    gyro = new Pigeon2(10);
    Pigeon2Configuration config = new Pigeon2Configuration();

    // set mount pose as rolled 90 degrees counter-clockwise
    config.MountPoseYaw = 0;
    config.MountPosePitch = 0;
    config.MountPoseRoll = 90;
    
    gyro.configAllSettings(config);

    //Motion tracking and trajetory code for auto.
    dt_kinematics = new DifferentialDriveKinematics(0.6223);
    //dt_feedforward = new SimpleMotorFeedforward(0.12923, 2.7944, 0.32061);
    // dt_odometry = new DifferentialDriveOdometry(dt_gyro.getRotation2d());

  //  CameraServer.startAutomaticCapture();


  }

  //Return average values of encoder from motors
  public double getLeftEncoderRate(){
    return (dt_LeftMotor_Encoder_L.getVelocity() + dt_LeftMotor_Encoder_F.getVelocity()) / 2;
  }

  public double getRightEncoderRate(){
    return (dt_RightMotor_Encoder_L.getVelocity() + dt_RightMotor_Encoder_F.getVelocity()) / 2;
  }

  public double getLeftEncoderPosition(){
    return (dt_LeftMotor_Encoder_L.getPosition() + dt_LeftMotor_Encoder_F.getPosition()) / 2;
  }

  public double getRightEncoderPosition(){
    return (dt_RightMotor_Encoder_L.getPosition() + dt_RightMotor_Encoder_F.getPosition()) / 2;
  }

  //Reset the encoders value to zero
  public void resetEncoders() {
    dt_LeftMotor_Encoder_L.setPosition(0);
    dt_LeftMotor_Encoder_F.setPosition(0);
    dt_RightMotor_Encoder_L.setPosition(0);
    dt_RightMotor_Encoder_F.setPosition(0);
  }

  public void resetGyro(){
    gyro.setYaw(0);
  }

  public double getAngle(){
  
    double angle = gyro.getYaw();
    while (angle<-180 || angle>180){
      if(angle<-180){
        angle += 360;
      }
      else if (angle>180) {
        angle -= 360;
      }
    }
    return angle;
  }


  //TODO add pigeon and uncomment these methods.
  // public void zeroHeading() {
  //   dt_gyro.reset();
  // }

  // public double getHeading() {
  //   return dt_gyro.getRotation2d().getDegrees();
  // }

  // public double getTurnRate() {
  //   return -dt_gyro.getRate();
  // }

  //Return pose/position of the robot based on the gyro and motor encoders
  public Pose2d getPose() {
    return dt_odometry.getPoseMeters();
  }

  //Send voltage to the motors instead of percentage
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    dt_LeftMotors.setVoltage(leftVolts);
    dt_RightMotors.setVoltage(rightVolts);
    //dt_Drive.feed();

  }

  //Set speed of the drive train based on wheel speed
  private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    dt_LeftMotors.set(speeds.leftMetersPerSecond);
    dt_RightMotors.set(speeds.rightMetersPerSecond);
  }

  //Teleop Control
  public void drive(double xSpeed, double rot) {
    SmartDashboard.putNumber("Gyro Angle", getAngle());
    SmartDashboard.putNumber("Drive Encoder Left", getLeftEncoderPosition());
    SmartDashboard.putNumber("Drive Encoder Right", getRightEncoderPosition());


    var wheelSpeeds = dt_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot * DRIVE_TRAIN.Z_AXIS_TELEOP_ADJUSTMENT));
    setSpeeds(wheelSpeeds);
  }

  @Override
  public void periodic() {
    // dt_odometry.update(Rotation2D gyro.getYaw(), getLeftEncoderPosition(), getRightEncoderPosition());
  }
}
