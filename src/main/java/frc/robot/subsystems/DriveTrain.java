// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax dt_LeftMotor_L;
  private CANSparkMax dt_LeftMotor_F;
  private CANSparkMax dt_RightMotor_L;
  private CANSparkMax dt_RightMotor_F;

  private MotorControllerGroup dt_LeftMotors;
  private MotorControllerGroup dt_RightMotors;

  private final DifferentialDrive dt_Drive;


  private RelativeEncoder dt_LeftMotor_Encoder_L;
  private RelativeEncoder dt_LeftMotor_Encoder_F;
  private RelativeEncoder dt_RightMotor_Encoder_L;
  private RelativeEncoder dt_RightMotor_Encoder_F;

  private AHRS dt_gyro;

  public  DifferentialDriveKinematics dt_kinematics;
  public DifferentialDriveOdometry dt_odometry;
  public SimpleMotorFeedforward dt_feedforward;




  /** Creates a new DriveTrain. */
  public DriveTrain() {
    dt_LeftMotor_L = new CANSparkMax(33, MotorType.kBrushless);
    dt_LeftMotor_F = new CANSparkMax(32, MotorType.kBrushless);
    dt_RightMotor_L = new CANSparkMax(31, MotorType.kBrushless);
    dt_RightMotor_F = new CANSparkMax(30, MotorType.kBrushless);

    dt_LeftMotors = new MotorControllerGroup(dt_LeftMotor_L, dt_LeftMotor_F);
    dt_RightMotors = new MotorControllerGroup(dt_RightMotor_L, dt_RightMotor_F);
    dt_LeftMotors.setInverted(true);

    dt_Drive = new DifferentialDrive(dt_LeftMotors, dt_RightMotors);

    dt_LeftMotor_Encoder_L = dt_LeftMotor_F.getEncoder();
    dt_LeftMotor_Encoder_L.setPositionConversionFactor(0.044705);
    dt_LeftMotor_Encoder_L.setVelocityConversionFactor(0.044705);
    
    dt_LeftMotor_Encoder_F = dt_LeftMotor_F.getEncoder();
    dt_LeftMotor_Encoder_F.setPositionConversionFactor(0.044705);
    dt_LeftMotor_Encoder_F.setVelocityConversionFactor(0.044705);

    dt_RightMotor_Encoder_L = dt_RightMotor_F.getEncoder();
    dt_RightMotor_Encoder_L.setPositionConversionFactor(0.044705);
    dt_RightMotor_Encoder_L.setVelocityConversionFactor(0.044705);

    dt_RightMotor_Encoder_F = dt_RightMotor_F.getEncoder();
    dt_RightMotor_Encoder_F.setPositionConversionFactor(0.044705);
    dt_RightMotor_Encoder_F.setVelocityConversionFactor(0.044705);
    

    dt_gyro = new AHRS(SPI.Port.kMXP);
    dt_gyro.calibrate();
    dt_gyro.zeroYaw();

    dt_kinematics = new DifferentialDriveKinematics(0.6784);
    dt_feedforward = new SimpleMotorFeedforward(0.12923, 2.7944, 0.32061);
    dt_odometry = new DifferentialDriveOdometry(dt_gyro.getRotation2d());

  }

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

  public void resetEncoders() {
    dt_LeftMotor_Encoder_L.setPosition(0);
    dt_LeftMotor_Encoder_F.setPosition(0);
    dt_RightMotor_Encoder_L.setPosition(0);
    dt_RightMotor_Encoder_F.setPosition(0);
  }

  public void zeroHeading() {
    dt_gyro.reset();
  }

  public double getHeading() {
    return dt_gyro.getRotation2d().getDegrees();
  }

  public double getDegrees(){
    return dt_gyro.getAngle();
  }

  public double getTurnRate() {
    return -dt_gyro.getRate();
  }

  public Pose2d getPose() {
    return dt_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    dt_odometry.resetPosition(pose, dt_gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    dt_LeftMotors.setVoltage(leftVolts);
    dt_RightMotors.setVoltage(rightVolts);
    dt_Drive.feed();

  }

  public void arcadeDrive(double fwd, double rot) {
    dt_Drive.arcadeDrive(fwd, rot);
  }

  @Override
  public void periodic() {
    dt_odometry.update(dt_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    // This method will be called once per scheduler run
  }
}
