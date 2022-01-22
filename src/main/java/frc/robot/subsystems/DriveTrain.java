// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  private AnalogGyro dt_gyro;

  private DifferentialDriveKinematics dt_kinematics;
  private DifferentialDriveOdometry dt_odometry;
  private SimpleMotorFeedforward dt_feedforward;




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
    dt_LeftMotor_Encoder_L.setPositionConversionFactor(0.4788);
    
    dt_LeftMotor_Encoder_F = dt_LeftMotor_F.getEncoder();
    dt_LeftMotor_Encoder_F.setPositionConversionFactor(0.4788);

    dt_RightMotor_Encoder_L = dt_RightMotor_F.getEncoder();
    dt_RightMotor_Encoder_L.setPositionConversionFactor(0.4788);

    dt_RightMotor_Encoder_F = dt_RightMotor_F.getEncoder();
    dt_RightMotor_Encoder_F.setPositionConversionFactor(0.4788);
    

    dt_gyro = new AnalogGyro(0);
    dt_gyro.reset();

    dt_kinematics = new DifferentialDriveKinematics(0.6784);
    dt_feedforward = new SimpleMotorFeedforward(0.12923, 2.7944, 0.32061);

    dt_LeftPID = new PIDController(1, 0, 0);
    dt_RightPID = new PIDController(1, 0, 0);
  }

  public double getLeftEncoderRate(){
    return (dt_LeftMotor_Encoder_L.getVelocity() + dt_LeftMotor_Encoder_F.getVelocity()) / 2;
  }

  public double getRightEncoderRate(){
    return (dt_RightMotor_Encoder_L.getVelocity() + dt_RightMotor_Encoder_F.getVelocity()) / 2;
  }

  public void arcadeDrive(double fwd, double rot) {
    dt_Drive.arcadeDrive(fwd, rot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
