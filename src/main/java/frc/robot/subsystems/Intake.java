// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax i_IntakeMotor;
  private static final double INTAKE_SPEED = 0.8;
  public Intake() {
    i_IntakeMotor = new CANSparkMax(21, MotorType.kBrushless);
  }

  public void setForward() {
    i_IntakeMotor.set(INTAKE_SPEED);
  }
  public void setBackward() {
    i_IntakeMotor.set(-INTAKE_SPEED);
  }
  public void setOff() {
    i_IntakeMotor.set(0);
  }
  public void setSpeed(double speed){
    i_IntakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
