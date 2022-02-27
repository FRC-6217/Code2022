// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;

public class Intake extends SubsystemBase {
  private CANSparkMax i_IntakeMotor;
  public Intake() {
    //instantiate motor 
    this.i_IntakeMotor = new CANSparkMax(INTAKE.MOTOR_ID, MotorType.kBrushless);
  }

  //Run motor in the postive direction
  public void setForward() {
    i_IntakeMotor.set(0.6);
  }
  
  //Run motor in the negative direction
  public void setBackward() {
    i_IntakeMotor.set(-INTAKE.SPEED);
  }
  
  //Turn motor off
  public void setOff() {
    i_IntakeMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}
