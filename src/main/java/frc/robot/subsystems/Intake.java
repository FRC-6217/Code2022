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
  public enum MotorState {
    OFF,
    REVERSE,
    SLOW,
    FAST
  }

  private CANSparkMax i_IntakeMotor;
  private MotorState motorState = MotorState.OFF;
  public Intake() {
    //instantiate motor 
    this.i_IntakeMotor = new CANSparkMax(INTAKE.MOTOR_ID, MotorType.kBrushless);
  }

  //Run motor in the postive direction
  public void setForward() {
    motorState = MotorState.FAST;
    i_IntakeMotor.set(INTAKE.SPEED);
  }
  
  public void setSlow() {
    motorState = MotorState.SLOW;
    i_IntakeMotor.set(INTAKE.SPEED/2);
  }
  //Run motor in the negative direction
  public void setReverse() {
    motorState = MotorState.REVERSE;
    i_IntakeMotor.set(-INTAKE.SPEED);
  }
  
  //Turn motor off
  public void setOff() {
    motorState = MotorState.OFF;
    i_IntakeMotor.set(0);
  }

  public MotorState getMotorState(){
    return motorState;
  }

  public double getPostion(){
    return i_IntakeMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
  }
}
