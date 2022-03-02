// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VelocityPID extends SubsystemBase {
  private double p = 0;
  private double i = 0;
  private double d = 0;
  private double dT = 1;
  private double previousError = 0;
  private double sumError = 0;
  private String name;
  private double setPoint = 0;
  private CANSparkMax motorController;
  boolean motorState = false;
  private double deadZone = 10;
  /** Creates a new CustomPID. */


  public VelocityPID(String name, int motorID) {
    this.name = name;
    this.motorController = new CANSparkMax(motorID, MotorType.kBrushless);
    this.motorController.setInverted(true);
    // set PID defaults
    // @todo move to constansts
    SmartDashboard.putNumber(name + " P Gain",0.023000);
    SmartDashboard.putNumber(name + " I Gain", 0.000040);
    SmartDashboard.putNumber(name + " D Gain", 0);
    SmartDashboard.putNumber(name + " Set Point", 2250.000000);
  }

  private double calculate(double setPoint, double currentPoint) {
    double error = setPoint - currentPoint;
    double pE = p * error;
    double dE = d * (error - previousError)/dT;
    previousError = error;
        
    sumError += error;
    double iE = i * sumError * dT;
    if (Math.abs(error) <= deadZone)
    {

    }

    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("pE", pE);
    SmartDashboard.putNumber("dE", dE);
    SmartDashboard.putNumber("sumError", sumError);
    SmartDashboard.putNumber("iE", iE);

    return pE + dE + iE;
  }

  public void updateConstants () {
    double getP = SmartDashboard.getNumber(name + " P Gain", 0);
    double getI = SmartDashboard.getNumber(name + " I Gain", 0);
    double getD = SmartDashboard.getNumber(name + " D Gain", 0);
    double getSetPoint = SmartDashboard.getNumber(name + " Set Point", 0);
    if((getP != p)) { 
      p = getP;
      }

    if((getI != i)) {
      i = getI;
    }

    if((getD != d)) {
      d = getD;
    }

    if((getSetPoint != setPoint)) {
      setPoint = getSetPoint;
    }
    
  }


  public void turnOn() {
    motorState = true;
  }

  public void turnOff() {
    motorState = false;
    sumError = 0;
    previousError = 0;
  }
  
  public boolean getMotorState(){
    return motorState;
  }

  public boolean isAtSetpoint(){
    return Math.abs(motorController.getEncoder().getVelocity() - setPoint) < setPoint * 0.05;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current RPM", motorController.getEncoder().getVelocity());
    SmartDashboard.putNumber("Current RPM Graph", motorController.getEncoder().getVelocity());
    updateConstants();
    if (motorState == true)
    {
      double newMotorSpeed = calculate(setPoint, motorController.getEncoder().getVelocity());
      if (newMotorSpeed >= 10.5)
      {
        newMotorSpeed = 10.5;
      }

      else if(newMotorSpeed <= -10.5)
      {
        newMotorSpeed = -10.5;
      }

      SmartDashboard.putNumber("Motor Speed", newMotorSpeed);
      SmartDashboard.putNumber("Applied Output", motorController.getAppliedOutput());
      SmartDashboard.putNumber("Current", motorController.getOutputCurrent());
      SmartDashboard.putNumber("Bus Voltage", motorController.getBusVoltage());
      SmartDashboard.putNumber("Sticky Faults", motorController.getStickyFaults());
      
      

      motorController.setVoltage(newMotorSpeed);

    }

    else
    {
      motorController.set(0);
    }
    // This method will be called once per scheduler run
  }
}
