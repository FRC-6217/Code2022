// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


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
  private double spinnerOffset = 0;
  private double timeInSetPoint = Double.POSITIVE_INFINITY;
  private boolean isAtSetpoint = false;
  /** Creates a new CustomPID. */


  public VelocityPID(String name, int motorID) {
    this.name = name;
    this.motorController = new CANSparkMax(motorID, MotorType.kBrushless);
    this.motorController.setInverted(true);
    // set PID defaults
    // TODO move to constansts
    SmartDashboard.putNumber(name + " P Gain",0.023000);
    SmartDashboard.putNumber(name + " I Gain", 0.000040);
    SmartDashboard.putNumber(name + " D Gain", 0);
    SmartDashboard.putNumber(name + " Set Point", 1750.000000);
  }

  private double calculate(double setPoint, double currentPoint) {
    double error = setPoint - currentPoint;
    double pE = p * error;
    double dE = d * (error - previousError)/dT;
    previousError = error;
        
    sumError += error;
    double iE = i * sumError * dT;

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

  public void setSetpoint(double setpoint){
    SmartDashboard.putNumber(name + " Set Point", setpoint);
  }
  
  public boolean getMotorState(){
    return motorState;
  }

  public boolean isAtSetpoint(){
    return isAtSetpoint;
  }

  public void increaseSetpoint(){
    //spinnerOffset += 100;
  }
  public void decreaseSetpoint(){
    //spinnerOffset -= 100;
  }
  public void clearSpinnerOffset(){
    spinnerOffset = 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current RPM", motorController.getEncoder().getVelocity());
    SmartDashboard.putNumber("Current RPM Graph", motorController.getEncoder().getVelocity());
    SmartDashboard.putNumber("Spinner Offset", spinnerOffset);
    updateConstants();
    if (motorState == true)
    {
      double newMotorSpeed = calculate(setPoint  + spinnerOffset, motorController.getEncoder().getVelocity());
      if (newMotorSpeed >= Constants.PID.MAX_VOLTAGE)
      {
        newMotorSpeed = Constants.PID.MAX_VOLTAGE;
      }

      else if(newMotorSpeed <= -Constants.PID.MAX_VOLTAGE)
      {
        newMotorSpeed = -Constants.PID.MAX_VOLTAGE;
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

    boolean inSetpointThreshold = Math.abs(motorController.getEncoder().getVelocity() - setPoint) < setPoint * 0.08;

    if(inSetpointThreshold && timeInSetPoint == Double.POSITIVE_INFINITY){
      timeInSetPoint = Timer.getFPGATimestamp();
    }
    else if (inSetpointThreshold && Timer.getFPGATimestamp() - timeInSetPoint > 0.5){
      isAtSetpoint = true;
    }
    else if (!(inSetpointThreshold)){
      isAtSetpoint = false;
      timeInSetPoint = Double.POSITIVE_INFINITY;
    }
    else{
      isAtSetpoint = false;
    }

    SmartDashboard.putBoolean(name + "State", getMotorState());
    // This method will be called once per scheduler run
  }
}
