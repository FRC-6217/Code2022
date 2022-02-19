// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//DONT USE, DOESNT WORK
package frc.robot.subsystem;

import java.lang.System.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDMotorControl extends SubsystemBase {
  private CANSparkMax myMotorController;
  private SparkMaxPIDController myPidController;
  private RelativeEncoder myEncoder;
  private String myName;
  private int mySetPoint = 0;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  /** Creates a new PIDMotorControl. */
  public PIDMotorControl(int motorID, String name) {
    myMotorController = new CANSparkMax(motorID, MotorType.kBrushless);
    myPidController = myMotorController.getPIDController();
    myEncoder = myMotorController.getEncoder();
    myName = name;
    smartDashboardStuff();

    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    myPidController.setP(kP);
    myPidController.setI(kI);
    myPidController.setD(kD);
    myPidController.setIZone(kIz);
    myPidController.setFF(kFF);
    myPidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void smartDashboardStuff() {
    int smartMotionSlot = 0;

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber(myName + " P Gain", kP);
    SmartDashboard.putNumber(myName + " I Gain", kI);
    SmartDashboard.putNumber(myName + " D Gain", kD);
    SmartDashboard.putNumber(myName + " I Zone", kIz);
    SmartDashboard.putNumber(myName + " Feed Forward", kFF);

    // display Smart Motion coefficients
    SmartDashboard.putNumber(myName + " Max Velocity", maxVel);
    SmartDashboard.putNumber(myName + " Min Velocity", minVel);
    SmartDashboard.putNumber(myName + " Set Velocity", 0);
  }

  public void turnOn() {
    System.out.println("motor on and " + mySetPoint);
    mySetPoint = (int) SmartDashboard.getNumber(myName + " Set Velocity", 0);
   
    myPidController.setReference(mySetPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void turnOff() {
    System.out.println("motor off");
    myMotorController.set(0);
  }

  @Override
  public void periodic() {
    double p = SmartDashboard.getNumber(myName + " P Gain", 0);
    double i = SmartDashboard.getNumber(myName + " I Gain", 0);
    double d = SmartDashboard.getNumber(myName + " D Gain", 0);
    double iz = SmartDashboard.getNumber(myName + " I Zone", 0);
    double ff = SmartDashboard.getNumber(myName + " Feed Forward", 0);
    double max = SmartDashboard.getNumber(myName + " Max Output", 0);
    double min = SmartDashboard.getNumber(myName + " Min Output", 0);
    double maxV = SmartDashboard.getNumber(myName + " Max Velocity", 0);
    double minV = SmartDashboard.getNumber(myName + " Min Velocity", 0);
    double maxA = SmartDashboard.getNumber(myName + " Max Acceleration", 0);
    double allE = SmartDashboard.getNumber(myName + " Allowed Closed Loop Error", 0);
    double actualSpeed = myEncoder.getVelocity();
    SmartDashboard.putNumber(myName + " Actual Speed", actualSpeed);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { myPidController.setP(p); kP = p; }
    if((i != kI)) { myPidController.setI(i); kI = i; }
    if((d != kD)) { myPidController.setD(d); kD = d; }
    if((iz != kIz)) { myPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { myPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      myPidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { myPidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { myPidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }

    SmartDashboard.putNumber(myName + " Output", myMotorController.getAppliedOutput());
    // This method will be called once per scheduler run
  }
}
