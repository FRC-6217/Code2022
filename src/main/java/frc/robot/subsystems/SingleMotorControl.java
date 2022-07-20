// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorControl extends SubsystemBase {
  private CANSparkMax myMotorController;
  private double myMotorSpeed = 0.0;
  private boolean myMotorState = false;
  private double myPercentSpeedChange;
  private String name;
  /** Creates a new SingleMotorControl. */
  public SingleMotorControl(int motorID, String name, MotorType motorType, boolean isInverted, double percentSpeedChange, double startPercentSpeed) {
    myMotorController = new CANSparkMax(motorID, motorType);
    myMotorController.setInverted(isInverted);
    myMotorSpeed = startPercentSpeed;
    myPercentSpeedChange = percentSpeedChange;
    this.name = name;
    resetEncoders();
  }

  public void turnOnForward() {
    myMotorState = true;
    myMotorController.set(myMotorSpeed);
  }

  public void turnOnReverse() {
    myMotorState = true;
    myMotorController.set(-myMotorSpeed);
  }

  public void turnOff() {
    myMotorState = false;
    myMotorController.set(0);
  }

  public void increaseSpeed() {
    if (1 >= myMotorSpeed) {
      myMotorSpeed = myPercentSpeedChange;
      if (true == myMotorState) {
        myMotorController.set(myMotorSpeed);
      }
  }
  }

  public void decreaseSpeed() {
    if (0 <= myMotorSpeed) {
      myMotorSpeed -= myPercentSpeedChange;
      if (true == myMotorState) {
        myMotorController.set(myMotorSpeed);
      }
    }
  }
  public void resetEncoders(){
    myMotorController.getEncoder().setPosition(0);
  }
  //Throws bad stuff if brushed. 
  public double getPostion(){
    if(myMotorController.getMotorType() == MotorType.kBrushed){
      return Double.NaN;
    }
    return myMotorController.getEncoder().getPosition();
  }
  
  //Throws bad stuff if brushed.
  public double getVelocity(){
    if(myMotorController.getMotorType() == MotorType.kBrushed){
      return Double.NaN;
    }
    return myMotorController.getEncoder().getVelocity();
  }

  public double getPercent(){
    return myMotorSpeed;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Power of Winch", myMotorSpeed);
    SmartDashboard.putBoolean(name + "State", myMotorState);
    // This method will be called once per scheduler run
  }
}
