// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorControl extends SubsystemBase {
  private CANSparkMax myMotorController;
  private double myMotorSpeed = 0.0;
  private boolean myMotorState = false;
  private int myDirection;
  private double myPercentSpeedChange;
  /** Creates a new SingleMotorControl. */
  public SingleMotorControl(int motorID, MotorType motorType, int direction, double percentSpeedChange) {
    myMotorController = new CANSparkMax(motorID, motorType);
    myDirection = direction;
    myPercentSpeedChange = percentSpeedChange;
  }

  public void turnOn() {
    myMotorState = true;
    myMotorController.set(myMotorSpeed * myDirection);
  }

  public void turnOff() {
    myMotorState = false;
    myMotorController.set(0);
  }

  public void increaseSpeed() {
    if (1 >= myMotorSpeed) {
      myMotorSpeed = myPercentSpeedChange;
      if (true == myMotorState) {
        myMotorController.set(myMotorSpeed * myDirection);
      }
  }
  }

  public void decreaseSpeed() {
    if (0 <= myMotorSpeed) {
      myMotorSpeed -= myPercentSpeedChange;
      if (true == myMotorState) {
        myMotorController.set(myMotorSpeed * myDirection);
      }
  }
  }

  public void changeMotorDirection() {
    myDirection *= -1;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
