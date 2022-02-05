// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter_2 extends SubsystemBase {
  /** Creates a new Shooter_2. */
  CANSparkMax motor = new CANSparkMax(29, MotorType.kBrushless);
   double speed = 0;
   boolean s = true;
  public Shooter_2() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void on(){
    s = true;
    motor.set(speed);
  }
  public void off(){
    s=false;
    motor.set(0);
  }
  public void speedup(){
    speed = speed + 0.1;
  }
  public void speeddown(){
    speed = speed - 0.1;
  }
  public void reverse(){
    speed = speed * -1;
  }
  public double getSpeed(){
    return speed;
  }
  public boolean getS(){
    return s;
  }
}
