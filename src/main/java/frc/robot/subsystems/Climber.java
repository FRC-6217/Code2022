// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax winchUp;
  private CANSparkMax winchDown;
  private static double speed = 0.5; 
  public Climber() {
    winchUp = new CANSparkMax(27, MotorType.kBrushed);
    winchDown = new CANSparkMax(32, MotorType.kBrushed);
  }

  public void winchUpUp(){
    winchUp.set(speed);
  }
  public void winchUpDown(){
    winchUp.set(-speed);
  }
  public void winchUpOff(){
    winchUp.set(0);
  }
  public void winchDownUp(){
    winchDown.set(speed);
  }
  public void winchDownDown(){
    winchDown.set(-speed);
  }
  public void winchDownOff(){
    winchDown.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
