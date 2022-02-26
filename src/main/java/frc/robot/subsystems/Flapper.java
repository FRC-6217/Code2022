// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FLAPPER;

public class Flapper extends SubsystemBase {
  private CANSparkMax f_flapper;
  private SparkMaxLimitSwitch f_forwardSwitch;
  private SparkMaxLimitSwitch f_reverseSwitch;

  public Flapper(int motorID) {
    //Initlize motor
    f_flapper = new CANSparkMax(motorID, MotorType.kBrushless);
    
    //Get the limit switch of the motor
    f_forwardSwitch = f_flapper.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    f_reverseSwitch = f_flapper.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  //Run the motors in the positive direction
  public void setForward(){
      f_flapper.set(FLAPPER.SPEED);
  }

  //Run the motors in the negative direction
  public void setReverse(){
      f_flapper.set(-FLAPPER.SPEED);
    }

  //Turn the motors off
  public void setOff(){
    f_flapper.set(0);
  }

  @Override
  public void periodic() {
  }
}
