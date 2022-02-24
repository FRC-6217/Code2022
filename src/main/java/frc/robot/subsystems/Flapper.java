// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flapper extends SubsystemBase {
  /** Creates a new Flapper. */
  private CANSparkMax f_flapper;
  private SparkMaxLimitSwitch f_forwardSwitch;
  private SparkMaxLimitSwitch f_reverseSwitch;
  private static final double FLAPPER_SPEED = 0.6;

  public Flapper(int id) {
    f_flapper = new CANSparkMax(id, MotorType.kBrushless);
    
    f_forwardSwitch = f_flapper.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    f_reverseSwitch = f_flapper.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  public void setForward(){
    if(f_forwardSwitch.isPressed()){
      f_flapper.set(FLAPPER_SPEED);
    }
    else{
      f_flapper.set(0);
    }
  }

  public void setReverse(){
    if(f_reverseSwitch.isPressed()){
      f_flapper.set(-FLAPPER_SPEED);
    }
    else{
      f_flapper.set(0);
    }
  }

  public void setOff(){
    f_flapper.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
