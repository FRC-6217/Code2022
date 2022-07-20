// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FLAPPER;

public class Flapper extends SubsystemBase {
  private CANSparkMax f_flapper;
  private String name;
  private boolean on;

  public Flapper(int motorID, String name) {
    //Initlize motor
    f_flapper = new CANSparkMax(motorID, MotorType.kBrushless);
    f_flapper.setInverted(true);
    this.name = name;
    on = false;
  }

  //Run the motors in the positive direction
  public void setForward(){
      f_flapper.set(FLAPPER.SPEED);
      on = true;
  }

  //Run the motors in the negative direction
  public void setReverse(){
      f_flapper.set(-FLAPPER.SPEED);
      on = true;
    }

  //Turn the motors off
  public void setOff(){
    f_flapper.set(0);
    on = false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Flapper " + name + "State", on);
  }
}
