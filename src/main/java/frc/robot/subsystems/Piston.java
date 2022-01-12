// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Piston extends SubsystemBase {
  /** Creates a new Piston. */
  private DoubleSolenoid piston1;

  public Piston() {
    piston1 = new DoubleSolenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.PISTON_1_ID_1, Constants.PISTON_1_ID_2);
  }

  public void offPiston1() {
    piston1.set(Value.kOff);

  }

  public void forwardPiston1() {
    piston1.set(Value.kForward);

  }

  public void reversePiston1() {
    piston1.set(Value.kReverse);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
