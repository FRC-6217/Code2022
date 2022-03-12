// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIGEON;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2_Faults;

public class Pigeon extends SubsystemBase {
  /** Creates a new Pigeon. */
  private WPI_Pigeon2 pigeon;
  private double angle, pitch, yaw, roll;
  public Pigeon() {
    pigeon = new WPI_Pigeon2(PIGEON.DEVICE_ID);
    pigeon.reset();

  }

  @Override
  public void periodic() {
    angle = pigeon.getAngle();
    pitch = pigeon.getPitch();
    yaw  = pigeon.getYaw();
    roll = pigeon.getRoll();
  }

  double getAngle() {
    return angle;
  }

  double getPitch() {
    return pitch;
  }

  double getYaw() {
    return yaw;
  }

  double getRoll() {
    return roll;
  }
}
