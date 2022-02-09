// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  

  private NetworkTable table;
  private NetworkTableEntry  tx;
  private NetworkTableEntry  ty;
  private NetworkTableEntry  tv;


  private Scalar angle;
  private Scalar distance;

  public Limelight(Scalar angle, Scalar distance) {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");

    this.angle = angle;
    this.distance = distance;
  }

  private double getAngleHorizontal(){
    return tx.getDouble(Double.POSITIVE_INFINITY);
  }
  
  private double getAngleVertical(){
    return ty.getDouble(-1);
  }

  private boolean getValid(){
    return tv.getDouble(0) == 1;
  }

  @Override
  public void periodic() {
    angle.setValid(getValid());
    distance.setValid(getValid());

    angle.setValue(getAngleHorizontal());
    distance.setValue(());

    // This method will be called once per scheduler run
  }
}

class Scalar{
  public double value;
  public boolean valid;

  public Scalar(){
  }

  public void setValue(double value){
    this.value = value;
  }

  /*
   * If you use this doublecheck that the isValid is true
  */
  public double getValue(){
      return value;
  }

  public boolean isValid(){
    return valid;
  }

  public void setValid(boolean valid){
    this.valid = valid;
  }

}