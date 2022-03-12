// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID {

    String name;
    double p,i,d;

    private double dT = 1;
    private double previousError = 0;
    private double sumError = 0;
    private double setPoint = 0;
    private double deadZone = 0;

    boolean debug = false;

    PID(String name, double p, double i, double d, double setPoint, double deadZone, boolean debug) {
        this.name = name;
        this.p = p;
        this.i = i;
        this.d = d;
        this.setPoint = setPoint;
        this.deadZone = deadZone;
        this.debug = debug;
    }

    public void enableDebug() {
        debug = true;
    }

    public void disableDebug() {
        debug = false;
    }

    public double calculate (double setPoint, double currentPoint) {
        double error = setPoint - currentPoint;
        double pE = p * error;
        double dE = d * (error - previousError)/dT;
        previousError = error;
            
        sumError += error;
        double iE = i * sumError * dT;
        if (Math.abs(error) <= deadZone)
        {
    
        }
        if (debug) {
            SmartDashboard.putNumber(name + "Error", error);
            SmartDashboard.putNumber(name + "pE", pE);
            SmartDashboard.putNumber(name + "dE", dE);
            SmartDashboard.putNumber(name + "sumError", sumError);
            SmartDashboard.putNumber(name + "iE", iE);
        }

    
        return pE + dE + iE;
    }

    public void updateConstants () {
        if (debug) {
            double getP = SmartDashboard.getNumber(name + " P Gain", 0);
            double getI = SmartDashboard.getNumber(name + " I Gain", 0);
            double getD = SmartDashboard.getNumber(name + " D Gain", 0);
            double getSetPoint = SmartDashboard.getNumber(name + " Set Point", 0);

            if((getP != p)) { 
              p = getP;
              }
        
            if((getI != i)) {
              i = getI;
            }
        
            if((getD != d)) {
              d = getD;
            }
        
            if((getSetPoint != setPoint)) {
              setPoint = getSetPoint;
            }
        }

    }
    
}
