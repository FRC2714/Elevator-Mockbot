// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  private static double ty, tx, distance;

  private static NetworkTable limelight;

  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public static double getDistanceToGoal() {
    distance = Math.tan(Math.toRadians(Constants.CameraConstants.kMountingAngle + getYAngleOffset()));
    return distance;

  }

  public static double getYAngleOffset() {
    return ty;
  }

  public double getXAngleOffset() {
    return -1 * tx;
  }

  public double getXRadianOffset() {
    return getXAngleOffset() * Math.PI / 180;
  }

  public static boolean targetVisible() {
    double tv = limelight.getEntry("tv").getDouble(0.0);
    return tv != 0.0;
  }

  public void setPipeline(double numPipeline) {
    limelight.getEntry("pipeline").setDouble(numPipeline);
  }

  public static void enable() {
    limelight.getEntry("ledMode").setNumber(0);
}

public static void disable() {
    limelight.getEntry("ledMode").setNumber(1);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Est. Distance (ft)", Units.metersToFeet(distance));

    tx = limelight.getEntry("tx").getDouble(-1);
    ty = limelight.getEntry("ty").getDouble(-1);

  }
}
