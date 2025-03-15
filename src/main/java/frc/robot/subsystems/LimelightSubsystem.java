// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable m_limelightNetworkTable;
  private Pose3d m_visionMeasurement;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem(String key) {

    this.m_limelightNetworkTable = NetworkTableInstance.getDefault().getTable(key);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double[] targetPose = m_limelightNetworkTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[7]);
    SmartDashboard.putBoolean("tv", this.m_limelightNetworkTable.getEntry("tv").getDouble(0.0) == 1.0);
    SmartDashboard.putNumber("translationX", targetPose[0]);
    SmartDashboard.putNumber("translationY", targetPose[1]);
    SmartDashboard.putNumber("translationZ", targetPose[2]);
    SmartDashboard.putNumber("pitch", targetPose[3]);
    SmartDashboard.putNumber("yaw", targetPose[4]);
    SmartDashboard.putNumber("roll", targetPose[5]);

    this.m_visionMeasurement = new Pose3d(
      targetPose[0],
      targetPose[1],
      targetPose[2],
      new Rotation3d(
        targetPose[5], // Roll
        targetPose[3], // Pitch
        targetPose[4]  // Yaw
    ));

  }

  public boolean hasValidTarget() {

    return this.m_limelightNetworkTable.getEntry("tv").getDouble(0.0) == 1.0;
  }

  public Pose3d getVisionMeasurement() {

    return this.m_visionMeasurement;
  }

  public int getCurrentID() {
    return (int) this.m_limelightNetworkTable.getEntry("tid").getInteger(0);

  }

}
