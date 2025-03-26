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

  private String m_limelightName;
  private NetworkTable m_limelightNetworkTable;

  private Pose3d m_targetPoseCameraSpace = new Pose3d();
  private Pose2d m_robotPoseFieldSpace = new Pose2d();

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem(String key) {

    this.m_limelightName = key;
    this.m_limelightNetworkTable = NetworkTableInstance.getDefault().getTable(key);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double[] targetPoseCameraSpace = m_limelightNetworkTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[7]);
    SmartDashboard.putBoolean(this.m_limelightName + " tv", this.m_limelightNetworkTable.getEntry("tv").getDouble(0.0) == 1.0);
    SmartDashboard.putNumber(this.m_limelightName + " translationX", targetPoseCameraSpace[0]);
    // SmartDashboard.putNumber(this.m_limelightName + " translationY", targetPose[1]);
    SmartDashboard.putNumber(this.m_limelightName + " translationZ", targetPoseCameraSpace[2]);
    // SmartDashboard.putNumber(this.m_limelightName + " pitch", targetPose[3]);
    SmartDashboard.putNumber(this.m_limelightName + " yaw", targetPoseCameraSpace[4]);
    // SmartDashboard.putNumber(this.m_limelightName + " roll", targetPose[5]);

    this.m_targetPoseCameraSpace = new Pose3d(
      targetPoseCameraSpace[0], // X
      targetPoseCameraSpace[1], // Y
      targetPoseCameraSpace[2], // Z
      new Rotation3d(
        targetPoseCameraSpace[5], // Roll
        targetPoseCameraSpace[3], // Pitch
        targetPoseCameraSpace[4]  // Yaw
    ));

    double[] robotPoseFieldSpace = m_limelightNetworkTable.getEntry("botpose").getDoubleArray(new double[7]);

    this.m_robotPoseFieldSpace = new Pose2d(
      robotPoseFieldSpace[0] + (17.551 / 2.0), // X
      robotPoseFieldSpace[1] + (8.052 / 2.0), // Y
      new Rotation2d(
        Math.toRadians(robotPoseFieldSpace[5])  // Yaw
    ));

  }

  public boolean hasValidTarget() {

    return this.m_limelightNetworkTable.getEntry("tv").getDouble(0.0) == 1.0;
  }

  public Pose3d getTargetPoseInCameraSpace() {

    return this.m_targetPoseCameraSpace;
  }

  public Pose2d getRobotPoseInFieldSpace() {
    return this.m_robotPoseFieldSpace;
  }

  public int getCurrentID() {
    return (int) this.m_limelightNetworkTable.getEntry("tid").getInteger(0);

  }

}
