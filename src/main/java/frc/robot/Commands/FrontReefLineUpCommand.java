// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TakeOverTelopConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FrontReefLineUpCommand extends Command {

  private DriveSubsystem m_drive;
  private LimelightSubsystem m_limelight;

  private Pose3d m_lastKnownPose;

  private boolean m_allowEndCondition;

  private double m_lastTimeSeen;

  /** Creates a new LimelightReefLevel4. */
  public FrontReefLineUpCommand(DriveSubsystem drive, LimelightSubsystem limelight, boolean allowEndCondition) {

    this.m_drive = drive;
    this.m_limelight = limelight;

    this.m_lastKnownPose = null;

    this.m_allowEndCondition = allowEndCondition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!this.m_limelight.hasValidTarget()) {
      //No target found then stop
      this.m_drive.drive(0, 0, 0, false);
      return;
    } else {
      this.m_lastTimeSeen = DriverStation.getMatchTime();
    }

    this.m_lastKnownPose = this.m_limelight.getTargetPoseInCameraSpace();
    double limelightToApriltagZ = TakeOverTelopConstants.kReefXDistance + TakeOverTelopConstants.kFrontLimeLightToFrame;

    int currentID = this.m_limelight.getCurrentID();

    // Set goal rotation
    double targetAngle = 0.0;

    if(currentID == 7 || currentID == 18) {
      targetAngle = 0.0;
    } else if(currentID == 10 || currentID == 21) {
      targetAngle = 180.0;
    } else if(currentID == 8 || currentID == 19) {
      targetAngle = -60.0;
    } else if(currentID == 6 || currentID == 17) {
      targetAngle = 60;
    } else if(currentID == 9 || currentID == 22) {
      targetAngle = -120.0;
    } else if(currentID == 11 || currentID == 20) {
      targetAngle = 120.0;
    }
    
    targetAngle = Math.toRadians(targetAngle);

    //Pulls offsets
    double ySpeed = 7.0 * (TakeOverTelopConstants.kReefYDistance + this.m_lastKnownPose.getX());
    double xSpeed = -7.0 * (limelightToApriltagZ - this.m_lastKnownPose.getZ());

    // // When both speeds are too low to move, increase them
    if(Math.abs(TakeOverTelopConstants.kReefYDistance + this.m_lastKnownPose.getX()) < TakeOverTelopConstants.kMaxErrorDistance &&
    Math.abs(limelightToApriltagZ - this.m_lastKnownPose.getZ()) < TakeOverTelopConstants.kMaxErrorDistance) {

      xSpeed *= 6.0;
      ySpeed *= 6.0;
    }

    //Check if with in error, if so, zero them
    if(Math.abs(TakeOverTelopConstants.kReefYDistance + this.m_lastKnownPose.getX()) < TakeOverTelopConstants.kMaxErrorDistance) {
      ySpeed = 0.0;
    }
    if(Math.abs(limelightToApriltagZ - this.m_lastKnownPose.getZ()) < TakeOverTelopConstants.kMaxErrorDistance) {
      xSpeed = 0.0;
    }

    //limits max speed
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);

    SmartDashboard.putString("To Drive", "xSpeed: " + xSpeed + ", ySpeed: " + ySpeed + ", rot: " + targetAngle);

    this.m_drive.headingDrive(TakeOverTelopConstants.kMaxSpeed, xSpeed, ySpeed, Math.cos(targetAngle), Math.sin(targetAngle), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //
    if(!this.m_limelight.hasValidTarget() && Math.abs(this.m_lastTimeSeen - DriverStation.getMatchTime()) > 3.0) {

      return m_allowEndCondition;
    }


    this.m_lastKnownPose = this.m_limelight.getTargetPoseInCameraSpace();
    double limelightToApriltagZ = TakeOverTelopConstants.kReefXDistance + TakeOverTelopConstants.kFrontLimeLightToFrame;

    double errorX = Math.abs(TakeOverTelopConstants.kReefYDistance + this.m_lastKnownPose.getX());
    double errorY = Math.abs(limelightToApriltagZ - this.m_lastKnownPose.getZ());

    return
    errorX < TakeOverTelopConstants.kMaxCommandEndError && // Left-Right offset in with error
    errorY < TakeOverTelopConstants.kMaxCommandEndError && // Forward-Backward offset in with error
    this.m_allowEndCondition;
  }
} 