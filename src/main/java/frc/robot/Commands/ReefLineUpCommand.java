// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TakeOverTelopConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefLineUpCommand extends Command {

  private DriveSubsystem m_drive;
  private LimelightSubsystem m_limelight;

  private Pose3d m_lastKnownPose;

  /** Creates a new LimelightReefLevel4. */
  public ReefLineUpCommand(DriveSubsystem drive, LimelightSubsystem limelight) {

    this.m_drive = drive;
    this.m_limelight = limelight;

    this.m_limelight = null;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!this.m_limelight.hasValidTarget() && this.m_lastKnownPose != null) {
      //if no limelight target is found but was found in a past frame then
      //the attempt to point at the last known Pose

      double rot = -10 * this.m_lastKnownPose.getX();

      this.m_drive.drive(0.0, 0.0, rot, false);
      return;
    }

    this.m_lastKnownPose = this.m_limelight.getVisionMeasurement();

    int currentTagID = this.m_limelight.getCurrentID();

    boolean canSeeBlueReef = 17 >= currentTagID && currentTagID <= 22; //April Tags on Blue reef are ids: 17-22
    boolean canSeeRedReef = 6 >= currentTagID && currentTagID <= 11; //April Tags on Red reef are ids: 6-11

    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rot = 0.0;


    //Checks if the limelight's found tag is on the robot's team reef
    if((canSeeBlueReef && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ||
      (canSeeRedReef && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) {

      xSpeed = -10 * this.m_lastKnownPose.getRotation().getZ();
      ySpeed = 10 * (TakeOverTelopConstants.kReefYDistance - this.m_lastKnownPose.getZ());
      rot = -10 * this.m_lastKnownPose.getX();
    }

    //If the limelight is near the edge of the frame then
    //stop translation and rotation until the tag is more centered
    if(rot > TakeOverTelopConstants.kTranslationLockOut) {
      xSpeed = 0.0;
      ySpeed = 0.0;
    }


    this.m_drive.drive(xSpeed, ySpeed, rot, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    this.m_lastKnownPose = this.m_limelight.getVisionMeasurement();

    return
    Math.abs(this.m_lastKnownPose.getX()) < TakeOverTelopConstants.kMaxErrorDistance && // Left-Right offset in with error
    Math.abs(TakeOverTelopConstants.kReefYDistance - this.m_lastKnownPose.getZ()) < TakeOverTelopConstants.kMaxErrorDistance && // Forward-Backward offset in with error
    Math.abs(this.m_lastKnownPose.getRotation().getZ()) < TakeOverTelopConstants.kMaxErrorRotation;// Yaw rotation offset in with error
  }
}
