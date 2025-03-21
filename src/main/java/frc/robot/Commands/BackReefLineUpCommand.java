// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TakeOverTelopConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BackReefLineUpCommand extends Command {
  
  private DriveSubsystem m_drive;
  private LimelightSubsystem m_limelight;

  private Pose3d m_lastKnownPose;

  /** Creates a new BackReefLineUpCommand. */
  public BackReefLineUpCommand(DriveSubsystem drive, LimelightSubsystem limelight) {

    this.m_drive = drive;
    this.m_limelight = limelight;

    this.m_lastKnownPose = null;
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
      return;
    }

    this.m_lastKnownPose = this.m_limelight.getVisionMeasurement();
    double limelightToApriltagZ = TakeOverTelopConstants.kReefYDistance + TakeOverTelopConstants.kBackLimeLightToFrame;

    //Pulls offsets
    double ySpeed = -5.0 * this.m_lastKnownPose.getX();
    double xSpeed = 5.0 * (limelightToApriltagZ - this.m_lastKnownPose.getZ());

    //limits max speed
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);

    this.m_drive.headingDrive(TakeOverTelopConstants.kMaxSpeed, xSpeed, ySpeed, 0.0, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    this.m_lastKnownPose = this.m_limelight.getVisionMeasurement();
    double limelightToApriltagZ = TakeOverTelopConstants.kReefYDistance + TakeOverTelopConstants.kBackLimeLightToFrame;

    double errorX = Math.abs(this.m_lastKnownPose.getX());
    double errorY = Math.abs(limelightToApriltagZ - this.m_lastKnownPose.getZ());

    return
    errorX < TakeOverTelopConstants.kMaxErrorDistance && // Left-Right offset in with error
    errorY < TakeOverTelopConstants.kMaxErrorDistance; // Forward-Backward offset in with error
  }
}
