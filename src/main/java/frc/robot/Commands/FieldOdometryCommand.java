// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FieldOdometryCommand extends Command {

  DriveSubsystem m_drive;
  LimelightSubsystem m_frontLimeight;
  LimelightSubsystem m_backLimelight;

  Field2d m_field;

  Pose2d m_lastLimelightPose;
  Pose2d m_lastDrivePose;

  /** Creates a new FieldOdometryCommand. */
  public FieldOdometryCommand(DriveSubsystem drive, LimelightSubsystem frontLimelight, LimelightSubsystem backLimelight) {

    this.m_drive = drive;
    this.m_frontLimeight = frontLimelight;
    this.m_backLimelight = backLimelight;

    this.m_field = new Field2d();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_backLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.m_lastLimelightPose = new Pose2d();


    SmartDashboard.putData("Field Real", m_field);
    System.out.println("SHOWING!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(this.m_frontLimeight.hasValidTarget()) {
      this.m_lastLimelightPose = this.m_frontLimeight.getRobotPoseInFieldSpace();
      this.m_lastDrivePose = this.m_drive.getPose();
    } else if(this.m_backLimelight.hasValidTarget()) {
      this.m_lastLimelightPose = this.m_backLimelight.getRobotPoseInFieldSpace().plus(new Transform2d(0.0, 0.305, new Rotation2d(Math.PI)));
      this.m_lastDrivePose = this.m_drive.getPose();
    }

    if(this.m_lastLimelightPose == null || this.m_lastDrivePose == null){
      this.m_field.setRobotPose(this.m_drive.getPose());
      return;
    }


    Pose2d currentPose = new Pose2d(
      this.m_drive.getPose().getX() - this.m_lastDrivePose.getX() + this.m_lastLimelightPose.getX(),
      this.m_drive.getPose().getY() - this.m_lastDrivePose.getY()  + this.m_lastLimelightPose.getY(),
      this.m_drive.getPose().getRotation().minus(this.m_lastDrivePose.getRotation()).plus(this.m_lastLimelightPose.getRotation())
    );


    this.m_field.setRobotPose(currentPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Returns true if the command should run when disabled.
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
