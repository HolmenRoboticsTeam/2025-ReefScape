// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GripperIntakeConstants;
import frc.robot.subsystems.GripperPivotSubsystem;
import frc.robot.subsystems.GripperIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperGrabCommand extends Command {

  private GripperIntakeSubsystem m_gripper;

  /** Creates a new GripperDropCommand. */
  public GripperGrabCommand(GripperIntakeSubsystem gripper) {

    this.m_gripper = gripper;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.m_gripper.setIntakeSpeed(-GripperIntakeConstants.kMaxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.m_gripper.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return this.m_gripper.getIntakeMotorVoltage() > GripperIntakeConstants.kVoltageThreshHold;
  }
}
