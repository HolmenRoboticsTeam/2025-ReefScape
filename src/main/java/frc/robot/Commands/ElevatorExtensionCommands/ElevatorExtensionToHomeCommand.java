// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorExtensionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorExtensionConstants;
import frc.robot.subsystems.ElevatorExtensionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorExtensionToHomeCommand extends Command {

  private ElevatorExtensionSubsystem m_elevatorExtension;

  /** Creates a new ElevatorExtensionToHomeCommand. */
  public ElevatorExtensionToHomeCommand(ElevatorExtensionSubsystem elevatorExtension) {

    this.m_elevatorExtension = elevatorExtension;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.m_elevatorExtension.setTargetExtension(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double positionError = Math.abs(this.m_elevatorExtension.getCurrentExtension() - ElevatorExtensionConstants.kLevel1Extend);

    return positionError < ElevatorExtensionConstants.kExtensionErrorAllowed;
  }
}
