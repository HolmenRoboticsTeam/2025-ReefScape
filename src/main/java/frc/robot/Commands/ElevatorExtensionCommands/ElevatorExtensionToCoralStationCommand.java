// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorExtensionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorExtensionConstants;
import frc.robot.subsystems.ElevatorExtensionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorExtensionToCoralStationCommand extends Command {

  private ElevatorExtensionSubsystem m_elevatorExtension;

  private boolean m_allowEndCondition;

  /** Creates a new ElevatorExtensionToCoralStationCommand. */
  public ElevatorExtensionToCoralStationCommand(ElevatorExtensionSubsystem elevatorExtension, boolean allowEndCondition) {

    this.m_elevatorExtension = elevatorExtension;
    
    this.m_allowEndCondition = allowEndCondition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.m_elevatorExtension.setTargetExtension(ElevatorExtensionConstants.kCoralStationExtend);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.m_elevatorExtension.setTargetExtension(ElevatorExtensionConstants.kHomeExtension);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double positionError = Math.abs(this.m_elevatorExtension.getCurrentExtension() - ElevatorExtensionConstants.kCoralStationExtend);

    return positionError < ElevatorExtensionConstants.kExtensionErrorAllowed &&
    this.m_allowEndCondition;
  }
}
