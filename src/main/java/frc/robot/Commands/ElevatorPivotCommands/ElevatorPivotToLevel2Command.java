// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.Constants.GlobalVariables;
import frc.robot.subsystems.ElevatorPivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPivotToLevel2Command extends Command {

  private ElevatorPivotSubsystem m_elevatorPivot;

  private boolean m_allowEndCondition;

  /** Creates a new ElevatorPivotToLevel1Command. */
  public ElevatorPivotToLevel2Command(ElevatorPivotSubsystem elevatorPivot, boolean allowEndCondition) {
    

    this.m_elevatorPivot = elevatorPivot;

    this.m_allowEndCondition = allowEndCondition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GlobalVariables.gripperAtSetpoint = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.m_elevatorPivot.setTargetAngle(ElevatorPivotConstants.kLevel2Angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.m_elevatorPivot.setTargetAngle(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double positionError = Math.abs(this.m_elevatorPivot.getCurrentAngle() - ElevatorPivotConstants.kLevel2Angle);

    return positionError < ElevatorPivotConstants.kAngleErrorAllowed &&
    this.m_allowEndCondition;
  }
}
