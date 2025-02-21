// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorExtensionConstants;
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.subsystems.ElevatorPivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPivotToLevel1Command extends Command {

  private ElevatorPivotSubsystem elevatorPivot;

  /** Creates a new ElevatorPivotToLevel1Command. */
  public ElevatorPivotToLevel1Command(ElevatorPivotSubsystem elevatorPivot) {

    this.elevatorPivot = elevatorPivot;

    addRequirements(elevatorPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elevatorPivot.setTargetAngle(ElevatorPivotConstants.kLevel1Angle);
    elevatorPivot.setTargetExtension(ElevatorExtensionConstants.kLevel1Extend);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    elevatorPivot.setTargetAngle(0.0);
    elevatorPivot.setTargetExtension(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
