// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.GripperPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GripperPivotConstants;
import frc.robot.subsystems.GripperPivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperPivotToLevel1Command extends Command {

  private GripperPivotSubsystem m_gripperPivot;
  private boolean m_allowEndCondition;

  /** Creates a new GripperPivotToLevel1Command. */
  public GripperPivotToLevel1Command(GripperPivotSubsystem gripperPivot, boolean allowEndCondition) {

    this.m_gripperPivot = gripperPivot;
    this.m_allowEndCondition = allowEndCondition;

    addRequirements(gripperPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.m_gripperPivot.setTargetAngle(GripperPivotConstants.kLevel1Angle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angleError = Math.abs(this.m_gripperPivot.getCurrentAngle() - GripperPivotConstants.kLevel1Angle);

    return angleError < GripperPivotConstants.kAngleErrorAllowed + 0.5 &&
    this.m_allowEndCondition;
  }
}
