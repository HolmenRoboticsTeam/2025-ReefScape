// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RumbleCommand extends Command {

  private CommandXboxController m_xBoxController;

  private double m_rumbleLeft;
  private double m_rumbleRight;


  /** Creates a new RumbleCommand. */
  public RumbleCommand(CommandXboxController xBoxController, double rumbleLeft, double rumbleRight) {

    this.m_xBoxController = xBoxController;

    this.m_rumbleLeft = rumbleLeft;
    this.m_rumbleRight = rumbleRight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.m_xBoxController.setRumble(RumbleType.kLeftRumble, this.m_rumbleLeft);
    this.m_xBoxController.setRumble(RumbleType.kRightRumble, this.m_rumbleRight);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_xBoxController.setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
