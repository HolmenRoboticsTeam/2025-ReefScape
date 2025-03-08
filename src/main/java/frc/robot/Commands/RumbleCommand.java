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

  private double m_rumble;
  private double m_maxRumble;


  private boolean m_isPulsing;
  private boolean m_isIncreasing;
  /** Creates a new RumbleCommand. */
  public RumbleCommand(CommandXboxController xBoxController, double rumble, boolean isPulsing) {

    this.m_xBoxController = xBoxController;

    this.m_rumble = rumble;
    this.m_maxRumble = rumble;

    //Pulsing rumble might just be pointless or could be useful, needs testing
    this.m_isPulsing = isPulsing;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(this.m_isPulsing) {
      this.m_rumble += this.m_isIncreasing ? 0.01 : -0.01;
    }
    if(this.m_rumble < 0.0 || this.m_rumble > this.m_maxRumble) {
      this.m_isIncreasing = !this.m_isIncreasing;
    }

    this.m_xBoxController.setRumble(RumbleType.kBothRumble, this.m_rumble);

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
