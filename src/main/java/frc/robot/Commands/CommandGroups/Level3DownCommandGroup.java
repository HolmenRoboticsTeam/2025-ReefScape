// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToHomeCommand;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel3Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel3Command;
import frc.robot.Commands.GripperPivotCommands.GipperPivotToHomeCommand;
import frc.robot.subsystems.ElevatorExtensionSubsystem;
import frc.robot.subsystems.ElevatorPivotSubsystem;
import frc.robot.subsystems.GripperIntakeSubsystem;
import frc.robot.subsystems.GripperPivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Level3DownCommandGroup extends SequentialCommandGroup {
  /** Creates a new Level3DownCommandGroup. */
  public Level3DownCommandGroup(ElevatorPivotSubsystem elevatorPivot, ElevatorExtensionSubsystem elevatorExtension, GripperPivotSubsystem gripperPivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ParallelDeadlineGroup(
          new GipperPivotToHomeCommand(gripperPivot, true),  //Group waits on this command
        new ElevatorExtensionToLevel3Command(elevatorExtension, false),
        new ElevatorPivotToLevel3Command(elevatorPivot, false)
      ),

      new ParallelDeadlineGroup(
          new ElevatorExtensionToHomeCommand(elevatorExtension, true),  //Group waits on this command
        new ElevatorPivotToLevel3Command(elevatorPivot, false)
      )

    );
  }
}
