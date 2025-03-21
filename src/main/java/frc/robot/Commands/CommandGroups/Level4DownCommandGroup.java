// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.GripperDropCommand;
import frc.robot.Commands.RumbleCommand;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToHomeCommand;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel3Command;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel4Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel3Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel4Command;
import frc.robot.Commands.GripperPivotCommands.GipperPivotToHomeCommand;
import frc.robot.Commands.GripperPivotCommands.GripperPivotToLevel3Command;
import frc.robot.subsystems.ElevatorExtensionSubsystem;
import frc.robot.subsystems.ElevatorPivotSubsystem;
import frc.robot.subsystems.GripperIntakeSubsystem;
import frc.robot.subsystems.GripperPivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Level4DownCommandGroup extends SequentialCommandGroup {
  /** Creates a new Level3DownCommandGroup. */
  public Level4DownCommandGroup(CommandXboxController controller, ElevatorPivotSubsystem elevatorPivot,
    ElevatorExtensionSubsystem elevatorExtension, GripperPivotSubsystem gripperPivot, GripperIntakeSubsystem gripperIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    //Scores coral, and keeps pivot, extension, and gripper pivot up
      new ParallelDeadlineGroup(
          new GipperPivotToHomeCommand(gripperPivot, true),
        new ElevatorExtensionToLevel4Command(elevatorExtension, false),
        new ElevatorPivotToLevel4Command(elevatorPivot, false)
      ),

      //Drops gripper pivot, extension, and rumbles, but keeps pivot up
      new ParallelDeadlineGroup(
        new ElevatorExtensionToHomeCommand(elevatorExtension, true),
        new ElevatorPivotToLevel4Command(elevatorPivot, false)
      )

    );
  }
}
