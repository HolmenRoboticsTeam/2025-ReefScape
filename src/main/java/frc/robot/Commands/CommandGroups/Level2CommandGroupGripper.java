// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.GripperDropCommand;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel2Command;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToSecondStageCommand;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel2Command;
import frc.robot.Commands.GripperPivotCommands.GripperPivotToLevel2Command;
import frc.robot.subsystems.ElevatorExtensionSubsystem;
import frc.robot.subsystems.ElevatorPivotSubsystem;
import frc.robot.subsystems.GripperPivotSubsystem;
import frc.robot.subsystems.GripperIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Level2CommandGroupGripper extends SequentialCommandGroup {
  /** Creates a new AprilTagLevel2Command. */
  public Level2CommandGroupGripper(ElevatorPivotSubsystem elevatorPivot, ElevatorExtensionSubsystem elevatorExtension, GripperPivotSubsystem gripperPivot, GripperIntakeSubsystem gripperIntake) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ElevatorPivotToLevel2Command(elevatorPivot, true),

      new ParallelDeadlineGroup(
          new ElevatorExtensionToSecondStageCommand(elevatorExtension, true), //Group waits on this command
        new ElevatorPivotToLevel2Command(elevatorPivot, false)
      ),

      new ParallelDeadlineGroup(
          new GripperPivotToLevel2Command(gripperPivot, true),
        new ElevatorExtensionToLevel2Command(elevatorExtension, false),
        new ElevatorPivotToLevel2Command(elevatorPivot, false)
      ),

      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new GripperDropCommand(gripperIntake),
        new GripperPivotToLevel2Command(gripperPivot, false),
        new ElevatorExtensionToLevel2Command(elevatorExtension, false),
        new ElevatorPivotToLevel2Command(elevatorPivot, false)
      )

    );
  }
}
