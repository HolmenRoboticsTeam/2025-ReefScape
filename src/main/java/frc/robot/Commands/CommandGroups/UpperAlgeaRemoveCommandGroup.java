// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.GripperDropCommand;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel2Command;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel3Command;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToSecondStageCommand;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel2Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel3Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToUpperAgleaCommand;
import frc.robot.Commands.GripperPivotCommands.GripperPivotToLowerAlgeaRemoveCommand;
import frc.robot.Commands.GripperPivotCommands.GripperPivotToUpperAlgeaRemoveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorExtensionSubsystem;
import frc.robot.subsystems.ElevatorPivotSubsystem;
import frc.robot.subsystems.GripperPivotSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.GripperIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UpperAlgeaRemoveCommandGroup extends SequentialCommandGroup {
  /** Creates a new AprilTagLevel2Command. */
  public UpperAlgeaRemoveCommandGroup(ElevatorPivotSubsystem elevatorPivot, ElevatorExtensionSubsystem elevatorExtension,
  GripperPivotSubsystem gripperPivot, GripperIntakeSubsystem gripperIntake) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ElevatorPivotToUpperAgleaCommand(elevatorPivot, true),

      new ParallelDeadlineGroup(
          new ElevatorExtensionToLevel3Command(elevatorExtension, true), //Group waits on this command
        new ElevatorPivotToUpperAgleaCommand(elevatorPivot, false)
      ),

      new ParallelCommandGroup(
        new GripperDropCommand(gripperIntake),
        new GripperPivotToUpperAlgeaRemoveCommand(gripperPivot, false),
        new ElevatorExtensionToLevel3Command(elevatorExtension, false),
        new ElevatorPivotToUpperAgleaCommand(elevatorPivot, false)
      )

    );
  }

}
