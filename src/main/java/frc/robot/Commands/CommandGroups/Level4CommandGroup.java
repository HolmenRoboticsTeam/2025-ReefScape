// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel4Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel4Command;
import frc.robot.Commands.GripperPivotCommands.GripperPivotToLevel4Command;
import frc.robot.subsystems.ElevatorExtensionSubsystem;
import frc.robot.subsystems.ElevatorPivotSubsystem;
import frc.robot.subsystems.GripperPivotSubsystem;
import frc.robot.subsystems.GripperIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Level4CommandGroup extends SequentialCommandGroup {
  /** Creates a new AprilTagLevel2Command. */
  public Level4CommandGroup(CommandXboxController controller, ElevatorPivotSubsystem elevatorPivot,
    ElevatorExtensionSubsystem elevatorExtension, GripperPivotSubsystem gripperPivot, GripperIntakeSubsystem gripperIntake) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ElevatorPivotToLevel4Command(elevatorPivot, true),

      //Moves extension, but keeps pivot up
      new ParallelDeadlineGroup(
          new ElevatorExtensionToLevel4Command(elevatorExtension, true),
        new ElevatorPivotToLevel4Command(elevatorPivot, false)
      ),

      //Moves gripper pivot and extension, but keeps pivot up
      new ParallelDeadlineGroup(
          new GripperPivotToLevel4Command(gripperPivot, false), //Group waits on this command
        new ElevatorExtensionToLevel4Command(elevatorExtension, false),
        new ElevatorPivotToLevel4Command(elevatorPivot, false)
      )

      // //Scores coral, and keeps pivot, extension, and gripper pivot up
      // new ParallelDeadlineGroup(
      //     new WaitCommand(0.5), //Group waits on this command
      //   new GripperDropCommand(gripperIntake),
      //   new GripperPivotToLevel4Command(gripperPivot, true),
      //   new ElevatorExtensionToLevel4Command(elevatorExtension, false),
      //   new ElevatorPivotToLevel4Command(elevatorPivot, false)
      // ),

      // //Drops gripper pivot, extension, and rumbles, but keeps pivot up
      // new ParallelDeadlineGroup(
      //     new WaitCommand(0.5), //Group waits on this command
      //   new RumbleCommand(controller, 0.25, true),
      //   new ElevatorExtensionToLevel4Command(elevatorExtension, false),
      //   new ElevatorPivotToLevel4Command(elevatorPivot, false)
      // ),

      // //Tells driver all subsystem are going to home
      // new ParallelDeadlineGroup(
      //     new WaitCommand(0.1), //Group waits on this command
      //   new RumbleCommand(controller, 1.0, false),
      //   new ElevatorPivotToLevel4Command(elevatorPivot, false)
      // )

    );
  }
}
