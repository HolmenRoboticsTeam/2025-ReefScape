// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.BackReefLineUpCommand;
import frc.robot.Commands.GripperDropCommand;
import frc.robot.Commands.RumbleCommand;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel2Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel2Command;
import frc.robot.Commands.GripperPivotCommands.GripperPivotToLevel2Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorExtensionSubsystem;
import frc.robot.subsystems.ElevatorPivotSubsystem;
import frc.robot.subsystems.GripperPivotSubsystem;
import frc.robot.subsystems.GripperIntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilTagLevel2CommandGroup extends SequentialCommandGroup {
  /** Creates a new AprilTagLevel2Command. */
  public AprilTagLevel2CommandGroup(CommandXboxController controller, DriveSubsystem drive, LimelightSubsystem limelight, ElevatorPivotSubsystem elevatorPivot,
    ElevatorExtensionSubsystem elevatorExtension, GripperPivotSubsystem gripperPivot, GripperIntakeSubsystem gripperIntake) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BackReefLineUpCommand(drive, limelight),

      new ElevatorPivotToLevel2Command(elevatorPivot, true),

      //Moves extension, and keeps pivot up
      new ParallelDeadlineGroup(
          new ElevatorExtensionToLevel2Command(elevatorExtension, true), //Group waits on this command
        new ElevatorPivotToLevel2Command(elevatorPivot, false)
      ),

      //Moves gripper pivot, and keeps pivot and extension up
      new ParallelDeadlineGroup(
          new GripperPivotToLevel2Command(gripperPivot, true), //Group waits on this command
        new ElevatorExtensionToLevel2Command(elevatorExtension, false),
        new ElevatorPivotToLevel2Command(elevatorPivot, false)
      ),

      //Scores coral, and keeps pivot, extension, and gripper pivot up
      new ParallelDeadlineGroup(
          new WaitCommand(0.33), //Group waits on this command
        new GripperDropCommand(gripperIntake),
        new GripperPivotToLevel2Command(gripperPivot, true),
        new ElevatorExtensionToLevel2Command(elevatorExtension, false),
        new ElevatorPivotToLevel2Command(elevatorPivot, false)
      ),

      //Drops gripper pivot and rumbles, and keeps pivot and extension up
      new ParallelDeadlineGroup(
          new WaitCommand(0.33), //Group waits on this command
        new RumbleCommand(controller, 0.25, true),
        new ElevatorExtensionToLevel2Command(elevatorExtension, false),
        new ElevatorPivotToLevel2Command(elevatorPivot, false)
      ),

      //Tells driver all subsystem are going to home
      new ParallelDeadlineGroup(
          new WaitCommand(0.1), //Group waits on this command
        new RumbleCommand(controller, 1.0, false)
      )

    );
  }
}
