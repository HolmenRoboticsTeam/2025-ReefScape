// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  public Level4CommandGroup(ElevatorPivotSubsystem elevatorPivot, ElevatorExtensionSubsystem elevatorExtension, GripperPivotSubsystem gripperPivot) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ElevatorPivotToLevel4Command(elevatorPivot, true),

      new ParallelDeadlineGroup(
          new ElevatorExtensionToLevel4Command(elevatorExtension, true),  //Group waits on this command
        new ElevatorPivotToLevel4Command(elevatorPivot, false)
      ),

      new ParallelDeadlineGroup(
          new GripperPivotToLevel4Command(gripperPivot, true), //Group waits on this command
        new ElevatorExtensionToLevel4Command(elevatorExtension, false),
        new ElevatorPivotToLevel4Command(elevatorPivot, false)
      ),

      new ParallelCommandGroup(
        new GripperPivotToLevel4Command(gripperPivot, false),
        new ElevatorExtensionToLevel4Command(elevatorExtension, false),
        new ElevatorPivotToLevel4Command(elevatorPivot, false)
      )
    );
  }
}
