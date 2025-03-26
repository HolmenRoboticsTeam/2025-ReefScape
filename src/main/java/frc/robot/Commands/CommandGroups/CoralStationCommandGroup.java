// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.GripperGrabCommand;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToCoralStationCommand;
import frc.robot.Commands.GripperPivotCommands.GripperPivotToCoralStationCommand;
import frc.robot.Commands.GripperPivotCommands.GripperPivotToLevel1Command;
import frc.robot.subsystems.ElevatorPivotSubsystem;
import frc.robot.subsystems.GripperIntakeSubsystem;
import frc.robot.subsystems.GripperPivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralStationCommandGroup extends SequentialCommandGroup {
  /** Creates a new CoralStationCommandGroup. */
  public CoralStationCommandGroup(ElevatorPivotSubsystem elevatorPivot, GripperPivotSubsystem gripperPivot, GripperIntakeSubsystem gripperIntake) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ElevatorPivotToCoralStationCommand(elevatorPivot, true),

      new ParallelDeadlineGroup(
          new GripperPivotToLevel1Command(gripperPivot, true), //Group waits on this command
        new ElevatorPivotToCoralStationCommand(elevatorPivot, false)
      ),

      new ParallelDeadlineGroup(
          new GripperPivotToCoralStationCommand(gripperPivot, true), //Group waits on this command
        new ElevatorPivotToCoralStationCommand(elevatorPivot, false)
      ),

      new ParallelCommandGroup(
        new GripperGrabCommand(gripperIntake),
        new GripperPivotToCoralStationCommand(gripperPivot, false),
        new ElevatorPivotToCoralStationCommand(elevatorPivot, false)
      )

    );
  }
}
