// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.BackReefLineUpCommand;
import frc.robot.Commands.FrontReefLineUpCommand;
import frc.robot.Commands.GripperDropCommand;
import frc.robot.Commands.GripperGrabCommand;
import frc.robot.Commands.RumbleCommand;
import frc.robot.Commands.CommandGroups.AprilTagLevel2CommandGroup;
import frc.robot.Commands.CommandGroups.AprilTagLevel3CommandGroup;
import frc.robot.Commands.CommandGroups.AprilTagLevel4CommandGroup;
import frc.robot.Commands.CommandGroups.Level2CommandGroup;
import frc.robot.Commands.CommandGroups.Level3CommandGroup;
import frc.robot.Commands.CommandGroups.Level4CommandGroup;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToCoralStationCommand;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToHomeCommand;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel2Command;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel3Command;
import frc.robot.Commands.ElevatorExtensionCommands.ElevatorExtensionToLevel4Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToCoralStationCommand;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToHomeCommand;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel1Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel2Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel3Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel4Command;
import frc.robot.Commands.GripperPivotCommands.GipperPivotToHomeCommand;
import frc.robot.Commands.GripperPivotCommands.GripperPivotToLevel2Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorExtensionSubsystem;
import frc.robot.subsystems.ElevatorPivotSubsystem;
import frc.robot.subsystems.GripperPivotSubsystem;
import frc.robot.subsystems.GripperIntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ElevatorPivotSubsystem m_elevatorPivot = new ElevatorPivotSubsystem();
  private final ElevatorExtensionSubsystem m_elevatorExtension = new ElevatorExtensionSubsystem();
  // private final GripperPivotSubsystem m_gripperPivot = new GripperPivotSubsystem();
  private final GripperIntakeSubsystem m_gripperIntake = new GripperIntakeSubsystem();

  private final LimelightSubsystem m_limelightFront = new LimelightSubsystem("limelight-front");
  private final LimelightSubsystem m_limelightBack = new LimelightSubsystem("limelight-back");

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_buttonBox = new XboxController(OIConstants.kButtonBoxControllerPort);

  private final JoystickButton m_aprilTagLevel2Place = new JoystickButton(m_buttonBox, 4);
  private final JoystickButton m_aprilTagLevel3Place = new JoystickButton(m_buttonBox, 5);
  private final JoystickButton m_aprilTagLevel4Place = new JoystickButton(m_buttonBox,6);
  private final JoystickButton m_aprilTagCoralStationGrab = new JoystickButton(m_buttonBox, 3);
  private final JoystickButton m_gripperGrab = new JoystickButton(m_buttonBox, 7);
  private final JoystickButton m_gripperDrop = new JoystickButton(m_buttonBox, 2);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // NamedCommands.registerCommand("Place Level 2", new Level2CommandGroup(
    //   this.m_driverController, this.m_elevatorPivot, this.m_elevatorExtension, this.m_gripperPivot, this.m_gripperIntake
    // ));
    // NamedCommands.registerCommand("Place Level 3", new Level3CommandGroup(
    //   this.m_driverController, this.m_elevatorPivot, this.m_elevatorExtension, this.m_gripperPivot, this.m_gripperIntake
    // ));
    // NamedCommands.registerCommand("Place Level 4", new Level4CommandGroup(
    //   this.m_driverController, this.m_elevatorPivot, this.m_elevatorExtension, this.m_gripperPivot, this.m_gripperIntake
    // ));

    // NamedCommands.registerCommand("Place Level 2 (April Tag)", new AprilTagLevel2CommandGroup(
    //   this.m_driverController, this.m_drive, this.m_limelightBack, this.m_elevatorPivot, this.m_elevatorExtension, this.m_gripperPivot, this.m_gripperIntake
    // ));
    // NamedCommands.registerCommand("Place Level 3 (April Tag)", new AprilTagLevel3CommandGroup(
    //   this.m_driverController, this.m_drive, this.m_limelightFront, this.m_elevatorPivot, this.m_elevatorExtension, this.m_gripperPivot, this.m_gripperIntake
    // ));
    // NamedCommands.registerCommand("Place Level 4 (April Tag)", new AprilTagLevel4CommandGroup(
    //   this.m_driverController, this.m_drive, this.m_limelightFront, this.m_elevatorPivot, this.m_elevatorExtension, this.m_gripperPivot, this.m_gripperIntake
    // ));

    // NamedCommands.registerCommand("Gripper Grab", new GripperGrabCommand(this.m_gripperIntake));

    // Configure default commands
    m_drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the right stick.
        new RunCommand(
            () -> this.m_drive.headingDrive(
                MathUtil.applyDeadband(this.m_driverController.getRightTriggerAxis(), 0.05),
                MathUtil.applyDeadband(-this.m_driverController.getLeftY(), 0.1),
                MathUtil.applyDeadband(-this.m_driverController.getLeftX(), 0.1),
                MathUtil.applyDeadband(this.m_driverController.getRightX(), 0.1),
                MathUtil.applyDeadband(this.m_driverController.getRightY(), 0.1),
                true
                ),
            this.m_drive));

    this.m_elevatorPivot.setDefaultCommand(
      new ElevatorPivotToHomeCommand(this.m_elevatorPivot)
    );

    this.m_elevatorExtension.setDefaultCommand(
      new ElevatorExtensionToHomeCommand(this.m_elevatorExtension)
    );

    // this.m_gripperPivot.setDefaultCommand(
    //   new GipperPivotToHomeCommand(this.m_gripperPivot, false)
    // );

    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //Just for test april tag lineups
    //need to check if driver 1 using controller: overrides the command, overrides controller, or just wack stuff
    this.m_driverController.b().whileTrue(new BackReefLineUpCommand(this.m_drive, this.m_limelightBack));
    this.m_driverController.a().whileTrue(new FrontReefLineUpCommand(this.m_drive, this.m_limelightFront));

    //Planned for real Matches (need to add levels 3, 4, and station. Also check wait time.) (AND TEST BEFORE COMP! please...)
    // this.m_aprilTagLevel2Place.whileTrue(
    //   new AprilTagLevel2CommandGroup(
    //     this.m_driverController, this.m_drive, this.m_limelightBack, this.m_elevatorPivot,
    //     this.m_elevatorExtension, this.m_gripperPivot, this.m_gripperIntake
    //   )
    // );

    this.m_aprilTagLevel2Place.whileTrue(new SequentialCommandGroup(

      new ElevatorPivotToLevel2Command(this.m_elevatorPivot, true),
      new ParallelCommandGroup(
        new ElevatorExtensionToLevel2Command(this.m_elevatorExtension, false),
        new ElevatorPivotToLevel2Command(this.m_elevatorPivot, false)
      ),
      new ParallelDeadlineGroup(
        new ElevatorExtensionToHomeCommand(this.m_elevatorExtension),
        new ElevatorPivotToLevel2Command(this.m_elevatorPivot, false)
        )
    ));

    this.m_aprilTagLevel3Place.whileTrue(new SequentialCommandGroup(
      new ElevatorPivotToLevel3Command(this.m_elevatorPivot, true),
      new ParallelCommandGroup(
        new ElevatorExtensionToLevel3Command(this.m_elevatorExtension, false),
        new ElevatorPivotToLevel3Command(this.m_elevatorPivot, false)
      ),
      new ParallelDeadlineGroup(
        new ElevatorExtensionToHomeCommand(this.m_elevatorExtension),
        new ElevatorPivotToLevel2Command(this.m_elevatorPivot, false)
        )
    ));

    this.m_aprilTagLevel4Place.whileTrue(new SequentialCommandGroup(
      new ElevatorPivotToLevel4Command(this.m_elevatorPivot, true),
      new ParallelCommandGroup(
        new ElevatorExtensionToLevel4Command(this.m_elevatorExtension, false),
        new ElevatorPivotToLevel4Command(this.m_elevatorPivot, false)
      ),
      new ParallelDeadlineGroup(
        new ElevatorExtensionToHomeCommand(this.m_elevatorExtension),
        new ElevatorPivotToLevel2Command(this.m_elevatorPivot, false)
        )
    ));

    this.m_aprilTagCoralStationGrab.whileTrue(new SequentialCommandGroup(
      new ElevatorPivotToCoralStationCommand(this.m_elevatorPivot, true),
      new ParallelCommandGroup(
        new ElevatorExtensionToCoralStationCommand(this.m_elevatorExtension, false),
        new ElevatorPivotToCoralStationCommand(this.m_elevatorPivot, false)
      ),
      new ParallelDeadlineGroup(
        new ElevatorExtensionToHomeCommand(this.m_elevatorExtension),
        new ElevatorPivotToCoralStationCommand(this.m_elevatorPivot, false)
        )
    ));

    this.m_gripperGrab.whileTrue(
      new GripperGrabCommand(this.m_gripperIntake)
    );

    this.m_gripperDrop.whileTrue(
      new GripperDropCommand(this.m_gripperIntake)
    );


    this.m_driverController.x()
      .whileTrue(new RunCommand(
        () -> m_drive.setX(),
        m_drive
      ));


    // this.m_driverController.povDown().whileTrue(
    //   new ElevatorPivotToLevel2Command(this.m_elevatorPivot, false)
    // );
    // this.m_driverController.povRight().whileTrue(
    //   new ElevatorPivotToLevel3Command(this.m_elevatorPivot, false)
    // );
    // this.m_driverController.povUp().whileTrue(
    //   new ElevatorPivotToLevel4Command(this.m_elevatorPivot, false)
    // );
    // this.m_driverController.povLeft().whileTrue(
    //   new ElevatorPivotToCoralStationCommand(this.m_elevatorPivot, false)
    // );

    // this.m_driverController.povDown().whileTrue(
    //   new ElevatorExtensionToLevel2Command(this.m_elevatorExtension, false)
    // );
    // this.m_driverController.povRight().whileTrue(
    //   new ElevatorExtensionToLevel3Command(this.m_elevatorExtension, false)
    // );
    // this.m_driverController.povUp().whileTrue(
    //   new ElevatorExtensionToLevel4Command(this.m_elevatorExtension, false)
    // );
    // this.m_driverController.povLeft().whileTrue(
    //   new ElevatorExtensionToCoralStationCommand(this.m_elevatorExtension, false)
    // );

    this.m_driverController.leftBumper().whileTrue(new ElevatorPivotToLevel2Command(this.m_elevatorPivot, false));
    this.m_driverController.rightBumper().whileTrue(new ElevatorExtensionToLevel2Command(this.m_elevatorExtension, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
