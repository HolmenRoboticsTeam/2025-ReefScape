// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ReefLineUpCommand;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToHomeCommand;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel2Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel3Command;
import frc.robot.Commands.ElevatorPivotCommands.ElevatorPivotToLevel4Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorPivotSubsystem;
import frc.robot.subsystems.GripperPivotSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorPivotSubsystem m_robotElevatorPivot = new ElevatorPivotSubsystem();
  private final GripperPivotSubsystem m_gripper = new GripperPivotSubsystem();
  private final LimelightSubsystem m_limelightFront = new LimelightSubsystem("limelight-front");
  private final LimelightSubsystem m_limelightBack = new LimelightSubsystem("limelight-back");

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_buttonBox = new XboxController(OIConstants.kButtonBoxControllerPort);

  private final JoystickButton m_AprilTagLevel2Place = new JoystickButton(m_buttonBox, 0);
  private final JoystickButton m_AprilTagLevel3Place = new JoystickButton(m_buttonBox, 1);
  private final JoystickButton m_AprilTagLevel4Place = new JoystickButton(m_buttonBox, 2);
  private final JoystickButton m_AprilTagCoralStationGrab = new JoystickButton(m_buttonBox, 3);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  public RobotContainer() {

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.headingDrive(
                MathUtil.applyDeadband(this.m_driverController.getRightTriggerAxis(), 0.05),
                MathUtil.applyDeadband(-this.m_driverController.getLeftY(), 0.1),
                MathUtil.applyDeadband(-this.m_driverController.getLeftX(), 0.1),
                MathUtil.applyDeadband(this.m_driverController.getRightX(), 0.1),
                MathUtil.applyDeadband(this.m_driverController.getRightY(), 0.1),
                true
                ),
            m_robotDrive));

    this.m_robotElevatorPivot.setDefaultCommand(
      new ElevatorPivotToHomeCommand(m_robotElevatorPivot)
    );

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
    this.m_AprilTagLevel2Place.onTrue(new ReefLineUpCommand(this.m_robotDrive, this.m_limelightFront));
    this.m_AprilTagLevel2Place.onTrue(new ReefLineUpCommand(this.m_robotDrive, this.m_limelightBack));

    //Planned for real Matches (need to add levels 3, 4, and station. Also check wait time.) (AND TEST BEFORE COMP!)
    //This got a lot bigger than planned, think about making this shorter/braking up (especially for testing!).
    // this.m_AprilTagLevel2Place.whileTrue(new SequentialCommandGroup(
    //   new ReefLineUpCommand(this.m_robotDrive, this.m_limelightFront),
    //   new ElevatorPivotToLevel2Command(this.m_robotElevatorPivot, true),
    //   new ParallelRaceGroup(
    //     new WaitCommand(0.33),
    //     new GripperDropCommand(this.m_gripper),
    //     new ElevatorPivotToLevel2Command(this.m_robotElevatorPivot, false)
    //   ),
    //   new ParallelRaceGroup(
    //     new WaitCommand(0.33),
    //     new RumbleCommand(m_driverController, 0.25, 0.25),
    //     new ElevatorPivotToLevel2Command(this.m_robotElevatorPivot, false)
    //   )
    // ));

    this.m_driverController.x()
      .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive
        ));


    this.m_driverController.povDown().whileTrue(
      new ElevatorPivotToLevel2Command(this.m_robotElevatorPivot, false)
    );
    this.m_driverController.povRight().whileTrue(
      new ElevatorPivotToLevel3Command(this.m_robotElevatorPivot, false)
    );
    this.m_driverController.povUp().whileTrue(
      new ElevatorPivotToLevel4Command(this.m_robotElevatorPivot, false)
    );
    this.m_driverController.povLeft().whileTrue(
      new ElevatorPivotToLevel4Command(this.m_robotElevatorPivot, false)
    );
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
