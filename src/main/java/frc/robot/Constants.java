// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 2.0;

    public static final double kMaxAngularSpeed = 2 * Math.PI ; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(29.0);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(29.0);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 9;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 2;

    public static final boolean kGyroReversed = true;

    //PID Controller
    public static final double kTurningP = 0.005;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.00025;
    public static final TrapezoidProfile.Constraints kTurningControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeed, kMaxAngularSpeed);

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kButtonBoxControllerPort = 1;

    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ElevatorPivotConstants {

    public static final double kMaxAngleOutput = 0.65;

    public static final double kAngleErrorAllowed = 0.1;

    public static final int kLeftMotorID = 10;
    public static final int kRightMotorID = 11;

    public static final double kPivotP = 0.7;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;

    public static final double kHomeAngle = Rotation2d.fromDegrees(25.0).getRadians();

    public static final double kLevel1Angle = Rotation2d.fromDegrees(0).getRadians();
    public static final double kLevel2Angle = Rotation2d.fromDegrees(57.643).getRadians();
    public static final double kLevel3Angle = Rotation2d.fromDegrees(70.0).getRadians();
    public static final double kLevel4Angle = Rotation2d.fromDegrees(76.0).getRadians();
    public static final double kCoralStationAngle = Rotation2d.fromDegrees(85.0).getRadians();

  }

  public static final class ElevatorExtensionConstants {

    public static final double kMaxExtensionOutput = 0.001;

    public static final double kExtensionErrorAllowed = 0.1;

    public static final int kLeftMotorID = 12;
    public static final int kRightMotorID = 13;

    public static final double kExtensionP = 0.8;
    public static final double kExtensionI = 0.0;
    public static final double kExtensionD = 0.0;

    public static final double kHomeExtension = 0.0;

    public static final double kLevel1Extend = 0.0;
    public static final double kLevel2Extend = 0.255;
    public static final double kLevel3Extend = 0.486;
    public static final double kLevel4Extend = 1.17;
    public static final double kCoralStationExtend = 0.03;

  }

  public static final class GripperPivotConstants {
  
    public static final double kMaxSpeed = 0.1;

    public static final double kAngleErrorAllowed = 0.1;

    public static final int kMotorID = 14;

    public static final double kPivotP = 0.001;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;

    public static final double kHomeAngle = Rotation2d.fromDegrees(0.0 + 150.0).getRadians();

    public static final double kLevel1Angle = Rotation2d.fromDegrees(0.0).getRadians();
    public static final double kLevel2Angle = Rotation2d.fromDegrees(0.0 - 2.357).getRadians();
    public static final double kLevel3Angle = Rotation2d.fromDegrees(0.0 + 0.0).getRadians();
    public static final double kLevel4Angle = Rotation2d.fromDegrees(0.0 + 15).getRadians();
    public static final double kCoralStationAngle = Rotation2d.fromDegrees(0.0 - 170.0).getRadians();
  }

  public static final class GripperIntakeConstants {

    public static final double kMaxSpeed = 1.0;

    public static final int kMotorID = 15;

    //Need check if voltage is even the right thing to base it off of, and if so, what's the right number
    public static final double kVoltageThreshHold = 1.0;

  }

  public static final class TakeOverTelopConstants {

    public static final double kMaxSpeed = 1.0; // Scale based on kMaxSpeed
    public static final double kMaxErrorDistance = 0.005; //Meters
    public static final double kMaxErrorRotation = 1.0; //Degrees

    public static final double kFrontLimeLightToFrame = 0.36; //Meters
    public static final double kBackLimeLightToFrame = 0.05; //Meters

    //Y Distance is the forward distance when the robot is facing the april tag
    public static final double kReefYDistance = 0.089; //Meters (from frame to reef wall)
    public static final double kCoralStationYDistance = 0.089; //Meters

    // This double is the threshold that stops and xy movement to prevent losing the april tag
    public static final double kTranslationLockOut = 15.0; //Degrees
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}


