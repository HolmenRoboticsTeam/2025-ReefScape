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

  public static final class GlobalVariables {

    public static boolean gripperAtSetpoint = false;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3.0;

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
    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;

    public static final double kTurningP = 0.01;
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

    public static final int kLeftMotorID = 10;
    public static final int kRightMotorID = 11;

    public static final double kMotorToPivotGearRatio = 213.33;

    public static final int kMaxCurrentLimit = 20;
    public static final double kMaxVoltage = 12;

    public static final double kMaxVelocity = 0.05;
    public static final double kMaxAcceleration = 0.01;

    public static final double kPivotP = 1.2;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;

    public static final double kHomeAngle = Rotation2d.fromDegrees(30).getRadians();

    public static final double kLevel1Angle = Rotation2d.fromDegrees(0).getRadians();
    public static final double kLevel2Angle = Rotation2d.fromDegrees(62.0).getRadians();
    public static final double kLevel3Angle = Rotation2d.fromDegrees(75.0).getRadians();
    public static final double kLevel4Angle = Rotation2d.fromDegrees(82.0).getRadians();
    public static final double kCoralStationAngle = Rotation2d.fromDegrees(70.0).getRadians();

    public static final double kUpperAlgeaRemove = Rotation2d.fromDegrees(70.0).getRadians();

    public static final double kAngleErrorAllowed = 0.1;

  }

  public static final class ElevatorExtensionConstants {

    public static final int kLeftMotorID = 12;
    public static final int kRightMotorID = 13;

    public static final double kMotorToDrumGearRatio = 7.2;

    public static final int kMaxCurrentLimit = 60;
    public static final double kMaxVoltage = 12;

    public static final double kMaxVelocity = 0.000000000005;
    public static final double kMaxAcceleration = 0.0001;

    public static final double kExtensionP = 2.7;
    public static final double kExtensionI = 0.0;
    public static final double kExtensionD = 0.0;

    //All of the following are in meters
    public static final double kHomeExtension = 0.01;

    //These 2 are mid points to stop gravity from slamming the elevator into the end (Should not be needed)
    // public static final double kPreHomeExtension = 0.28;
    // public static final double kFarHomeExtension = 0.48;

    public static final double kLevel1Extend = 0.0;
    public static final double kLevel2Extend = 0.3;
    public static final double kLevel3Extend = 0.68;
    public static final double kLevel4Extend = 1.19;
    public static final double kCoralStationExtend = 0.03;

    public static final double kSecondStageTrip = 0.75;

    public static final double kExtensionErrorAllowed = 0.05;
  }

  public static final class GripperPivotConstants {

    public static final int kMotorID = 14;

    public static final int kMaxCurrentLimit = 25;
    public static final double kMaxVoltage = 12;

    public static final double kMaxVelocity = 0.0005;
    public static final double kMaxAcceleration = 0.0001;

    public static final double kPivotP = 0.4;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;

    public static final double kSlackError = -15.0;

    public static final double kHomeAngle = Rotation2d.fromDegrees(0.0).getRadians();

    public static final double kLevel1Angle = Rotation2d.fromDegrees(90.0).getRadians();
    public static final double kLevel2Angle = Rotation2d.fromDegrees(170.0 + kSlackError).getRadians();
    public static final double kLevel3Angle = Rotation2d.fromDegrees(182.0 + kSlackError).getRadians();
    public static final double kLevel4Angle = Rotation2d.fromDegrees(175.0 + kSlackError).getRadians();
    public static final double kCoralStationAngle = Rotation2d.fromDegrees(27.0).getRadians();

    public static final double kUpperAlgeaRemove = Rotation2d.fromDegrees(160.0 + kSlackError).getRadians();
    public static final double kLowerAlgeaRemove = Rotation2d.fromDegrees(160.0 + kSlackError).getRadians();

    public static final double kAngleErrorAllowed = 0.1;
  }

  public static final class GripperIntakeConstants {

    public static final double kMaxSpeed = 0.66;

    public static final int kMotorID = 15;

    public static final int kMaxCurrentLimit = 40;
    public static final double kMaxVoltage = 12;
  }

  public static final class TakeOverTelopConstants {

    public static final double kMaxSpeed = 0.333;
    public static final double kMaxRotationSpeed = 0.1;
    public static final double kMaxErrorDistance = 0.03; //Meters
    public static final double kFrontLimeLightToFrame = 0.38; //Meters
    public static final double kBackLimeLightToFrame = 0.06; //Meters

    //Y Distance is the forward distance when the robot is facing the april tag
    public static final double kReefYDistance = 0.09; //Meters (from frame to reef wall)
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}


