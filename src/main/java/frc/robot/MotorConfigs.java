package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorExtensionConstants;
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.Constants.GripperIntakeConstants;
import frc.robot.Constants.GripperPivotConstants;
import frc.robot.Constants.ModuleConstants;

public final class MotorConfigs {

  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
          / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20);
      turningConfig.absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class ElevatorPivotConfig {
    public static final SparkMaxConfig leftPivotConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightPivotConfig = new SparkMaxConfig();

    static {
      leftPivotConfig.inverted(false);
      rightPivotConfig.inverted(true);

      // Changes all inputs and outputs from motor rotations to pivot angle in radians
      double positionConversionFactorRelative = (2.0 * Math.PI) / ElevatorPivotConstants.kMotorToPivotGearRatio;
      leftPivotConfig.encoder.positionConversionFactor(positionConversionFactorRelative);
      rightPivotConfig.encoder.positionConversionFactor(positionConversionFactorRelative);

      rightPivotConfig.absoluteEncoder.inverted(true);

      leftPivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorPivotConstants.kMaxCurrentLimit)
          .voltageCompensation(ElevatorPivotConstants.kMaxVoltage);
      rightPivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorPivotConstants.kMaxCurrentLimit)
          .voltageCompensation(ElevatorPivotConstants.kMaxVoltage);
    }
  }

  public static final class ElevatorExtensionConfig {
    public static final SparkMaxConfig leftExtensionConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightExtensionConfig = new SparkMaxConfig();

    static {
      leftExtensionConfig.inverted(false);
      rightExtensionConfig.inverted(true);

      // Turns inputs and outputs from motor rotation into the extension
      // 0.02425 is r (2.0 * Math.PI * 0.02425) = 0.15236724 meters, 0.16269 seems to
      // be a better conversion?
      double positionConversionFactor = (0.16269) / ElevatorExtensionConstants.kMotorToDrumGearRatio;

      leftExtensionConfig.encoder.positionConversionFactor(positionConversionFactor);
      rightExtensionConfig.encoder.positionConversionFactor(positionConversionFactor);

      leftExtensionConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorExtensionConstants.kMaxCurrentLimit)
          .voltageCompensation(ElevatorExtensionConstants.kMaxVoltage);
      rightExtensionConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorExtensionConstants.kMaxCurrentLimit)
          .voltageCompensation(ElevatorExtensionConstants.kMaxVoltage);
    }
  }

  public static final class GripperPivotConfig {
    public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    static {
      pivotConfig.inverted(true);

      // Changes all inputs and outputs from motor rotations to pivot angle in radians
      double positionConversionFactorRelative = (2 * Math.PI) / 48.0;
      pivotConfig.encoder.positionConversionFactor(positionConversionFactorRelative);
      pivotConfig.encoder.positionConversionFactor(positionConversionFactorRelative);

      pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(GripperPivotConstants.kMaxCurrentLimit)
          .voltageCompensation(GripperPivotConstants.kMaxVoltage);
    }
  }

  public static final class GripperIntakeConfig {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      intakeConfig.inverted(false);

      intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(GripperIntakeConstants.kMaxCurrentLimit)
          .voltageCompensation(GripperIntakeConstants.kMaxVoltage);
    }
  }
}
