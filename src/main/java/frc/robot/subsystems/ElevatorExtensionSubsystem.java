// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorExtensionConstants;
import frc.robot.MotorConfigs.ElevatorExtensionConfig;

public class ElevatorExtensionSubsystem extends SubsystemBase {

  private SparkMax m_leftExtensionMotor;
  private SparkMax m_rightExtensionMotor;
  private RelativeEncoder m_extensionRelativeEncoder;

  private ProfiledPIDController m_rightExtensionPIDController;
  private ProfiledPIDController m_leftExtensionPIDController;


  /** Creates a new ElevatorExtensionSubsystem. */
  public ElevatorExtensionSubsystem() {

    this.m_leftExtensionMotor = new SparkMax(ElevatorExtensionConstants.kLeftMotorID, MotorType.kBrushless);
    this.m_rightExtensionMotor = new SparkMax(ElevatorExtensionConstants.kRightMotorID, MotorType.kBrushless);
    this.m_extensionRelativeEncoder = this.m_rightExtensionMotor.getEncoder();

    this.m_rightExtensionPIDController = new ProfiledPIDController(ElevatorExtensionConstants.kExtensionP, ElevatorExtensionConstants.kExtensionI, ElevatorExtensionConstants.kExtensionD,
      new TrapezoidProfile.Constraints(ElevatorExtensionConstants.kMaxVelocity, ElevatorExtensionConstants.kMaxAcceleration)
    );

    this.m_leftExtensionPIDController = new ProfiledPIDController(ElevatorExtensionConstants.kExtensionP, ElevatorExtensionConstants.kExtensionI, ElevatorExtensionConstants.kExtensionD,
      new TrapezoidProfile.Constraints(ElevatorExtensionConstants.kMaxVelocity, ElevatorExtensionConstants.kMaxAcceleration)
    );

    SparkMaxConfig leftExtensionConfig = new SparkMaxConfig();
    SparkMaxConfig rightExtensionConfig = new SparkMaxConfig();

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

    this.m_leftExtensionMotor.configure(leftExtensionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_rightExtensionMotor.configure(rightExtensionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CurrentElevatorExtension", getCurrentExtension());

  }

  public void changePIDConrolles(String type) {
    
    if(type.equals("Extension")) {System.out.println("Switching To Extension");

      this.m_rightExtensionPIDController = new ProfiledPIDController(ElevatorExtensionConstants.kExtensionP, ElevatorExtensionConstants.kExtensionI, ElevatorExtensionConstants.kExtensionD,
        new TrapezoidProfile.Constraints(ElevatorExtensionConstants.kMaxVelocity, ElevatorExtensionConstants.kMaxAcceleration)
      );

      this.m_leftExtensionPIDController = new ProfiledPIDController(ElevatorExtensionConstants.kExtensionP, ElevatorExtensionConstants.kExtensionI, ElevatorExtensionConstants.kExtensionD,
        new TrapezoidProfile.Constraints(ElevatorExtensionConstants.kMaxVelocity, ElevatorExtensionConstants.kMaxAcceleration)
      );
    } else if(type.equals("Retraction")) {System.out.println("Switching To Retraction");

      this.m_rightExtensionPIDController = new ProfiledPIDController(ElevatorExtensionConstants.kRetractionP, ElevatorExtensionConstants.kRetractionI, ElevatorExtensionConstants.kRetractionD,
        new TrapezoidProfile.Constraints(ElevatorExtensionConstants.kMaxVelocity, ElevatorExtensionConstants.kMaxAcceleration)
      );

      this.m_leftExtensionPIDController = new ProfiledPIDController(ElevatorExtensionConstants.kRetractionP, ElevatorExtensionConstants.kRetractionI, ElevatorExtensionConstants.kRetractionD,
        new TrapezoidProfile.Constraints(ElevatorExtensionConstants.kMaxVelocity, ElevatorExtensionConstants.kMaxAcceleration)
      );
    }
  }

  /**
   * 
   * @param targetLength - the length the motors are set to extend to.
   */
  public void setTargetExtension(double targetLength) {

    

    System.out.println(targetLength);

    double deltaLength = targetLength - this.getCurrentExtension();

    // System.out.println(deltaLength);

    double leftOutput = -this.m_leftExtensionPIDController.calculate(deltaLength);
    double rightOutput = -this.m_rightExtensionPIDController.calculate(deltaLength);

    // System.out.println(leftOutput + ", " + rightOutput);

    this.m_leftExtensionMotor.set(leftOutput);
    this.m_rightExtensionMotor.set(rightOutput);
  }

  /**
   * 
   * @return returns the current length value of the motor
   */
  public double getCurrentExtension() {
    return this.m_extensionRelativeEncoder.getPosition();
  }
}
