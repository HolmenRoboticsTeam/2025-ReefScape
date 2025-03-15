// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPivotConstants;

public class ElevatorPivotSubsystem extends SubsystemBase {

  private SparkMax m_leftPivotMotor;
  private SparkMax m_rightPivotMotor;
  private AbsoluteEncoder m_pivotAbsoluteEncoder;
  private RelativeEncoder m_leftPivotEncoder;
  private RelativeEncoder m_rightPivotEncoder;

  private ProfiledPIDController m_leftPIDController;
  private ProfiledPIDController m_rightPIDController;


  /** Creates a new PivotSubsystem. */
  public ElevatorPivotSubsystem() {

    this.m_leftPivotMotor = new SparkMax(ElevatorPivotConstants.kLeftMotorID, MotorType.kBrushless);
    this.m_rightPivotMotor = new SparkMax(ElevatorPivotConstants.kRightMotorID, MotorType.kBrushless);
    this.m_pivotAbsoluteEncoder = this.m_rightPivotMotor.getAbsoluteEncoder();

    this.m_leftPivotEncoder = this.m_leftPivotMotor.getEncoder();
    this.m_rightPivotEncoder = this.m_rightPivotMotor.getEncoder();

    this.m_leftPIDController = new ProfiledPIDController(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD,
      new TrapezoidProfile.Constraints(0.1, 0.1)
    );

    this.m_rightPIDController = new ProfiledPIDController(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD,
      new TrapezoidProfile.Constraints(0.1, 0.1)
    );

    SparkMaxConfig configLeftPivot = new SparkMaxConfig();
    SparkMaxConfig configRightPivot = new SparkMaxConfig();

    configLeftPivot.inverted(false);
    configRightPivot.inverted(true);

    double positionConversionFactorRelative = (2.0 * Math.PI) / (213.33);  // 213.33 is gear ratio

    configLeftPivot.encoder.positionConversionFactor(positionConversionFactorRelative);
    configRightPivot.encoder.positionConversionFactor(positionConversionFactorRelative);

    configRightPivot.absoluteEncoder.inverted(true);

    configLeftPivot.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);
    configRightPivot.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

    this.m_rightPivotMotor.configure(configRightPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_leftPivotMotor.configure(configLeftPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.m_leftPivotEncoder.setPosition(this.m_pivotAbsoluteEncoder.getPosition() * (Math.PI / 2.0));
    this.m_rightPivotEncoder.setPosition(this.m_pivotAbsoluteEncoder.getPosition() * (Math.PI / 2.0));

  }

  /**
   *
   * @param targetAngle - the angle the motors are set to move to.
   */
  public void setTargetAngle(double targetAngle) {

    double leftOutput = -1.0 * this.m_leftPIDController.calculate(targetAngle - this.getCurrentAngle());
    double rightOutput = -1.0 * this.m_rightPIDController.calculate(targetAngle - this.getCurrentAngle());

    this.m_leftPivotMotor.set(leftOutput);
    this.m_rightPivotMotor.set(rightOutput);
  }

  /**
   * 
   * @return returns the current angle value of the motor
   */
  public double getCurrentAngle() {
    return this.m_rightPivotEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CurrentAngle", getCurrentAngle());

  }
}
