// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPivotConstants;

public class ElevatorPivotSubsystem extends SubsystemBase {

  private SparkMax m_leftPivotMotor;
  private SparkMax m_rightPivotMotor;
  private AbsoluteEncoder m_pivotAbsoluteEncoder;

  private SparkClosedLoopController m_rightPivotPIDController;
  private SparkClosedLoopController m_leftPivotPIDController;

  /** Creates a new PivotSubsystem. */
  public ElevatorPivotSubsystem() {

    this.m_leftPivotMotor = new SparkMax(ElevatorPivotConstants.kLeftMotorID, MotorType.kBrushless);
    this.m_rightPivotMotor = new SparkMax(ElevatorPivotConstants.kRightMotorID, MotorType.kBrushless);
    this.m_pivotAbsoluteEncoder = this.m_rightPivotMotor.getAbsoluteEncoder();

    this.m_rightPivotPIDController = m_rightPivotMotor.getClosedLoopController();
    this.m_leftPivotPIDController = m_leftPivotMotor.getClosedLoopController();

    SparkMaxConfig configLeftPivot = new SparkMaxConfig();
    SparkMaxConfig configRightPivot = new SparkMaxConfig();

    configLeftPivot.inverted(false);
    configRightPivot.inverted(true);

    configLeftPivot.absoluteEncoder.positionConversionFactor(1);
    configRightPivot.absoluteEncoder.positionConversionFactor(1);

    configLeftPivot.closedLoop.pid(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD);
    configRightPivot.closedLoop.pid(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD);

    this.m_rightPivotMotor.configure(configRightPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_leftPivotMotor.configure(configLeftPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("PivotP", ElevatorPivotConstants.kPivotP);
    SmartDashboard.putNumber("PivotI", ElevatorPivotConstants.kPivotI);
    SmartDashboard.putNumber("PivotD", ElevatorPivotConstants.kPivotD);

  }

  /**
   *
   * @param targetAngle - the angle the motors are set to move to.
   */
  public void setTargetAngle(double targetAngle) {

    System.out.println(targetAngle);

    this.m_rightPivotPIDController.setReference(targetAngle, ControlType.kPosition);
    this.m_leftPivotPIDController.setReference(targetAngle, ControlType.kPosition);
  }

  /**
   * 
   * @return returns the current angle value of the motor
   */
  public double getCurrentAngle() {
    return this.m_pivotAbsoluteEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CurrentAngle", getCurrentAngle());

  }
}
