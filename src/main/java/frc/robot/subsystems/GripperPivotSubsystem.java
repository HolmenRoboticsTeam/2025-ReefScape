// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperIntakeConstants;
import frc.robot.Constants.GripperPivotConstants;

public class GripperPivotSubsystem extends SubsystemBase {

  private SparkMax m_intakeMotor;
  private SparkMax m_pivotMotor;

  private SparkClosedLoopController m_intakeMotorPIDController;
  private SparkClosedLoopController m_pivotMotorPIDController;

  /** Creates a new GripperSubsystem. */
  public GripperPivotSubsystem() {
    this.m_intakeMotor = new SparkMax(GripperIntakeConstants.kMotorID, MotorType.kBrushless);
    this.m_pivotMotor = new SparkMax(GripperPivotConstants.kMotorID, MotorType.kBrushless);

    this.m_intakeMotorPIDController = this.m_intakeMotor.getClosedLoopController();
    this.m_pivotMotorPIDController = this.m_pivotMotor.getClosedLoopController();

    SparkMaxConfig configIntake = new SparkMaxConfig();
    SparkMaxConfig configPivot = new SparkMaxConfig();

    configIntake.inverted(false);
    configPivot.inverted(false);

    configIntake.closedLoop.maxOutput(GripperIntakeConstants.kMaxSpeed);
    configPivot.closedLoop.maxOutput(GripperPivotConstants.kMaxSpeed);

    configIntake.closedLoop.pid(GripperIntakeConstants.kIntakeP, GripperIntakeConstants.kIntakeI, GripperIntakeConstants.kIntakeD);
    configPivot.closedLoop.pid(GripperPivotConstants.kPivotP, GripperPivotConstants.kPivotI, GripperPivotConstants.kPivotD);

    this.m_intakeMotor.configure(configIntake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_pivotMotor.configure(configPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTargetAngle(double angle) {
    this.m_pivotMotorPIDController.setReference(angle, ControlType.kPosition);
  }

  public void setIntakeSpeed(double speed) {
    this.m_intakeMotorPIDController.setReference(speed, ControlType.kVelocity);
  }

  public double getCurrentAngle() {
    return this.m_pivotMotor.getEncoder().getPosition();
  }

  public double getIntakeMotorVoltage() {
    return this.m_intakeMotor.getBusVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
