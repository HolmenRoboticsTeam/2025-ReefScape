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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperPivotConstants;

public class GripperPivotSubsystem extends SubsystemBase {

  private SparkMax m_pivotMotor;

  private SparkClosedLoopController m_pivotMotorPIDController;

  /** Creates a new GripperSubsystem. */
  public GripperPivotSubsystem() {

    this.m_pivotMotor = new SparkMax(GripperPivotConstants.kMotorID, MotorType.kBrushless);

    this.m_pivotMotorPIDController = this.m_pivotMotor.getClosedLoopController();

    SparkMaxConfig configPivot = new SparkMaxConfig();

    configPivot.inverted(false);
    configPivot.closedLoop.pid(GripperPivotConstants.kPivotP, GripperPivotConstants.kPivotI, GripperPivotConstants.kPivotD);

    this.m_pivotMotor.configure(configPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTargetAngle(double angle) {
    this.m_pivotMotorPIDController.setReference(angle, ControlType.kPosition);
  }

  public double getCurrentAngle() {
    return this.m_pivotMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Gripper Angle", getCurrentAngle());
  }
}
