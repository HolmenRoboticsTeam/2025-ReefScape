// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.Constants.GripperPivotConstants;

public class GripperPivotSubsystem extends SubsystemBase {

  private SparkMax m_pivotMotor;

  private ProfiledPIDController  m_pivotMotorPIDController;

  /** Creates a new GripperSubsystem. */
  public GripperPivotSubsystem() {

    this.m_pivotMotor = new SparkMax(GripperPivotConstants.kMotorID, MotorType.kBrushless);

    this.m_pivotMotorPIDController = new ProfiledPIDController(GripperPivotConstants.kPivotP, GripperPivotConstants.kPivotI, GripperPivotConstants.kPivotD,
      new TrapezoidProfile.Constraints(0.01, 0.01)
    );

    SparkMaxConfig configPivot = new SparkMaxConfig();

    configPivot.inverted(true);

    double positionConversionFactor = 2.0 * Math.PI;

    configPivot.encoder.positionConversionFactor(positionConversionFactor);

    configPivot.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

    this.m_pivotMotor.configure(configPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTargetAngle(double targetAngle) {

    double output = this.m_pivotMotorPIDController.calculate(targetAngle - this.getCurrentAngle());

    this.m_pivotMotor.set(output);
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
