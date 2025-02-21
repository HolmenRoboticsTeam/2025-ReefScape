// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {

  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;

  private SparkClosedLoopController m_leftMotorPIDController;
  private SparkClosedLoopController m_rightMotorPIDController;

  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    this.m_leftMotor = new SparkMax(GripperConstants.kLeftMotorID, MotorType.kBrushless);
    this.m_rightMotor = new SparkMax(GripperConstants.kRightMotorID, MotorType.kBrushless);

    this.m_leftMotorPIDController = this.m_leftMotor.getClosedLoopController();
    this.m_rightMotorPIDController = this.m_rightMotor.getClosedLoopController();
  }

  public void setSpeed(double speed) {
    this.m_leftMotorPIDController.setReference(speed, ControlType.kVelocity);
    this.m_rightMotorPIDController.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
