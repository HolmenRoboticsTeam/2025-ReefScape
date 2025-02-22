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

  private SparkMax m_motor;

  private SparkClosedLoopController m_motorPIDController;

  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    this.m_motor = new SparkMax(GripperConstants.kMotorID, MotorType.kBrushless);

    this.m_motorPIDController = this.m_motor.getClosedLoopController();
  }

  public void setSpeed(double speed) {
    this.m_motorPIDController.setReference(speed, ControlType.kVelocity);
  }

  public double getCurrentVoltage() {
    return this.m_motor.getBusVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
