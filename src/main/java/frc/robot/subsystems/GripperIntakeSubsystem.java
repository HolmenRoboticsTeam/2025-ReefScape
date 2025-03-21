// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GripperIntakeConstants;
import frc.robot.MotorConfigs.GripperIntakeConfig;

public class GripperIntakeSubsystem extends SubsystemBase {

  private SparkMax m_intakeMotor;

  /** Creates a new GripperSubsystem. */
  public GripperIntakeSubsystem() {

    this.m_intakeMotor = new SparkMax(GripperIntakeConstants.kMotorID, MotorType.kBrushless);

    this.m_intakeMotor.configure(GripperIntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeSpeed(double speed) {
    this.m_intakeMotor.set(speed);
  }

  public double getIntakeMotorVoltage() {
    return this.m_intakeMotor.getBusVoltage();
  }


  @Override
  public void periodic() {
  
  }
}
