// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperIntakeConstants;

public class GripperIntakeSubsystem extends SubsystemBase {

  private SparkMax m_intakeMotor;

  /** Creates a new GripperSubsystem. */
  public GripperIntakeSubsystem() {

    this.m_intakeMotor = new SparkMax(GripperIntakeConstants.kMotorID, MotorType.kBrushless);

    SparkMaxConfig configIntake = new SparkMaxConfig();

    configIntake.inverted(false);

    this.m_intakeMotor.configure(configIntake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
