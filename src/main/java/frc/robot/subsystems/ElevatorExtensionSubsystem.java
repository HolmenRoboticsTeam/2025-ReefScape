// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorExtensionConstants;

public class ElevatorExtensionSubsystem extends SubsystemBase {

  private SparkMax m_leftExtensionMotor;
  private SparkMax m_rightExtensionMotor;
  private RelativeEncoder m_extensionRelativeEncoder;

  private SparkClosedLoopController m_rightExtensionPIDController;
  private SparkClosedLoopController m_leftExtensionPIDController;


  /** Creates a new ElevatorExtensionSubsystem. */
  public ElevatorExtensionSubsystem() {

    this.m_leftExtensionMotor = new SparkMax(ElevatorExtensionConstants.kLeftMotorID, MotorType.kBrushless);
    this.m_rightExtensionMotor = new SparkMax(ElevatorExtensionConstants.kRightMotorID, MotorType.kBrushless);
    this.m_extensionRelativeEncoder = this.m_rightExtensionMotor.getEncoder();

    this.m_rightExtensionPIDController = m_rightExtensionMotor.getClosedLoopController();
    this.m_leftExtensionPIDController = m_leftExtensionMotor.getClosedLoopController();

    SparkMaxConfig configLeftExtension = new SparkMaxConfig();
    SparkMaxConfig configRightExtension = new SparkMaxConfig();

    configLeftExtension.inverted(false);
    configRightExtension.inverted(true);


    configLeftExtension.closedLoop.outputRange(-ElevatorExtensionConstants.kMaxExtensionOutput, ElevatorExtensionConstants.kMaxExtensionOutput);
    configRightExtension.closedLoop.outputRange(-ElevatorExtensionConstants.kMaxExtensionOutput, ElevatorExtensionConstants.kMaxExtensionOutput);

    configLeftExtension.closedLoop.pid(ElevatorExtensionConstants.kExtensionP, ElevatorExtensionConstants.kExtensionI, ElevatorExtensionConstants.kExtensionD);
    configRightExtension.closedLoop.pid(ElevatorExtensionConstants.kExtensionP, ElevatorExtensionConstants.kExtensionI, ElevatorExtensionConstants.kExtensionD);

    this.m_rightExtensionMotor.configure(configRightExtension, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_leftExtensionMotor.configure(configLeftExtension, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("ExtensionP", ElevatorExtensionConstants.kExtensionP);
    SmartDashboard.putNumber("ExtensionI", ElevatorExtensionConstants.kExtensionI);
    SmartDashboard.putNumber("ExtensionD", ElevatorExtensionConstants.kExtensionD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CurrentExtension", getCurrentExtension());

  }

  /**
   * 
   * @param targetLength - the length the motors are set to extend to.
   */
  public void setTargetExtension(double targetLength) {

    this.m_rightExtensionPIDController.setReference(targetLength, ControlType.kPosition);
    this.m_leftExtensionPIDController.setReference(targetLength, ControlType.kPosition);
  }

  /**
   * 
   * @return returns the current length value of the motor
   */
  public double getCurrentExtension() {
    return this.m_extensionRelativeEncoder.getPosition();
  }
}
