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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorExtensionConstants;
import frc.robot.Constants.ElevatorPivotConstants;

public class ElevatorPivotSubsystem extends SubsystemBase {

  private SparkMax m_leftPivotMotor;
  private SparkMax m_rightPivotMotor;
  private AbsoluteEncoder m_pivotAbsoluteEncoder;

  private SparkMax m_leftExtensionMotor;
  private SparkMax m_rightExtensionMotor;
  private RelativeEncoder m_extensionRelativeEncoder;

  private SparkClosedLoopController m_rightPivotPIDController;
  private SparkClosedLoopController m_leftPivotPIDController;

  private SparkClosedLoopController m_rightExtensionPIDController;
  private SparkClosedLoopController m_leftExtensionPIDController;


  /** Creates a new PivotSubsystem. */
  public ElevatorPivotSubsystem() {

    this.m_leftPivotMotor = new SparkMax(ElevatorPivotConstants.kLeftMotorID, MotorType.kBrushless);
    this.m_rightPivotMotor = new SparkMax(ElevatorPivotConstants.kRightMotorID, MotorType.kBrushless);
    this.m_pivotAbsoluteEncoder = this.m_rightPivotMotor.getAbsoluteEncoder();

    this.m_leftExtensionMotor = new SparkMax(ElevatorExtensionConstants.kLeftMotorID, MotorType.kBrushless);
    this.m_rightExtensionMotor = new SparkMax(ElevatorExtensionConstants.kRightMotorID, MotorType.kBrushless);
    this.m_extensionRelativeEncoder = this.m_rightExtensionMotor.getEncoder();

    this.m_rightPivotPIDController = m_rightPivotMotor.getClosedLoopController();
    this.m_leftPivotPIDController = m_leftPivotMotor.getClosedLoopController();

    this.m_rightExtensionPIDController = m_rightExtensionMotor.getClosedLoopController();
    this.m_leftExtensionPIDController = m_leftExtensionMotor.getClosedLoopController();

    SparkMaxConfig configLeftPivot = new SparkMaxConfig();
    SparkMaxConfig configRightPivot = new SparkMaxConfig();
    SparkMaxConfig configLeftExtension = new SparkMaxConfig();
    SparkMaxConfig configRightExtension = new SparkMaxConfig();

    configLeftPivot.inverted(true);
    configRightPivot.inverted(false);

    configLeftPivot.closedLoop.maxOutput(ElevatorPivotConstants.kMaxAngleSpeed);
    configRightPivot.closedLoop.maxOutput(ElevatorPivotConstants.kMaxAngleSpeed);
    
    configLeftExtension.inverted(true);
    configRightExtension.inverted(false);

    configLeftPivot.closedLoop.pid(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD);
    configRightPivot.closedLoop.pid(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD);
    configLeftExtension.closedLoop.pid(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD);
    configRightExtension.closedLoop.pid(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD);

    this.m_rightPivotMotor.configure(configRightPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_leftPivotMotor.configure(configLeftPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.m_rightExtensionMotor.configure(configRightExtension, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_leftExtensionMotor.configure(configLeftExtension, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("PivotP", ElevatorPivotConstants.kPivotP);
    SmartDashboard.putNumber("PivotI", ElevatorPivotConstants.kPivotI);
    SmartDashboard.putNumber("PivotD", ElevatorPivotConstants.kPivotD);

    SmartDashboard.putNumber("ExtensionP", ElevatorExtensionConstants.kExtensionP);
    SmartDashboard.putNumber("ExtensionI", ElevatorExtensionConstants.kExtensionI);
    SmartDashboard.putNumber("ExtensionD", ElevatorExtensionConstants.kExtensionD);

  }

  /**
   *
   * @param targetAngle - the angle the motors are set to move to.
   */
  public void setTargetAngle(double targetAngle) {

    this.m_rightPivotPIDController.setReference(-targetAngle, ControlType.kPosition);
    this.m_leftPivotPIDController.setReference(-targetAngle, ControlType.kPosition);
  }

  /**
   * 
   * @param targetLength - the length the motors are set to extend to.
   */
  public void setTargetExtension(double targetLength) {

    this.m_rightExtensionPIDController.setReference(targetLength, ControlType.kMAXMotionPositionControl);
    this.m_leftExtensionPIDController.setReference(targetLength, ControlType.kMAXMotionPositionControl);
  }

  /**
   * 
   * @return returns the current angle value of the motor
   */
  public double getCurrentAngle() {
    return this.m_pivotAbsoluteEncoder.getPosition();
  }

  /**
   * 
   * @return returns the current length value of the motor
   */
  public double getCurrentExtension() {
    return this.m_extensionRelativeEncoder.getPosition();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("CurrentAngle", getCurrentAngle());
    SmartDashboard.putNumber("CurrentExtension", getCurrentExtension());

    ElevatorPivotConstants.kPivotP = SmartDashboard.getNumber("PivotP", ElevatorPivotConstants.kPivotP);
    ElevatorPivotConstants.kPivotI = SmartDashboard.getNumber("PivotI", ElevatorPivotConstants.kPivotI);
    ElevatorPivotConstants.kPivotD = SmartDashboard.getNumber("PivotD", ElevatorPivotConstants.kPivotD);

    ElevatorExtensionConstants.kExtensionP = SmartDashboard.getNumber("ExtensionP", ElevatorExtensionConstants.kExtensionP);
    ElevatorExtensionConstants.kExtensionI = SmartDashboard.getNumber("ExtensionI", ElevatorExtensionConstants.kExtensionI);
    ElevatorExtensionConstants.kExtensionD = SmartDashboard.getNumber("ExtensionD", ElevatorExtensionConstants.kExtensionD);


    // This method will be called once per scheduler run
  }
}
