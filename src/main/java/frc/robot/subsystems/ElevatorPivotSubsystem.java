// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.MotorConfigs.ElevatorPivotConfig;

public class ElevatorPivotSubsystem extends SubsystemBase {

  private SparkMax m_leftPivotMotor;
  private SparkMax m_rightPivotMotor;
  private AbsoluteEncoder m_pivotAbsoluteEncoder;
  private RelativeEncoder m_leftPivotEncoder;
  private RelativeEncoder m_rightPivotEncoder;

  private ProfiledPIDController m_leftPIDController;
  private ProfiledPIDController m_rightPIDController;


  /** Creates a new PivotSubsystem. */
  public ElevatorPivotSubsystem() {

    this.m_leftPivotMotor = new SparkMax(ElevatorPivotConstants.kLeftMotorID, MotorType.kBrushless);
    this.m_rightPivotMotor = new SparkMax(ElevatorPivotConstants.kRightMotorID, MotorType.kBrushless);
    this.m_pivotAbsoluteEncoder = this.m_rightPivotMotor.getAbsoluteEncoder();

    this.m_leftPivotEncoder = this.m_leftPivotMotor.getEncoder();
    this.m_rightPivotEncoder = this.m_rightPivotMotor.getEncoder();

    this.m_leftPIDController = new ProfiledPIDController(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD,
      new TrapezoidProfile.Constraints(ElevatorPivotConstants.kMaxVelocity, ElevatorPivotConstants.kMaxAcceleration)
    );

    this.m_rightPIDController = new ProfiledPIDController(ElevatorPivotConstants.kPivotP, ElevatorPivotConstants.kPivotI, ElevatorPivotConstants.kPivotD,
      new TrapezoidProfile.Constraints(ElevatorPivotConstants.kMaxVelocity, ElevatorPivotConstants.kMaxAcceleration)
    );

    this.m_leftPivotMotor.configure(ElevatorPivotConfig.leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_rightPivotMotor.configure(ElevatorPivotConfig.rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Gives the motor controllers the current angle
    //This currently requires the code to be uploaded every time the robot starts to initially set the position
    this.m_leftPivotEncoder.setPosition(this.m_pivotAbsoluteEncoder.getPosition() * (Math.PI / 2.0));
    this.m_rightPivotEncoder.setPosition(this.m_pivotAbsoluteEncoder.getPosition() * (Math.PI / 2.0));

    SmartDashboard.putNumber("Pivot-kLevel1Angle", ElevatorPivotConstants.kLevel1Angle);
    SmartDashboard.putNumber("Pivot-kLevel2Angle", ElevatorPivotConstants.kLevel2Angle);
    SmartDashboard.putNumber("Pivot-kLevel3Angle", ElevatorPivotConstants.kLevel3Angle);
    SmartDashboard.putNumber("Pivot-kLevel4Angle", ElevatorPivotConstants.kLevel4Angle);
    SmartDashboard.putNumber("Pivot-kCoralStationAngle", ElevatorPivotConstants.kCoralStationAngle);
    SmartDashboard.putNumber("Pivot-kUpperAlgeaRemove", ElevatorPivotConstants.kUpperAlgeaRemove);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CurrentPivotAngle", Math.toDegrees(this.getCurrentAngle()));

    ElevatorPivotConstants.kLevel1Angle = SmartDashboard.getNumber("Pivot-kLevel1Angle", ElevatorPivotConstants.kLevel1Angle);
    ElevatorPivotConstants.kLevel2Angle = SmartDashboard.getNumber("Pivot-kLevel2Angle", ElevatorPivotConstants.kLevel2Angle);
    ElevatorPivotConstants.kLevel3Angle = SmartDashboard.getNumber("Pivot-kLevel3Angle", ElevatorPivotConstants.kLevel3Angle);
    ElevatorPivotConstants.kLevel4Angle = SmartDashboard.getNumber("Pivot-kLevel4Angle", ElevatorPivotConstants.kLevel4Angle);
    ElevatorPivotConstants.kCoralStationAngle = SmartDashboard.getNumber("Pivot-kCoralStationAngle", ElevatorPivotConstants.kCoralStationAngle);
    ElevatorPivotConstants.kUpperAlgeaRemove = SmartDashboard.getNumber("Pivot-kUpperAlgeaRemove", ElevatorPivotConstants.kUpperAlgeaRemove);

  }

  /**
   *
   * @param targetAngle - the angle the motors are set to move to.
   */
  public void setTargetAngle(double targetAngle) {

    double deltaAngle = targetAngle - this.getCurrentAngle();

    double leftOutput = -this.m_leftPIDController.calculate(deltaAngle);
    double rightOutput = -this.m_rightPIDController.calculate(deltaAngle);

    this.m_leftPivotMotor.set(leftOutput);
    this.m_rightPivotMotor.set(rightOutput);

  }

  /**
   *
   * @return returns the current angle value of the motor
   */
  public double getCurrentAngle() {
    return this.m_rightPivotEncoder.getPosition();
  }

}
