// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.Constants.GripperPivotConstants;
import frc.robot.MotorConfigs.GripperPivotConfig;

public class GripperPivotSubsystem extends SubsystemBase {

  private SparkMax m_pivotMotor;

  private ProfiledPIDController  m_pivotMotorPIDController;

  /** Creates a new GripperSubsystem. */
  public GripperPivotSubsystem() {

    this.m_pivotMotor = new SparkMax(GripperPivotConstants.kMotorID, MotorType.kBrushless);

    this.m_pivotMotorPIDController = new ProfiledPIDController(GripperPivotConstants.kPivotP, GripperPivotConstants.kPivotI, GripperPivotConstants.kPivotD,
      new TrapezoidProfile.Constraints(GripperPivotConstants.kMaxVelocity, GripperPivotConstants.kMaxAcceleration)
    );

    this.m_pivotMotor.configure(GripperPivotConfig.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Griper-kLevel1Angle", GripperPivotConstants.kLevel1Angle);
    SmartDashboard.putNumber("Griper-kLevel2Angle", GripperPivotConstants.kLevel2Angle);
    SmartDashboard.putNumber("Griper-kLevel3Angle", GripperPivotConstants.kLevel3Angle);
    SmartDashboard.putNumber("Griper-kLevel4Angle", GripperPivotConstants.kLevel4Angle);
    SmartDashboard.putNumber("Griper-kCoralStationAngle", GripperPivotConstants.kCoralStationAngle);
    SmartDashboard.putNumber("Griper-kUpperAlgeaRemove", GripperPivotConstants.kUpperAlgeaRemove);
    SmartDashboard.putNumber("Griper-kLowerAlgeaRemove", GripperPivotConstants.kLowerAlgeaRemove);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("CurrentGripperAngle", Math.toDegrees(getCurrentAngle()));

    GripperPivotConstants.kLevel1Angle = SmartDashboard.getNumber("Griper-kLevel1Angle", GripperPivotConstants.kLevel1Angle);
    GripperPivotConstants.kLevel2Angle = SmartDashboard.getNumber("Griper-kLevel2Angle", GripperPivotConstants.kLevel2Angle);
    GripperPivotConstants.kLevel3Angle = SmartDashboard.getNumber("Griper-kLevel3Angle", GripperPivotConstants.kLevel3Angle);
    GripperPivotConstants.kLevel4Angle = SmartDashboard.getNumber("Griper-kLevel4Angle", GripperPivotConstants.kLevel4Angle);
    GripperPivotConstants.kCoralStationAngle = SmartDashboard.getNumber("Griper-kCoralStationAngle", GripperPivotConstants.kCoralStationAngle);
    GripperPivotConstants.kUpperAlgeaRemove = SmartDashboard.getNumber("Griper-kUpperAlgeaRemove", GripperPivotConstants.kUpperAlgeaRemove);
    GripperPivotConstants.kLowerAlgeaRemove = SmartDashboard.getNumber("Griper-kLowerAlgeaRemove", GripperPivotConstants.kLowerAlgeaRemove);
  }

  public void setTargetAngle(double targetAngle) {

    double deltaAngle = targetAngle - this.getCurrentAngle();

    double output = -this.m_pivotMotorPIDController.calculate(deltaAngle);

    this.m_pivotMotor.set(output);
  }

  public double getCurrentAngle() {
    return this.m_pivotMotor.getEncoder().getPosition();
  }

}
