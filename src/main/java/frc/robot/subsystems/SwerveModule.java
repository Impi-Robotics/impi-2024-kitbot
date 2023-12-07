// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;


public class SwerveModule {
  
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private AbsoluteEncoder turnEncoder;

  private SparkMaxPIDController drivePIDController;
  private SparkMaxPIDController turnPIDController;

  private double chassisAngleOffset;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(int driveCANId, int turnCANId, double chassisAngleOffset) {
    driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    drivePIDController = driveMotor.getPIDController();
    turnPIDController = turnMotor.getPIDController();
    drivePIDController.setFeedbackDevice(driveEncoder);
    turnPIDController.setFeedbackDevice(turnEncoder);

    driveEncoder.setPositionConversionFactor(SwerveConstants.CONVERSION.DRIVE_ROTATIONS_TO_METERS);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.CONVERSION.DRIVE_RPM_TO_MPS);
    turnEncoder.setPositionConversionFactor(SwerveConstants.CONVERSION.TURN_CLICKS_TO_RADIANS);
    turnEncoder.setVelocityConversionFactor(SwerveConstants.CONVERSION.TURN_RPM_TO_RPS);
    turnEncoder.setInverted(SwerveConstants.PID.TURN_IS_INVERTED);

    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(SwerveConstants.PID.TURN_MIN_PID_INPUT);
    turnPIDController.setPositionPIDWrappingMaxInput(SwerveConstants.PID.TURN_MAX_PID_INPUT);
    
    drivePIDController.setP(SwerveConstants.PID.DRIVE_P);
    drivePIDController.setI(SwerveConstants.PID.DRIVE_I);
    drivePIDController.setD(SwerveConstants.PID.DRIVE_D);
    drivePIDController.setFF(SwerveConstants.PID.DRIVE_FF);

    turnPIDController.setP(SwerveConstants.PID.TURN_P);
    turnPIDController.setI(SwerveConstants.PID.TURN_I);
    turnPIDController.setD(SwerveConstants.PID.TURN_D);
    turnPIDController.setFF(SwerveConstants.PID.TURN_FF);

    drivePIDController.setOutputRange(SwerveConstants.PID.DRIVE_MIN_OUTPUT, SwerveConstants.PID.DRIVE_MAX_OUTPUT);
    turnPIDController.setOutputRange(SwerveConstants.PID.TURN_MIN_OUTPUT, SwerveConstants.PID.TURN_MAX_OUTPUT);

    driveMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(SwerveConstants.MISC.DRIVE_AMP_LIMIT);
    turnMotor.setSmartCurrentLimit(SwerveConstants.MISC.TURN_AMP_LIMIT);

    driveMotor.burnFlash();
    turnMotor.burnFlash();

    this.chassisAngleOffset = chassisAngleOffset;
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(),
      new Rotation2d(turnEncoder.getPosition(), - chassisAngleOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), 
      new Rotation2d(turnEncoder.getPosition() - chassisAngleOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
      SwerveModuleState correctedDesiredState = new SwerveModuleState();
      correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
      correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngleOffset));

      SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
       new Rotation2d(turnEncoder.getPosition()));

      drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
      turnPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

      this.desiredState = desiredState;
  }

  public void resetEncoder() {
    driveEncoder.setPosition(0.0);
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }
}
