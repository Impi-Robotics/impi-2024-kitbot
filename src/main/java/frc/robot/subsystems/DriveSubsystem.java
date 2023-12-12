// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  
  private final SwerveModule frontLeft = new SwerveModule(
    SwerveConstants.CAN.FRONT_LEFT_DRIVE,
    SwerveConstants.CAN.FRONT_LEFT_TURN,
    SwerveConstants.PID.FRONT_LEFT_ANGLE_OFFSET);

  private final SwerveModule frontRight = new SwerveModule(
    SwerveConstants.CAN.FRONT_RIGHT_DRIVE,
    SwerveConstants.CAN.FRONT_RIGHT_TURN,
    SwerveConstants.PID.FRONT_RIGHT_ANGLE_OFFSET);

  private final SwerveModule backLeft = new SwerveModule(
    SwerveConstants.CAN.BACK_LEFT_DRIVE,
    SwerveConstants.CAN.BACK_LEFT_TURN,
    SwerveConstants.PID.BACK_LEFT_ANGLE_OFFSET);

  private final SwerveModule backRight = new SwerveModule(
    SwerveConstants.CAN.BACK_RIGHT_DRIVE,
    SwerveConstants.CAN.BACK_RIGHT_TURN,
    SwerveConstants.PID.BACK_RIGHT_ANGLE_OFFSET);

  private final Pigeon2 gyro = new Pigeon2(SwerveConstants.CAN.GYRO);

  private double currentRotation = 0.0;
  private double currentTranslationDirection = 0.0;
  private double currentTranslationMagnitude = 0.0;
   
  private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(SwerveConstants.MISC.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MISC.ROTATIONAL_SLEW_RATE);
  private double previousTime = WPIUtilJNI.now() * 1e-6;

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    SwerveConstants.MISC.DRIVE_KINEMATICS,
    gyro.getRotation2d(),
    new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    }
  );

  public DriveSubsystem() {

  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }, pose);
  }

  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      double inputTranslationDirection = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
      
      double directionSlewRate;
      if (currentTranslationMagnitude != 0.0) {
        directionSlewRate = Math.abs(SwerveConstants.MISC.DIRECTION_SLEW_RATE / currentTranslationMagnitude);
      } else {
        directionSlewRate = 500.0;
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - previousTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDirection, currentTranslationDirection);

      if (angleDif < 0.85 * Math.PI) {
        currentTranslationDirection = SwerveUtils.StepTowardsCircular(currentTranslationDirection, inputTranslationDirection, directionSlewRate * elapsedTime);
        currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
      }
      else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMagnitude > 1e-4) {
          currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
        }
        else {
          currentTranslationDirection = SwerveUtils.WrapAngle(currentTranslationDirection + Math.PI);
          currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
        }
      }
      else {
        currentTranslationDirection = SwerveUtils.StepTowardsCircular(currentTranslationDirection, inputTranslationDirection, directionSlewRate * elapsedTime);
        currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
      }
      previousTime = currentTime;

      xSpeedCommanded = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
      ySpeedCommanded = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
      currentRotation = rotationLimiter.calculate(rotation);
    }
    else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rotation;
    }

    double xSpeedDelivered = xSpeedCommanded * SwerveConstants.MISC.MAX_SPEED;
    double ySpeedDelivered = ySpeedCommanded * SwerveConstants.MISC.MAX_SPEED;
    double rotDelivered = currentRotation * SwerveConstants.MISC.MAX_ANGULAR_SPEED;

    var swerveModuleStates = SwerveConstants.MISC.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, gyro.getRotation2d())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, SwerveConstants.MISC.MAX_SPEED);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MISC.MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    frontLeft.resetEncoder();
    frontRight.resetEncoder();
    backLeft.resetEncoder();
    backRight.resetEncoder();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  } 

  public double getTurnRate() {
    return gyro.getRate() * (SwerveConstants.MISC.GYRO_REVERSED ? -1.0 : 1.0);
  }

  @Override
  public void periodic() {
    odometry.update(
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }
    );
  }
}
