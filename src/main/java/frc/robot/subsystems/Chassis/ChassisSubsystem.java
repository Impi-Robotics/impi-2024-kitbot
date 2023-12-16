// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.SwerveConstants.ChassisConstants;

public class ChassisSubsystem extends SubsystemBase {
  
  //Create the 4 swerve drive modules
  private SwerveModuleMAX frontRightModule;
  private SwerveModuleMAX frontLeftModule;
  private SwerveModuleMAX rearRightModule;
  private SwerveModuleMAX rearLeftModule;

  //Pigeon
  private Pigeon2 gyro;

  //Odometry
  private SwerveDriveOdometry odometry;

  //Maybe include slew rate variables?
  public ChassisSubsystem() {
    frontLeftModule = new SwerveModuleMAX("Front Left Module",
      ChassisConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
      ChassisConstants.FRONT_LEFT_TURN_MOTOR_ID,
      false,
      true,
      ChassisConstants.FRONT_LEFT_ANGLE_OFFSET);

    frontRightModule = new SwerveModuleMAX("Front Right Module",
      ChassisConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      ChassisConstants.FRONT_RIGHT_TURN_MOTOR_ID,
      false,
      true,
      ChassisConstants.FRONT_RIGHT_ANGLE_OFFSET);

    rearLeftModule = new SwerveModuleMAX("Rear Left Module",
      ChassisConstants.REAR_LEFT_DRIVE_MOTOR_ID,
      ChassisConstants.REAR_LEFT_TURN_MOTOR_ID,
      false,
      true,
      ChassisConstants.REAR_LEFT_ANGLE_OFFSET);

    rearRightModule = new SwerveModuleMAX("Rear Right Module",
      ChassisConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
      ChassisConstants.REAR_RIGHT_TURN_MOTOR_ID,
      false,
      true,
      ChassisConstants.REAR_RIGHT_ANGLE_OFFSET);
    
    gyro = new Pigeon2(ChassisConstants.PIGEON_ID);
    gyro.reset();
    odometry = new SwerveDriveOdometry(
      ChassisConstants.SWERVE_DRIVE_KINEMATICS,
      gyro.getRotation2d(),
      new SwerveModulePosition[]{
        frontLeftModule.getCurrentPosition(),
        frontRightModule.getCurrentPosition(),
        rearLeftModule.getCurrentPosition(),
        rearRightModule.getCurrentPosition()
      });


  }

  @Override
  public void periodic() {
    frontLeftModule.periodic();
    frontRightModule.periodic();
    rearLeftModule.periodic();
    rearRightModule.periodic();

    SmartDashboard.putNumber("Gyro Heading Degrees: ", getGyroHeading());
    odometry.update(
      gyro.getRotation2d(),
      new SwerveModulePosition[]{
        frontLeftModule.getCurrentPosition(),
        frontRightModule.getCurrentPosition(),
        rearLeftModule.getCurrentPosition(),
        rearRightModule.getCurrentPosition()
      });
  }
  /**
   * 
   * @param xSpeed -> Forward speed
   * @param ySpeed -> Sideways speed
   * @param rot -> Rotation speed
   * @param fieldRelative 
   * @param rateLimit
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
                    boolean rateLimit ){
      var swerveModuleStates = ChassisConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot));
      SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ChassisConstants.MAX_SPEED_METERS_PER_SECOND);
      frontLeftModule.setDesiredState(swerveModuleStates[0]);
      frontRightModule.setDesiredState(swerveModuleStates[1]);
      rearLeftModule.setDesiredState(swerveModuleStates[2]);
      rearRightModule.setDesiredState(swerveModuleStates[3]);

  }
  /**
   * Returns the current estimated pose of robot
   * @returns x and y of robot on field in meters
   */
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  /**
   * Resets odometry to specified position
   * Useful for teleop if we want robot to rotate towards goal automatically 
   */
  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(
      gyro.getRotation2d(),
      new SwerveModulePosition[]{
        frontLeftModule.getCurrentPosition(),
        frontRightModule.getCurrentPosition(),
        rearLeftModule.getCurrentPosition(),
        rearRightModule.getCurrentPosition()
        }, 
      pose);
  }

  /**
   * Sets the wheel into an X
   */
  public void setX(){
    frontLeftModule.setDesiredState(new SwerveModuleState(0., Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0.,Rotation2d.fromDegrees(-45.)));
    rearLeftModule.setDesiredState(new SwerveModuleState(0., Rotation2d.fromDegrees(-45.)));
    rearRightModule.setDesiredState(new SwerveModuleState(0., Rotation2d.fromDegrees(45.)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      ChassisConstants.MAX_SPEED_METERS_PER_SECOND);
      frontLeftModule.setDesiredState(desiredStates[0]);
      frontRightModule.setDesiredState(desiredStates[1]);
      rearLeftModule.setDesiredState(desiredStates[2]);
      rearRightModule.setDesiredState(desiredStates[3]);
  }

  public void resetDriveEncoders(){
    frontLeftModule.resetDriveEncoder();
    frontRightModule.resetDriveEncoder();
    rearLeftModule.resetDriveEncoder();
    rearRightModule.resetDriveEncoder();
  }

  public void zeroGyroHeading(){
    gyro.reset();
  }

  public double getGyroHeading(){
    return gyro.getRotation2d().getDegrees();
  }
}
