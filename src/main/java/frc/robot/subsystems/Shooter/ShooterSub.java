// CopyFollower (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSub extends SubsystemBase {

  private double dashboardShooterRPM = 0.0;
  //Master
  private CANSparkMax shooterMotorMaster;
  //Follower
  private CANSparkMax shooterMotorFollower;

  private SparkMaxPIDController shooterPIDController;

  private RelativeEncoder shooterEncoder;

  private double dashboardShooterSetpoint = 0;
  
  public ShooterSub() {

    shooterMotorMaster = new CANSparkMax(9, MotorType.kBrushless);
    shooterMotorFollower = new CANSparkMax(2, MotorType.kBrushless);

    shooterPIDController = shooterMotorMaster.getPIDController();

    shooterMotorMaster.restoreFactoryDefaults();
    shooterMotorFollower.restoreFactoryDefaults();


    shooterMotorMaster.setSmartCurrentLimit(ShooterConstants.Main.CurrentLimit);
    shooterMotorFollower.setSmartCurrentLimit(ShooterConstants.Main.CurrentLimit);
    shooterMotorMaster.setIdleMode(IdleMode.kCoast);
    shooterMotorFollower.follow(shooterMotorMaster, true);

    shooterEncoder = shooterMotorMaster.getEncoder();
    // shooterEncoder.setPositionConversionFactor(0);
    // shooterEncoder.setVelocityConversionFactor(0);

    shooterPIDController.setOutputRange(0, 1);

    shooterPIDController.setP(ShooterConstants.Main.ShooterP);
    shooterPIDController.setI(ShooterConstants.Main.ShooterI);
    shooterPIDController.setD(ShooterConstants.Main.ShooterD);
    shooterPIDController.setFF(ShooterConstants.Main.ShooterFF);

    shooterMotorMaster.burnFlash();
    shooterMotorFollower.burnFlash();
  }

  @Override
  public void periodic() {
    dashboardShooterSetpoint = SmartDashboard.getNumber("Dashboard Shooter", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Dashboard Shooter", dashboardShooterSetpoint);
    SmartDashboard.putNumber("Dashboard Shooter RPM:", getCurrentRPM());
    // dashboardShooterRPM = SmartDashboard.getNumber("Dashboard Shooter RPM:", getCurrentRPM());
    // shooterPIDController.setReference(dashboardShooterRPM, ControlType.kVelocity);
    // This method will be called once per scheduler run
  }

  public void dashboardShoot(){
		shooterPIDController.setReference(dashboardShooterSetpoint, ControlType.kVelocity);
	}

  public void shoot5000() {
    shooterPIDController.setReference(ShooterConstants.Main.FullRPM/*5000rpm?*/, ControlType.kVelocity);
  }

  public void shoot(double rpm) {
    shooterMotorMaster.set(0.5);
    // shooterPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void stop() {
    shooterMotorMaster.set(0.);
  }

  public double getCurrentRPM(){
    return shooterEncoder.getVelocity();
  }
}
