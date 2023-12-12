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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSub extends SubsystemBase {

  //Master
  private CANSparkMax ShooterMotorMaster;
  //Follower
  private CANSparkMax ShooterMotorFollower;

  private SparkMaxPIDController ShooterPIDController;

  private RelativeEncoder ShooterEncoder;

  public ShooterSub() {
    this.ShooterMotorMaster = ShooterMotorMaster;
    this.ShooterMotorFollower = ShooterMotorFollower;
    this.ShooterPIDController = ShooterPIDController;
    this.ShooterEncoder = ShooterEncoder;

    ShooterMotorMaster = new CANSparkMax(1, MotorType.kBrushless);
    ShooterMotorFollower = new CANSparkMax(2, MotorType.kBrushless);

    ShooterPIDController = ShooterMotorMaster.getPIDController();

    ShooterMotorMaster.restoreFactoryDefaults();
    ShooterMotorFollower.restoreFactoryDefaults();


    ShooterMotorMaster.setSmartCurrentLimit(ShooterConstants.Main.CurrentLimit);
    ShooterMotorFollower.setSmartCurrentLimit(ShooterConstants.Main.CurrentLimit);
    ShooterMotorMaster.setIdleMode(IdleMode.kCoast);
    ShooterMotorFollower.follow(ShooterMotorMaster, true);

    ShooterEncoder = ShooterMotorMaster.getEncoder();
    // ShooterEncoder.setPositionConversionFactor(0);
    // ShooterEncoder.setVelocityConversionFactor(0);

    ShooterPIDController.setOutputRange(0, 1);

    ShooterPIDController.setP(ShooterConstants.Main.ShooterP);
    ShooterPIDController.setI(ShooterConstants.Main.ShooterI);
    ShooterPIDController.setD(ShooterConstants.Main.ShooterD);
    ShooterPIDController.setFF(ShooterConstants.Main.ShooterFF);

    ShooterMotorMaster.burnFlash();
    ShooterMotorFollower.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot5000() {
    ShooterPIDController.setReference(1000/*5000rpm?*/, ControlType.kVelocity);
  }

  public void shoot(double rpm) {
    ShooterMotorMaster.set(0.5);
    // ShooterPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void stop() {
    ShooterMotorMaster.set(0);
  }
}
