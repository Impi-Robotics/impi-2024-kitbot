// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSub extends SubsystemBase {

  private CANSparkMax ElevatorMotorMaster;
  private CANSparkMax ElevatorMotorFollower;
  private RelativeEncoder ElevatorMasterEncoder;
  private RelativeEncoder ElevatorFollowerEncoder;
  private SparkMaxPIDController ElevatorMasterPIDController;
  private DigitalInput LimitSwitch;

  /** Creates a new ElevatorSub. */
  public ElevatorSub() {
    ElevatorMotorMaster = new CANSparkMax(0, MotorType.kBrushless);
    ElevatorMotorFollower = new CANSparkMax(1, MotorType.kBrushless);
    ElevatorMotorMaster.setIdleMode(IdleMode.kBrake);
    ElevatorMotorFollower.follow(ElevatorMotorMaster, true);
    ElevatorMotorMaster.setSmartCurrentLimit(ElevatorConstants.Main.CurrentLimit);

    //bottom limit
    ElevatorMotorMaster.setSoftLimit(SoftLimitDirection.kReverse, (float)ElevatorConstants.Main.LimitBottomPos);
    ElevatorMotorMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);

    ElevatorMasterPIDController = ElevatorMotorMaster.getPIDController();
    ElevatorMasterPIDController.setP(ElevatorConstants.Main.ElevatorP);
    ElevatorMasterPIDController.setI(ElevatorConstants.Main.ElevatorI);
    ElevatorMasterPIDController.setD(ElevatorConstants.Main.ElevatorD);
    ElevatorMasterPIDController.setFF(ElevatorConstants.Main.ElevatorFF);

    // top limit
    LimitSwitch = new DigitalInput(0);
  }

  @Override
  public void periodic() {
    //telem
    // This method will be called once per scheduler run
  }

  public void stop() {
    ElevatorMotorMaster.set(0);
  }

  public void elevate(double speed) {
    if(LimitSwitch.get() && speed > 0) {
      speed = 0;
    }
    ElevatorMotorMaster.set(speed);
  }

  public void elevateToPos(double setSpeed, double pos) {
    ElevatorMasterPIDController.setReference(pos, ControlType.kPosition);
  }
}
