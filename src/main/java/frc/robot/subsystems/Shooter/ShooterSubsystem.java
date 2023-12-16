// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.TunableNumber;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();

  private final TunableNumber shooterRpm = new TunableNumber("Shooter/ShooterRPM");
  private final TunableNumber shooterP = new TunableNumber("Shooter/ShooterP");
  private final TunableNumber shooterI = new TunableNumber("Shooter/ShooterI");
  private final TunableNumber shooterD = new TunableNumber("Shooter/ShooterD");

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    shooterRpm.setDefault(0.);
    shooterP.setDefault(0.);
    shooterI.setDefault(0.);
    shooterD.setDefault(0.);

    io.setCoastMode();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    if(shooterP.hasChanged() || shooterI.hasChanged() || shooterD.hasChanged()){
      io.configurePID(shooterP.get(), shooterI.get(), shooterD.get());
    }
    if(shooterRpm.hasChanged()){
      shootRPM(shooterRpm.get());
    }
  }

  public void shootRPM(double rpm){
    io.setShooterRPM(rpm);
  }

  public void shootSpeed(double speed){
    io.setShooterVoltage(speed);
  }

  public void stop(){
    shootSpeed(0.);
  }

  public double getLeftVelocity(){
    return inputs.leftRPM;
  }
  public double getRightVelocity(){
    return inputs.rightRPM;
  }
}
