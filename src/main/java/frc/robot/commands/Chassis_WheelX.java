// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis.ChassisSubsystem;

public class Chassis_WheelX extends InstantCommand {
  private final ChassisSubsystem chassisSubsystem;
  public Chassis_WheelX(ChassisSubsystem chassisSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassisSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassisSubsystem.setX();
  }
}
