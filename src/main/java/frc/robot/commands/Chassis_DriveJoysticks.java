// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ImpiLib2024;
import frc.robot.subsystems.Chassis.ChassisSubsystem;

public class Chassis_DriveJoysticks extends Command {
  private final ChassisSubsystem chassisSubsystem;
  
  private DoubleSupplier turnTriggerLeft;
  private DoubleSupplier turnTriggerRight;
  private DoubleSupplier driveJoystickX;
  private DoubleSupplier driveJoystickY;

  public Chassis_DriveJoysticks(ChassisSubsystem chassisSubsystem,
        DoubleSupplier turnTriggerLeft, DoubleSupplier turnTriggerRight,
        DoubleSupplier driveJoystickX, DoubleSupplier driveJoystickY) {
          this.chassisSubsystem = chassisSubsystem;
          this.turnTriggerLeft = turnTriggerLeft;
          this.turnTriggerRight = turnTriggerRight;
          this.driveJoystickX = driveJoystickX;
          this.driveJoystickY = driveJoystickY;
          addRequirements(chassisSubsystem);
    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double rotation = ImpiLib2024.parseJoystick(turnTriggerLeft, 0.05) - 
                      ImpiLib2024.parseJoystick(turnTriggerRight, 0.05);
    double xValue = ImpiLib2024.parseJoystick(driveJoystickX, 0.05);
    double yValue = ImpiLib2024.parseJoystick(driveJoystickY, 0.05);
    chassisSubsystem.drive(xValue, yValue, rotation, true, false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
