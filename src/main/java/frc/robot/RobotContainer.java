// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RobotOrientedDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // Xbox Controller Stuff
  private final XboxController driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController buttonsController = new XboxController(OperatorConstants.kButtonsControllerPort);

  private final DoubleSupplier driverLeftJoystickX = () -> driverController.getLeftX();
  private final DoubleSupplier driverLeftJoystickY = () -> driverController.getLeftY();

  private final DoubleSupplier buttonsLeftJoystickX = () -> buttonsController.getLeftX();
  private final DoubleSupplier buttonsLeftJoystickY = () -> buttonsController.getLeftY();
  private final DoubleSupplier buttonsLeftTrigger = () -> buttonsController.getLeftTriggerAxis();
  private final DoubleSupplier buttonsRightJoystickX = () -> buttonsController.getRightX();
  private final DoubleSupplier buttonsRightJoystickY = () -> buttonsController.getRightY();
  private final DoubleSupplier buttonsRightTrigger = () -> buttonsController.getRightTriggerAxis();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();

    driveSubsystem.setDefaultCommand(
      new RunCommand(() -> driveSubsystem.drive(
        -MathUtil.applyDeadband(driverController.getLeftY(), SwerveConstants.MISC.DRIVE_DEADBAND),  
        -MathUtil.applyDeadband(driverController.getLeftX(), SwerveConstants.MISC.DRIVE_DEADBAND),
        -MathUtil.applyDeadband(driverController.getRightX(), SwerveConstants.MISC.DRIVE_DEADBAND),
        true, true), driveSubsystem)); 
    
  }

  // public void setDefaultCommands() {
  //   driveSubsystem.setDefaultCommand(new RobotOrientedDriveCommand(STUFFFFFF));
  // }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //new JoystickButton(driverController, Button.kR1.value).whileTrue(new RunCommand(null, null))
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
