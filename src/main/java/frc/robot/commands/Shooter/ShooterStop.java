package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter.ShooterSub;

public class ShooterStop extends InstantCommand {

  public ShooterSub shooterSub;

  public ShooterStop(ShooterSub shooterSub) {
    this.shooterSub = shooterSub;
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
    shooterSub.stop();
  }
}
