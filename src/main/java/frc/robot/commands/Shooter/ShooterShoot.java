package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSub;

public class ShooterShoot extends InstantCommand {

  public ShooterSub shooterSub;

  public ShooterShoot(ShooterSub shooterSub) {
    this.shooterSub = shooterSub;
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
    shooterSub.shoot(ShooterConstants.Main.FullRPM);
    // shooterSub.shoot5000();
  }
}
