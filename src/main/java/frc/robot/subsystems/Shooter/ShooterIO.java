package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    default void updateInputs(ShooterIOInputs inputs) {}
    @AutoLog
    public static class ShooterIOInputs{
        public double targetRPM;
        public double rightRPM;
        public double leftRPM;
        public double rightAppliedVolts;
        public double leftAppliedVolts;
        public double[] rightCurrentDrawAmps;
        public double[] leftCurrentDrawAmps;

        public double[] rightShooterTemp;
        public double[] leftShooterTemp;

    }
    default void setBreakMode(){}
    default void setCoastMode(){}
    default void setShooterVoltage(double percentOutput){}
    default void setShooterRPM(double rpm){}
    default void configurePID(double p, double i, double d){}
}
