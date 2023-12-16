package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;

public class ShooterIOSparkMax implements ShooterIO{
    private final CANSparkMax leftShooterMotor;
    private final CANSparkMax rightShooterMotor;
    private final RelativeEncoder rightShooterEncoder;
    private final RelativeEncoder leftShooterEncoder;

    private final SparkMaxPIDController shooterPidController;

    public ShooterIOSparkMax(){
        leftShooterMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(1, MotorType.kBrushless);

        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();
        rightShooterEncoder = rightShooterMotor.getEncoder();
        leftShooterEncoder = leftShooterMotor.getEncoder();

        leftShooterMotor.follow(rightShooterMotor, true);

        shooterPidController = rightShooterMotor.getPIDController();

        leftShooterMotor.burnFlash();
        rightShooterMotor.burnFlash();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        inputs.rightAppliedVolts = rightShooterMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.leftAppliedVolts = leftShooterMotor.getAppliedOutput() * RobotController.getBatteryVoltage();

        inputs.rightRPM = rightShooterEncoder.getVelocity();
        inputs.leftRPM = leftShooterEncoder.getVelocity();

        inputs.rightShooterTemp = new double[] {rightShooterMotor.getMotorTemperature(),};
        inputs.leftShooterTemp = new double[] {leftShooterMotor.getMotorTemperature(),};
        inputs.rightCurrentDrawAmps = new double[] {rightShooterMotor.getOutputCurrent(),};
        inputs.leftCurrentDrawAmps = new double[] {leftShooterMotor.getOutputCurrent(),};

    }
    @Override
    public void setBreakMode(){
        rightShooterMotor.setIdleMode(IdleMode.kBrake);
    }
    @Override
    public void setCoastMode(){
        rightShooterMotor.setIdleMode(IdleMode.kCoast);
    }
    @Override 
    public void setShooterVoltage(double speed){
        shooterPidController.setReference(speed, ControlType.kDutyCycle);
    }
    @Override 
    public void setShooterRPM(double rpm){
        shooterPidController.setReference(rpm, ControlType.kVelocity);
    }
    @Override 
    public void configurePID(double p, double i, double d){
        shooterPidController.setP(p);
        shooterPidController.setI(i);
        shooterPidController.setD(d);
        shooterPidController.setFF(0.);
    }
}
