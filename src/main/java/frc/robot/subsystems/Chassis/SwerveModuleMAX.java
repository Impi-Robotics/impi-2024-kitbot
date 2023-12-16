package frc.robot.subsystems.Chassis;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Constants.Constants;
import frc.robot.util.Constants.SwerveConstants.SwerveModuleConstants;

public class SwerveModuleMAX {
    String moduleName;
    //Motors for swerve module
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    //Encoders for both motors
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;
    //Pid controllers for drive and turn
    private final SparkMaxPIDController drivePidController;
    private final SparkMaxPIDController turnPidController;

    //Offset of wheels... found when pointing wheel forward
    private double wheelAngleOffset;
    //Contains velocity and angle of swerve module
    private SwerveModuleState desiredModuleState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructor for Swerve Module using MAXSwerve Modules
     * @param driveCANID -> ID of Drive Motor
     * @param turnCANID -> ID of Turn Motor
     * @param wheelAngleOffset -> Offset of Wheel in Radians
     */
    public SwerveModuleMAX(String moduleName, int driveCANID, int turnCANID, boolean driveBreakMode, boolean turnBreakMode, double wheelAngleOffset){
        driveMotor = new CANSparkMax(driveCANID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnCANID, MotorType.kBrushless);
        //Factory Reset both spark maxs. This gets Spark Maxs to be in a known state before configuration
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        drivePidController = driveMotor.getPIDController();
        turnPidController = turnMotor.getPIDController();
        drivePidController.setFeedbackDevice(driveEncoder);
        //REV through bore encoder hooked up to NEO... No need to specify port
        turnPidController.setFeedbackDevice(turnEncoder);

        /**
         * Driving
         * Position Factor: Rotations -> Meters
         * Velocity Factor: RPM -> Meters per second
         */
        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_ENCODER_POSITION_FACTOR);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR);
        /**
         * Turning
         * Position Factor: Rotations -> Radians
         * Velocity Factor: RPM -> Radians Per Seconds Squared
         */
        turnEncoder.setPositionConversionFactor(SwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnEncoder.setVelocityConversionFactor(SwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        //Invert the turning motor -> shaft rotates opposite of the steering motor in module
        turnEncoder.setInverted(SwerveModuleConstants.INVERT_TURN_ENCODER);

        /*
         * PID wrap on turn encoder: Allows controller to go through 0 to get to setpoint
         * ex: 350 degrees will go through 0 opposed to going long way around
         * REMEMBER: In radians... that's why it's wrapped between 0 and 2pi.
         */
        turnPidController.setPositionPIDWrappingEnabled(true);
        turnPidController.setPositionPIDWrappingMinInput(SwerveModuleConstants.TURNING_ENCODER_PID_MIN_INPUT);
        turnPidController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.TURNING_ECONDER_PID_MAX_INPUT);

        //Set PID gains for drive and turn
        //NOTE: RIGHT NOW USING REV GAINS... CHANGE WHEN TESTING
        drivePidController.setP(SwerveModuleConstants.DRIVE_P);
        drivePidController.setI(SwerveModuleConstants.DRIVE_I);
        drivePidController.setD(SwerveModuleConstants.DRIVE_D);
        drivePidController.setFF(SwerveModuleConstants.DRIVE_FF);
        drivePidController.setOutputRange(SwerveModuleConstants.DRIVE_MIN_OUTPUT, SwerveModuleConstants.DRIVE_MAX_OUTPUT);

        turnPidController.setP(SwerveModuleConstants.TURN_P);
        turnPidController.setI(SwerveModuleConstants.TURN_I);
        turnPidController.setD(SwerveModuleConstants.TURN_D);
        turnPidController.setFF(SwerveModuleConstants.TURN_FF);
        turnPidController.setOutputRange(SwerveModuleConstants.TURN_MIN_OUTPUT, SwerveModuleConstants.TURN_MAX_OUTPUT);

        if(driveBreakMode) driveMotor.setIdleMode(IdleMode.kBrake);
        else driveMotor.setIdleMode(IdleMode.kCoast);
        if(turnBreakMode) turnMotor.setIdleMode(IdleMode.kBrake);
        else turnMotor.setIdleMode(IdleMode.kCoast);

        driveMotor.setSmartCurrentLimit(SwerveModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(SwerveModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

        //Save Configurations for drive and turn Spark Maxs
        driveMotor.burnFlash();
        turnMotor.burnFlash();

        this.wheelAngleOffset = wheelAngleOffset;
        desiredModuleState.angle = new Rotation2d(turnEncoder.getPosition());
        driveEncoder.setPosition(0);

        this.moduleName = moduleName;
    }
    
    public void periodic(){
        SmartDashboard.putNumber(moduleName + " Wheel Angle Radians", turnEncoder.getPosition());
        SmartDashboard.putNumber(moduleName + " Wheel Angle Degrees", Units.radiansToDegrees(turnEncoder.getPosition()));
        SmartDashboard.putNumber(moduleName + " Speed Meters Per Sec", driveEncoder.getVelocity());
        SmartDashboard.putNumber(moduleName + " Current Zero Angle Offset", this.wheelAngleOffset);
        SmartDashboard.putBoolean(moduleName + " Drive Motor Break Mode", (driveMotor.getIdleMode().equals(IdleMode.kBrake)) ? true: false);

        if(Constants.Mode.TUNING_MODE){
            //PID Tuning Smart Dashboard
            double p = SmartDashboard.getNumber(moduleName + " Turn P", turnPidController.getP());
            SmartDashboard.putNumber(moduleName + " Turn P", p);
            double i = SmartDashboard.getNumber(moduleName + " Turn I", turnPidController.getI());
            SmartDashboard.putNumber(moduleName + " Turn I", i);
            double d = SmartDashboard.getNumber(moduleName + " Turn P", turnPidController.getD());
            SmartDashboard.putNumber(moduleName + " Turn D", d);
            double ff = SmartDashboard.getNumber(moduleName + " Turn FF", turnPidController.getFF());
            SmartDashboard.putNumber(moduleName + " Turn P", ff);

            turnPidController.setP(p);
            turnPidController.setI(i);
            turnPidController.setD(d);

            /*
            * double p = SmartDashboard.getNumber(moduleName + " Drive P", drivePidController.getP());
            SmartDashboard.putNumber(moduleName + " Turn P", p);
            double i = SmartDashboard.getNumber(moduleName + " Drive I", drivePidController.getI());
            SmartDashboard.putNumber(moduleName + " Turn I", i);
            double d = SmartDashboard.getNumber(moduleName + " Drive P",  drivePidController.getD());
            SmartDashboard.putNumber(moduleName + " Turn D", d);
            double ff = SmartDashboard.getNumber(moduleName + " Drive FF", drivePidController.getFF());
            SmartDashboard.putNumber(moduleName + " Turn P", ff);

            drivePidController.setP(p);
            drivePidController.setI(i);
            drivePidController.setD(d);
            */

        }
        
    }
    /*
     * Returns current state of the module
     * @return -> State which includes velocity and angle of swerve module
     * NOTE: Need to subtract wheel angle offset
     */
    public SwerveModuleState getCurrentState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), 
        new Rotation2d(turnEncoder.getPosition() - this.wheelAngleOffset));
    }
    /*
     * Returns current position of the module
     * @return - > Position of current swerve module
     * NOTS: Need to subtract wheel angle offset
     */
    public SwerveModulePosition getCurrentPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(),
        new Rotation2d(turnEncoder.getPosition() - this.wheelAngleOffset));
    }
    /**
     * Sets the desired state for the module
     * 
     * @param desiredModuleState -> Desired state with speed and angle
     */
    public void setDesiredState(SwerveModuleState desiredModuleState){
        //Apply chassis angular offset to desired state
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond  = desiredModuleState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredModuleState.angle.plus(Rotation2d.fromRadians(this.wheelAngleOffset));

        //Optimize the reference state so module doesn't spin more than 90 degrees
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, 
        new Rotation2d(turnEncoder.getPosition()));

        //This part commands the spark maxs to go to respective setpoints
        drivePidController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        turnPidController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

        this.desiredModuleState = desiredModuleState;
    }

    public void resetDriveEncoder(){
        driveEncoder.setPosition(0.);
    }

}
