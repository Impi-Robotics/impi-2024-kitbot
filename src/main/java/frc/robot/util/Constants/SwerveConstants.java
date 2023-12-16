package frc.robot.util.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Constants.MotorConstants.NeoMotorConstants;

public final class SwerveConstants {
    
    public static class SwerveModuleConstants{
        public static final boolean INVERT_TURN_ENCODER = false;

        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50;
        public static final int TURN_MOTOR_CURRENT_LIMIT = 20;
        public static final double DRIVE_MOTOR_PINION_TEETH = 14.;
        public static final double DRIVE_MOTOR_FIRST_STAGE_SPUR_TEETH = 22.;
        public static final double DRIVE_MOTOR_BEVEL_TEETH = 45.;
        public static final double DRIVE_MOTOR_BEVEL_PINION_TEETH = 15.;

        //Calculations Required for Conversion Factors
        public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.NEO_FREE_SPEED_RPM / 60.;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.0);
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        //Drive Motor Gear Ratio = teeth on bevel gear * first stage spur gear / Pinion teeth * bevel pinon
        public static final double DRIVE_GEAR_RATIO = 
        (DRIVE_MOTOR_BEVEL_TEETH * DRIVE_MOTOR_FIRST_STAGE_SPUR_TEETH) / 
        (DRIVE_MOTOR_PINION_TEETH * DRIVE_MOTOR_BEVEL_PINION_TEETH);
        
        public static final double DRIVE_FREE_SPEED_RPS = 
        (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVE_GEAR_RATIO;

        //Conversion Factors
        //Convert Rotations to Meters
        public static final double DRIVE_ENCODER_POSITION_FACTOR =
        (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_RATIO;
        //Convert RPM to Meters per second
        public static final double DRIVE_ENCODER_VELOCITY_FACTOR = 
        ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_RATIO) / 60.;

        //Convert to Radians
        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI);
        //Covert to Radians per Second
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = 
        (2 * Math.PI) / 60.;
        //Both of these min, max are in radians
        public static final double TURNING_ENCODER_PID_MIN_INPUT = 0;
        public static final double TURNING_ECONDER_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR;

        //PID Constants -> Right now REV Constants
        public static final double DRIVE_P = 0.;
        public static final double DRIVE_I = 0.;
        public static final double DRIVE_D = 0.;
        public static final double DRIVE_FF = 1 / DRIVE_FREE_SPEED_RPS;
        public static final double DRIVE_MIN_OUTPUT = -1.0;
        public static final double DRIVE_MAX_OUTPUT = 1.0;

        public static final double TURN_P = 0.;
        public static final double TURN_I = 0.;
        public static final double TURN_D = 0.;
        public static final double TURN_FF = 0.;
        public static final double TURN_MIN_OUTPUT = -1.0;
        public static final double TURN_MAX_OUTPUT = 1.0;

    }

    public static final class ChassisConstants{
        /*
         * Make sure everything needed for subsystem is here. Reference CAN ID and include constants here
         */
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = CANConstants.FRONT_LEFT_DRIVE_MOTOR_ID;
        public static final int FRONT_LEFT_TURN_MOTOR_ID = CANConstants.FRONT_LEFT_TURN_MOTOR_ID;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = CANConstants.FRONT_RIGHT_DRIVE_MOTOR_ID;
        public static final int FRONT_RIGHT_TURN_MOTOR_ID = CANConstants.FRONT_RIGHT_TURN_MOTOR_ID;
        public static final int REAR_LEFT_DRIVE_MOTOR_ID = CANConstants.REAR_LEFT_DRIVE_MOTOR_ID;
        public static final int REAR_LEFT_TURN_MOTOR_ID = CANConstants.REAR_LEFT_TURN_MOTOR_ID;
        public static final int REAR_RIGHT_DRIVE_MOTOR_ID = CANConstants.REAR_RIGHT_DRIVE_MOTOR_ID;
        public static final int REAR_RIGHT_TURN_MOTOR_ID = CANConstants.REAR_RIGHT_TURN_MOTOR_ID;
        public static final int PIGEON_ID = CANConstants.PIGEON_ID;
        /*
         * THESE ARE IN RADIANS
         * To find these:
         * -Turn all to 0 in code
         * -Point wheels physically to face front of robot with gear on same side
         * -Look at smart dashboard to find offsets
         */
        public static final double FRONT_LEFT_ANGLE_OFFSET = Units.degreesToRadians(-90.);
        public static final double FRONT_RIGHT_ANGLE_OFFSET = Units.degreesToRadians(0.);
        public static final double REAR_LEFT_ANGLE_OFFSET = Units.degreesToRadians(180.);
        public static final double REAR_RIGHT_ANGLE_OFFSET = Units.degreesToRadians(90.);

        /*
         * Driving Params -> Not at max capable speeds but at max allowed speed
         */
        public static final double MAX_SPEED_METERS_PER_SECOND = 3.8;
        public static final double MAX_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;

        //Chassis Config
        public static final double TRACK_WIDTH = Units.inchesToMeters(27.);
        public static final double WHEEL_BASE = Units.inchesToMeters(27.);
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2 , TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
        
        
        public static final boolean GYRO_REVERSED = false;
        
    }
}
