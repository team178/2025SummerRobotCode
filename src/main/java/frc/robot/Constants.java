package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    /* ROBOT MODE */
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        REAL, // on real robot
        SIM, // on simulated robot
        REPLAY // from a advtgkit log file
    }

    /* --------------- */

    public static class SwerveConstants {
        public static final SwerveModuleState[] xPosStates = new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
        };

        /* Device CAN ID's */
        public static final int[] turnCANIds = { 1, 2, 3, 4 }; // FL, FR, BL, BR
        public static final int[] driveCANIds = { 5, 6, 7, 8 }; // FL, FR, BL, BR
        public static final int pigeonCANId = 9;
        
        /* Physical Robot Details */
        public static final double wheelDistanceMeters = Units.inchesToMeters(18);
        
        public static final double wheelDiameterMeters = Units.inchesToMeters(3);
        public static final double wheelRadiusMeters = wheelDiameterMeters / 2;
        public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
        
        /* Turn Motor Configuration */
        public static final SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
        public static final int turnMotorCurrentLimit = 20;
        public static final boolean turnEncoderInverted = true;
        
        public static final double turnRampRate = 0.1; // seconds from 0 to full PID output
        public static final double turn_kP = 0;
        public static final double turn_kI = 0;
        public static final double turn_kD = 0;

        public static final double turnPositionConversionFactor =
            Units.rotationsToRadians(1);                          // Shaft Rotations -> Shaft Radians
        public static final double turnVelocityConversionFactor =
            Units.rotationsToRadians(1) / 60;                     // Shaft RPM       -> Shaft Radians/Second
            
        /* Drive Motor Configuration */
        public static final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
        public static final int driveMotorCurrentLimit = 50;
        public static final double driveMotorReduction = (45.0 * 22.0) / (13.0 * 15.0); // see product datasheet & parker's code
        
        public static final double driveRampRate = 0.1; // seconds from 0 to full PID output
        public static final double drive_kP = 0;
        public static final double drive_kI = 0;
        public static final double drive_kD = 0;
        public static final double drive_kS = 0;
        public static final double drive_kV = 0;

        public static final double drivePositionConversionFactor =
        (2 * Math.PI) / driveMotorReduction;                       // Rotor Rotations -> Wheel Radians
        public static final double driveVelocityConversionFactor =
        (2 * Math.PI) / (60 * driveMotorReduction);                // Rotor RPM       -> Wheel Radians/Second

        /* Driver Input Limits */
        public static final double maxChassisSpeedMagnitudeMetersPerSec = 3;
        public static final double maxWheelSpeedMetersPerSec = 15;
        public static final double maxRotationalVelocityRadiansPerSec = 10;
        
        public static final Rotation2d[] zeroRotations = {
            new Rotation2d(0.0),
            new Rotation2d(0.0),
            new Rotation2d(0.0),
            new Rotation2d(0.0),
        };

        static {
            /* Configure Turn */
            turnMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0)
                .closedLoopRampRate(turnRampRate)
            ;
            turnMotorConfig.absoluteEncoder
                .inverted(turnEncoderInverted)
                .positionConversionFactor(turnPositionConversionFactor)
                .velocityConversionFactor(turnVelocityConversionFactor)
            ;
            turnMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(turn_kP, turn_kI, turn_kP, 0.0)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, Units.rotationsToRadians(1)) // wrap to radians rather than rotations
                .outputRange(-1, 1)
            ;
            
            /* Configure Drive */
            driveMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(driveMotorCurrentLimit)
                .voltageCompensation(12.0)
                .closedLoopRampRate(driveRampRate)
            ;
            driveMotorConfig.absoluteEncoder
                .positionConversionFactor(drivePositionConversionFactor)
                .velocityConversionFactor(driveVelocityConversionFactor)
            ;
            driveMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(drive_kP, drive_kI, drive_kP, 0.0)
                .outputRange(-1, 1) // cap at full voltage
            ;
        }
    }
}
