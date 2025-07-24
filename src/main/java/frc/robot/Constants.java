package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class SwerveConstants {
        public static final SwerveModuleState[] xPosStates = new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
        };

        public static final int[] turnCANIds = { 0, 1, 2, 3 }; // FL, FR, BL, BR
        public static final int[] driveCANIds = { 4, 5, 6, 7 }; // FL, FR, BL, BR
        public static final int pigeonCANId = 8;

        public static final SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();

        public static final double robotSizeMeters = Units.inchesToMeters(18);
        public static final double wheelDiameter = 3; // meters
        public static final double wheelCircumference = wheelDiameter * Math.PI; // meters
        public static final double gearRatio = 2;

        public static final double maxVelocityMetersPerSec = 5;
        

        static {
            turnMotorConfig
                .smartCurrentLimit(30)
                .voltageCompensation(12)
                .inverted(false);
            turnMotorConfig.absoluteEncoder
                .inverted(false);
            turnMotorConfig.closedLoop
                .positionWrappingInputRange(0, 1)
                .pid(0, 0, 0);
        }
    }
}
