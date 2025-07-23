package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class SwerveConstants {
        public static final int[] turnCANIds = { 0, 1, 2, 3 }; // FL, FR, BL, BR
        public static final int[] driveCANIds = { 4, 5, 6, 7 }; // FL, FR, BL, BR
        public static final int pigeonCANId = 8;

        public static final SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();

        public static final double robotSizeMeters = Units.inchesToMeters(18);

        public static final double maxVelocityMetersPerSec = 5;

        static {
            
        }
    }
}
