package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

public class Constants {
    public static class SwerveModuleConstants {
        public static final int[] turnCANIds = {
            0,
            1,
            2,
            3
        };
        public static final int[] driveCANIds = {
            4,
            5,
            6,
            7
        };

        public static final SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    }
}
