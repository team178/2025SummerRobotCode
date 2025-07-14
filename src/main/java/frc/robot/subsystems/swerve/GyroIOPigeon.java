package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.SwerveConstants;

public class GyroIOPigeon implements GyroIO {
    private Pigeon2 pigeon;

    public GyroIOPigeon() {
        pigeon = new Pigeon2(SwerveConstants.pigeonCANId);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.angle = pigeon.getRotation2d();
    }
}
