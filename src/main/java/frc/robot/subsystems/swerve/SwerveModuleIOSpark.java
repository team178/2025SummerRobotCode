package frc.robot.subsystems.swerve;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SwerveModuleIOSpark implements SwerveModuleIO {
    private SparkMax turnMotor;
    private SparkMax driveMotor;

    private RelativeEncoder turnEncoder;
    private AbsoluteEncoder driveEncoder;

    public SwerveModuleIOSpark(int index) {
        turnMotor = new SparkMax(SwerveConstants.turnCANIds[index], MotorType.kBrushless);
        driveMotor = new SparkMax(SwerveConstants.driveCANIds[index], MotorType.kBrushless);

        turnEncoder = turnMotor.getEncoder();
        driveEncoder = driveMotor.getAbsoluteEncoder();

        turnMotor.configure(SwerveConstants.turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTurnPosition(Rotation2d angle) {
        turnMotor.getClosedLoopController().setReference(angle.getRotations(), ControlType.kPosition);
    }

    public void setDriveVelocity(double speedMetersPerSecond) {
        driveMotor.getClosedLoopController().setReference(speedMetersPerSecond, ControlType.kVelocity);
    }

    // public SwerveModuleState optimizeState(SwerveModuleState defaultState) {
    //     double angle = defaultState.angle.getRadians();
    //     double speed = defaultState.speedMetersPerSecond;
    //     turnMotor.getAbsoluteEncoder().getPosition();


    // }

   @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.turnPosition = Rotation2d.fromRotations(turnEncoder.getPosition());
        inputs.speedMetersPerSecond = driveEncoder.getVelocity();
    }
}