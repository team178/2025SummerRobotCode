package frc.robot.subsystems.swerve;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SwerveModuleIOSpark implements SwerveModuleIO {
    private SparkMax turnMotor;
    private SparkMax driveMotor;

    private AbsoluteEncoder turnEncoder;
    private RelativeEncoder driveEncoder;

    private SparkClosedLoopController turnController;
    private SparkClosedLoopController driveController;

    private Rotation2d zeroRotation;

    private int index;

    public SwerveModuleIOSpark(int index) {
        turnMotor = new SparkMax(SwerveConstants.turnCANIds[index], MotorType.kBrushless);
        driveMotor = new SparkMax(SwerveConstants.driveCANIds[index], MotorType.kBrushless);
        zeroRotation = SwerveConstants.zeroRotations[index];

        turnEncoder = turnMotor.getAbsoluteEncoder();
        driveEncoder = driveMotor.getEncoder();

        turnController = turnMotor.getClosedLoopController();
        driveController = driveMotor.getClosedLoopController();

        this.index = index;

        turnMotor.configure(SwerveConstants.turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(SwerveConstants.driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTurnPosition(Rotation2d angle) {
        double setpoint = MathUtil.inputModulus(angle.rotateBy(zeroRotation).getRadians(), 0, 2 * Math.PI);
        turnController.setReference(setpoint, ControlType.kPosition);
    }

    public void setDriveVelocity(double velocityRadPerSec) {
        driveController.setReference(velocityRadPerSec, ControlType.kVelocity);
    }

   @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.turnPosition = new Rotation2d(turnEncoder.getPosition()).minus(zeroRotation);
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.drivePositionRad = driveEncoder.getPosition();
    }
}