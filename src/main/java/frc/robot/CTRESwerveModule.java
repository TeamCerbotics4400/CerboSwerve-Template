// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import team4400.Util.Conversions;
import team4400.Util.Swerve.CANModuleOptimizer;
import team4400.Util.Swerve.SwerveModuleConstants;

/** Add your docs here. */
public class CTRESwerveModule {

    public final int moduleNumber;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    //private final RelativeEncoder driveEncoder;

    private final AnalogEncoder absoluteEncoder;

    private final PIDController turnController;
 
    private final double absoluteEncoderOffset;

    private Rotation2d lastAngle;
    
    public CTRESwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){

        this.moduleNumber = moduleNumber;

        driveMotor = new TalonFX(moduleConstants.driveMotorID, "rio");
        turnMotor = new TalonFX(moduleConstants.driveMotorID, "rio");

        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        turnMotor.getConfigurator().apply(new TalonFXConfiguration());

        var driveConfiguration = driveMotor.getConfigurator();
        var turnConfiguration = turnMotor.getConfigurator();

        var driveMotorConfigs = new MotorOutputConfigs();
        var turnMotorConfigs = new MotorOutputConfigs();

        absoluteEncoderOffset = moduleConstants.angleOffset;
        absoluteEncoder = new AnalogEncoder(moduleConstants.absoluteEncoderID);

        driveMotor.setInverted(moduleConstants.driveReversed);
        turnMotor.setInverted(moduleConstants.turnReversed);

        driveMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        turnMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        var driveSlot0Configs = new Slot0Configs();

        driveSlot0Configs.kP = ModuleConstants.kP;
        driveSlot0Configs.kI = ModuleConstants.kI;
        driveSlot0Configs.kD = ModuleConstants.kD;
        driveSlot0Configs.kS = ModuleConstants.kS;
        driveSlot0Configs.kV = ModuleConstants.kV;
        driveSlot0Configs.kA = ModuleConstants.kA;

        driveConfiguration.apply(driveSlot0Configs, 0.050);
        driveConfiguration.apply(driveMotorConfigs);
        turnConfiguration.apply(turnMotorConfigs);

        turnController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        Timer.delay(1.0);
        resetEncoders();

        lastAngle = getState().angle;
    }

    public double getDrivePosition(){
        return Conversions.TalonFXRotationsToDistanceMeters(
                                                driveMotor.getPosition().getValueAsDouble());
    }

    public double getDriveVelocity(){
        return Conversions.TalonFXRotationsToDistanceMeters(
                                                driveMotor.getVelocity().getValueAsDouble());
    }

    public double getRawAbsoluteVolts(){
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getAngleDeegrees(){
        double rawAngle = 
                (absoluteEncoder.getAbsolutePosition() * 360 - absoluteEncoderOffset) % 360;
        double angle;
        if(rawAngle > 180.0 && rawAngle < 360.0){
            angle = -180 + rawAngle % 180.0;
        } else {
            angle = rawAngle;
        }

        return angle;
    }

    public double turningDeegreesToRadians(){
        return Units.degreesToRadians(getAngleDeegrees());
    }

    public void resetEncoders(){
        driveMotor.setPosition(0);
        absoluteEncoder.reset();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), 
        Rotation2d.fromDegrees(getAngleDeegrees()));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){

        desiredState = 
            CANModuleOptimizer.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);  

        SmartDashboard.putString("Swerve [" + moduleNumber + "] state", desiredState.toString());
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = 
                desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveMotor.set(percentOutput);
        } else {
            var request = new VelocityVoltage(0).withSlot(0);

            driveMotor.setControl(request.withVelocity(
                    Conversions.meters2TalonFXRotations(desiredState.speedMetersPerSecond)));
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= 
        (DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.01)) ? lastAngle : desiredState.angle;

        turnMotor
        .set(turnController.calculate(turningDeegreesToRadians(), desiredState.angle.getRadians()));
        lastAngle = angle;
    }

    public void lockModule(){
        double targetAngle = -45;
        turnMotor.set(turnController.calculate(targetAngle));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(), 
            Rotation2d.fromDegrees(getAngleDeegrees()));
    }

    public void stop(){
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    //Debug
    public void tuneModulePID(double speedMtsPerSec){
        var request = new VelocityVoltage(0).withSlot(0);
        driveMotor.setControl(request.withVelocity(speedMtsPerSec));
    }
}
