// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.DrivetrainConstants.ModuleConstants;
import frc.robot.DrivetrainConstants.DriveConstants.*;
import frc.robot.DrivetrainConstants.ModuleConstants.*;

/** Add your docs here. */
public class SwerveModuleSim {
    private final DCMotorSim drivingMotorSim = new DCMotorSim(
        DCMotor.getNEO(1), 
        Drive.DRIVE_MOTOR_REDUCTION, 
        0.025 //from mechanicaladvantage
        );
    // DCMotorSim(LinearSystemId.createDCMotorSystem(0, 0), null, 0); << can't be used because we don't have ff valued ;-;

    private final DCMotorSim turningMotorSim = new DCMotorSim(
        DCMotor.getNeo550(1), 
        Turning.TURN_MOTOR_REDUCTION, 
        0.004096955
        );

    private final PIDController drivePID = new PIDController(Drive.kP, Drive.kI, Drive.kD);
    private final PIDController turnPID = new PIDController(Turning.kP, Turning.kI, Turning.kD);

    private final SimpleMotorFeedforward driveFF;
    private final SimpleMotorFeedforward turnFF;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());


    private double chassisAngularOffset = 0;

    public SwerveModuleSim(double chassisAngularOffset){
        driveFF = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
        turnFF = new SimpleMotorFeedforward(Turning.kS, Turning.kV, Turning.kA);
        this.chassisAngularOffset = chassisAngularOffset;

        drivingMotorSim.setState(VecBuilder.fill(0, 0));
        turningMotorSim.setState(VecBuilder.fill(0, 0));

        desiredState.angle = new Rotation2d(getTurningPosition());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(getTurningPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePID.calculate(optimizedDesiredState.speedMetersPerSecond, (drivingMotorSim.getAngularVelocityRPM() * ModuleConstants.WHEEL_CIRCUMFERENCE));
    turnPID.calculate(optimizedDesiredState.angle.getRadians(), turningMotorSim.getAngularPositionRad());

    this.desiredState = desiredState;
    }
    
    public double getTurningPosition(){
        return turningMotorSim.getAngularPositionRotations() * Turning.ENCODER_PFACTOR;
    }

    public double getDrivingVelocity(){
        return drivingMotorSim.getAngularVelocityRPM() * Drive.DRIVE_ENCODER_VFACTOR;
    }


    
}
