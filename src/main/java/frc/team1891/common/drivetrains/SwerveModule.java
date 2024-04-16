// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public abstract class SwerveModule {
    protected int ignorCount = 20;
    protected int m_moduleNumber;
    public static SwerveModule createFromDriveFalconAndSteerFalcon(TalonFX driveKraken,
                                                                   TalonFX steeringKraken,
                                                                   CANcoder encoder,
                                                                   DrivetrainConfig config,
                                                                   double driveP,
                                                                   double driveI,
                                                                   double driveD,
                                                                   double steerP,
                                                                   double steerI,
                                                                   double steerD) {
        return new SwerveModule(encoder, driveP, driveI, driveD, steerP, steerI, steerD, 1) {
            @Override
            public SwerveModuleState getState() {
                return new SwerveModuleState(config.nativeUnitsToVelocityMeters(driveKraken.get()), getCANCoderRotation2d());
            }

            @Override
            public SwerveModulePosition getPosition() {
                // Need to add refresh() in swerve drive train periodic
                return new SwerveModulePosition(config.nativeUnitsToDistanceMeters(driveKraken.getPosition().getValue()), getCANCoderRotation2d());
            }

            @Override
            public void drive(SwerveModuleState desiredState) {
                // Optimize the reference state to avoid spinning further than 90 degrees
                SwerveModuleState state =
                        SwerveModuleState.optimize(desiredState, getCANCoderRotation2d());

                // Calculate the drive output from the drive PID controller.
                final double driveOutput =
                        drivePIDController.calculate(config.nativeUnitsToVelocityMeters(driveKraken.get()), state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                final double turnOutput =
                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), state.angle.getRadians());

                // Calculate the turning motor output from the turning PID controller.
                driveKraken.set(driveOutput);
                steeringKraken.set(turnOutput);
            }
        };
    }

    public static SwerveModule createFromDriveKrakenAndSteeringKraken(TalonFX driveKraken,
                                                                   TalonFX steeringKraken,
                                                                   CANcoder encoder,
                                                                   DrivetrainConfig config,
                                                                   double driveP,
                                                                   double driveI,
                                                                   double driveD,
                                                                   double steerP,
                                                                   double steerI,
                                                                   double steerD,
                                                                   int moduleNumber) {
        return new SwerveModule(encoder, driveP, driveI, driveD, steerP, steerI, steerD, moduleNumber) {
            
            @Override
            public SwerveModuleState getState() {
                return new SwerveModuleState(config.nativeUnitsToVelocityMeters(driveKraken.get()), getCANCoderRotation2d());
            }

            @Override
            public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(config.nativeUnitsToDistanceMeters(driveKraken.getPosition().getValue()), getCANCoderRotation2d());
            }

            @Override
            public void drive(SwerveModuleState desiredState) {
                // Optimize the reference state to avoid spinning further than 90 degrees
                SwerveModuleState state2 = new SwerveModuleState();
                state2.angle = Rotation2d.fromDegrees(RobotContainer.setAngle);
                SwerveModuleState state =
                        SwerveModuleState.optimize(desiredState, getCANCoderRotation2d());

                // Calculate the drive output from the drive PID controller.
                final double driveOutput =
                        drivePIDController.calculate(config.nativeUnitsToVelocityMeters(driveKraken.get()), state.speedMetersPerSecond);
                        // SmartDashboard.putNumber("Position Measurment" + moduleNumber, getPosition().angle.getRadians());
                        // SmartDashboard.putNumber("Position Goal" + moduleNumber, state.angle.getRadians());
                // Calculate the turning motor output from the turning PID controller.
                double turnOutput =
                         turningPIDController.calculate(getPosition().angle.getRadians(), state.angle.getRadians());
                
                //double   turnOutput = 0.5;
                SmartDashboard.putNumber("getCANCoderRotation2d"+ this.m_moduleNumber, getCANCoderRotation2d().getDegrees());
                SmartDashboard.putNumber("desiredState"+ this.m_moduleNumber, desiredState.angle.getDegrees());
                SmartDashboard.putNumber("SwerveModule angle drive"+ this.m_moduleNumber, state.angle.getDegrees());
                SmartDashboard.putNumber("turnOutput-" + this.m_moduleNumber, turnOutput);

                // double testOutput = turningPIDController.calculate(90,0);
                //double testOutput = turningPIDController.calculate(-Math.PI / 2, 0);

                // if (moduleNumber == 1){
                //     SmartDashboard.putNumber("testOutput" + moduleNumber, testOutput);
                // }

                driveKraken.set(driveOutput);//-drive out
          
                // System.out.println("M:" + moduleNumber + " speed:" + driveKraken.get()/(13824/0.31918581360472297881)); // ticks/s -> m/s
                if(ignorCount > 0){
                    ignorCount--;
                    steeringKraken.set(turnOutput/8);
                    //for some reason the motors want to move in order for the PID to snap out of it
                }else{
                    steeringKraken.set(turnOutput);
                }
                //-turnOutput
                if(this.m_moduleNumber == 1){
                    //System.out.printf("#%d desired angle %.02f real angle %.02f PID output %.02f%n", moduleNumber,Math.toDegrees(state.angle.getRadians()), Math.toDegrees(getAbsoluteCANCoderRadians()), turnOutput);
                    //System.out.printf("#%d targetMPS %f%n", moduleNumber, state.speedMetersPerSecond);
                }
                //SmartDashboard.putNumber(m_moduleNumber + "-EncoderRadians", getPosition().angle.getRadians());
                //SmartDashboard.putNumber(m_moduleNumber + "-EncoderDeg", getPosition().angle.getDegrees());
                //SmartDashboard.putNumber(m_moduleNumber + "-TurningPow",turnOutput);
                //SmartDashboard.putNumber(m_moduleNumber + "-SetAngle",state.angle.getDegrees());
            }
        };
    }

    private CANcoder encoder;
    
    private SwerveModuleState desiredState = new SwerveModuleState();

    protected ProfiledPIDController drivePIDController;

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    protected ProfiledPIDController turningPIDController;

    private SwerveModule(CANcoder encoder, double driveP, double driveI, double driveD, double steerP, double steerI, double steerD, int moduleNumber) {
        this.encoder = encoder;
        this.m_moduleNumber = moduleNumber;
        drivePIDController =
                new ProfiledPIDController(driveP, driveI, driveD, new TrapezoidProfile.Constraints(90, 500000));
        turningPIDController =
                new ProfiledPIDController(
                        steerP, steerI, steerD,
                        new TrapezoidProfile.Constraints(
                                // Max angular velocity and acceleration of the module
                                9*Math.PI,
                                9*Math.PI
                        )
                );
                //turningPIDController.setTolerance(Math.toRadians(.5));
        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public abstract SwerveModuleState getState();

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public abstract SwerveModulePosition getPosition();

    protected abstract void drive(SwerveModuleState desiredState);

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        drive(desiredState);
        this.desiredState = desiredState;
    }

    public void setDesiredState(double speedMetersPerSecond, Rotation2d angle) {
        setDesiredState(new SwerveModuleState(speedMetersPerSecond, angle));
    }

    public double getAbsoluteCANCoderRadians() {
        double angle = encoder.getAbsolutePosition().getValue() * 2 * Math.PI;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public Rotation2d getCANCoderRotation2d() {
        return new Rotation2d(getAbsoluteCANCoderRadians());
    }

    public void configureShuffleboard(ShuffleboardLayout moduleLayout) {
        moduleLayout.addNumber("Current Position (Meters)", () -> getPosition().distanceMeters);
        moduleLayout.addNumber("Current Velocity (Meters per Second)", () -> getState().speedMetersPerSecond);
        moduleLayout.addNumber("Current Angle (Radians)", this::getAbsoluteCANCoderRadians);
        moduleLayout.addNumber("Desired Velocity (Meters per Second)", () -> desiredState.speedMetersPerSecond);
        moduleLayout.addNumber("Desired Angle (Radians)", () -> desiredState.angle.getRadians());
    }
}