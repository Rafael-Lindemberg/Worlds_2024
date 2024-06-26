// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1891.common.hardware.NavX;

/** Swerve Drivetrain base. */
public class SwerveDrivetrain extends HolonomicDrivetrain {
  public static final double MAX_VOLTAGE = 12.0;

  public static final SwerveModuleState[] EMPTY_SWERVE_MODULE_STATES = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };

  protected final SwerveDrivePoseEstimator poseEstimator;
  private final Translation2d frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation;
  protected final SwerveDriveKinematics kinematics;

  private final SwerveModule frontLeft, frontRight, backLeft, backRight;

  protected final Field2d modulesField;
  private final double modulesFieldXOffset, modulesFieldYOffset;
  private double inputScale = 1;
  /**
   * Creates a new SwerveDrivetrain, assuming the center of the robot is the center of the drivebase.
   * @param config the config of the drivetrain
   * @param driveBaseWidth the width between left and right modules
   * @param driveBaseLength the length between front and back modules
   * @param gyro the gyro of the drivetrain
   * @param frontLeft the front left module
   * @param frontRight the front right module
   * @param backLeft the back left module
   * @param backRight the back right module
   */
  public SwerveDrivetrain(
    ShuffleboardTab shuffleboardTab,
    DrivetrainConfig config,
    double driveBaseWidth,
    double driveBaseLength,
    NavX gyro,
    SwerveModule frontLeft,
    SwerveModule frontRight,
    SwerveModule backLeft,
    SwerveModule backRight
  ) {
    this(
      shuffleboardTab,
      config,
      new Translation2d(driveBaseLength / 2d, driveBaseWidth / 2d),
      new Translation2d(driveBaseLength / 2d, -driveBaseWidth / 2d),
      new Translation2d(-driveBaseLength / 2d, driveBaseWidth / 2d),
      new Translation2d(-driveBaseLength / 2d, -driveBaseWidth / 2d),
      gyro,
      frontLeft,
      frontRight,
      backLeft,
      backRight
    );
  }

  /**
   * Creates a new SwerveDrivetrain.
   * @param config the config of the drivetrain
   * @param frontLeftLocation the location of the front left module relative to the robot center
   * @param frontRightLocation the location of the front right module relative to the robot center
   * @param backLeftLocation the location of the back left module relative to the robot center
   * @param backRightLocation the location of the back right module relative to the robot center
   * @param gyro the gyro of the drivetrain
   * @param frontLeft the front left module
   * @param frontRight the front right module
   * @param backLeft the back left module
   * @param backRight the back right module
   */
  public SwerveDrivetrain(
    ShuffleboardTab shuffleboardTab,
    DrivetrainConfig config,
    Translation2d frontLeftLocation,
    Translation2d frontRightLocation,
    Translation2d backLeftLocation,
    Translation2d backRightLocation,
    NavX gyro,
    SwerveModule frontLeft,
    SwerveModule frontRight,
    SwerveModule backLeft,
    SwerveModule backRight
  ) {
    super(shuffleboardTab, config, gyro);

    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;

    this.frontLeftLocation = frontLeftLocation;
    this.frontRightLocation = frontRightLocation;
    this.backLeftLocation = backLeftLocation;
    this.backRightLocation = backRightLocation;

    this.kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getSwerveModulePositions(), new Pose2d());

    this.modulesField = new Field2d();
    this.modulesFieldXOffset = Math.abs(frontLeftLocation.getX()) + Math.abs(backLeftLocation.getX());
    this.modulesFieldYOffset = Math.abs(frontLeftLocation.getY()) + Math.abs(backLeftLocation.getY());
    modulesField.setRobotPose(modulesFieldXOffset, modulesFieldYOffset, new Rotation2d());
    SmartDashboard.putBoolean("Modules Robot Relative", true);
    SmartDashboard.putBoolean("Modules Show Desired States", true);
    //SmartDashboard.putData("Modules (Field2d)", modulesField);

    //SmartDashboard.putNumber("1-Radians", frontLeft.getCANCoderRotation2d().getRadians());
    //SmartDashboard.putNumber("2-Radians", backLeft.getCANCoderRotation2d().getRadians());
    //SmartDashboard.putNumber("3-Radians", frontRight.getCANCoderRotation2d().getRadians());
    //SmartDashboard.putNumber("4-Radians", backRight.getCANCoderRotation2d().getRadians());
  }

  public void holonomicDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // if(Math.abs(xSpeed) < .08 && Math.abs(ySpeed) < .08){
    //   xSpeed = 0;
    //   ySpeed = 0;
    // }
    // if(Math.abs(rot) < .08){
    //   rot = 0;
    // }
    SmartDashboard.putNumber("yJoy", ySpeed);
    SmartDashboard.putNumber("xJoy", xSpeed);
    SmartDashboard.putNumber("twistJoy", rot);
    xSpeed *= config.chassisMaxVelocityMetersPerSecond * inputScale;
    ySpeed *= config.chassisMaxVelocityMetersPerSecond * inputScale;
    rot *= config.chassisMaxAngularVelocityRadiansPerSecond * inputScale;

    SwerveModuleState[] swerveModuleStates;
    if (Math.abs(xSpeed) < 0.2 && Math.abs(ySpeed) < 0.2 && Math.abs(rot) < 0.2) {
      swerveModuleStates = new SwerveModuleState[] {
        new SwerveModuleState(0, frontLeft.getState().angle),
        new SwerveModuleState(0, frontRight.getState().angle),
        new SwerveModuleState(0, backLeft.getState().angle),
        new SwerveModuleState(0, backRight.getState().angle)
      };
    } else {
      swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative?
            (DriverStation.getAlliance().equals(Alliance.Blue)?
              ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose2d().getRotation())
            :
              ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose2d().getRotation().rotateBy(Rotation2d.fromDegrees(180))))
        :
            new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
    }
    // for(int i = 0; i < 4; i++){
      
    //   SmartDashboard.putNumber("swerveModuleStates speedMetersPerSecond"+i, swerveModuleStates[i].speedMetersPerSecond);
    //   SmartDashboard.putNumber("swerveModuleStates degrees"+i, swerveModuleStates[i].angle.getDegrees());
    // }
    setSwerveModuleStates(swerveModuleStates);
  }
  public void setInputScale(double scale){
    inputScale = scale;
    //System.out.println("set input scale to: " + scale);
  }
  @Override
  public void fromChassisSpeeds(ChassisSpeeds speeds) {
      setSwerveModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * Sets the 4 swerve modules to the desired states.
   * @param swerveModuleStates An array of length 4, of the desired module states
   */
  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    // Normalize wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, config.chassisMaxVelocityMetersPerSecond);
// for(int i = 0; i < 4; i++){
      
//       SmartDashboard.putNumber("swerveModuleStates speedMetersPerSecond"+i, swerveModuleStates[i].speedMetersPerSecond);
//       SmartDashboard.putNumber("swerveModuleStates degrees"+i, swerveModuleStates[i].angle.getDegrees());
//     }
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Stops all drive motors.
   */
  public void stop() {
    frontLeft.setDesiredState(0, frontLeft.getState().angle);
    frontRight.setDesiredState(0, frontRight.getState().angle);
    backLeft.setDesiredState(0, backLeft.getState().angle);
    backRight.setDesiredState(0, backRight.getState().angle);
  }

  
  @Override
  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void resetOdometry(Pose2d pose2d) {
      poseEstimator.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), pose2d);
      field.setRobotPose(pose2d);
  }

  /**
   * Updates the field relative position of the robot.
   * 
   * Odometry is measured in meters.
   */
  public void updateOdometry() {
    poseEstimator.update(
      gyro.getRotation2d(),
      getSwerveModulePositions()
    );
  }

  @Override
  protected void configureShuffleboard() {
    //frontLeft.configureShuffleboard(shuffleboardTab.getLayout("Front Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0));
    //frontRight.configureShuffleboard(shuffleboardTab.getLayout("Front Right", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0));
    //backLeft.configureShuffleboard(shuffleboardTab.getLayout("Back Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0));
    //backRight.configureShuffleboard(shuffleboardTab.getLayout("Back Right", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0));

    //gyroLayout.addNumber("Radians", gyro::getRadians);
    //gyroLayout.addNumber("Degrees", gyro::getDegrees);
    //gyroLayout.addNumber("Degrees (Looped)", gyro::getDegreesLooped);
    //shuffleboardTab.addNumber("Chassis x Speed (Meters per Second)", () -> kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()).vxMetersPerSecond);
    //shuffleboardTab.addNumber("Chassis y Speed (Meters per Second)", () -> kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()).vyMetersPerSecond);
    //shuffleboardTab.addNumber("Chassis omega Speed (Radians per Second)", () -> kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()).omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {
    super.periodic();
    //System.out.println("set input scale to: " + inputScale);
    // modulesField
    Translation2d frontLeftT = new Translation2d(frontLeftLocation.getX(), frontLeftLocation.getY());
    Translation2d frontRightT =  new Translation2d(frontRightLocation.getX(), frontRightLocation.getY());
    Translation2d backLeftT =  new Translation2d(backLeftLocation.getX(), backLeftLocation.getY());
    Translation2d backRightT =  new Translation2d(backRightLocation.getX(), backRightLocation.getY());
    if (SmartDashboard.getBoolean("Modules Robot Relative", true)) {
      frontLeftT = frontLeftT.rotateBy(gyro.getRotation2d());
      frontRightT = frontRightT.rotateBy(gyro.getRotation2d());
      backLeftT = backLeftT.rotateBy(gyro.getRotation2d());
      backRightT = backRightT.rotateBy(gyro.getRotation2d());
      modulesField.setRobotPose(modulesFieldXOffset, modulesFieldYOffset, gyro.getRotation2d());
    } else {
      modulesField.setRobotPose(modulesFieldXOffset, modulesFieldYOffset, new Rotation2d());
    }

    modulesField.getObject("frontLeft").setPose(frontLeftT.getX()+modulesFieldXOffset, frontLeftT.getY()+modulesFieldYOffset, frontLeft.getCANCoderRotation2d().rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
    modulesField.getObject("frontRight").setPose(frontRightT.getX()+modulesFieldXOffset, frontRightT.getY()+modulesFieldYOffset, frontRight.getCANCoderRotation2d().rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
    modulesField.getObject("backLeft").setPose(backLeftT.getX()+modulesFieldXOffset, backLeftT.getY()+modulesFieldYOffset, backLeft.getCANCoderRotation2d().rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
    modulesField.getObject("backRight").setPose(backRightT.getX()+modulesFieldXOffset, backRightT.getY()+modulesFieldYOffset, backRight.getCANCoderRotation2d().rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));

    if (SmartDashboard.getBoolean("Modules Show Desired States", true)) {
      modulesField.getObject("frontLeftDesired").setPose(frontLeftT.getX() + modulesFieldXOffset, frontLeftT.getY() + modulesFieldYOffset, frontLeft.getDesiredState().angle.rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
      modulesField.getObject("frontRightDesired").setPose(frontRightT.getX() + modulesFieldXOffset, frontRightT.getY() + modulesFieldYOffset, frontRight.getDesiredState().angle.rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
      modulesField.getObject("backLeftDesired").setPose(backLeftT.getX() + modulesFieldXOffset, backLeftT.getY() + modulesFieldYOffset, backLeft.getDesiredState().angle.rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
      modulesField.getObject("backRightDesired").setPose(backRightT.getX() + modulesFieldXOffset, backRightT.getY() + modulesFieldYOffset, backRight.getDesiredState().angle.rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
    }
  }
}
