// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SerialPort;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveLibrary.*;

public class DriveTrainSubsystems extends SubsystemBase implements DriveTrainConstants {
  /** Creates a new DriveTrainSubsystems. */


  // search up kauai labs frc gyro port
  AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

  // Odometry class for tracking robot pose
    private final SwerveDriveOdometry odo = new SwerveDriveOdometry(Constants.m_kinematics, ahrs.getRotation2d());

    // These are the modules initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    SwerveModuleState[] states = Constants.m_kinematics.toSwerveModuleStates(chassisSpeeds);


  public DriveTrainSubsystems() {

    frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
      // This can either be STANDARD or FAST depending on your gear configuration
      Mk4SwerveModuleHelper.GearRatio.L1,
      // This is the ID of the drive motor
      frontLeftDriveMotor,
      // This is the ID of the steer motor
      frontLeftSteerMotor,
      // This is the ID of the steer encoder
      frontLeftSteerEncoder,
      // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
      frontLeftModuleSteerOffset);

    frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
      Mk4SwerveModuleHelper.GearRatio.L1,
      frontRightDriveMotor,
      frontRightSteerMotor,
      frontRightSteerEncoder,
      frontRightModuleSteerOffset
    );

      backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        Mk4SwerveModuleHelper.GearRatio.L1,
        backLeftDriveMotor,
        backLeftSteerMotor,
        backLeftSteerEncoder,
        backLeftModuleSteerOffset
      );

      backRightModule = Mk4SwerveModuleHelper.createFalcon500(
        // Shuffleboard.getTab("SwerveData").getLayout("SwerveData"),
        Mk4SwerveModuleHelper.GearRatio.L1,
        backRightDriveMotor,
        backRightSteerMotor,
        backRightSteerEncoder,
        backRightModuleSteerOffset
      );
  }

  public void zeroGyroscope() {
    //zeros gyroscope
    ahrs.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() 
  {
    if (ahrs.isMagnetometerCalibrated()) {
     // We will only get valid fused headings if the magnetometer is calibrated     
     return Rotation2d.fromDegrees(ahrs.getFusedHeading());
    }     
    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - ahrs.getYaw());
  }

  public void drive(ChassisSpeeds speeds)
  {
    states = Constants.m_kinematics.toSwerveModuleStates(speeds);
    frontLeftModule.set(states[0].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage, states[3].angle.getRadians());
  }

  public void setModules(SwerveModuleState[] speeds)
  {
    drive(Constants.m_kinematics.toChassisSpeeds(speeds));
  }

  @Override
  public void periodic() {
    // This method will be called once per sched uler run
    SmartDashboard.putString("Gyro Rotation: ", getGyroscopeRotation().toString());
    SmartDashboard.putString("chassis speeds: ", chassisSpeeds.toString());

    // updateStates();
    // setModules();
    updateOdo();
  }

  public void updateOdo()
  {
    odo.update(ahrs.getRotation2d(), states[0],states[1],states[2],states[3]);
    SmartDashboard.putString("Odo", ""+odo.getPoseMeters());
  }

  public Pose2d getPose() 
  {
   return odo.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) 
  {
    odo.resetPosition(pose, ahrs.getRotation2d());
  }

  // private void resetEncoders() {
  //   // Some  has to go here! //FIXME figure out encoder reset
  //   }
}
 