// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystems;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  DriveTrainSubsystems driveSub;
  DoubleSupplier translationXSupplier;
  DoubleSupplier translationYSupplier;
  DoubleSupplier rotationSupplier;

  public DriveCommand(DriveTrainSubsystems d, DoubleSupplier xtrans, DoubleSupplier ytrans, DoubleSupplier rot) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSub = d;
    translationXSupplier = xtrans;
    translationYSupplier = ytrans;
    rotationSupplier = rot;

    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // +++++++ THIS IS THE FIELD ORIENTED DRIVE ++++++++++
    // if this doesn't work then check in gyro is plugged in
    
      driveSub.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          translationXSupplier.getAsDouble(),
          translationYSupplier.getAsDouble(),
          rotationSupplier.getAsDouble(),
          driveSub.getGyroscopeRotation()
      ));
  

    // =++++++++++ THIS IS THE NON FIELD ORIENTED DRIVE+++++++++
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
      // driveSub.drive(
      //   new ChassisSpeeds(
      //     translationXSupplier.getAsDouble(),
      //     translationYSupplier.getAsDouble(),
      //     rotationSupplier.getAsDouble())
      // );
    
     // Constants.boo = RobotContainer.driver.getRawButtonPressed(2);
      SmartDashboard.putNumber("transX", translationXSupplier.getAsDouble());
      SmartDashboard.putNumber("transY", translationYSupplier.getAsDouble());
      SmartDashboard.putNumber("rotSupplier", rotationSupplier.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}