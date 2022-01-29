package frc.robot.subsystems;


public interface DriveTrainConstants {

  //USE CHARACTERIZATION TOOL FOR THIS INFO https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
  public static final double maxVoltage = 7;


  // this is just an estimate In radians
  public static final double maxVelocityPerSecond = 2;

  public static final double maxAngularVelocityPerSecond = maxVelocityPerSecond/Math.hypot(0.4041,.4041);
  //when upside down, decreasing offset = turn left/counterclockwise, increasing offset = turn right/clockwise

  public static final int frontLeftDriveMotor = 3; // FIXME Set front left module drive motor ID
  public static final int frontLeftSteerMotor = 7; // FIXME Set front left module steer motor ID
  public static final int frontLeftSteerEncoder = 11; // FIXME Set front left steer encoder ID
  public static final double frontLeftModuleSteerOffset = -Math.toRadians(111); // FIXME Measure and set front left steer offset

  public static final int frontRightDriveMotor = 2; // FIXME Set front right drive motor ID
  public static final int frontRightSteerMotor = 6; // FIXME Set front right steer motor ID
  public static final int frontRightSteerEncoder = 8; // FIXME Set front right steer encoder ID
  public static final double frontRightModuleSteerOffset = -Math.toRadians(85); // FIXME Measure and set front right steer offset

  public static final int backLeftDriveMotor = 0; // FIXME Set back left drive motor ID
  public static final int backLeftSteerMotor = 4; // FIXME Set back left steer motor ID
  public static final int backLeftSteerEncoder = 10; // FIXME Set back left steer encoder ID
  public static final double backLeftModuleSteerOffset = -Math.toRadians(354); // FIXME Measure and set back left steer offset

  public static final int backRightDriveMotor = 1; // FIXME Set back right drive motor ID
  public static final int backRightSteerMotor = 5; // FIXME Set back right steer motor ID
  public static final int backRightSteerEncoder= 9; // FIXME Set back right steer encoder ID
  public static final double backRightModuleSteerOffset = -Math.toRadians(188); // FIXME Measure and set back right steer offset


  public static final int volts = 12;
  public static final int voltsSecondsPerMeter = 7;
  public static final int voltSecondsSquaredPerMeter = 5;


}