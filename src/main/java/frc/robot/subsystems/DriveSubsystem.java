package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  private CANSparkMax frontLeftSpark = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rearLeftSpark = new CANSparkMax(4, MotorType.kBrushless);

  private CANSparkMax frontRightSpark = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rearRightSpark = new CANSparkMax(2, MotorType.kBrushless);

  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(frontLeftSpark,
                               rearLeftSpark);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(frontRightSpark,
                               rearRightSpark);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final CANEncoder m_leftEncoder = rearLeftSpark.getEncoder();

  // The right-side drive encoder
  private final CANEncoder m_rightEncoder = rearRightSpark.getEncoder();

  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private double gear_ratio = 12.75;
  private double circumference = 0.478536;



  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    init();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  public void init(){
    double converter = circumference/(gear_ratio);
    SmartDashboard.putNumber("Converter", converter);
    m_leftEncoder.setPositionConversionFactor(converter);
    m_rightEncoder.setPositionConversionFactor(converter);
    m_leftEncoder.setVelocityConversionFactor(circumference/(gear_ratio*60));
    m_rightEncoder.setVelocityConversionFactor(circumference/(gear_ratio*60));

    resetEncoders();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(),
                      -m_rightEncoder.getPosition());
                      
    SmartDashboard.putNumber("Right", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left", m_leftEncoder.getPosition());

    SmartDashboard.putNumber("Right_vel", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left_vel", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("x", getPose().getTranslation().getX());
    SmartDashboard.putNumber("y", getPose().getTranslation().getY());
    SmartDashboard.putNumber("Angle", m_gyro.getRotation2d().getDegrees());

    m_drive.feed();

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
    m_drive.feed();
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}