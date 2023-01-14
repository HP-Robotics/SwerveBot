/// Copyright (c) FIRST and other WPILib contributors.
//\ Open Source Software; you can modify and/or share it under the terms of
//     the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 4.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI*4; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.244, 0.244);
  private final Translation2d m_frontRightLocation = new Translation2d(0.244, -0.244);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.244, 0.244);
  private final Translation2d m_backRightLocation = new Translation2d(-0.244, -0.244);

  private final SwerveModule m_frontLeft = new SwerveModule(13, 12); //BIG BONGO 2
  private final SwerveModule m_frontRight = new SwerveModule(2, 3); //BIG BONGO 1
  private final SwerveModule m_backLeft = new SwerveModule(14, 15); //BIG BONGO 3
  private final SwerveModule m_backRight = new SwerveModule(50, 1); //BIG BONGO 4

  ///private final DutyCycleEncoder m_testEncoder = new DutyCycleEncoder(id);
  private final DutyCycleEncoder m_frontLeftEncoder = new DutyCycleEncoder(11);
  private final DutyCycleEncoder m_frontRighttEncoder = new DutyCycleEncoder(12);
  private final DutyCycleEncoder m_backLeftEncoder = new DutyCycleEncoder(13);
  private final DutyCycleEncoder m_backRightEncoder = new DutyCycleEncoder(14);
  //Duty Encoders have the wrong values

  private final AHRS m_gyro = new AHRS(Port.kUSB);
  private final PigeonIMU m_pGyro = new PigeonIMU(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public Drivetrain() {
    m_pGyro.enterCalibrationMode(CalibrationMode.Temperature);
    //m_testEncoder.setDistancePerRotation(0.5);
    m_frontLeftEncoder.setDistancePerRotation(0.5);
    m_frontRighttEncoder.setDistancePerRotation(0.5);
    m_backLeftEncoder.setDistancePerRotation(0.5);
    m_backRightEncoder.setDistancePerRotation(0.5);
  }
  
  /**
   * Method  to  drive  the  robot  using  joystick  info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the ro bot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    System.out.println("Yaw " + m_pGyro.getYaw() + "  Pitch " + m_pGyro.getPitch());
    //System.out.println("X Speed: " + xSpeed + " | Y Speed: " + ySpeed + " | Rotation: " + rot);
    //System.out.println(m_gyro.getYaw());
    //System.out.println("Swerve Stuff" + swerveModuleStates[0].angle + " y " + ySpeed);
    
    //SmartDashboard.putNumber("Duty Encoder", m_testEncoder.get());
  }
}


