// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;



/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot
{
    private static final int FRONT_LEFT_CHANNEL = 2;
    private static final int REAR_LEFT_CHANNEL = 3;
    private static final int FRONT_RIGHT_CHANNEL = 1;
    private static final int REAR_RIGHT_CHANNEL = 4;
    
    private static final int JOYSTICK_CHANNEL = 0;
    
    private final MecanumDrive robotDrive;
    private final Joystick stick;
    
    
    /** Called once at the beginning of the robot program. */
    public Robot()
    {
        SparkMax frontLeft = new SparkMax(FRONT_LEFT_CHANNEL, SparkLowLevel.MotorType.kBrushless);
        SparkMax rearLeft = new SparkMax(REAR_LEFT_CHANNEL, SparkLowLevel.MotorType.kBrushless);
        SparkMax frontRight = new SparkMax(FRONT_RIGHT_CHANNEL, SparkLowLevel.MotorType.kBrushless);
        SparkMax rearRight = new SparkMax(REAR_RIGHT_CHANNEL, SparkLowLevel.MotorType.kBrushless);
        
        // Invert the right side motors.
        // You may need to change or remove this to match your robot.
        rearRight.setInverted(true);
        frontRight.setInverted(true);
        
        robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);
        
        stick = new Joystick(JOYSTICK_CHANNEL);
        
        SendableRegistry.addChild(robotDrive, frontLeft);
        SendableRegistry.addChild(robotDrive, rearLeft);
        SendableRegistry.addChild(robotDrive, frontRight);
        SendableRegistry.addChild(robotDrive, rearRight);
    }
    
    
    @Override
    public void teleopPeriodic()
    {
        // Use the joystick Y axis for forward movement, X axis for lateral
        // movement, and Z axis for rotation.
        robotDrive.driveCartesian(stick.getY(), -stick.getX(), -stick.getZ());
    }
}
