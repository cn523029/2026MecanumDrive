// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.drive.MecanumDrive

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class.  */
class Robot : TimedRobot() {
    private val robotDrive: MecanumDrive
    private val stick: Joystick


    /** Called once at the beginning of the robot program.  */
    init {
        val frontLeft = SparkMax(FRONT_LEFT_CHANNEL, SparkLowLevel.MotorType.kBrushless)
        val rearLeft = SparkMax(REAR_LEFT_CHANNEL, SparkLowLevel.MotorType.kBrushless)
        val frontRight = SparkMax(FRONT_RIGHT_CHANNEL, SparkLowLevel.MotorType.kBrushless)
        val rearRight = SparkMax(REAR_RIGHT_CHANNEL, SparkLowLevel.MotorType.kBrushless)


        // Invert the right side motors.
        // You may need to change or remove this to match your robot.
        rearRight.inverted
        frontRight.inverted

        robotDrive = MecanumDrive(
            { speed: Double -> frontLeft.set(speed) },
            { speed: Double -> rearLeft.set(speed) },
            { speed: Double -> frontRight.set(speed) },
            { speed: Double -> rearRight.set(speed) })

        stick = Joystick(JOYSTICK_CHANNEL)

        SendableRegistry.addChild(robotDrive, frontLeft)
        SendableRegistry.addChild(robotDrive, rearLeft)
        SendableRegistry.addChild(robotDrive, frontRight)
        SendableRegistry.addChild(robotDrive, rearRight)
    }


    override fun teleopPeriodic() {
        // Use the joystick Y axis for forward movement, X axis for lateral
        // movement, and Z axis for rotation.
        robotDrive.driveCartesian(stick.getY(), -stick.getX(), -stick.getZ())
    }

    companion object {
        private const val FRONT_LEFT_CHANNEL = 2
        private const val REAR_LEFT_CHANNEL = 3
        private const val FRONT_RIGHT_CHANNEL = 1
        private const val REAR_RIGHT_CHANNEL = 4

        private const val JOYSTICK_CHANNEL = 0
    }
}