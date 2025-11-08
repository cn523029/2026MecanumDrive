// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package org.hangar84.mecanum2026

import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {
    override fun robotInit() {
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)

        // Reference the RobotContainer object to ensure that the static initializer is run
        RobotContainer
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {
    }

    override fun disabledPeriodic() {
    }

    override fun teleopInit() {

    }

    override fun teleopPeriodic() {
    }

    override fun autonomousInit() {
        RobotContainer.autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {
    }

    override fun simulationInit() {
    }

    override fun simulationPeriodic() {
    }
}