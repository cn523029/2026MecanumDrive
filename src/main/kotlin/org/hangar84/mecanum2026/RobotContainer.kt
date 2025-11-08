package org.hangar84.mecanum2026

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.hangar84.mecanum2026.subsystems.DriveSubsystem
import org.hangar84.mecanum2026.subsystems.DriveSubsystem.mecanumDrive
import org.hangar84.mecanum2026.subsystems.LauncherSubsystem

object RobotContainer {
    private val controller : CommandXboxController = CommandXboxController(0)

    private var autoChooser: SendableChooser<Command>? = null

    val autonomousCommand: Command
        get() {
            return autoChooser?.selected ?: InstantCommand()
        }

    init {
        configureBindings()
        cameraSettings()
        configureNamedCommands()

    }

    //private val  controller: XboxController = XboxController(0)


    private fun configureBindings() {

        DriveSubsystem.defaultCommand =
            DriveSubsystem.run {
                mecanumDrive(controller.leftY, -controller.leftX, -controller.rightX)
            }

        LauncherSubsystem.defaultCommand =
            LauncherSubsystem.run {
                LauncherSubsystem.launcherMotor.set(-controller.leftTriggerAxis + controller.rightTriggerAxis)
            }
    }

    private fun cameraSettings() {

    }

    private fun configureNamedCommands() {
        NamedCommands.registerCommand("Launch", LauncherSubsystem.LAUNCH_COMMAND)

        NamedCommands.registerCommand("Intake", LauncherSubsystem.INTAKE_COMMAND)
    }
}