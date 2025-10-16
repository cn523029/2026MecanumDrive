package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.Joystick
import subsystems.DriveSubsystem
import subsystems.DriveSubsystem.mecanumDrive

object RobotContainer {

    init{
        configureBindings()
    }

    //private val  controller: XboxController = XboxController(0)
    private val stick: Joystick = Joystick(0)

    private fun configureBindings() {

        DriveSubsystem.defaultCommand =
            DriveSubsystem.run {
                mecanumDrive(stick.getY(), -stick.getX(), -stick.getZ())
            }
    }
}