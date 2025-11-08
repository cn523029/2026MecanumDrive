package org.hangar84.mecanum2026.subsystems

import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

object LauncherSubsystem : Subsystem {

    val launcherMotor = SparkMax(10, MotorType.kBrushless)
    private val followerLauncherMotor = SparkMax(9, MotorType.kBrushless)

    private val followerConfig = SparkMaxConfig()
    private val launcherConfig = SparkMaxConfig()

    val LAUNCH_COMMAND: Command
        get() =
            runOnce {
                launcherMotor.set(1.0)
            }
                .withTimeout(1.0)
                .andThen({
                    launcherMotor.set(0.0)
                })

    val INTAKE_COMMAND: Command
        get() =
            runOnce {
                launcherMotor.set(-1.0)
            }
                .withTimeout(1.0)
                .andThen({
                    launcherMotor.set(0.0)
                })

    init {
        followerLauncherMotor.configure(
            followerConfig.follow(10),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
        launcherMotor.configure(
            launcherConfig.inverted(true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )
    }
}