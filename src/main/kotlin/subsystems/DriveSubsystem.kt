package subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.RobotController.getBatteryVoltage
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import kotlin.math.abs
import org.photonvision.PhotonCamera

data object DataTable {

    private val table = NetworkTableInstance.getDefault().getTable("DriveData")

    val frontleftVelocityEntry: NetworkTableEntry = table.getEntry("Front Left Velocity")
    val frontrightVelocityEntry: NetworkTableEntry = table.getEntry("Front Right Velocity")
    val rearleftVelocityEntry: NetworkTableEntry = table.getEntry("Rear Left Velocity")
    val rearrightVelocityEntry: NetworkTableEntry = table.getEntry("Rear Right Velocity")

    val frontleftVoltageEntry: NetworkTableEntry = table.getEntry("Front Left Voltage")
    val frontrightVoltageEntry: NetworkTableEntry = table.getEntry("Front Right Voltage")
    val rearleftVoltageEntry: NetworkTableEntry = table.getEntry("Rear Left Voltage")
    val rearrightVoltageEntry: NetworkTableEntry = table.getEntry("Rear Right Voltage")
}


object DriveSubsystem : Subsystem {

    private val rightconfig: SparkMaxConfig = SparkMaxConfig()

    private val frontleftmotor = SparkMax(2, SparkLowLevel.MotorType.kBrushless)
    private val frontrightmotor = SparkMax(1, SparkLowLevel.MotorType.kBrushless)
    private val rearleftmotor = SparkMax(3, SparkLowLevel.MotorType.kBrushless)
    private val rearrightmotor = SparkMax(4, SparkLowLevel.MotorType.kBrushless)

    private val imu = ADIS16470_IMU()

    private val frontleftencoder = frontleftmotor.absoluteEncoder
    private val frontrightencoder = frontrightmotor.absoluteEncoder
    private val rearleftencoder = rearleftmotor.absoluteEncoder
    private val rearrightencoder = rearrightmotor.absoluteEncoder

    var mecanumDrive: MecanumDrive = MecanumDrive(
        frontleftmotor, rearleftmotor,
        frontrightmotor, rearrightmotor
    )

    private var frontLeftLocation: Translation2d = Translation2d(0.833, 1.200)
    private var frontRightLocation: Translation2d = Translation2d(0.833, -1.200)
    private var rearLeftLocation: Translation2d = Translation2d(-0.833, 1.200)
    private var rearRightLocation: Translation2d = Translation2d(-0.833, -1.200)

    private val mecanumDriveKinematics: MecanumDriveKinematics = MecanumDriveKinematics(
        frontLeftLocation, frontRightLocation,
        rearLeftLocation, rearRightLocation
    )

    private val mecanumDriveWheelPositions: MecanumDriveWheelPositions = MecanumDriveWheelPositions(
    frontleftencoder.position, frontrightencoder.position,
    rearleftencoder.position, rearrightencoder.position
    )

    private val mecanumDriveOdometry =
        MecanumDriveOdometry(
            mecanumDriveKinematics,
            Rotation2d.fromDegrees(imu.angle),
            mecanumDriveWheelPositions,
            Pose2d()
        )

    private var frontleftFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(4.0, 2.3000, 1.1004)
    private var frontrightFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(4.0, 2.3000, 1.1004)
    private var rearleftFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(4.0, 2.3048, 1.0419)
    private var rearrightFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(4.0, 2.3048, 1.0419)

    private val frontleftVelocityPIDController: PIDController = PIDController(0.0001, 0.0, 0.0)
    private val frontrightVelocityPIDController: PIDController = PIDController(0.0001, 0.0, 0.0)
    private val rearleftVelocityPIDController: PIDController = PIDController(0.0003, 0.000004, 0.007)
    private val rearrightVelocityPIDController: PIDController = PIDController(0.0003, 0.000004, 0.007)

    val pose: Pose2d
        get() = mecanumDriveOdometry.poseMeters

    private val rotation2d
        get() = Rotation2d.fromDegrees(imu.angle)

    val chassisSpeeds: ChassisSpeeds
        get() =
            mecanumDriveKinematics.toChassisSpeeds(
                MecanumDriveWheelSpeeds(
                    frontleftencoder.velocity, frontrightencoder.velocity,
                    rearleftencoder.velocity, rearrightencoder.velocity
                )
            )

private val appliedVoltage = Volts.mutable(0.0)

private val distance = Meters.mutable(0.0)

private val velocity = MetersPerSecond.mutable(0.0)


private val identificationRoutine =
    SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            // drive =
            { voltage: Measure<VoltageUnit> ->
                val volts = voltage.`in`(Volts)
                frontleftmotor.setVoltage(volts)
                frontrightmotor.setVoltage(volts)
                rearleftmotor.setVoltage(volts)
                rearrightmotor.setVoltage(volts)
            },
            // log =
            { log: SysIdRoutineLog ->
                log.motor("drive/front left")
                    .voltage(appliedVoltage.mut_replace(frontleftmotor.get() * getBatteryVoltage(), Volts))
                    .linearPosition(distance.mut_replace(frontleftencoder.position, Meters))
                    .linearVelocity(velocity.mut_replace(frontleftencoder.velocity, MetersPerSecond))

                log.motor("drive/front right")
                    .voltage(appliedVoltage.mut_replace(frontrightmotor.get() * getBatteryVoltage(), Volts))
                    .linearPosition(distance.mut_replace(frontrightencoder.position, Meters))
                    .linearVelocity(velocity.mut_replace(frontrightencoder.velocity, MetersPerSecond))

                log.motor("drive/rear left")
                    .voltage(appliedVoltage.mut_replace(rearleftmotor.get() * getBatteryVoltage(), Volts))
                    .linearPosition(distance.mut_replace(rearleftencoder.position, Meters))
                    .linearVelocity(velocity.mut_replace(rearleftencoder.velocity, MetersPerSecond))

                log.motor("drive/rear right")
                    .voltage(appliedVoltage.mut_replace(rearrightmotor.get() * getBatteryVoltage(), Volts))
                    .linearPosition(distance.mut_replace(rearrightencoder.position, Meters))
                    .linearVelocity(velocity.mut_replace(rearrightencoder.velocity, MetersPerSecond))
            },
            // subsystem =
            this,
        ),
    )

init {
    rightconfig.inverted(false)
    rearrightmotor.configure(rightconfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

    SmartDashboard.putData("Front Left PID Controller", frontleftVelocityPIDController)
    SmartDashboard.putData("Front Right PID Controller", frontrightVelocityPIDController)
    SmartDashboard.putData("Rear Left PID Controller", rearleftVelocityPIDController)
    SmartDashboard.putData("Rear Right PID Controller", rearrightVelocityPIDController)
    SmartDashboard.putData("IMU", imu)
}

override fun periodic() {
    mecanumDriveOdometry.update(
        Rotation2d.fromDegrees(imu.angle),
        mecanumDriveWheelPositions
    )

    DataTable.frontleftVoltageEntry.setDouble(frontleftmotor.busVoltage)
    DataTable.frontrightVoltageEntry.setDouble(frontrightmotor.busVoltage)
    DataTable.rearleftVoltageEntry.setDouble(rearleftmotor.busVoltage)
    DataTable.rearrightVoltageEntry.setDouble(rearrightmotor.busVoltage)

    DataTable.frontleftVelocityEntry.setDouble(frontleftencoder.velocity)
    DataTable.frontrightVelocityEntry.setDouble(frontrightencoder.velocity)
    DataTable.rearleftVelocityEntry.setDouble(rearleftencoder.velocity)
    DataTable.rearrightVelocityEntry.setDouble(rearrightencoder.velocity)
}

fun resetpose (pose: Pose2d) {
    mecanumDriveOdometry.resetPosition(
        rotation2d, mecanumDriveWheelPositions, pose
    )
}

fun driverelative(relativeSpeeds: ChassisSpeeds) {
    val wheelSpeeds = mecanumDriveKinematics.toWheelSpeeds(relativeSpeeds)

    val frontleftFed = frontleftFeedForward.calculate(wheelSpeeds.frontLeftMetersPerSecond)
    val frontrightFed = frontrightFeedForward.calculate(wheelSpeeds.frontRightMetersPerSecond)
    val rearleftFed = rearleftFeedForward.calculate(wheelSpeeds.rearLeftMetersPerSecond)
    val rearrightFed = rearrightFeedForward.calculate(wheelSpeeds.rearRightMetersPerSecond)

    val frontleftOutput = frontleftVelocityPIDController.calculate(frontleftencoder.velocity, wheelSpeeds.frontLeftMetersPerSecond)
    val frontrightOutput = frontrightVelocityPIDController.calculate(frontrightencoder.velocity, wheelSpeeds.frontRightMetersPerSecond)
    val rearleftOutput = rearleftVelocityPIDController.calculate(rearleftencoder.velocity, wheelSpeeds.rearLeftMetersPerSecond)
    val rearrightOutput = rearrightVelocityPIDController.calculate(rearrightencoder.velocity, wheelSpeeds.rearRightMetersPerSecond)

    frontleftmotor.setVoltage(frontleftFed + frontleftOutput)
    frontrightmotor.setVoltage(frontrightFed + frontrightOutput)
    rearleftmotor.setVoltage(rearleftFed + rearleftOutput)
    rearrightmotor.setVoltage(rearrightFed + rearrightOutput)
}

fun mecanumDrive(
    xspeed: Double,
    yspeed: Double,
    zRotation: Double,
){

    var xSpeed = xspeed
    var ySpeed = yspeed

    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation)
}
}