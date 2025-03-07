from typing import override
import wpilib
import wpilib.drive
import wpilib.simulation
import navx
import wpimath
import wpimath.estimator
import wpimath.kinematics
import wpimath.geometry
import rev


class TestRobot(wpilib.TimedRobot):
    pd: wpilib.PowerDistribution # pyright: ignore[reportUninitializedInstanceVariable]
    motors: dict[str, rev.SparkMax] = {}
    drivetrain: wpilib.drive.DifferentialDrive # pyright: ignore[reportUninitializedInstanceVariable]
    pose_estim: wpimath.kinematics.DifferentialDriveOdometry # pyright: ignore[reportUninitializedInstanceVariable]
    controller: wpilib.XboxController # pyright: ignore[reportUninitializedInstanceVariable]
    field: wpilib.Field2d # pyright: ignore[reportUninitializedInstanceVariable]
    gyro: navx.AHRS # pyright: ignore[reportUninitializedInstanceVariable]
    timer: wpilib.Timer # pyright: ignore[reportUninitializedInstanceVariable]

    @override
    def robotInit(self) -> None:
        self.pd = wpilib.PowerDistribution()

        self.controller = wpilib.XboxController(0)

        self.motors["right_back"] = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushed)
        self.motors["right_front"] = rev.SparkMax(3, rev.SparkMax.MotorType.kBrushed)
        self.motors["left_front"] = rev.SparkMax(4, rev.SparkMax.MotorType.kBrushed)
        self.motors["left_back"] = rev.SparkMax(5, rev.SparkMax.MotorType.kBrushed)

        globalConfig = rev.SparkMaxConfig()
        rLeaderConfig = rev.SparkMaxConfig()
        lFollowerConfig = rev.SparkMaxConfig()
        rFollowerConfig = rev.SparkMaxConfig()

        _ = rLeaderConfig.apply(globalConfig).inverted(True)

        _ = lFollowerConfig.apply(globalConfig)
        _ = rFollowerConfig.apply(rLeaderConfig)

        _ = self.motors["left_front"].configure(
            globalConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )
        _ = self.motors["right_front"].configure(
            rLeaderConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )
        
        _ = self.motors["left_back"].configure(
            lFollowerConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )
        _ = self.motors["right_back"].configure(
            rFollowerConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        left_group = wpilib.MotorControllerGroup(self.motors["left_back"], self.motors["left_front"])
        right_group = wpilib.MotorControllerGroup(self.motors["right_back"], self.motors["right_front"])

        self.drivetrain = wpilib.drive.DifferentialDrive(left_group, right_group)

        self.motors["shooter"] = rev.SparkMax(8, rev.SparkMax.MotorType.kBrushless)

        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        self.gyro.reset()

        self.timer = wpilib.Timer()
        self.timer.start()

    @override
    def robotPeriodic(self) -> None:
        _ = wpilib.SmartDashboard.putNumber(
            "gyro_angle",
            self.gyro.getAngle()
        )
        _ = wpilib.SmartDashboard.putNumber(
            "gyro_pitch",
            self.gyro.getPitch()
        )

    @override
    def autonomousPeriodic(self) -> None:
        self.drivetrain.tankDrive(0.5, 0.5)
    
    @override
    def autonomousExit(self) -> None:
        self.drivetrain.tankDrive(
            0.0, 0.0
        )

        for motor in self.motors.values():
            motor.set(0.0)

    @override
    def teleopPeriodic(self) -> None:
        self.drivetrain.tankDrive(
            -self.controller.getLeftY(), -self.controller.getRightY()
        )
        
        if self.controller.getLeftBumperButton():
            self.motors["shooter"].set(-1.0)
        elif self.controller.getRightBumperButton():
            self.motors["shooter"].set(1.0)
        else:
            self.motors["shooter"].set(0.0)