from typing import override
import wpilib
import wpilib.drive
import rev


class TestRobot(wpilib.TimedRobot):
    pd: wpilib.PowerDistribution # pyright: ignore[reportUninitializedInstanceVariable]
    motors: dict[str, rev.SparkMax] = {}
    drivetrain: wpilib.drive.DifferentialDrive # pyright: ignore[reportUninitializedInstanceVariable]
    controller: wpilib.XboxController # pyright: ignore[reportUninitializedInstanceVariable]
    

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

        _ = lFollowerConfig.apply(globalConfig).follow(self.motors["left_front"]).inverted(True)
        _ = rFollowerConfig.apply(globalConfig).follow(self.motors["right_front"])

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

        self.drivetrain = wpilib.drive.DifferentialDrive(self.motors["left_front"], self.motors["right_front"])

        self.motors["elevator1"] = rev.SparkMax(6, rev.SparkMax.MotorType.kBrushed)
        self.motors["elevator2"] = rev.SparkMax(7, rev.SparkMax.MotorType.kBrushed)

        elevatorFollower = rev.SparkMaxConfig().inverted(True)

        _ = self.motors["elevator2"].configure(
            elevatorFollower,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        self.motors["thingy"] = rev.SparkMax(8, rev.SparkMax.MotorType.kBrushless)

    @override
    def autonomousPeriodic(self) -> None:
        self.drivetrain.tankDrive(
            0.25, 0.25
        )
    
    @override
    def autonomousExit(self) -> None:
        self.drivetrain.tankDrive(
            0.0, 0.0
        )

    @override
    def teleopPeriodic(self) -> None:
        self.drivetrain.tankDrive(
            self.controller.getLeftY(), self.controller.getRightY()
        )
        if self.controller.getAButton():
            self.motors["elevator1"].set(1.0)
            self.motors["elevator2"].set(1.0)
        elif self.controller.getBButton():
            self.motors["elevator1"].set(-1.0)
            self.motors["elevator2"].set(-1.0)
        else:
            self.motors["elevator1"].set(0.0)
            self.motors["elevator2"].set(0.0)
        
        if self.controller.getLeftBumperButton():
            self.motors["thingy"].set(-1.0)
        elif self.controller.getRightBumperButton():
            self.motors["thingy"].set(1.0)
        else:
            self.motors["thingy"].set(0.0)