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
import math

class CurveController(wpilib.Joystick):
    @staticmethod
    def sigmoid(x: float, k: float = 0.1, s: float=2) -> float:
        return ((1 / (1 + math.exp(-x / k))) - 0.5) * s

    @staticmethod
    def cubic(x: float):
        # FIXME: impl
        return x

    @override
    def getY(self) -> float:
        y = super().getY()
        y = self.sigmoid(y)
        return y

# FIXME: cubed controller needed but i forgot how geometric stretching works

class TestRobot(wpilib.TimedRobot):
    pd: wpilib.PowerDistribution # pyright: ignore[reportUninitializedInstanceVariable]
    motors: dict[str, rev.SparkMax | wpilib.MotorControllerGroup] = {}
    drivetrain: wpilib.drive.DifferentialDrive # pyright: ignore[reportUninitializedInstanceVariable]
    pose_estim: wpimath.kinematics.DifferentialDriveOdometry # pyright: ignore[reportUninitializedInstanceVariable]
    controller: CurveController # pyright: ignore[reportUninitializedInstanceVariable]
    field: wpilib.Field2d # pyright: ignore[reportUninitializedInstanceVariable]
    gyro: navx.AHRS # pyright: ignore[reportUninitializedInstanceVariable]
    timer: wpilib.Timer # pyright: ignore[reportUninitializedInstanceVariable]

    @override
    def robotInit(self) -> None:
        self.pd = wpilib.PowerDistribution()

        self.controller = CurveController(0)

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

        self.motors["shooter"] = rev.SparkMax(8, type=rev.SparkMax.MotorType.kBrushless)

        self.motors["grabber1"] = rev.SparkMax(6,rev.SparkMax.MotorType.kBrushed)
        self.motors["grabber2"] = rev.SparkMax(7,rev.SparkMax.MotorType.kBrushed)
        self.motors["grabber"] = wpilib.MotorControllerGroup(self.motors["grabber1"], self.motors["grabber2"])

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

        if self.timer.get() > 2.5:
            self.motors["shooter"].set(-0.275)
        elif self.timer.get() > 2.75:
            self.motors["shooter"].set(0)
    
    @override
    def autonomousExit(self) -> None:
        self.drivetrain.tankDrive(
            0.0, 0.0
        )

        for motor in self.motors.values():
            motor.set(0.0)

    @override
    def teleopPeriodic(self) -> None:
        real_points = {
            (0, 1): (1, 1),    # Forward
            (0, -1): (-1, -1), # Backward
            (1, 0): (-1, 1),   # Turn right
            (-1, 0): (1, -1)   # Turn left
        }

        point = (
            self.controller.getZ(),
            -self.controller.getY()
        )

        z, y = point

        # Interpolate between the known points
        if abs(z) < 0.1 and abs(y) < 0.1:
            # Near center - stop
            leftSpeed, rightSpeed = 0.0, 0.0
        else:
            # Find weights based on distance to known points
            total_weight = 0.0
            leftSpeed = 0.0
            rightSpeed = 0.0
            
            for known_point, speeds in real_points.items():
                # Calculate Euclidean distance
                kz, ky = known_point
                distance = math.sqrt((z - kz)**2 + (y - ky)**2)
                
                # Avoid division by zero
                if distance < 0.001:
                    leftSpeed, rightSpeed = speeds
                    break
                
                # Inverse distance weighting
                weight = 1.0 / (distance**2)
                total_weight += weight
                
                # Add weighted contribution
                leftSpeed += speeds[0] * weight
                rightSpeed += speeds[1] * weight
            
            # Normalize by total weight
            if total_weight > 0:
                leftSpeed /= total_weight
                rightSpeed /= total_weight
            
            # Clamp to valid range [-1, 1]
            leftSpeed = max(-1.0, min(1.0, leftSpeed))
            rightSpeed = max(-1.0, min(1.0, rightSpeed))

        self.drivetrain.tankDrive(
            leftSpeed, rightSpeed
        )

        pov = self.controller.getPOV()

        if pov is 0:
            self.motors["grabber"].set(0.25)
        elif pov is 180:
            self.motors["grabber"].set(-0.25)
        else:
            self.motors["grabber"].set(0)
            
        
        if self.controller.getRawButton(1):
            self.motors["shooter"].set(-0.275) 
        else:
            self.motors["shooter"].set(0.0)