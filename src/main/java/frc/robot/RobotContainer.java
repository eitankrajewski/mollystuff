// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/2 of a rotation per second max angular velocity

    // Translational acceleration limits: accel > decel (m/s^2). Tune as needed.
    private final AsymmetricSlewRateLimiter vxLimiter = new AsymmetricSlewRateLimiter(3.0, 10.0);
    private final AsymmetricSlewRateLimiter vyLimiter = new AsymmetricSlewRateLimiter(3.0, 10.0);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(vxLimiter.calculate(-joystick.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(vyLimiter.calculate(-joystick.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        // Zero heading on X press (but not when using Back/Start combos for SysId)
        joystick.x()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    // Asymmetric limiter: faster when increasing |speed|, slower when decreasing |speed|.
    private static final class AsymmetricSlewRateLimiter {
        private final double accelRate; // units per second when speeding up
        private final double decelRate; // units per second when slowing down
        private double prevVal;
        private double prevTime;

        AsymmetricSlewRateLimiter(double accelRate, double decelRate) {
            this(accelRate, decelRate, 0.0);
        }

        AsymmetricSlewRateLimiter(double accelRate, double decelRate, double initial) {
            this.accelRate = accelRate;
            this.decelRate = decelRate;
            this.prevVal = initial;
            this.prevTime = Timer.getFPGATimestamp();
        }

        double calculate(double input) {
            double now = Timer.getFPGATimestamp();
            double dt = now - prevTime;
            prevTime = now;

            double delta = input - prevVal;
            boolean increasingMagnitude = Math.abs(input) > Math.abs(prevVal);
            double rate = increasingMagnitude ? accelRate : decelRate;
            double maxChange = rate * dt;

            if (delta > maxChange) delta = maxChange;
            if (delta < -maxChange) delta = -maxChange;

            prevVal += delta;
            return prevVal;
        }

        void reset(double value) {
            prevVal = value;
            prevTime = Timer.getFPGATimestamp();
        }

        double get() { return prevVal; }
    }
}
