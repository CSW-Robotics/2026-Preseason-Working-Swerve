// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.LimeLightTracking;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(2);
    private final Joystick l_joystick = new Joystick(0);
    private final Joystick r_joystick = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final LimeLight f_limelight = new LimeLight("limelight-front",0,0,0);

    

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(r_joystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(r_joystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-l_joystick.getX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        
        // Theoretically resets the field reletive possitioning
        new JoystickButton(r_joystick,3).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // Theoretically applies the break works great in the sim
        new JoystickButton(r_joystick,5).whileTrue(drivetrain.applyRequest(() -> brake));


        new JoystickButton(r_joystick,1).whileTrue(drivetrain.applyRequest(() -> LimeLightTracking.Track(drivetrain,f_limelight)));
        
        
        
        new JoystickButton(r_joystick,1).onTrue(new InstantCommand(()->System.out.println(f_limelight.tv)));

        // Should theoretically switch us to robot reletive mode. I want to test in person
        new JoystickButton(l_joystick,1).whileTrue(
            drivetrain.applyRequest( () -> new SwerveRequest.RobotCentric() // Robot-centric mode
                .withVelocityX(-r_joystick.getY()*4)   
                .withVelocityY(-r_joystick.getX()*4)   
                .withRotationalRate(-l_joystick.getX()*4)
        ));



        // We will want to use these system ids to tune the pid constants 

             // // Run SysId routines when holding back/start and X/Y.
            // // Note that each routine should be run exactly once in a single log.
            // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));



        // ALL OF THE STUFF THAT WAS GENERATED THAT I COMMENTED OUT


        // // Idle while the robot is disabled. This ensures the configured
        // // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
