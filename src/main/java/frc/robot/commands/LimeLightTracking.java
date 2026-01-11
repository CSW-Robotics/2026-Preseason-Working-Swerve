package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;


public class LimeLightTracking extends Command {

  // track with the front limelight
  public static SwerveRequest Track(CommandSwerveDrivetrain drivetrain, LimeLight limelight) {
  
    return new SwerveRequest.RobotCentric() // Robot-centric mode
                .withVelocityX(
                    Math.copySign(
                        Math.min(
                            Math.abs((limelight.targetData[2]-limelight.zoffset))*10,
                             1), // the speed that we limit the limelights at
                        limelight.targetData[2]-limelight.zoffset
                    )
                )   

                .withVelocityY(
                    Math.copySign(
                        Math.min(
                            Math.abs((limelight.targetData[0]-limelight.xoffset)),
                            0.8), // the speed that we limit the limelights at
                        -limelight.targetData[0]-limelight.xoffset
                    )
                )   
                .withRotationalRate(
                    Math.copySign(
                        Math.min(
                            Math.abs((limelight.targetData[4]-limelight.rotoffset)),
                            0.3), // the speed that we limit the limelights at
                        -limelight.targetData[4]-limelight.rotoffset
                    )
                );


}
}
