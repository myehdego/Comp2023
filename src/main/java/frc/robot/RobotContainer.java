package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmRun;
import frc.robot.commands.GripperOpenClose;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Arm arm = new Arm();
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick mechJoytick = new Joystick(OIConstants.kDRiverCOntrollerPort2);
    private final Gripper gripper = new Gripper();
    
    public RobotContainer() {
        /*  swapped out to put drive function in teleopPeriodic
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
        */
        configureButtonBindings();
    }

    private void configureButtonBindings() { 
        // driverJoytick Buttons
        new JoystickButton(driverJoytick, OIConstants.kDriverResetGyroButtonIdx).
        onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoytick, OIConstants.kDriverResetOdometryButtonIdx).
        onTrue(new InstantCommand(() -> 
          swerveSubsystem.resetOdometry(new Pose2d(0., 0., new Rotation2d(0.0)))));
        // whenPressed(() -> swerveSubsystem.resetOdometry(new Pose2d(0., 0., new Rotation2d(0.0))));
        // mechJoytick Buttons
        new JoystickButton(mechJoytick, OIConstants.kArmExtendPos1Button).
        onTrue(new ArmRun(arm,ArmConstants.cubeDepth1));
        new JoystickButton(mechJoytick, OIConstants.kArmExtendPos2Button).
        onTrue(new ArmRun(arm,ArmConstants.cubeDepth2));
        new JoystickButton(mechJoytick, OIConstants.kArmExtendPos0Button).
        onTrue(new ArmRun(arm,ArmConstants.retracto0));
       // onTrue(arm.extensionCommand(ArmConstants.cubeDepth2));
       // onTrue(arm.extensionCommand(ArmConstants.cubeDepth1));
       new JoystickButton(mechJoytick, OIConstants.kgripperopenbutton).
       onTrue(new GripperOpenClose(gripper, true));
     }
    

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        double scale = -.4;

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(

                        // new Translation2d(  -1 * scale,   0 * scale),
                        // new Translation2d(  -1 * scale,   1 * scale),
                        // new Translation2d(  -2 * scale ,  1 * scale),
                        // new Translation2d(  -2 * scale ,  0 * scale),
                        // new Translation2d(  -1 * scale,   0 * scale),
                        // new Translation2d(  -1 * scale,   1 * scale),
                        // new Translation2d(   0 * scale,   1 * scale),
                        // new Translation2d(   0 * scale,   0 * scale),

                     //   new Translation2d(  -1 * scale,   0 * scale),
                     //   new Translation2d(  -1 * scale,   1 * scale),
                     //   new Translation2d(  -2 * scale ,  1 * scale),
                     //   new Translation2d(  -2 * scale ,  0 * scale),
                     //   new Translation2d(  -1 * scale,   0 * scale),
                        new Translation2d(  -1 * scale,   1 * scale),
                        new Translation2d(   0 * scale,   1 * scale)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(180)),  //360)),
                trajectoryConfig);



        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public SwerveSubsystem getSwerveSS() {
            return swerveSubsystem;
    }

    public Arm getarmSS() {
        return arm;
    }

}
