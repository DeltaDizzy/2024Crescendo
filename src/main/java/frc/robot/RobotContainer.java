// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveController = new CommandXboxController(0); // My joystick

  
  private final Telemetry logger = new Telemetry();
  private final Intake intakeSub = new Intake();
  private final CommandSwerveDrivetrain driveSub = TunerConstants.DriveTrain;
  private final Vision vision = new Vision(driveSub::getRobotPose2d); // TODO: get the orangePi set up on the main robot asap
  private final Shooter shooterSub = new Shooter(vision);

  private void configureBindings() {
    driveSub.setDefaultCommand(driveSub.driveFieldRelative(driveController::getLeftY, driveController::getLeftX, driveController::getRightX));
    driveController.rightBumper().whileTrue(driveSub.brake());

    /*driveController.leftBumper().whileTrue(
      drivetrain.applyRequest(
        () -> point.withModuleDirection(
          new Rotation2d(
            -driveController.getLeftY(),
            -driveController.getLeftX()
          )
        )
      )
     );*/
    //driveController.a().onTrue(ShootSpeaker());
    driveController.a().onTrue(shooterSub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driveController.b().onTrue(shooterSub.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    driveController.x().whileTrue(intakeSub.Run()).onFalse(intakeSub.stop());
    driveController.y().whileTrue(intakeSub.Reverse()).whileFalse(intakeSub.stop());
    //driveController.y().whileTrue(shooterSub.RunAtVelocity(1));
    driveController.pov(0).whileTrue(shooterSub.Run()).whileFalse(shooterSub.stop());
    // reset the field-centric heading on left bumper press
    //driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driveController.leftBumper().onTrue(shooterSub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driveController.rightBumper().onTrue(shooterSub.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driveController.a().onTrue(shootWhileMoving(shooterSub, driveSub));
    if (Utils.isSimulation()) {
      driveSub.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    driveSub.registerTelemetry(logger::telemeterize);
  }
  public Command ShootSpeaker(){
    return 
    intakeSub.Reverse()
    .andThen(shooterSub.Run())
    .andThen(new WaitCommand(0.3))
    .andThen(intakeSub.stop())
    .andThen(new WaitCommand(0.5))
    .andThen(intakeSub.Run())
    .andThen(new WaitCommand(0.5))
    .andThen(shooterSub.stop())
    .andThen(intakeSub.stop());
  }

  public RobotContainer() {
    NamedCommands.registerCommand("IntakeCommand", intakeSub.Run().andThen(new WaitCommand(1)));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    driveSub.registerTelemetry((state) -> logger.telemeterize(state));
  }

  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
  }

  public Command shootWhileMoving(Shooter shooter, CommandSwerveDrivetrain drivetrain) {
    return Commands.run(() -> {
      double speedPerpendicularToSpeaker = drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond;
      Rotation2d currentRotation =  drivetrain.getRobotPose2d().getRotation();

      //these make a right triangle
      double distanceToSpeaker = vision.GetRobotToSpeakerDistance();
      double goalDeltaX = (shooter.GetApproxExitVelocity()*distanceToSpeaker) * speedPerpendicularToSpeaker;

      //trig to find angle after robot has moved    
      Rotation2d CorrectedShotAngle = currentRotation.plus(new Rotation2d(Math.atan(goalDeltaX/distanceToSpeaker)));

      //pythagorean therom to find the new shot distance
      double NewShotDistance = Math.sqrt(Math.pow(distanceToSpeaker, 2) + Math.pow(goalDeltaX, 2));

      //correct for shot

      //if we are within 5 degrees then stop trying to correct
      if(Math.abs(currentRotation.getDegrees() - CorrectedShotAngle.getDegrees()) > 5){
        drivetrain.applyRequest(()-> new SwerveRequest.FieldCentricFacingAngle().withTargetDirection(CorrectedShotAngle));
      }
    }, shooter, drivetrain);
  }

  public void runPeriodics() {
    vision.periodic();
  }
}
