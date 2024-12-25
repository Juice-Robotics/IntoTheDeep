package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LoopAction;
import org.firstinspires.ftc.teamcode.commands.primitives.InstantCommand;
import org.firstinspires.ftc.teamcode.commands.primitives.LoopCommand;
import org.firstinspires.ftc.teamcode.commands.primitives.ParallelCommand;
import org.firstinspires.ftc.teamcode.commands.primitives.SequentialCommand;
import org.firstinspires.ftc.teamcode.commands.primitives.SleepCommand;
import org.firstinspires.ftc.teamcode.commands.util.Commands;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseKeeper;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.misc.FullPose2d;
import org.firstinspires.ftc.teamcode.zestyzoom.Drive;
import org.firstinspires.ftc.teamcode.zestyzoom.trajectory.TrajectoryCommand;

@Autonomous(name = "a", group = "Autonomous")
public class AutonTest extends LinearOpMode {
    public void runOpMode() {
        Pose2D beginPose = new Pose2D(DistanceUnit.INCH, -12.0, -60.0, AngleUnit.RADIANS, Math.PI);
        Drive drive = new Drive(hardwareMap, beginPose, telemetry);

        TrajectoryCommand preloadDrive = drive.trajectoryBuilder(drive.pose)
                .addPoint(new Pose2D(DistanceUnit.INCH, 0, -28, AngleUnit.RADIANS, Math.PI))
                .waitSeconds(0.5)
                .build();
        TrajectoryCommand spike1 = drive.trajectoryBuilder(preloadDrive.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH,-38, -43, AngleUnit.RADIANS, Math.toRadians(23)))
                .waitSeconds(0.5)
                .build();
//        TrajectoryCommand observation1 = drive.trajectoryBuilder(spike1.endPose())
//                .addPoint(new Pose2D(DistanceUnit.INCH,35, -44.5, AngleUnit.RADIANS, Math.toRadians(-55)))
//                .waitSeconds(0.5)
//                .build();
//        TrajectoryCommand spike2 = drive.trajectoryBuilder(observation1.endPose())
//                .addPoint(new Pose2D(DistanceUnit.INCH,35, -44.5, AngleUnit.RADIANS, Math.toRadians(-55)))
//                .waitSeconds(0.5)
//                .build();
//
//        TrajectoryCommand spike2 = drive.actionBuilder(new Pose2d(35, -44.5, Math.toRadians(-55)))
//                .splineToLinearHeading(new Pose2d(51.5, -42, Math.toRadians(80)), Math.toRadians(70))
//                .build();
//        TrajectoryCommand observation2 = drive.actionBuilder(new Pose2d(51.5, -42, Math.toRadians(80)))
//                .splineToLinearHeading(new Pose2d(38, -44.5, Math.toRadians(-60)), Math.toRadians(0))
//                .build();
//        Action back = drive.actionBuilder(new Pose2d(38, -44.5, Math.toRadians(-60)))
//                .setReversed(true)
//                .setTangent(9 * Math.PI/10)
//                .splineToLinearHeading(new Pose2d(21, -36, Math.toRadians(-45)), 9 * Math.PI/10)
//                .build();
//        Action intake1 = drive.actionBuilder(new Pose2d(21, -36, Math.toRadians(-45)))
//                .setReversed(false)
//                .lineToX(27)
//                .build();
//        Action deposit = drive.actionBuilder(new Pose2d(22, -37, Math.toRadians(-45)))
//                .setTangent(9 * Math.PI/10)
//                .splineToLinearHeading(new Pose2d(5, -27, Math.toRadians(-90)), Math.PI/2)
//                .build();
//        Action intake2 = drive.actionBuilder(new Pose2d(5, -27, Math.toRadians(-90)))
//                .setReversed(false)
//                .setTangent(-4*Math.PI/10)
//                .splineToLinearHeading(new Pose2d(25, -42,  Math.toRadians(-45)), 9*Math.PI/10)
//                .build();
//        Action deposit2 = drive.actionBuilder(new Pose2d(23, -39, Math.toRadians(-45)))
//                .setTangent(9 * Math.PI/10)
//                .splineToLinearHeading(new Pose2d(0, -27, Math.toRadians(-90)), Math.PI/2)
//                .waitSeconds(0.5)
//                .build();
//        Action intake3 = drive.actionBuilder(new Pose2d(0, -27, Math.toRadians(-90)))
//                .setReversed(false)
//                .setTangent(-4*Math.PI/10)
//                .splineToLinearHeading(new Pose2d(25, -42,  Math.toRadians(-45)), 9*Math.PI/10)
//                .build();
//        Action deposit3 = drive.actionBuilder(new Pose2d(23, -39, Math.toRadians(-45)))
//                .setTangent(9 * Math.PI/10)
//                .splineToLinearHeading(new Pose2d(3, -27.5, Math.toRadians(-90)), Math.PI/2)
//                .build();
        telemetry.addData("Status","starting");
        telemetry.update();
//        robot.initSubsystems();
        waitForStart();
        if (isStopRequested()) return;
        //                            robot.lift.update();
        //                             PoseKeeper.set(robot.drive.pose);
        Commands.runBlocking(
                new ParallelCommand(
                        new SequentialCommand(
                            // PRELOAD DEPOSIT

                            new ParallelCommand(
                                    preloadDrive
//                                        new InstantCommand(()->robot.climbWinch.setPower(1)),
//                                        new SequentialAction(
//                                                new InstantAction(robot::highRung)
//                                        )
                            ),
//                                new InstantAction(()->robot.climbWinch.setPower(0)),
//                                robot.outtakeSpecimen(true),
//                                new InstantAction(() -> robot.lift.runToPosition(810)),
                            //new SleepCommand(0.1),
//                                new InstantAction(() -> robot.intermediatePreset()),
//                               // SPIKE RIGHT
                            new ParallelCommand(
                                    spike1
                                    //robot.retractedIntakePreset(true)

                            )
//                                new InstantAction(()->robot.extension.runToPosition(225)),
//                                new SleepAction(0.4),
//                                new InstantAction(() -> robot.arm.runToPreset(Levels.INTAKE_INTERMEDIATE)),
//                                observation1,
//                                robot.claw.ejectOpsAuton(true),
//                                new InstantAction(()->robot.extension.runToPosition(190)),
//                                new InstantAction(() -> robot.arm.runToPreset(Levels.INTAKE)),
//                                spike2,
//                                new InstantAction(()->robot.extension.runToPosition(225)),
//                                new SleepAction(0.4),
//                                new ParallelAction(
//                                        new InstantAction(()->robot.extension.runToPosition(200)),
//                                        observation2)
//                                ,
//                                robot.claw.ejectOpsAuton(true),
//                                back,
//                                //cycle1
//                                new SleepAction(0.5),
//                                new ParallelAction(
//                                        new SequentialAction(new SleepAction(1),
//                                                new InstantAction(()->robot.extension.runToPosition(225))),
//                                        intake1
//                                ),
//                                new SleepAction(0.3),
//                                new InstantAction(robot::highRung),
//                                new SleepAction(0.1),
//                                deposit,
//                                robot.outtakeSpecimen(true),
//                                new InstantAction(() -> robot.lift.runToPosition(810)),
//                                new SleepAction(0.1),
//                                new InstantAction(() -> robot.intermediatePreset()),
//
//                                //cycle2
//                                new SleepAction(0.5),
//                                new ParallelAction(
//                                        intake2,
//                                        new SequentialAction(
//                                                robot.retractedIntakePreset(true),
//                                                new SleepAction(2),
//                                                new InstantAction(()->robot.extension.runToPosition(225))
//                                        )
//                                ),
//                                new SleepAction(0.3),
//                                new InstantAction(robot::highRung),
//                                new SleepAction(0.1),
//                                deposit2,
//                                robot.outtakeSpecimen(true),
//                                new InstantAction(() -> robot.lift.runToPosition(810)),
//                                new SleepAction(0.1),
//                                new InstantAction(() -> robot.intermediatePreset()),
//
//                                //cycle3
//                                new SleepAction(0.5),
//                                new ParallelAction(
//                                        intake3,
//                                        new SequentialAction(
//                                                robot.retractedIntakePreset(true),
//                                                new SleepAction(2),
//                                                new InstantAction(()->robot.extension.runToPosition(225))
//                                        )
//                                ),
//                                new SleepAction(0.3),
//                                new InstantAction(robot::highRung),
//                                new SleepAction(0.1),
//                                deposit3,
//                                robot.outtakeSpecimen(true),
//                                new InstantAction(() -> robot.lift.runToPosition(810)),
//                                new SleepAction(0.1),
//                                new InstantAction(() -> robot.intermediatePreset())

                    ),
                        new LoopCommand(drive::update, this::isStopRequested)
                )
        );
        // robot.cv.kill();
    }
}