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

@Autonomous(name = "a2", group = "Autonomous")
public class AutonTestPush extends LinearOpMode {
    public void runOpMode() {
        //Pose2D beginPose = new Pose2D(DistanceUnit.INCH, -12.0, -60.0, AngleUnit.RADIANS, Math.PI);
        Pose2D beginPose = new Pose2D(DistanceUnit.INCH, -12.0, -60.0, AngleUnit.RADIANS, 0);
        Drive drive = new Drive(hardwareMap, beginPose, telemetry);

        TrajectoryCommand auton = drive.trajectoryBuilder(drive.pose)
                //drive to sub (preload)
                .addPoint(new Pose2d(0, -29, Math.toRadians(-90)))
                .waitSeconds(0.25)

                //drive in front of left spike
                .addPoint(new Pose2d(0, -42, Math.toRadians(-90)))
                .addPoint(new Pose2d(36, -42, Math.toRadians(-90)))

                .addPoint(new Pose2d(36, -36, Math.toRadians(-90)))
                .addPoint(new Pose2d(36, -12, Math.toRadians(-90)))
                .addPoint(new Pose2d(40, -12, Math.toRadians(-90)))
                .waitSeconds(0.1)

                //drive to observation zone part1
                .addPoint(new Pose2d(40, -52, Math.toRadians(-90)))
                .waitSeconds(0.1)

                //drive in front of middle spike
                .addPoint(new Pose2d(40, -12, Math.toRadians(-90)))
                .addPoint(new Pose2d(52, -12, Math.toRadians(-90)))
                .waitSeconds(0.1)

                //drive to observation zone part 2
                .addPoint(new Pose2d(52, -52, Math.toRadians(-90)))
                .waitSeconds(0.1)

                //drive in front of right spike
                .addPoint(new Pose2d(52, -12, Math.toRadians(-90)))
                .addPoint(new Pose2d(60, -12, Math.toRadians(-90)))
                .waitSeconds(0.1)

                //drive to observation zone part 3
                .addPoint(new Pose2d(60, -52, Math.toRadians(-90)))
                .waitSeconds(0.1)

                .build();

        LoopCommand cmdLoop = new LoopCommand(drive::update, this::isStopRequested);
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
                                auton,
                                new InstantCommand(cmdLoop::kill)
//                        new ParallelCommand(
//                                preloadDrive
////                                        new InstantCommand(()->robot.climbWinch.setPower(1)),
////                                        new SequentialAction(
////                                                new InstantAction(robot::highRung)
////                                        )
//                        ),
////                                new InstantAction(()->robot.climbWinch.setPower(0)),
////                                robot.outtakeSpecimen(true),
////                                new InstantAction(() -> robot.lift.runToPosition(810)),
//                        //new SleepCommand(0.1),
////                                new InstantAction(() -> robot.intermediatePreset()),
////                               // SPIKE RIGHT
//                        new ParallelCommand(
//                                spike1
//                                //robot.retractedIntakePreset(true)
//
//                        )
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
                        cmdLoop
                )
        );
        // robot.cv.kill();
    }
}