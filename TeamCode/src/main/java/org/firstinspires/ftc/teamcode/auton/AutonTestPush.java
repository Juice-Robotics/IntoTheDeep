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

        double HPDeposit = -53;
        double spikeBack = -12;
        double waits = 0.2;
        double intakeWait = 0.3;

        TrajectoryCommand preload = drive.trajectoryBuilder(beginPose)
                .lineToY(-30)
                .build();

        TrajectoryCommand allSpikes = drive.trajectoryBuilder(preload.endPose())
                .addPoint(new Pose2d(18, -42, Math.toRadians(-90)))
                .addPoint(new Pose2d(36, spikeBack, Math.toRadians(-90)))

                .addPoint(new Pose2d(46, spikeBack, Math.toRadians(-90)))

                .lineToY(HPDeposit)

                .addPoint(new Pose2d(46, spikeBack, Math.toRadians(-90)))
                .addPoint(new Pose2d(55, spikeBack, Math.toRadians(-90)))

                .lineToY(HPDeposit)

                .addPoint(new Pose2d(55, spikeBack, Math.toRadians(-90)))
                .addPoint(new Pose2d(61, spikeBack, Math.toRadians(-90)))

                .lineToY(HPDeposit)
                .build();

        TrajectoryCommand intakeSpec2 = drive.trajectoryBuilder(allSpikes.endPose())
                .addPoint(new Pose2d(19, -48, Math.toRadians(-45)))
                .build();

        TrajectoryCommand depositSpec2 = drive.trajectoryBuilder(intakeSpec2.endPose())
                .addPoint(new Pose2d(3, -30, Math.toRadians(-92)))
                .build();

        TrajectoryCommand intakeSpec3 = drive.trajectoryBuilder(depositSpec2.endPose())
                .addPoint(new Pose2d(19, -48, Math.toRadians(-45)))
                .build();

        TrajectoryCommand depositSpec3 = drive.trajectoryBuilder(intakeSpec3.endPose())
                .addPoint(new Pose2d(0, -30, Math.toRadians(-92)))
                .build();

        TrajectoryCommand intakeSpec4 = drive.trajectoryBuilder(depositSpec3.endPose())
                .addPoint(new Pose2d(19, -48, Math.toRadians(-45)))
                .build();

        TrajectoryCommand depositSpec4 = drive.trajectoryBuilder(intakeSpec4.endPose())
                .addPoint(new Pose2d(-3, -30, Math.toRadians(-92)))
                .build();

        TrajectoryCommand intakeSpec5 = drive.trajectoryBuilder(depositSpec4.endPose())
                .addPoint(new Pose2d(19, -48, Math.toRadians(-45)))
                .build();

        TrajectoryCommand depositSpec5 = drive.trajectoryBuilder(intakeSpec5.endPose())
                .addPoint(new Pose2d(-6, -30, Math.toRadians(-92)))
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
                                preload,
                                allSpikes,
                                intakeSpec2,
                                depositSpec2,
                                intakeSpec3,
                                depositSpec3,
                                intakeSpec4,
                                depositSpec4,
                                intakeSpec5,
                                depositSpec5,
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