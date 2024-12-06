package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LoopAction;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseKeeper;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.misc.FullPose2d;

@Autonomous(name = "HPSideRedV1.1", group = "Autonomous")
public class HPSideRedV1_1 extends LinearOpMode {
    Robot robot;
    KalmanDrive drive;
    CVMaster cv;
    Limelight3A limelight;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(-90));
        PoseKeeper.set(beginPose);
        robot = new Robot(hardwareMap, true);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        cv = new CVMaster(limelight, null);
        drive = new KalmanDrive(hardwareMap, beginPose, limelight);

        Action preloadDrive = drive.actionBuilder(drive.pose)
                .setTangent(2.03444)
                .splineToLinearHeading(new Pose2d(0, -29, Math.toRadians(-90)), Math.toRadians(110))
                .waitSeconds(0.1)
                .build();
        Action spike1 = drive.actionBuilder(new Pose2d(0, -29, Math.toRadians(-90)))
                .setReversed(true)
                .setTangent(Math.toRadians(-17))
                .splineToLinearHeading(new Pose2d(38.5, -42.5, Math.toRadians(67)), Math.toRadians(67))
                .waitSeconds(0.5)
                .build();
        Action observation1 = drive.actionBuilder(new Pose2d(39.5, -43, Math.toRadians(67)))
                .splineToLinearHeading(new Pose2d(35, -44.5, Math.toRadians(-55)), Math.toRadians(0))
                .waitSeconds(0.5)
                .build();
        Action spike2 = drive.actionBuilder(new Pose2d(35, -44.5, Math.toRadians(-55)))
                .splineToLinearHeading(new Pose2d(51.5, -42, Math.toRadians(80)), Math.toRadians(70))
                .waitSeconds(0.5)
                .build();
        Action observation2 = drive.actionBuilder(new Pose2d(51.5, -42, Math.toRadians(80)))
                .splineToLinearHeading(new Pose2d(38, -44.5, Math.toRadians(-60)), Math.toRadians(0))
                .waitSeconds(0.5)
                .build();
        Action back = drive.actionBuilder(new Pose2d(38, -44.5, Math.toRadians(-60)))
                .setReversed(true)
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(20, -35, Math.toRadians(-45)), 9 * Math.PI/10)
                .build();
        Action intake1 = drive.actionBuilder(new Pose2d(20, -35, Math.toRadians(-45)))
                .setReversed(false)
                .lineToX(26.5)
//                .setTangent(9 * Math.PI/10)
//                .splineToLinearHeading(new Pose2d(38, -44.5, Math.toRadians(-60)), 9 * Math.PI/10)
                .build();
        Action back1 = drive.actionBuilder(new Pose2d(22, -37, Math.toRadians(-45)))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(-6, -27, Math.toRadians(-90)), Math.PI/2)
                .build();
        Action intake2 = drive.actionBuilder(new Pose2d(-6, -27, Math.toRadians(-90)))
                .setReversed(false)
                .setTangent(-4*Math.PI/10)
                .splineToLinearHeading(new Pose2d(23, -39,  Math.toRadians(-45)), 9*Math.PI/10)
//                .setTangent(9 * Math.PI/10)
//                .splineToLinearHeading(new Pose2d(38, -44.5, Math.toRadians(-60)), 9 * Math.PI/10)
                .build();
        Action back2 = drive.actionBuilder(new Pose2d(23, -39, Math.toRadians(-45)))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -27, Math.toRadians(-90)), Math.PI/2)
                .waitSeconds(0.5)
                .build();
        Action intake3 = drive.actionBuilder(new Pose2d(0, -27, Math.toRadians(-90)))
                .setReversed(false)
                .setTangent(-4*Math.PI/10)
                .splineToLinearHeading(new Pose2d(23, -39,  Math.toRadians(-45)), 9*Math.PI/10)
//                .setTangent(9 * Math.PI/10)
//                .splineToLinearHeading(new Pose2d(38, -44.5, Math.toRadians(-60)), 9 * Math.PI/10)
                .build();
        Action back3 = drive.actionBuilder(new Pose2d(23, -39, Math.toRadians(-45)))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(5, -27.5, Math.toRadians(-90)), Math.PI/2)
                .waitSeconds(0.5)
                .build();
        telemetry.addData("is","starting");
        telemetry.update();
        robot.initSubsystems();
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // PRELOAD DEPOSIT
                                new ParallelAction(
                                        preloadDrive,
                                        new SequentialAction(
//                                            new SleepAction(1),
                                            new InstantAction(robot::highRung)
                                        )
                                ),
//                                new SleepAction(0.5),
                                robot.outtakeSpecimen(true),
                                new InstantAction(() -> robot.lift.runToPosition(810)),
                                new SleepAction(0.1),
                                new InstantAction(() -> robot.intermediatePreset()),
//                                // SPIKE RIGHT
                                new ParallelAction(
                                        spike1,
                                        robot.retractedIntakePreset(true)

                                ),
                                new InstantAction(()->robot.extension.runToPosition(225)),
                                new SleepAction(1.4),
                                new InstantAction(() -> robot.arm.runToPreset(Levels.INTAKE_INTERMEDIATE)),
//                                robot.commands.stopIntake(SampleColors.RED),
                                observation1,
                                robot.claw.ejectOpsAuton(true),
                                new InstantAction(()->robot.extension.runToPosition(190)),
                                new SleepAction(0.3),
                                new InstantAction(() -> robot.arm.runToPreset(Levels.INTAKE)),
                                spike2,
                                new InstantAction(()->robot.extension.runToPosition(225)),
                                new SleepAction(1.4),
                                new ParallelAction(
                                        new InstantAction(()->robot.extension.runToPosition(200)),
                                        observation2)
                                ,
                                robot.claw.ejectOpsAuton(true),
                                back,
                                //cycle1
                                new SleepAction(0.9),
                                new ParallelAction(
                                    new SequentialAction(new SleepAction(0.5),
                                    new InstantAction(()->robot.extension.runToPosition(225))),
                                    intake1
                                ),
                                new SleepAction(1.4),
                                new ParallelAction(
                                        back1,
                                        new InstantAction(robot::highRung)
//                                        new SequentialAction(
//                                            new SleepAction(0.5),
//
//                                        )
                                ),
                                robot.outtakeSpecimen(true),
                                new InstantAction(() -> robot.lift.runToPosition(810)),
                                new SleepAction(0.1),
                                new InstantAction(() -> robot.intermediatePreset()),

                                //cycle2
                                new SleepAction(0.5),
                                new ParallelAction(
                                        intake2,

                                        new SequentialAction(
                                                new SleepAction(0.1),
                                                robot.retractedIntakePreset(true),
                                                new SleepAction(0.8),
                                                new InstantAction(()->robot.extension.runToPosition(225))
                                        )
                                ),
                                new SleepAction(1.2),
                                new ParallelAction(
                                        back2,
                                        new InstantAction(robot::highRung)
                //                                        new SequentialAction(
                //                                            new SleepAction(0.5),
                //
                //                                        )
                                ),
                                robot.outtakeSpecimen(true),
                                new InstantAction(() -> robot.lift.runToPosition(810)),
                                new SleepAction(0.1),
                                new InstantAction(() -> robot.intermediatePreset()),

                                //cycle3
                                new SleepAction(0.5),
                                new ParallelAction(
                                        intake3,

                                        new SequentialAction(
                                                new SleepAction(0.1),
                                                robot.retractedIntakePreset(true),
                                                new SleepAction(0.75),
                                                new InstantAction(()->robot.extension.runToPosition(225))
                                        )
                                ),
                                new SleepAction(1.2),
                                new ParallelAction(
                                        back3,
                                        new InstantAction(robot::highRung)
                                        //                                        new SequentialAction(
                                        //                                            new SleepAction(0.5),
                                        //
                                        //                                        )
                                ),
                                robot.outtakeSpecimen(true),
                                new InstantAction(() -> robot.lift.runToPosition(810)),
                                new SleepAction(0.1),
                                new InstantAction(() -> robot.intermediatePreset())


//
//                                // SPIKE CENTER
//                                new ParallelAction(
//                                        driveToSpikeC,
//                                        robot.intakePreset(50, true)
//                                ),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToObC,
//                                        new InstantAction(robot::preloadDropPreset)
//                                ),
//                                robot.outtakeSample(true),
//
//                                //SPIKE LEFT
//                                new ParallelAction(
//                                        driveToSpikeL,
//                                        robot.intakePreset(50, true)
//                                ),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToObL,
//                                        new InstantAction(robot::preloadDropPreset)
//                                ),
//                                robot.outtakeSample(true),
//
//                                // **CYCLE #1**
//                                driveToObservation1,
//                                new InstantAction(() -> robot.intakePreset(50)),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToChamber1,
//                                        new SequentialAction(
//                                                new SleepAction(0),
//                                                new InstantAction(robot::highRung)
//                                        )
//                                ),
//                                robot.smartOuttake(true),
//
//                                // **CYCLE #2**
//                                new ParallelAction(
//                                        driveToObservation2,
//                                        new SequentialAction(
//                                                new SleepAction(0.1),
//                                                robot.intakePreset(50, true)
//                                        )
//                                ),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToChamber2,
//                                        new SequentialAction(
//                                                new SleepAction(0),
//                                                new InstantAction(robot::highRung)
//                                        )
//                                ),
//                                robot.smartOuttake(true),
//
//                                // **CYCLE #3**
//                                new ParallelAction(
//                                        driveToObservation3,
//                                        new SequentialAction(
//                                                new SleepAction(0.1),
//                                                robot.intakePreset(50, true)
//                                        )
//                                ),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToChamber3,
//                                        new SequentialAction(
//                                                new SleepAction(0),
//                                                new InstantAction(robot::highRung)
//                                        )
//                                ),
//                                robot.smartOuttake(true),
//
//                                // **PARK**
//                                new ParallelAction(
//                                        park,
//                                        new SequentialAction(
//                                                new InstantAction(robot::autonObParkPreset)
//                                        )
//                                )
                        ),
                        new LoopAction(() -> {
                            robot.lift.update();
//                            PoseKeeper.set(robot.drive.pose);
                        }, this::isStopRequested)
                )
        );
       // robot.cv.kill();
    }
}