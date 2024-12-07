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

@Autonomous(name = "SPECIMENAUTO", group = "Autonomous")
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
                .splineToLinearHeading(new Pose2d(0, -28.5, Math.toRadians(-90)), Math.toRadians(110))
                .waitSeconds(0.1)
                .build();
        Action spike1 = drive.actionBuilder(new Pose2d(0, -29, Math.toRadians(-90)))
                .setReversed(true)
                .setTangent(Math.toRadians(-17))
                .splineToLinearHeading(new Pose2d(38, -43, Math.toRadians(67)), Math.toRadians(67))
                .build();
        Action observation1 = drive.actionBuilder(new Pose2d(39.5, -43, Math.toRadians(67)))
                .splineToLinearHeading(new Pose2d(35, -44.5, Math.toRadians(-55)), Math.toRadians(0))
                .build();
        Action spike2 = drive.actionBuilder(new Pose2d(35, -44.5, Math.toRadians(-55)))
                .splineToLinearHeading(new Pose2d(51.5, -42, Math.toRadians(80)), Math.toRadians(70))
                .build();
        Action observation2 = drive.actionBuilder(new Pose2d(51.5, -42, Math.toRadians(80)))
                .splineToLinearHeading(new Pose2d(38, -44.5, Math.toRadians(-60)), Math.toRadians(0))
                .build();
        Action back = drive.actionBuilder(new Pose2d(38, -44.5, Math.toRadians(-60)))
                .setReversed(true)
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(21, -36, Math.toRadians(-45)), 9 * Math.PI/10)
                .build();
        Action intake1 = drive.actionBuilder(new Pose2d(21, -36, Math.toRadians(-45)))
                .setReversed(false)
                .lineToX(27)
                .build();
        Action deposit = drive.actionBuilder(new Pose2d(22, -37, Math.toRadians(-45)))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(5, -27, Math.toRadians(-90)), Math.PI/2)
                .build();
        Action intake2 = drive.actionBuilder(new Pose2d(5, -27, Math.toRadians(-90)))
                .setReversed(false)
                .setTangent(-4*Math.PI/10)
                .splineToLinearHeading(new Pose2d(25, -42,  Math.toRadians(-45)), 9*Math.PI/10)
                .build();
        Action deposit2 = drive.actionBuilder(new Pose2d(23, -39, Math.toRadians(-45)))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -27, Math.toRadians(-90)), Math.PI/2)
                .waitSeconds(0.5)
                .build();
        Action intake3 = drive.actionBuilder(new Pose2d(0, -27, Math.toRadians(-90)))
                .setReversed(false)
                .setTangent(-4*Math.PI/10)
                .splineToLinearHeading(new Pose2d(25, -42,  Math.toRadians(-45)), 9*Math.PI/10)
                .build();
        Action deposit3 = drive.actionBuilder(new Pose2d(23, -39, Math.toRadians(-45)))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(3, -27.5, Math.toRadians(-90)), Math.PI/2)
                .build();
        telemetry.addData("Status","starting");
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
                                        new InstantAction(()->robot.climbWinch.setPower(-1)),
                                        new SequentialAction(
                                            new InstantAction(robot::highRung)
                                        )
                                ),
                                robot.outtakeSpecimen(true),
                                new InstantAction(() -> robot.lift.runToPosition(810)),
                                new SleepAction(0.1),
                                new InstantAction(() -> robot.intermediatePreset()),
//                               // SPIKE RIGHT
                                new ParallelAction(
                                        new InstantAction(()->robot.climbWinch.setPower(0)),
                                        spike1,
                                        robot.retractedIntakePreset(true)

                                ),
                                new InstantAction(()->robot.extension.runToPosition(225)),
                                new SleepAction(0.4),
                                new InstantAction(() -> robot.arm.runToPreset(Levels.INTAKE_INTERMEDIATE)),
                                observation1,
                                robot.claw.ejectOpsAuton(true),
                                new InstantAction(()->robot.extension.runToPosition(190)),
                                new InstantAction(() -> robot.arm.runToPreset(Levels.INTAKE)),
                                spike2,
                                new InstantAction(()->robot.extension.runToPosition(225)),
                                new SleepAction(0.4),
                                new ParallelAction(
                                        new InstantAction(()->robot.extension.runToPosition(200)),
                                        observation2)
                                ,
                                robot.claw.ejectOpsAuton(true),
                                back
//                                //cycle1
//                                new SleepAction(0.5),
//                                new ParallelAction(
//                                    new SequentialAction(new SleepAction(1),
//                                    new InstantAction(()->robot.extension.runToPosition(225))),
//                                    intake1
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
                        new LoopAction(() -> {
                            robot.lift.update();
//                            PoseKeeper.set(robot.drive.pose);
                        }, this::isStopRequested)
                )
        );
       // robot.cv.kill();
    }
}