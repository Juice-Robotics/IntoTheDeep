package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseKeeper;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.misc.FullPose2d;

@Autonomous(name = "HPSideRedV1.1", group = "Autonomous")
public class HPSideRedV1_1 extends LinearOpMode {
//    Robot robot;
    KalmanDrive drive;
    CVMaster cv;
    Limelight3A limelight;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(-90));
        PoseKeeper.set(beginPose);
//        robot = new Robot(hardwareMap, true);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        cv = new CVMaster(limelight, null);
        drive = new KalmanDrive(hardwareMap, beginPose, limelight);

        Action auton = drive.actionBuilder(drive.pose)
                .setTangent(2.03444)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), Math.toRadians(110))

//                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(70)), Math.toRadians(-17))
//                .waitSeconds(0.5)
                //.splineToLinearHeading(new Pose2d(34, -40, Math.toRadians(-30)), Math.toRadians(-17))
                .waitSeconds(0.25)
                .setReversed(true)
                .setTangent(Math.toRadians(-17))
                .splineToLinearHeading(new Pose2d(34, -40, Math.toRadians(35)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(34, -40, Math.toRadians(-45)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(41, -40, Math.toRadians(35)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(41, -40, Math.toRadians(-45)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(43, -40, Math.toRadians(35)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(43, -40, Math.toRadians(-45)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(40, -40, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(17, -42, Math.toRadians(-65)), -Math.PI/10)
                .waitSeconds(0.5)

                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(17, -42, Math.toRadians(-65)), -Math.PI/10)
                .waitSeconds(0.5)

                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(17, -42, Math.toRadians(-65)), -Math.PI/10)
//                .waitSeconds(0.5)

                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
//                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(17, -42, Math.toRadians(-65)), -Math.PI/10)

                .build();
        telemetry.addData("is","starting");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(auton);
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                preloadDrive,
//                                new InstantAction(robot::preloadHighRung)
//                        ),
//                        new InstantAction(robot::outtakeSpecimen),
//                        robot.commands.locateTargetsCV(CVMaster.EOCVPipeline.RED_SAMPLE)
//                ));
//
//        Pose3D target = robot.cv.findOptimalTarget(robot.drive.pose);
//        FullPose2d robotCapturePose = robot.cv.calculateRobotFullPose(target, target.getPosition().x, robot.drive.pose.position.y);
//        telemetry.addData("TARGET POSE", target.toString());
//        telemetry.update();
//
//        Action intakeAdjustment = robot.drive.actionBuilder(robot.drive.pose)
//                .setTangent(Math.toRadians(110))
//                .splineToLinearHeading(robotCapturePose.getRobotPose(), Math.toRadians(110))
//                .build();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                intakeAdjustment,
//                                new InstantAction(() -> robot.intakePreset(robotCapturePose.intakeExtension))
//                        ),
//                        robot.commands.stopIntake(SampleColors.RED),
//                        new SleepAction(1),
//                        auton2
//                )
//        );
//        PoseKeeper.set(robot.drive.pose);
    }
}