package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseKeeper;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;

@Autonomous(name = "HPSideRed", group = "Autonomous")
public class HPSideRedStatic extends LinearOpMode {
    KalmanDrive drive;
    CVMaster cv;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(90));
        cv = new CVMaster(hardwareMap.get(Limelight3A.class, "limelight"), hardwareMap.get(WebcamName.class, "Webcam 1"));
        cv.start();
        cv.setLLPipeline(CVMaster.LLPipeline.APRILTAGS);

        drive = new KalmanDrive(hardwareMap, beginPose, cv.limelight);

        Action auton = drive.actionBuilder(drive.pose)
//                .turn(Math.toRadians(180))

                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(110))
                .waitSeconds(0.5)

                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(58, -50, Math.toRadians(90)), -Math.PI/10)
                .waitSeconds(0.5)

                .turn(Math.toRadians(20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(1)

                // gonna see me cycling
                .turn(Math.toRadians(200))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(-90)), -Math.PI/10)
                .waitSeconds(0.5)

                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)

                .build();
        telemetry.addData("is","starting");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        auton
                ));
        PoseKeeper.set(drive.pose);
    }
}