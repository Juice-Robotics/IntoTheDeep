package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.drive.trajectory.TrajectoryBuilder;

import java.util.Locale;

public class Drive {
    private final PIDController xControl, yControl, headingControl;

    private final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    private final GoBildaPinpoint pinpoint;

    public static double pX = 0, iX = 0, dX = 0;
    public static double pY = 0, iY = 0, dY = 0;
    public static double pH = 0, iH = 0, dH = 0;;

    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetH = 0;
    public static double translationalError = 1;
    public static double headingError = 3;

    public Pose2D pose;

    private double lastHeading;


    public Telemetry telemetry;

    public VoltageSensor voltageSensor;

    public ElapsedTime timer;

//X IS FORWARD, Y IS STRAFE, H IS HEADING

    public Drive(HardwareMap hardwareMap, Pose2D startingPose, Telemetry telemetry) {
        xControl = new PIDController(pX, iX , dX);
        yControl = new PIDController(pY, iY , dY);
        headingControl = new PIDController(pH, iH , dH);

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");

        pinpoint = hardwareMap.get(GoBildaPinpoint.class, "pinpoint");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        pinpoint.setOffsets(DistanceUnit.MM.fromInches(0), DistanceUnit.MM.fromInches(5.13217677165));
        pinpoint.setEncoderResolution(GoBildaPinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpoint.EncoderDirection.FORWARD, GoBildaPinpoint.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pinpoint.setPosition(pose);
        pose = startingPose;

        targetX = startingPose.getX(DistanceUnit.INCH);
        targetY = startingPose.getY(DistanceUnit.INCH);
        targetH = startingPose.getHeading(AngleUnit.RADIANS);

        timer = new ElapsedTime();
    }

    public void update() {
        Pose2D pos = pinpoint.getPosition();

        double x = xControl.calculate(pos.getX(DistanceUnit.INCH), targetX);
        double y = yControl.calculate(pos.getY(DistanceUnit.INCH),targetY);
        double heading = headingControl.calculate(normalizeHeading(pinpoint.getPosition().getHeading(AngleUnit.RADIANS)), targetH);

        double x_rotated = x * Math.cos(heading) - y * Math.sin(heading);
        double y_rotated = x * Math.sin(heading) + y * Math.cos(heading);

        setDrivePower(x_rotated, y_rotated, heading);

        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        telemetry.update();
        pinpoint.update();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2D startingPose) {
        return new TrajectoryBuilder(this, startingPose, telemetry);
    }

    public void setTarget(Pose2D pose) {
        timer.reset();
        targetX = pose.getX(DistanceUnit.INCH);
        targetY = pose.getY(DistanceUnit.INCH);
        targetH = pose.getHeading(AngleUnit.RADIANS);
    }

    public boolean isCloseTo(Pose2D currentPose, Pose2D targetPose) {
        return Math.abs(currentPose.getX(DistanceUnit.INCH) - targetPose.getX(DistanceUnit.INCH)) < translationalError && Math.abs(currentPose.getY(DistanceUnit.INCH) - targetPose.getY(DistanceUnit.INCH)) < translationalError && Math.abs(currentPose.getHeading(AngleUnit.DEGREES) - targetPose.getHeading(AngleUnit.DEGREES)) < headingError;
    }

    private double normalizeHeading(double heading){
        while (heading > Math.PI) {
            heading -= 2 * Math.PI;
        }
        while (heading < -Math.PI) {
            heading += 2 * Math.PI;
        }
        return heading;
    }

    private void setDrivePower(double x_rotated, double y_rotated, double rx) {
        double powerFrontLeft = x_rotated + y_rotated + rx;
        double powerFrontRight = x_rotated - y_rotated - rx;
        double powerBackLeft = x_rotated - y_rotated + rx;
        double powerBackRight = x_rotated + y_rotated - rx;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }

        leftFront.setPower((float) powerFrontLeft);
        rightFront.setPower((float) powerFrontRight);
        leftBack.setPower((float) powerBackLeft);
        rightBack.setPower((float) powerBackRight);
    }
}
