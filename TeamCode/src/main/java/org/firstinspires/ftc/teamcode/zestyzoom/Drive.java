package org.firstinspires.ftc.teamcode.zestyzoom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.control.PIDFController;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.zestyzoom.trajectory.TrajectoryBuilder;

public class Drive {
    private PIDFController controllerForward;
    private PIDFController controllerStrafe;
    private PIDFController controllerHeading;

    public static double pF = 0.09, iF = 0, dF = 18, fF = 0.03;
    public static double pS = -0.06, iS = 0, dS = -5, fS = 0.03;
    public static double pH = -0.6, iH = 0, dH = 10;

    public static double targetF = 0;
    public static double targetS = 0;
    public static double targetH = 0;
    public static double e = 2;
    public static double eh = 3;

    public Pose2D pose;

    private double lastHeading;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private GoBildaPinpoint pinpoint;
    public Telemetry telemetry;

    public Drive(HardwareMap hardwareMap, Pose2D startingPose, Telemetry telemetry) {
        controllerForward = new PIDFController(pF, iF , dF, fF, false);
        controllerStrafe = new PIDFController(pS, iS , dS, fS, false);
        controllerHeading = new PIDFController(pH, iH , dH, 0, false);

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        pinpoint = hardwareMap.get(GoBildaPinpoint.class, "pinpoint");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint.setOffsets(DistanceUnit.MM.fromInches(-3), DistanceUnit.MM.fromInches(0.5));
        pinpoint.setEncoderResolution(GoBildaPinpoint.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpoint.EncoderDirection.REVERSED, GoBildaPinpoint.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startingPose.getX(DistanceUnit.INCH), startingPose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, startingPose.getHeading(AngleUnit.RADIANS)));
        targetS = startingPose.getX(DistanceUnit.INCH);
        targetF = startingPose.getY(DistanceUnit.INCH);
        targetH = startingPose.getHeading(AngleUnit.RADIANS);
        pose = startingPose;

    }

    public void update() {
        pinpoint.update();
        pose = new Pose2D(DistanceUnit.INCH, pinpoint.getPosition().getX(DistanceUnit.INCH), pinpoint.getPosition().getY(DistanceUnit.INCH), AngleUnit.RADIANS, normalizeH(pinpoint.getPosition().getHeading(AngleUnit.RADIANS), lastHeading));
        double forward = controllerForward.update(pose.getX(DistanceUnit.INCH), targetF);
        double strafe = controllerStrafe.update(pose.getY(DistanceUnit.INCH), targetS);
        double heading = controllerHeading.update(normalizeH(pinpoint.getHeading(), lastHeading), targetH);
        Vector2d r = rotateVector(new Vector2d(strafe, forward), -pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
        setDrivePower(r.x, r.y, heading);

        telemetry.addData("X ", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y ", pose.getY(DistanceUnit.INCH));
        telemetry.addData("H ", normalizeH(pose.getHeading(AngleUnit.RADIANS), lastHeading));

        telemetry.addData("forwardTarget ", targetF);
        telemetry.addData("strafeTarget ", targetS);
        telemetry.addData("headingTarget ", targetH);
        telemetry.addData("FError ", pose.getY(DistanceUnit.INCH) - targetF);
        telemetry.addData("SError ", pose.getX(DistanceUnit.INCH)-targetS);
        telemetry.addData("HError ", normalizeH(pose.getHeading(AngleUnit.RADIANS), lastHeading)-targetH);
        telemetry.update();
        lastHeading = pinpoint.getHeading();

    }

    public TrajectoryBuilder trajectoryBuilder(Pose2D startingPose) {
        return new TrajectoryBuilder(this, startingPose, telemetry);
    }

    public void setTarget(Pose2D pose) {
        targetF = pose.getX(DistanceUnit.INCH);
        targetS = pose.getY(DistanceUnit.INCH);
        targetH = pose.getHeading(AngleUnit.RADIANS);
    }

    public boolean isCloseTo(Pose2D currentPose, Pose2D targetPose) {
        return Math.abs(currentPose.getX(DistanceUnit.INCH) - targetPose.getX(DistanceUnit.INCH)) < e && Math.abs(currentPose.getY(DistanceUnit.INCH) - targetPose.getY(DistanceUnit.INCH)) < e && Math.abs(currentPose.getHeading(AngleUnit.DEGREES) - targetPose.getHeading(AngleUnit.DEGREES)) < eh;
    }

    private double normalizeH(double heading, double lastHeading){
        if ((heading < 0) && (lastHeading > 1)){
            return heading + 2*Math.PI;
        }else if ((heading > 0) && (lastHeading < -1)){
            return heading - 2*Math.PI;
        }
        return heading;
    }

    private Vector2d rotateVector(Vector2d vector, double angle){
        return new Vector2d(Math.cos(angle)*vector.x - Math.sin(angle)*vector.y, Math.sin(angle)*vector.x + Math.cos(angle)*vector.y);
    }

    private void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

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
        leftBack.setPower(-(float) powerBackLeft);
        rightBack.setPower(-(float) powerBackRight);
    }
}
