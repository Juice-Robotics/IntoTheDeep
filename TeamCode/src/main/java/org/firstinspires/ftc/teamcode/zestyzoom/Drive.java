
package org.firstinspires.ftc.teamcode.zestyzoom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.zestyzoom.trajectory.TrajectoryBuilder;

public class Drive {
    private PIDFController controllerY;
    private PIDFController controllerX;
    private PIDController controllerHeading;

    public static double pY = 0.08, iY = 0, dY = 0.01;
    public static double pX = -0.08, iX = -0.1, dX = -0.01;
    public static double pH = -0.6, iH = 0, dH = 0.000, fH = -0.003;
    public static double fMult = 0.003;

    public static double targetY = 0;
    public static double targetX = 0;
    public static double targetH = 0;
    public static double ef = 0.5;
    public static double es = 0.5;
    public static double eh = 2;

    public Pose2D pose;

    private double lastHeading;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private GoBildaPinpoint pinpoint;
    public Telemetry telemetry;
    public Vector2d r;
    public Drive(HardwareMap hardwareMap, Pose2D startingPose, Telemetry telemetry) {
        controllerY = new PIDFController(pY, iY, dY, 0);
        controllerX = new PIDFController(pX, iX, dX, 0);
        controllerHeading = new PIDController(pH, iH , dH);

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
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startingPose.getY(DistanceUnit.INCH), startingPose.getX(DistanceUnit.INCH), AngleUnit.RADIANS, startingPose.getHeading(AngleUnit.RADIANS)));
        targetX = startingPose.getX(DistanceUnit.INCH);
        targetY = startingPose.getY(DistanceUnit.INCH);
        targetH = startingPose.getHeading(AngleUnit.RADIANS);
        pose = startingPose;
    }

    public void update() {
        pinpoint.update();
        pose = new Pose2D(DistanceUnit.INCH, pinpoint.getPosition().getY(DistanceUnit.INCH), pinpoint.getPosition().getX(DistanceUnit.INCH), AngleUnit.RADIANS, normalizeH(pinpoint.getPosition().getHeading(AngleUnit.RADIANS), lastHeading));
        double forward = controllerY.calculate(pose.getY(DistanceUnit.INCH), targetY);
        double strafe = controllerX.calculate(pose.getX(DistanceUnit.INCH), targetX);
        double heading = controllerHeading.calculate(normalizeH(pinpoint.getHeading(), lastHeading), targetH);
        heading += fH*(heading/Math.abs(heading));
        r = rotateVector(new Vector2d(strafe, forward), -pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
        setDrivePower(r.x, r.y, heading);

        telemetry.addData("X ", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Y ", pose.getX(DistanceUnit.INCH));
        telemetry.addData("H ", pose.getHeading(AngleUnit.RADIANS));

        telemetry.addData("forwardTarget ", targetY);
        telemetry.addData("strafeTarget ", targetX);
        telemetry.addData("headingTarget ", targetH);
        telemetry.addData("FError ", pose.getY(DistanceUnit.INCH) - targetY);
        telemetry.addData("SError ", pose.getX(DistanceUnit.INCH)- targetX);
        telemetry.addData("HError ", normalizeH(pose.getHeading(AngleUnit.RADIANS), lastHeading)-targetH);
        telemetry.update();
        lastHeading = pinpoint.getHeading();
        controllerY.setF(fMult*r.x * Math.sin(pose.getHeading(AngleUnit.RADIANS)));
        controllerX.setF(fMult*r.x * Math.cos(pose.getHeading(AngleUnit.RADIANS)));

    }

    public TrajectoryBuilder trajectoryBuilder(Pose2D startingPose) {
        return new TrajectoryBuilder(this, startingPose, telemetry);
    }

    public void setTarget(Pose2D pose) {
        targetY = pose.getY(DistanceUnit.INCH);
        targetX = pose.getX(DistanceUnit.INCH);
        targetH = pose.getHeading(AngleUnit.RADIANS);
    }

    public boolean isCloseTo(Pose2D currentPose, Pose2D targetPose) {
        double hd = Math.abs(currentPose.getHeading(AngleUnit.DEGREES) - targetPose.getHeading(AngleUnit.DEGREES));

        return Math.abs(currentPose.getX(DistanceUnit.INCH) - targetPose.getX(DistanceUnit.INCH)) < es && Math.abs(currentPose.getY(DistanceUnit.INCH) - targetPose.getY(DistanceUnit.INCH)) < ef &&  (hd< eh || hd-360 < eh);
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
