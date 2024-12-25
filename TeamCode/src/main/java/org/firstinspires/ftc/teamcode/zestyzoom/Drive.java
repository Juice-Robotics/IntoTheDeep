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
    private PIDFController controllerForward;
    private PIDFController controllerStrafe;
    private PIDController controllerHeading;

    public static double pF = 0.08, iF = 0, dF = 0.01, fF = 0;
    public static double pS = -0.06, iS = -0.1, dS = -0.002, fS = 0;;
    public static double pH = -0.7, iH = 0, dH = 0.000;;

    public static double targetF = 0;
    public static double targetS = 0;
    public static double targetH = 0;
    public static double e = 3;
    public static double eh = 20;

    public Pose2D pose;

    private double lastHeading;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private GoBildaPinpoint pinpoint;
    public Telemetry telemetry;

    public Drive(HardwareMap hardwareMap, Pose2D startingPose, Telemetry telemetry) {
        controllerForward = new PIDFController(pF, iF , dF, fF);
        controllerStrafe = new PIDFController(pS, iS , dS, fS);
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
        targetS = startingPose.getX(DistanceUnit.INCH);
        targetF = startingPose.getY(DistanceUnit.INCH);
        targetH = startingPose.getHeading(AngleUnit.RADIANS);
        pose = startingPose;

    }

    public void update() {
        pinpoint.update();
        double forward = controllerForward.calculate(pinpoint.getPosition().getX(DistanceUnit.INCH), targetF);
        double strafe = controllerStrafe.calculate(pinpoint.getPosition().getY(DistanceUnit.INCH), targetS);
        double heading = controllerHeading.calculate(normalizeH(pinpoint.getPosition().getHeading(AngleUnit.RADIANS), lastHeading), targetH);
        Vector2d r = rotateVector(new Vector2d(strafe, forward), -pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
        setDrivePower(r.x, r.y, heading);

        telemetry.addData("X ", pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Y ", pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("H ", normalizeH(pinpoint.getPosition().getHeading(AngleUnit.RADIANS), lastHeading));
        telemetry.addData("forward ", forward);
        telemetry.addData("strafe ", strafe);
        telemetry.addData("heading ", heading);
        telemetry.addData("rforward ", r.y);
        telemetry.addData("rstrafe ", r.x);

        telemetry.addData("forwardTarget ", targetF);
        telemetry.addData("strafeTarget ", targetS);
        telemetry.addData("headingTarget ", targetH);
        telemetry.update();
        lastHeading = pinpoint.getHeading();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2D startingPose) {
        return new TrajectoryBuilder(this, startingPose);
    }

    public void setTarget(Pose2D pose) {
        targetF = pose.getY(DistanceUnit.INCH);
        targetS = pose.getX(DistanceUnit.INCH);
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
