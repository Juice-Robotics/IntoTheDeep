package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpoint;

@Config
@TeleOp
//@Disabled
public class PIDtoPoint extends OpMode {
    private PIDFController controllerY;
    private PIDFController controllerX;
    private PIDFController controllerHeading;

    public static double pY = 0.08, iY = 0, dY = 0.01, fY = 0;
    public static double pX = -0.06, iX = -0.1, dX = -0.002, fX = 0;
    public static double pH = -0.7, iH = 0, dH = 0.000, fH = 0;
    public static double targetY = 0;
    public static double targetX = 0;
    public static double targetH = 0;
    //private final double angleSlides = 60;
    private double lastHeading;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private GoBildaPinpoint pinpoint;
    //public Robot robot;
    double oldTime = 0;
    public Pose2D pose;

    @Override
    public void init() {
        controllerY = new PIDFController(pY, iY, dY, fY);
        controllerX = new PIDFController(pX, iX, dX, fX);
        controllerHeading = new PIDFController(pH, iH , dH, fH);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        pinpoint = hardwareMap.get(GoBildaPinpoint.class, "pinpoint");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // RR localizer note: don't love this conversion (change driver?)
//        pinpoint.setOffsets(DistanceUnit.MM.fromInches(0), DistanceUnit.MM.fromInches(5.125));
//        pinpoint.setEncoderResolution(GoBildaPinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(GoBildaPinpoint.EncoderDirection.FORWARD, GoBildaPinpoint.EncoderDirection.FORWARD);
//        pinpoint.resetPosAndIMU();
        pinpoint.setOffsets(DistanceUnit.MM.fromInches(-3), DistanceUnit.MM.fromInches(0.5));
        pinpoint.setEncoderResolution(GoBildaPinpoint.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpoint.EncoderDirection.REVERSED, GoBildaPinpoint.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

    }
    @Override
    public void loop(){
        controllerY.setPIDF(pY, iY, dY, fY);
        controllerX.setPIDF(pX, iX, dX, fX);
        controllerHeading.setPIDF(pH, iH , dH, fH);

        //robot.updatePinpoint();
        pinpoint.update();
//        telemetry.addData("X ", robot.pinpoint.getPosition().getY(DistanceUnit.INCH));
//        telemetry.addData("Y ", robot.pinpoint.getPosition().getX(DistanceUnit.INCH));
//        telemetry.addData("H ", robot.pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
//        double forward = controllerForward.calculate( robot.pinpoint.getPosition().getX(DistanceUnit.INCH), targetF);
//        double strafe = controllerStrafe.calculate( robot.pinpoint.getPosition().getY(DistanceUnit.INCH), targetS);
//        double heading = controllerHeading.calculate(normalizeH(robot.pinpoint.getPosition().getHeading(AngleUnit.RADIANS), lastHeading), targetH);
//        Vector2d r = rotateVector(new Vector2d(strafe, forward), -heading);
//        robot.setDrivePower(r.x, r.y, heading);
        pose = new Pose2D(DistanceUnit.INCH, pinpoint.getPosition().getY(DistanceUnit.INCH), pinpoint.getPosition().getX(DistanceUnit.INCH), AngleUnit.RADIANS, normalizeH(pinpoint.getPosition().getHeading(AngleUnit.RADIANS), lastHeading));
        double yPow = controllerY.calculate(pose.getY(DistanceUnit.INCH), targetY);
        double xPow = controllerX.calculate(pose.getX(DistanceUnit.INCH), targetX);
        double heading = controllerHeading.calculate(normalizeH(pinpoint.getHeading(), lastHeading), targetH);
        Vector2d r = rotateVector(new Vector2d(xPow, yPow), -pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
        setDrivePower(r.x, r.y, heading);
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        telemetry.addData("X ", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Y ", pose.getX(DistanceUnit.INCH));
        telemetry.addData("H ", normalizeH(pose.getHeading(AngleUnit.RADIANS), lastHeading));

        telemetry.addData("forwardTarget ", targetY);
        telemetry.addData("strafeTarget ", targetX);
        telemetry.addData("headingTarget ", targetH);
        telemetry.addData("FError ", pose.getY(DistanceUnit.INCH) - targetY);
        telemetry.addData("SError ", pose.getX(DistanceUnit.INCH)- targetX);
        telemetry.addData("HError ", normalizeH(pose.getHeading(AngleUnit.RADIANS), lastHeading)-targetH);
        telemetry.update();
        lastHeading = pinpoint.getHeading();
    }
    public double normalizeH(double heading, double lastHeading){
        if ((heading < 0) && (lastHeading > 1)){
            return heading + 2*Math.PI;
        }
        return heading;
    }
    public Vector2d rotateVector(Vector2d vector, double angle){
        return new Vector2d(Math.cos(angle)*vector.x - Math.sin(angle)*vector.y, Math.sin(angle)*vector.x + Math.cos(angle)*vector.y);
    }
    public void setDrivePower(double x, double y, double rx) {
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
