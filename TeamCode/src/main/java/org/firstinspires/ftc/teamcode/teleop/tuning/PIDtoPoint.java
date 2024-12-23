package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpoint;

@Config
@TeleOp
//@Disabled
public class PIDtoPoint extends OpMode {
    private PIDController controllerForward;
    private PIDController controllerStrafe;
    private PIDController controllerHeading;

    public static double pF = 0.09, iF = 0, dF = 0.01;
    public static double pS = -0.1, iS = -0.12, dS = -0.002;
    public static double pH = -0.7, iH = 0, dH = 0.000;

    public static double targetF = 0;
    public static double targetS = 0;
    public static double targetH = 0;
    private final double angleSlides = 60;
    private double lastHeading;

//    private DcMotorEx leftFront;
//    private DcMotorEx rightFront;
//    private DcMotorEx leftBack;
//    private DcMotorEx rightBack;
//    private GoBildaPinpoint pinpoint;
    public Robot robot;

    @Override
    public void init() {
        controllerForward = new PIDController(pF, iF , dF);
        controllerStrafe = new PIDController(pS, iS , dS);
        controllerHeading = new PIDController(pH, iH , dH);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, true);
        robot.initPinpoint();

    }
    @Override
    public void loop(){
        controllerForward.setPID(pF, iF , dF);
        controllerStrafe.setPID(pS, iS , dS);
        controllerHeading.setPID(pH, iH , dH);
        robot.updatePinpoint();
        telemetry.addData("X ", robot.pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Y ", robot.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("H ", robot.pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
        double forward = controllerForward.calculate( robot.pinpoint.getPosition().getX(DistanceUnit.INCH), targetF);
        double strafe = controllerStrafe.calculate( robot.pinpoint.getPosition().getY(DistanceUnit.INCH), targetS);
        double heading = controllerHeading.calculate(normalizeH(robot.pinpoint.getPosition().getHeading(AngleUnit.RADIANS), lastHeading), targetH);
        Vector2d r = rotateVector(new Vector2d(strafe, forward), -heading);
        robot.setDrivePower(r.x, r.y, heading);
        telemetry.addData("forward ", forward);
        telemetry.addData("strafe ", strafe);
        telemetry.addData("heading ", heading);

        telemetry.addData("forwardTarget ", targetF);
        telemetry.addData("strafeTarget ", targetS);
        telemetry.addData("headingTarget ", targetH);
        telemetry.update();
        lastHeading = robot.pinpoint.getHeading();
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
}

