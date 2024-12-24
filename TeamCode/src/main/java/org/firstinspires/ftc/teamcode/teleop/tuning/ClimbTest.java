package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

@TeleOp(group = "competition")
@Config
//@Disabled
public class ClimbTest extends LinearOpMode {
    ContinuousServo climb1;
    ContinuousServo climb2;
    Extension ext;
    Lift lift;
    DcMotorEx slides1;
    DcMotorEx slides2;
    public static int REVERSED1 = 1;

    public static int LIFT_TARGET = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        climb1 = new ContinuousServo(0, "climb1", hardwareMap);
        climb2 = new ContinuousServo(0, "climb2", hardwareMap);
        ext = new Extension(new StepperServo(0, "ext1", hardwareMap), new StepperServo(0, "ext2", hardwareMap));
        //lift = new Lift(new Motor(0, "lift1", hardwareMap, false), new Motor(0, "lift2", hardwareMap, false), hardwareMap.voltageSensor.iterator().next());
        slides1 = hardwareMap.get(DcMotorEx.class, "lift1");
        slides2 = hardwareMap.get(DcMotorEx.class, "lift2");
        ext.runToPosition(165);
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                climb1.setSpeed((float) -1);
                climb2.setSpeed((float) -1);
            } else if (gamepad1.dpad_down) {
                climb1.setSpeed((float) 1);
                climb2.setSpeed((float) 1);
            } else {
                climb1.setSpeed(0);
                climb2.setSpeed(0);
            }
            if (gamepad1.triangle) {
                slides1.setPower((float) REVERSED1*-1);
                slides2.setPower((float) 1);
            } else if (gamepad1.cross) {
                slides1.setPower((float) REVERSED1*1);
                slides2.setPower((float) -1);
            } else if (gamepad1.circle) {
                slides1.setPower((float) REVERSED1*0.5);
                slides2.setPower((float) -0.5);
            }else {
                slides1.setPower(0);
                slides2.setPower(0);
            }
            telemetry.addData("liftCurrentLeft: ", slides1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("liftCurrentRight: ", slides2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("listPos: ", slides1.getCurrentPosition());
            telemetry.update();
//            lift.runToPosition(LIFT_TARGET);
//            lift.update();
        }
    }
}


// INTAKE (DOWN)
// Arm: 103, elbow: 282, ext 190
// INTAKE (INTERMEDIATE)
// Arm: 130, elbow 282, ext 190
// INTERMEDIATE
// Arm: 260, Elbow: 190, Ext: 100

// LIFT MAX 2150
