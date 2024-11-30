package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

@TeleOp(group = "competition")
@Config
//@Disabled
public class PresetTuner extends LinearOpMode {
    public static double ARM_POS = 0;
    public static double CLAW_SPEED = 0;
    public static double CLIMB_SPEED = 0;
    public static double ELBOW_POS = 0;
    public static double EXT_POS = 270;
    public static int LIFT_POS = 0;


    StepperServo ext1;
    StepperServo ext2;
    StepperServo arm;
    StepperServo elbow;
    ContinuousServo intake1;
    ContinuousServo intake2;
    ContinuousServo climb1;
    ContinuousServo climb2;
    Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ext1 = new StepperServo(0, "ext1", hardwareMap);
        ext2 = new StepperServo(0, "ext2", hardwareMap);
        arm = new StepperServo(0, "arm", hardwareMap);
        elbow = new StepperServo(0, "elbow", hardwareMap);
        intake1 = new ContinuousServo(0, "intakeServo1", hardwareMap);
        intake2 = new ContinuousServo(0, "intakeServo2", hardwareMap);
        climb1 = new ContinuousServo(0, "climb1", hardwareMap);
        climb2 = new ContinuousServo(0, "climb2", hardwareMap);
        lift = new Lift(new Motor(0, "lift1", hardwareMap, false), new Motor(0, "lift2", hardwareMap, false), hardwareMap.voltageSensor.iterator().next());

        intake2.servo.setDirection(DcMotorSimple.Direction.REVERSE);
        int prevLiftTarget = LIFT_POS;
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            ext1.setAngle((float) EXT_POS);
            ext2.setAngle((float) EXT_POS);
            arm.setAngle((float) ARM_POS);
            elbow.setAngle((float) ELBOW_POS);
            intake1.setSpeed((float) CLAW_SPEED);
            intake2.setSpeed((float) CLAW_SPEED);
            climb1.setSpeed((float) CLIMB_SPEED);
            climb2.setSpeed((float) CLIMB_SPEED);

            if (prevLiftTarget != LIFT_POS) {
                lift.runToPosition(LIFT_POS);
            }
            prevLiftTarget = LIFT_POS;
        }
    }
}


// arm: 85
// claw open (hold): 70
// claw close (drop): 180
// intake lock: 320; 176 intake down
// elbow (capture): 280, wrist 272
// arm intermediate: 100
