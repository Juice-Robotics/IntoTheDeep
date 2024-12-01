package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.PoseKeeper;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.enums.ClimbType;
import org.firstinspires.ftc.teamcode.util.enums.Levels;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TimingTest")
@Config
public class TimingTest extends LinearOpMode {
    double oldTime = 0;
    AllianceColor allianceColor = AllianceColor.RED;

    // STATES
    boolean manualExtension = false;
    int climbOverride = 3;

    Gamepad oldGamepad = new Gamepad();
    boolean oldRBumper = false;
    boolean oldLBumper = false;
    boolean oldCross = false;
    boolean oldTriangle = false;
    double oldTrigger = 0;
    double oldRtrigger = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, false);
        List<Action> actionsQueue = new ArrayList<>();

        waitForStart();
        robot.initSubsystems();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

//            if (gamepad1.left_bumper && !oldGamepad.left_bumper) {
//                actionsQueue.add(new InstantAction(robot::teleDepositPreset));
//            }
            if (gamepad1.right_bumper && !oldRBumper) {
                if (robot.state != Levels.INTAKE_INTERMEDIATE) {
                    actionsQueue.add(
                            robot.teleIntakePreset(true)
                    );
                } else {
                    actionsQueue.add(
                            robot.intakeDrop()
                    );
                }
            }
            oldRBumper= gamepad1.right_bumper;

            if ((gamepad1.right_trigger > 0.3) && (oldRtrigger > 0.3)){
                actionsQueue.add(robot.stopIntakeAction());
            }
            oldRtrigger = gamepad1.right_trigger;

            if (gamepad1.left_bumper && !oldLBumper) {
                if (robot.mode == Robot.Gamepiece.SAMPLE){
                    actionsQueue.add(robot.highBasketAction());
                }else{
                    actionsQueue.add(robot.highRung(true));
                }
            }
            oldLBumper = gamepad1.left_bumper;
            if (gamepad1.cross &&!oldCross) {
                if (robot.mode == Robot.Gamepiece.SAMPLE) {
                    actionsQueue.add(robot.outtakeSample(true));
                } else{
                    actionsQueue.add(robot.outtakeSpecimen(true));
                }
            }
            oldCross = gamepad1.cross;
            if (gamepad1.triangle && !oldTriangle){
                robot.toggleGamepiece();
            }
            oldTriangle = gamepad1.triangle;

            if (gamepad1.left_trigger >= 0.5 && oldTrigger <= 0.5) {
                actionsQueue.add(
                        new SequentialAction(
                                new InstantAction(() -> {
                                    robot.lift.runToPosition(250);
                                    robot.arm.runToPreset(Levels.HIGH_BASKET);
                                }),
                                new SleepAction(0.5),
                                robot.claw.eject(true)
                        )
                );
            }
            oldTrigger = gamepad1.left_trigger;

            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actionsQueue = newActions;

            double x = -gamepad1.right_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = gamepad1.left_stick_x;
            robot.setDrivePower(-x, y, rx);

            robot.lift.update();
//            PoseKeeper.set(robot.drive.pose);

            telemetry.addData("rbumpero " ,oldRBumper);
            oldGamepad.copy(gamepad1);

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            telemetry.addData("MODE", robot.mode.toString());
            telemetry.addData("COLOR", robot.targetColor.toString());
            telemetry.addData("CLIMB", robot.climbMode.toString());
            telemetry.addData("LOOPTIME: ", frequency);
            telemetry.addData("state: ", robot.state);
            telemetry.addData("rbumper " ,gamepad1.right_bumper);
            telemetry.update();
        }
    }

    private void setManualExtension() {
        manualExtension = !manualExtension;
    }
}
