package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.PoseKeeper;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.enums.Levels;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TeleOp RED Main")
@Config
public class TeleOpMainRed extends LinearOpMode {
    double oldTime = 0;
    AllianceColor allianceColor = AllianceColor.RED;

    // STATES
    boolean manualExtension = false;
    int climbOverride = 3;

    Gamepad oldGamepad = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, false);
        List<Action> actionsQueue = new ArrayList<>();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.left_bumper && !oldGamepad.left_bumper) {
                actionsQueue.add(new InstantAction(robot::teleDepositPreset));
            }
            if (gamepad1.right_bumper && !oldGamepad.right_bumper && !manualExtension) {
                actionsQueue.add(robot.generateTeleOpAutomatedIntake(gamepad1));
            } else if (gamepad1.right_bumper && !oldGamepad.right_bumper && manualExtension) {
                actionsQueue.add(
                  robot.intakePreset(20, true)
                );
            }
            if (gamepad1.right_trigger >= 0.1 && robot.state == Levels.INTAKE) {
                actionsQueue.add(
                        new InstantAction(() -> {
                            robot.extension.runToPosition(robot.extension.getPosition(true) + gamepad1.right_trigger);
                        })
                );
            } else if (gamepad1.left_trigger >= 0.1 && robot.state == Levels.INTAKE) {
                actionsQueue.add(
                        new InstantAction(() -> {
                            robot.extension.runToPosition(robot.extension.getPosition(true) - gamepad1.left_trigger);
                        })
                );
            }
            if (gamepad1.triangle && !oldGamepad.triangle) {
                actionsQueue.add(new InstantAction(robot::toggleGamepiece));
            }
            if (gamepad1.square && !oldGamepad.square) {
                actionsQueue.add(new InstantAction(this::setManualExtension));
            }
            if (gamepad1.circle && !oldGamepad.circle) {
                actionsQueue.add(new InstantAction(() -> robot.toggleGamepieceColor(allianceColor)));
            }
            if (gamepad1.cross && !oldGamepad.cross) {
                actionsQueue.add(robot.smartOuttake(true));
            }

            if (gamepad1.dpad_up && !oldGamepad.dpad_up) {
                actionsQueue.add(robot.primeClimb());
            }
            if (gamepad1.dpad_down && !oldGamepad.dpad_down) {
                if (robot.state == Levels.CLIMB_PRIMED) {
                    actionsQueue.add(robot.startClimbL2(gamepad1));
                } else if (robot.state == Levels.CLIMB_L2) {
                    actionsQueue.add(robot.startClimbL3());
                } else if (robot.state == Levels.ASCENDING && climbOverride != 0) {
                    gamepad1.rumbleBlips(climbOverride);
                    climbOverride -= 1;
                } else if (robot.state == Levels.ASCENDING) {
                    robot.l3ClimbOverride = true;
                }
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actionsQueue = newActions;

            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            robot.setDrivePower(-x, y, rx);

            robot.lift.update();
            PoseKeeper.set(robot.drive.pose);

            oldGamepad.copy(gamepad1);

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            telemetry.addData("LOOPTIME: ", frequency);
            telemetry.addData("STATE: ", robot.state);
            telemetry.update();
        }
    }

    private void setManualExtension() {
        manualExtension = !manualExtension;
    }
}
