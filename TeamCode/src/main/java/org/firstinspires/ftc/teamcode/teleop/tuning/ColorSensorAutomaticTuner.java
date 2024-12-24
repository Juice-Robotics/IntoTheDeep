package org.firstinspires.ftc.teamcode.teleop.util;

import android.util.Range;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;

@TeleOp(group = "competition")
@Config
@Disabled
public class ColorSensorAutomaticTuner extends LinearOpMode {
    RevColorSensorV3 sensor;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(RevColorSensorV3.class, "Color");
        ((LynxI2cDeviceSynch) sensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        boolean oldCross = false;
        boolean oldSquare = false;
        boolean oldCircle = false;
        int brightness = 0;

        boolean currentlyTuning = false;

        /*
         * 0 - NONE<br/>
         * 1 - RED<br/>
         * 2 - BLUE<br/>
         * 3 - YELLOW
         */
        int tuning = 0;

        Range<Integer> redRange = new Range<Integer>(0, 100);
        Range<Integer> blueRange = new Range<Integer>(0, 100);
        Range<Integer> yellowRange = new Range<Integer>(0, 100);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.cross && !oldCross && !currentlyTuning) {
                currentlyTuning = true;
                tuning = 1;
            } else if (gamepad1.square && !oldSquare && !currentlyTuning) {
                currentlyTuning = true;
                tuning = 2;
            } else if (gamepad1.circle && !oldCircle && !currentlyTuning) {
                currentlyTuning = true;
                tuning = 3;
            }

            if (currentlyTuning) {
                if (tuning == 1) {
                    if (brightness > 100) {
                        gamepad1.rumbleBlips(3);
                        currentlyTuning = false;
                        tuning = 0;
                        brightness = 0;
                    }

                }
            }

        }
    }
}