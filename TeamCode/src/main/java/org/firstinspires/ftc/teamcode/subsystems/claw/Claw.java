package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class Claw {
    ContinuousServo servo1;
    ContinuousServo servo2;
    BrushlandColorSensor colorSensor;
    float power = 0;

    ElapsedTime sensorTimeout;

    public Claw(ContinuousServo s1, ContinuousServo s2, BrushlandColorSensor sensor) {
        servo1 = s1;
        servo2 = s2;
        colorSensor = sensor;

        sensorTimeout = new ElapsedTime();
    }

    public void setPower(float p) {
        servo1.servo.setPower(p);
        servo2.servo.setPower(-p);
        power = p;
    }

    public void startIntake() {
        setPower(1);
    }

    public void stopIntake() {
        setPower(0);
    }

    public void smartStopIntake(SampleColors... colors) {
        SampleColors s = detectSample();
        if (Arrays.stream(colors).anyMatch(x -> x == s )) {
            stopIntake();
        } else if (s != null) {
            eject();
        }
    }

    /**
     *
     * @param colors colors that you want to stop the intake for
     * @return 0: nothing in intake, 1: intaked targeted color, -1: intaked non-target (eject)
     */

    public int smartStopDetect(SampleColors... colors) {
        SampleColors s = detectSample();
        if (Arrays.stream(colors).anyMatch(x -> x == s )) {
            return 1;
        } else if (s != null) {
            return -1;
        }
        return 0;
    }

    public void eject() {
        setPower((float) -0.5);
        try {
            Thread.sleep(500);
            setPower(0);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void ejectOps() {
        setPower((float) -0.5);
        try {
            Thread.sleep(500);
            setPower(1);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public Action ejectOps(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> setPower((float) -0.5)),
                new SleepAction(0.5),
                new InstantAction(() -> setPower((float) 1))
        );
    }
    public Action ejectOpsAuton(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> setPower((float) -1)),
                new SleepAction(0.5),
                new InstantAction(() -> setPower((float) 1))
        );
    }

    public Action eject(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> setPower((float) -0.5)),
                new SleepAction(0.1),
                new InstantAction(() -> setPower(0))
        );
    }

    /**
     * Toggles a stall state on the claw in order to <b>temporarily</b> increase claw grip.
     * Typically used for clipped deposit.<br/>
     * <h3>WARNING: Don't have the servo in a stalled state for too long in order to extend servo lifetime</h3>
     */
    public void stall() {
        if (power == -0.1) {
            setPower((float) -0.1);
        } else {
            setPower(0);
        }
    }

    /**
     * Toggles a stall state on the claw in order to <b>temporarily</b> increase claw grip.
     * Typically used for clipped deposit.<br/>
     * <h3>WARNING: Don't have the servo in a stalled state for too long in order to extend servo lifetime</h3>
     * @param action input a <code>true</code> in order to activate action return method
     * @return returns <code>InstantAction</code> that toggles the stall
     */
    public Action stall(boolean action) {
        if (power == 0.2) {
            return new InstantAction( () -> setPower((float) 0.2));
        } else {
            return new InstantAction( () -> setPower(0));
        }
    }

    /**
     * Toggles a stall state on the claw in order to <b>temporarily</b> increase claw grip.
     * Typically used for clipped deposit.<br/>
     * <h3>WARNING: Don't have the servo in a stalled state for too long in order to extend servo lifetime</h3.
     * @param state <code>true</code> is a stalled claw and <code>false</code> is a unstalled claw
     */
    public void setStall(boolean state) {
        if (state) {
            setPower((float) 0.2);
        } else {
            setPower(0);
        }
    }

    /**
     * Toggles a stall state on the claw in order to <b>temporarily</b> increase claw grip.
     * Typically used for clipped deposit.<br/>
     * <h3>WARNING: Don't have the servo in a stalled state for too long in order to extend servo lifetime</h3>
     * @param action input a <code>true</code> in order to activate action return method
     * @param state <code>true</code> is a stalled claw and <code>false</code> is a unstalled claw
     * @return returns <code>InstantAction</code> that toggles the stall
     */
    public Action setStall(boolean state, boolean action) {
        if (state) {
            return new InstantAction( () -> setPower((float) 0.2));
        } else {
            return new InstantAction( () -> setPower(0));
        }
    }

    public SampleColors detectSample() {
//        if (sensorTimeout.time(TimeUnit.SECONDS) < 1f) {
//            return null;
//        }

        if (colorSensor.onlyPin0()) {
            return SampleColors.BLUE;
        } else if (colorSensor.onlyPin1()) {
            return SampleColors.RED;
        } else if (colorSensor.getBoth()) {
            return SampleColors.YELLOW;
        } else {
            return null;
        }
//        if (colorSensor.blue() > 0.5) {
//            return SampleColors.BLUE;
//        } else if (colorSensor.red() > 0.5) {
//            return SampleColors.RED;
//        } else if (colorSensor.getDistance(DistanceUnit.MM) > 3) {
//            return SampleColors.YELLOW;
//        }


//        sensorTimeout.reset();
//
//        return null;
    }
}
