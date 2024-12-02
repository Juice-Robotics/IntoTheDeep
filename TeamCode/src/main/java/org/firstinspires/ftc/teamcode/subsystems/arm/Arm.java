package org.firstinspires.ftc.teamcode.subsystems.arm;

import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

public class Arm {
    StepperServo servo;
    StepperServo wrist;

    float target = 0;

    public Arm(StepperServo s1, StepperServo s2) {
        servo = s1;
        wrist = s2;
    }

    public void runToPosition(float t) {
        servo.setAngle(t);

        target = t;
    }

    public void runToPreset(Levels level) {
        if (level == Levels.INIT) {
            runToPosition(286);
            setWristAngle(190);
        } else if (level == Levels.INTAKE) {
            runToPosition(121);//103
            setWristAngle(282);
        } else if (level == Levels.INTAKE_INTERMEDIATE) {
            runToPosition(180);
            setWristAngle(282);
        } else if (level == Levels.INTERMEDIATE) {
            runToPosition(238);
            setWristAngle(282);
        } else if (level == Levels.LOCATING_TARGETS) {
            runToPosition(126);
            setWristAngle(100);
        } else if (level == Levels.LOW_BASKET) {
            runToPosition(336);
            setWristAngle(110);
        } else if (level == Levels.HIGH_BASKET) {
            runToPosition(326);
            setWristAngle(110);
        } else if (level == Levels.LOW_RUNG) {
            runToPosition(126);
            setWristAngle(100);
        } else if (level == Levels.HIGH_RUNG) {
            runToPosition(346);
            setWristAngle(150);
        } else if (level == Levels.CLIMB_EXTENDED) {
            runToPosition(100);
            setWristAngle(100);
        } else if (level == Levels.CLIMB_RETRACTED) {
            runToPosition(100);
            setWristAngle(100);
        }
    }

    public float getPosition() {
        return target;
    }

    public void setWristAngle(float angle) {
        wrist.setAngle(angle);
    }

    public void runToWristPreset(Levels level) {
        if (level == Levels.INTAKE_INTERMEDIATE) {
            setWristAngle(10);
        }
    }
}
