package org.firstinspires.ftc.teamcode.subsystems.extension;

import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

public class Extension {
    StepperServo servo1;
    StepperServo servo2;
    //120 - 195
    float low = 90;

    float target = 0;

    public int MAXIMUM_EXTENSION = 100;

    public Extension(StepperServo s1, StepperServo s2) {
        servo1 = s1;
        servo2 = s2;
    }

    public void runToPosition(float t) {
        servo1.setAngle(t);
        servo2.setAngle(t);

        target = t;
    }

    public void runToPreset(Levels level) {
        if (level == Levels.INIT) {
            runToPosition(low);
        } else if (level == Levels.INTAKE) {
            runToPosition(190);
        } else if (level == Levels.INTERMEDIATE) {
            runToPosition(low);
        } else if (level == Levels.LOCATING_TARGETS) {
            runToPosition(low);
        } else if (level == Levels.LOW_BASKET) {
            runToPosition(low);
        } else if (level == Levels.HIGH_BASKET) {
            runToPosition(low);
        } else if (level == Levels.LOW_RUNG) {
            runToPosition(low);
        } else if (level == Levels.HIGH_RUNG) {
            runToPosition(low);
        } else if (level == Levels.CLIMB_EXTENDED) {
            runToPosition(low);
        } else if (level == Levels.CLIMB_RETRACTED) {
            runToPosition(low);
        }
    }

    public float getPosition() {
        return target;
    }
    public float getPosition(boolean useEncoder) {
        return servo1.getAngle(true);
    }
}
