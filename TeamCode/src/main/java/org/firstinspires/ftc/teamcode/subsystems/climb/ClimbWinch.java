package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.commands.WinchStopAtPosition;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;

public class ClimbWinch {
    public ContinuousServo servo1;
    ContinuousServo servo2;

    public float target;

    public static int PRIMED_POS = 0;
    public static int EXTENDED_POS = 120;
    public static int CLIMB_POS = 220;

    public ClimbWinch(ContinuousServo s1, ContinuousServo s2) {
        servo1 = s1;
        servo2 = s2;
    }

//    public void runToPosition(float t) {
//        servo1.setAngle(t);
//        servo2.setAngle(t);
//
//        target = t;
//    }

    public void setPower(float p) {
        servo1.setSpeed(p);
        servo2.setSpeed(p);
    }

    public Action prime() {
        return new SequentialAction(
                new InstantAction(() -> setPower(-1)),
                new WinchStopAtPosition(this, PRIMED_POS)
        );
    }

    public Action up() {
        return new SequentialAction(
                new InstantAction(() -> setPower(1)),
                new WinchStopAtPosition(this, EXTENDED_POS)
        );
    }

    public Action startClimb() {
        return new InstantAction(() -> setPower(-1));
    }

    public float getPosition() {
        return target;
    }
}
