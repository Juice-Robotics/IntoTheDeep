package org.firstinspires.ftc.teamcode.util.control;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * This is NOT an opmode.
 *
 * THis Class is used to define a PID controller of any kind
 *
 */

public class PIDFController
{
    private double lastError_;
    private double errorSum_;
    private double kp_;
    private double ki_;
    private double kd_;
    private double kf_;
    private long lastTime_;
    private boolean rot;
    public double error;
    public double derError;
    public double pTotal;
    public double iTotal;
    public double dTotal;
    public double fTotal;

    /* Public OpMode members. */
    private PIDFController() {}

    /* Constructor */
    public PIDFController(double kp, double ki, double kd, double kf, boolean rot){
        lastError_ = 0;
        lastTime_ = System.currentTimeMillis();
        errorSum_ = 0;
        this.rot = rot;

        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        kf_ = kf;
    }

    public void setPIDF(double kp, double ki, double kd, double kf){
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        kf_ = kf;
    }

    public void resetIntegrator() {
        errorSum_ = 0;
    }

    public float update(double newInput, double setPoint_){

        long time = System.currentTimeMillis();
        long period = time - lastTime_;


        if(rot){
            error = norm(setPoint_ - newInput);
        } else{
            error = setPoint_ - newInput;
        }

        if ((int)Math.signum(lastError_) != (int) Math.signum(error)) {
            errorSum_ = 0;
        }

        errorSum_ +=  (error * period);

        if (Math.abs(period) > 1E-6) {
            derError = (error - lastError_) / period;
        } else {
            derError = 0;
        }

        pTotal = kp_ * error;
        iTotal = ki_ * errorSum_;
        dTotal = kd_ * derError;
        fTotal = kf_ * Math.signum(error);

        double output = pTotal + iTotal + dTotal + fTotal;

        RobotLog.i("pTotal is " + pTotal);
        RobotLog.i("iTotal is " + iTotal);
        RobotLog.i("dTotal is " + dTotal);
        RobotLog.i("fTotal is " + fTotal);
        RobotLog.i("PID error is " + error + "; PID output is " + output + "; errorsum is " + errorSum_);

        lastError_ = error;
        lastTime_ = time;
        return (float) output;
    }

    public static double norm(double angle) {
        angle = angle % (Math.PI * 2);

        return angle;

    }
}