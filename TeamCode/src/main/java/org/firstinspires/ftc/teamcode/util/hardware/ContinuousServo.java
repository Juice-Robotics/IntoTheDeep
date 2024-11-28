package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.hardware.Component;

public class ContinuousServo extends Component {

    private float speed;
    public CRServo servo;
    public AnalogInput encoder;

    public ContinuousServo(int port, String name, HardwareMap map){
        super(port, name);
        servo = map.crservo.get(name);
    }

    public ContinuousServo(int port, String name, HardwareMap map, String encoder){
        super(port, name);
        servo = map.crservo.get(name);
        this.encoder = map.get(AnalogInput.class, encoder);
    }

    public void setSpeed(float speed) {
        this.speed = speed;
        servo.setPower(speed);
    }

    public float getSpeed(){
        return (float) servo.getPower();
    }

    public float getAngle() {
        return (float) (encoder.getVoltage() / 3.3 * 360);
    }

}