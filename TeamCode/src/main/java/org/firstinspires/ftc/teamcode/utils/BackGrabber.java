package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class BackGrabber {
    private final Servo grabberServo;
    private final TouchSensor limitSwitch;
    private final double closePosition;
    private final double openPosition;
    private boolean isClosed;


    public BackGrabber(double close, double open, HardwareMap hmap){
        grabberServo = hmap.servo.get("backGrabberServo");;
        limitSwitch = hmap.touchSensor.get("backGrabberSwitch");
        closePosition = close;
        openPosition = open;
        close();
    }

    public void close(){
        grabberServo.setPosition(closePosition);
        isClosed = true;
    }

    public void open(){
        grabberServo.setPosition(openPosition);
        isClosed = false;
    }

    public boolean isClosed() { return isClosed; }

    public boolean getSwitchState(){
        return limitSwitch.isPressed();
    }

}