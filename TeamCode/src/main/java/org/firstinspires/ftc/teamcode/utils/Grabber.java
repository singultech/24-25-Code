package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Grabber {
    private final Servo grabberServo;
    private final TouchSensor limitSwitch;
    private final double closePosition;
    private final double openPosition;
    private boolean isClosed;


    public Grabber(double close, double open, Servo grabber, TouchSensor limit){
        grabberServo = grabber;
        limitSwitch = limit;
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