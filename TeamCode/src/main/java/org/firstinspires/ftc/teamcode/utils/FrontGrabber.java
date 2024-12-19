package org.firstinspires.ftc.teamcode.utils;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class FrontGrabber {
    private final Servo grabberServo;
    private final TouchSensor limitSwitch;
    private final double closePosition;
    private final double openPosition;
    private boolean isClosed;

    public FrontGrabber(double close, double open){
         grabberServo = hardwareMap.servo.get("frontGrabber");
         limitSwitch = hardwareMap.touchSensor.get("frontGrabber");
         closePosition = close;
         openPosition = open;
         open();
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
