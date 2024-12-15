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
         grabberServo = hardwareMap.servo.get("frontGrabberServo");
         limitSwitch = hardwareMap.touchSensor.get("frontGrabberLimit");
         closePosition = close;
         openPosition = open;
         openGrabber();
    }

    public void closeGrabber(){
        grabberServo.setPosition(closePosition);
        isClosed = true;
    }

    public void openGrabber(){
        grabberServo.setPosition(openPosition);
        isClosed = false;
    }

    public boolean isClosed() { return isClosed; }

    public void getSwitchState(){
        limitSwitch.isPressed();
    }

}
