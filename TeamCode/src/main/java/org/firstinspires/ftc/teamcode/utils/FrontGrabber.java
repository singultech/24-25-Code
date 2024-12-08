package org.firstinspires.ftc.teamcode.utils;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class FrontGrabber {
    private final Servo grabberServo;
    private final TouchSensor limitSwitch;
    private final double closePosition;
    private final double openPosition;

    public FrontGrabber(double close, double open){
         grabberServo = hardwareMap.servo.get("frontGrabberServo");
         limitSwitch = hardwareMap.touchSensor.get("frontGrabberLimit");
         closePosition = close;
         openPosition = open;
    }

    public void closeGrabber(){
        grabberServo.setPosition(closePosition);
    }

    public void openGrabber(){
        grabberServo.setPosition(openPosition);
    }

    public void getSwitchState(){
        limitSwitch.isPressed();
    }

}
