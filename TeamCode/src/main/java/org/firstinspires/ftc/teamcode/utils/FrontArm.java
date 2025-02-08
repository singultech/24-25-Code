package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FrontArm {
    private final Servo armServo;
    private final double forwardPosition;
    private final double backPosition;
    private boolean isForward;

    public FrontArm(double forwardPos, double backPos, HardwareMap hmap){
        armServo = hmap.servo.get("frontFlip");
        armServo.setDirection(Servo.Direction.REVERSE);
        forwardPosition = forwardPos;
        backPosition = backPos;
        isForward = true;
    }

    public void forward(){
        armServo.setPosition(forwardPosition);
        isForward = true;
    }

    public void back(){
        armServo.setPosition(backPosition);
        isForward = false;
    }

    public boolean isForward(){ return  isForward; }
}
