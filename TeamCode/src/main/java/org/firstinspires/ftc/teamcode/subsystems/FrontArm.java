package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FrontArm {
    private final Servo armServo;
    private final double forwardPosition;
    private final double backPosition;
    private double currentPos;

    public FrontArm(HardwareMap hmap){
        armServo = hmap.servo.get("frontFlip");
        //armServo.setDirection(Servo.Direction.REVERSE);
        forwardPosition = 1;
        backPosition = 0;
    }

    public void forward(){
        setPosition(forwardPosition);
    }

    public void back(){
        setPosition(backPosition);
    }

    public boolean isForward(){ return (currentPos == 1); }

    public boolean isBack(){ return (currentPos == 0);}

    public void setPosition(double position){
        if(position >= 0 && position <= 1) {
            armServo.setPosition(position);
            currentPos = position;
        }
    }
    public double getSetPosition(){return currentPos;}
}
