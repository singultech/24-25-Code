package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FrontArm {
    private final Servo armServo;
    private final Servo wristServo;


    private final double ARM_FORWARD_POSITION = 1;
    private final double ARM_BACK_POSITION = 0;
    private final double WRIST_FORWARD_POSITION = 1;
    private final double WRIST_BACK_POSITION = 0;


    private double currentArmPos;
    private double currentWristPos;

    public FrontArm(HardwareMap hmap){
        armServo = hmap.servo.get("frontFlip");
        wristServo = hmap.servo.get("frontWrist");
        armServo.setDirection(Servo.Direction.REVERSE);
    }

    public void armForward(){
        setArmPosition(ARM_FORWARD_POSITION);
    }

    public void armBack(){
        setArmPosition(ARM_BACK_POSITION);
    }

    public boolean isArmForward(){ return (currentArmPos == ARM_FORWARD_POSITION); }

    public boolean isArmBack(){ return (currentArmPos == ARM_BACK_POSITION);}

    public void setArmPosition(double position){
        if(position >= 0 && position <= 1) {
            armServo.setPosition(position);
            currentArmPos = position;
        }
    }
    public double getArmPosition(){return currentArmPos;}

    public void wristForward(){
        setWristPosition(WRIST_FORWARD_POSITION);
    }

    public void wristBack(){
        setWristPosition(WRIST_BACK_POSITION);
    }

    public boolean isWristForward(){ return (currentWristPos == WRIST_FORWARD_POSITION); }

    public boolean isWristBack(){ return (currentWristPos == WRIST_BACK_POSITION);}

    public void setWristPosition(double position){
        if(position >= 0 && position <= 1) {
            wristServo.setPosition(position);
            currentWristPos = position;
        }
    }
    public double getWristPosition(){return currentWristPos;}


}
