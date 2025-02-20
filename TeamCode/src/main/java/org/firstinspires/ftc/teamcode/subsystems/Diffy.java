package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Diffy {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final AnalogInput leftEncoder;
    private final AnalogInput rightEncoder;

    private double leftLocalAngle;
    private double leftPreviousLocalAngle;
    private double leftTotalRotation;
    private double rightLocalAngle;
    private double rightPreviousLocalAngle;
    private double rightTotalRotation;
    private final double rightStartingAngle;
    private final double leftStartingAngle;
    private double leftTargetRotation;
    private double rightTargetRotation;
    private double grabberRotation;
    private final boolean runToPosition;

    public Diffy(HardwareMap hmap, boolean shouldRtp){
        runToPosition = shouldRtp;
        leftServo = hmap.crservo.get("leftDiffy");
        rightServo = hmap.crservo.get("rightDiffy");
        leftEncoder = hmap.get(AnalogInput.class, "leftDiffyEncoder");
        rightEncoder = hmap.get(AnalogInput.class, "rightDiffyEncoder");
        leftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        grabberRotation = 0;

        // Get initial angles
        leftLocalAngle = -leftEncoder.getVoltage() / 3.3 * 360.0;
        rightLocalAngle = rightEncoder.getVoltage() / 3.3 * 360.0;

        // Store starting angles for reference
        leftStartingAngle = leftLocalAngle;
        rightStartingAngle = rightLocalAngle;

        // Initialize previous angles to match current angles
        leftPreviousLocalAngle = leftLocalAngle;
        rightPreviousLocalAngle = rightLocalAngle;

        // Start total rotation at 0
        leftTotalRotation = 0;
        rightTotalRotation = 0;
    }
    private double normalizeAngleDifference(double angleDifference) {
        if (angleDifference < -180) {
            return angleDifference + 360;
        } else if (angleDifference > 180) {
            return angleDifference - 360;
        }
        return angleDifference;
    }

    public void setLeftPower(double power) {
        if (!runToPosition)leftServo.setPower(power);
    }
    public void setRightPower(double power) {
        if (!runToPosition)rightServo.setPower(power);
    }
    public void update() {
        leftLocalAngle = -leftEncoder.getVoltage() / 3.3 * 360.0;
        rightLocalAngle = rightEncoder.getVoltage() / 3.3 * 360.0;
        double leftAngleDifference = normalizeAngleDifference(leftLocalAngle - leftPreviousLocalAngle);
        double rightAngleDifference = normalizeAngleDifference(rightLocalAngle - rightPreviousLocalAngle);
        leftTotalRotation += leftAngleDifference;
        rightTotalRotation += rightAngleDifference;
        leftPreviousLocalAngle = leftLocalAngle;
        rightPreviousLocalAngle = rightLocalAngle;

        if (!runToPosition) return;

        double maxPower = 0.25;
        double kP = 0.015;
        double rightError = rightTargetRotation - rightTotalRotation;
        if (Math.abs(rightError) > 1) {
            double rightPower = Math.min(maxPower, Math.abs(rightError * kP)) * Math.signum(rightError);
            rightServo.setPower(-rightPower);
        } else {
            rightServo.setPower(0);
        }
        double leftError = leftTargetRotation - leftTotalRotation;
        if (Math.abs(leftError) > 1) {
            double leftPower = Math.min(maxPower, Math.abs(leftError * kP)) * Math.signum(leftError);
            leftServo.setPower(-leftPower);
        } else {
            leftServo.setPower(0);
        }

        /*double rightError = rightTargetRotation - rightTotalRotation;
        leftServo.setPower(0);
        if (Math.abs(rightError) > 5){
            if(rightError<0){
                rightServo.setPower(0.25);
            }else{
                rightServo.setPower(-0.25);
            }
        }*/

    }

    public double getLeftLocalAngle() {
        return leftLocalAngle;
    }
    public double getRightLocalAngle() {
        return rightLocalAngle;
    }
    public double getRightTotalRotation(){
        return rightTotalRotation;
    }
    public double getLeftTotalRotation(){
        return leftTotalRotation;
    }
    public double getRightStartingAngle(){return rightStartingAngle;}
    public double getLeftStartingAngle(){return leftStartingAngle;}
    public void setRightTargetRotation(double target){rightTargetRotation = target;}
    public double getRightTargetRotation(){return rightTargetRotation;}
    public void setLeftTargetRotation(double target){leftTargetRotation = target;}
    public double getLeftTargetRotation(){return leftTargetRotation;}
    public void changeLeftTargetRotation(double amount){
        leftTargetRotation += amount;
    }
    public void changeRightTargetRotation(double amount){
        rightTargetRotation += amount;
    }
    public void rotateGrabber(double degrees){
        if(grabberRotation+degrees<=50 && grabberRotation+degrees>=-50) {
            grabberRotation += degrees;
            changeLeftTargetRotation(degrees);
            changeRightTargetRotation(-degrees);
        }
    }
    public double getGrabberRotation(){
        return grabberRotation;
    }


}
