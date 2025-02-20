package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BackArm {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final AnalogInput rightEncoder;

    private double localAngle;
    private double previousAngle;
    private double totalRotation;
    private double targetRotation;
    private final double startingRotation;
    private final boolean runToPosition;

    public BackArm(HardwareMap hmap, boolean shouldRtp) {
        runToPosition = shouldRtp;
        leftServo = hmap.crservo.get("leftFlip");
        rightServo = hmap.crservo.get("rightFlip");
        rightEncoder = hmap.get(AnalogInput.class, "rightArmEncoder");

        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize encoder angles
        localAngle = getCurrentAngle();
        startingRotation = localAngle;

        // Set previous angle and total rotation
        previousAngle = localAngle;
        totalRotation = 0;
    }

    private double getCurrentAngle() {
        // Convert voltage to degrees (assuming 3.3V corresponds to full range)
        return (rightEncoder.getVoltage() / 3.3) * 360;
    }

    public void update() {
        localAngle = getCurrentAngle();
        double angleDifference = localAngle - previousAngle;

        // Handle rollover conditions
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }

        totalRotation += angleDifference;
        previousAngle = localAngle;

        if (!runToPosition) return;

        // Run-to-position control
        double maxPower = 0.25;
        double kP = 0.015;
        double error = targetRotation - totalRotation;

        if (Math.abs(error) > 1) {
            double power = Math.min(maxPower, Math.abs(error * kP)) * Math.signum(error);
            setPower(power);
        } else {
            setPower(0);
        }
    }

    public void setTargetRotation(double target) {
        targetRotation = target;
    }

    public double getTargetRotation() {
        return targetRotation;
    }

    public double getAngle() {
        return localAngle;
    }

    public double getPosition() {
        return totalRotation;
    }

    public void setPower(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    public double getRawOut() {
        return rightEncoder.getVoltage();
    }
}
