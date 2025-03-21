package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Back Arm Test")
public class BackArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BackArm backArm = new BackArm(hardwareMap);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        boolean manualMode = false;
        double maxPower = 0.5;
        
        waitForStart();
        while (!isStopRequested()) {
            gamepads.copyStates();
            backArm.update();

            if(gamepads.isPressed("triangle")){
                manualMode = !manualMode;
                backArm.setManualMode(manualMode);
            }

            if(manualMode) {
                if (gamepads.isPressed("left_dpad")) backArm.setManualPower(-maxPower);
                else if (gamepads.isPressed("right_dpad")) backArm.setManualPower(maxPower);
                else backArm.setManualPower(0);
            } else {
                if (gamepads.isPressed("up_dpad")) backArm.changeTargetRotation(10);
                if (gamepads.isPressed("down_dpad")) backArm.changeTargetRotation(-10);
            }

            telemetry.addLine(backArm.log());
            telemetry.update();
        }
    }
}