package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Underglow;

@TeleOp(name = "UnderglowTest")
public class UnderglowTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Underglow underglow = new Underglow(hardwareMap, RevBlinkinLedDriver.BlinkinPattern.RED);
        
        waitForStart();
        while (!isStopRequested()) {
            gamepads.copyStates();
            if (gamepads.isPressed("a")){
                if(underglow.getPattern() == RevBlinkinLedDriver.BlinkinPattern.RED)
                    underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                else if (underglow.getPattern() == RevBlinkinLedDriver.BlinkinPattern.BLUE){
                    underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                }
                else underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            
            telemetry.update();
        }
    }
}