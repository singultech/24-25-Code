package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Underglow {

    private final RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern curPattern;
    public Underglow(HardwareMap hmap) {
        blinkin = hmap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        curPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    }

    public Underglow(HardwareMap hmap, RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkin = hmap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(pattern);
        curPattern = pattern;
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkin.setPattern(pattern);
        curPattern = pattern;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern(){
        return curPattern;
    }

    @TeleOp(name = "UnderglowTest")
    public static class UnderglowTest extends LinearOpMode {
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
}
