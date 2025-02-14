package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Underglow {

    private static RevBlinkinLedDriver blinkin;
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
        curPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern(){
        return curPattern;
    }
}
