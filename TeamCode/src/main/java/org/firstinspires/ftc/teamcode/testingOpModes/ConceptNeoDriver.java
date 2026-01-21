package org.firstinspires.ftc.teamcode.testingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.NeoDriver;

@TeleOp(name="Concept: NeoDriver", group = "Concept")
//@Disabled
public class ConceptNeoDriver extends LinearOpMode {
    private NeoDriver LED;
    private static final int NUM_PIXELS =8 ; // The total number of pixels on your strip

    @Override
    public void runOpMode() {
        LED = hardwareMap.get(NeoDriver.class, "led");
        LED.initializeNeoPixels();

        // Variable to hold the starting hue for our rainbow pattern
        float currentHue = 0.0f;

        // An array to hold the HSV values. We'll reuse this to avoid creating new arrays in the loop.
        float[] hsvValues = {0f, 1f, 0.1f}; // Hue, Saturation, Value

        // Set all the LEDs to a single color (e.g., MINT) as we press initialize
        LED.setPixelRangeColor(0, NUM_PIXELS, NeoDriver.COLOR.BLACK);
        LED.show();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            for (int i = 0; i < NUM_PIXELS; i++) {
                // Set the pixel to the newly calculated RGB color.
                LED.setPixelColor(i, NeoDriver.COLOR.GREEN);
                sleep(10);
            }
            LED.show();
            sleep(10);
            //sleep(250);
        }
        LED.setPixelRangeColor(0, NUM_PIXELS, NeoDriver.COLOR.BLACK);
    }
}
