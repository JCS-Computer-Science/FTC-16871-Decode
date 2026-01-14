package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import android.util.Log;

/**
 * NeoPixel Driver for FTC Robotics.
 * Manages an internal pixel buffer and handles color format conversion (RGB to GRB)
 * for I2C-based NeoPixel drivers (like the Adafruit SeeSaw, product #5766).
 * * FINAL VERSION incorporating stability fixes, Big-Endian logic, and byte-width correction.
 */
@I2cDeviceType
@DeviceProperties(name = "Adafruit NeoDriver", xmlTag = "NeoDriver")
public class NeoDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // --- Internal State and Constants ---
    private static final String TAG = "NeoDriver";

    // SeeSaw Module Addresses
    public static final int NEOPIXEL_MODULE_ADDR = 0x0E; // The module for NeoPixel commands

    // --- User Methods ---
    /**
     * Initialization method for NeoPixels.
     */
    public void initializeNeoPixels() {
        write8(Register.SPEED, (byte) 0x00);
        write8(Register.PIN, (byte) 0x0F);
    }

    /**
     * Sends the SHOW command (Module 0x0E, Register 0x05) to update the LEDs.
     */
    public void show() {
        writeCommand();
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Sets the color of a single pixel using a predefined COLOR enum.
     * @param pixelIndex The index of the pixel to set.
     * @param red The red component of the color.
     * @param green The green component of the color.
     * @param blue The blue component of the color.
     */
    public void setPixelColor(int pixelIndex, int red, int green, int blue) {
        // For a single pixel, we need to send 5 bytes of data:
        // 2 bytes for start location
        // 3 bytes for the color
        byte[] pixelBuffer = new byte[5];

        int startaddress = pixelIndex * 3;

        if (pixelIndex >= 0) {
            pixelBuffer[0] = (byte) (startaddress >> 8 & 0xFF);
            pixelBuffer[1] = (byte) (startaddress & 0xFF);
            pixelBuffer[2] = (byte) (green & 0xFF);
            pixelBuffer[3] = (byte) (red & 0xFF);
            pixelBuffer[4] = (byte) (blue & 0xFF);
        } else {
            // Using Log.e for clearer error output
            Log.e(TAG, "Pixel index " + pixelIndex + " out of bounds.");
        }
        writeBytes(pixelBuffer);
    }

    /**
     * Sets the color of a single pixel using a predefined COLOR enum.
     * @param pixelIndex The index of the pixel to set.
     * @param color The named color from the COLOR enum (e.g., COLOR.GREEN).
     */
    public void setPixelColor(int pixelIndex, COLOR color) {
        // This is a convenience method. It calls the original setPixelColor
        // using the RGB values stored inside the provided enum constant.
        setPixelColor(pixelIndex, color.red, color.green, color.blue);
    }

    /**
     * Sets the color of a single pixel using a predefined COLOR enum.
     * @param startPixel The index of the pixel to start.
     * @param pixelCount The number of pixels to set.
     * @param red The red component of the color.
     * @param green The green component of the color.
     * @param blue The blue component of the color.
     */
    public void setPixelRangeColor(int startPixel, int pixelCount, int red, int green, int blue) {
        // Write only 9 pixels at a time (maximum).
        int numberOfPixels = pixelCount;

        while (numberOfPixels > 9) {
            byte[] pixelBuffer = new byte[32];
            int startAddress = startPixel * 3;

            pixelBuffer[0] = (byte) (startAddress >> 8 & 0xFF);
            pixelBuffer[1] = (byte) (startAddress & 0xFF);
            for (int i = 1; i <= 10; i++) {
                pixelBuffer[(i * 3) - 1] = (byte) (green & 0xFF);
                pixelBuffer[(i * 3)] = (byte) (red & 0xFF);
                pixelBuffer[(i * 3) + 1] = (byte) (blue & 0xFF);
            }

            startPixel += 9;
            numberOfPixels -= 9;

            writeBytes(pixelBuffer);
        }
        // now that there is not more than a full bank of 10, write the remainder
        byte[] pixelBuffer = new byte[(numberOfPixels * 3) + 2];
        int startAddress = startPixel * 3;
        pixelBuffer[0] = (byte) (startAddress >> 8 & 0xFF);
        pixelBuffer[1] = (byte) (startAddress & 0xFF);
        for (int i = 1; i <= numberOfPixels; i++) {
            pixelBuffer[(i * 3) - 1] = (byte) (green & 0xFF);
            pixelBuffer[(i * 3)] = (byte) (red & 0xFF);
            pixelBuffer[(i * 3) + 1] = (byte) (blue & 0xFF);
        }
        writeBytes(pixelBuffer);
    }

    /**
     * Sets the color of a single pixel using a predefined COLOR enum.
     * @param startPixel The index of the pixel to start.
     * @param pixelCount The number of pixels to set.
     * @param color The named color from the COLOR enum (e.g., COLOR.GREEN).
     */
    public void setPixelRangeColor(int startPixel, int pixelCount, COLOR color) {
        // This is a convenience method. It calls the original setPixelRangeColor
        // using the RGB values stored inside the provided enum constant.
        setPixelRangeColor(startPixel, pixelCount, color.red, color.green, color.blue);
    }

    // --- Helper Methods for I2C communication (Implementing the SeeSaw 2-byte protocol) ---
    /** Writes a command byte (Module 0x0E, Register R). */
    protected void writeCommand() {
        byte[] payload = new byte[]{(byte) Register.SHOW.bVal};
        deviceClient.write(NEOPIXEL_MODULE_ADDR, payload);
    }

    /** * Writes an 8-bit byte value (Module 0x0E, Register R, Data).
     * Used for single-byte registers like PIN and SPEED.
     */
    protected void write8(final Register reg, byte value) {
        byte[] payload = new byte[2];
        payload[0] = (byte) reg.bVal;
        payload[1] = value;

        deviceClient.write(NEOPIXEL_MODULE_ADDR, payload);
    }

    /** * Writes bytes for 0 to 10 pixels (Module 0x0E, Register BUF, Data).
     * Also writes to the BUF_LENGTH register to set the number of bytes to expect.
     *     (Module 0x0E, Register BUF_LENGTH, Buffer Length)
     * Used to write the colors to the appropriate registers and then use show() to display the colors.
     */
    protected void writeBytes(byte[] values) {
        byte[] payload = new byte[values.length + 1];
        payload[0] = (byte) Register.BUF.bVal;
        System.arraycopy(values, 0, payload, 1, values.length);

        byte[] bufLenPayload = new byte[2];
        bufLenPayload[0] = (byte) (Register.BUF_LENGTH.bVal & 0xFF);
        bufLenPayload[1] = (byte) (values.length +1 & 0XFF);

        deviceClient.write(NEOPIXEL_MODULE_ADDR, bufLenPayload);
        deviceClient.write(NEOPIXEL_MODULE_ADDR, payload);
    }

    // --- Registers and Config Settings ---
    public enum Register{
        PIN(0x01),
        SPEED(0x02),
        BUF_LENGTH(0x03),
        BUF(0x04),
        SHOW(0x05);

        public final int bVal;

        Register(int bVal){
            this.bVal = bVal;
        }
    }

    public enum COLOR {
        // Each enum constant is an object with r, g, and b fields.
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255),
        WHITE(255, 255, 255),
        BLACK(0, 0, 0), // Useful for turning pixels off
        YELLOW(255, 255, 0),
        CYAN(0, 255, 255),
        MAGENTA(255, 0, 255),
        ORANGE(255, 127, 0),
        PURPLE(128, 0, 128),
        PINK(255, 105, 180),
        LIME(191, 255, 0),
        MINT(62, 180, 137),
        TEAL(0, 128, 128),
        INDIGO(75, 0, 130),
        GOLD(255, 215, 0);

        // Member variables to hold the RGB values for each color
        public final int red;
        public final int green;
        public final int blue;

        // Constructor for the enum
        COLOR(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    // --- Construction and Initialization ---
    // Default I2C address for the Adafruit NeoDriver
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x60);

    public NeoDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.setLogging(true);
        this.deviceClient.setLoggingTag("NeoDriver");
        this.deviceClient.waitForWriteCompletions(I2cWaitControl.ATOMIC);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        initializeNeoPixels();
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "Adafruit NeoDriver - I2C to NeoPixel Driver";
    }
}
