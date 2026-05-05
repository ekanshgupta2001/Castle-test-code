package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {
    private static final float DEFAULT_GAIN = 2f;

    private final NormalizedColorSensor sensor;
    private final float[] hsv = new float[3];
    private NormalizedRGBA colors = new NormalizedRGBA();

    public ColorSensor(HardwareMap hardwareMap) {
        this(hardwareMap, "sensor_color", DEFAULT_GAIN, true);
    }

    public ColorSensor(HardwareMap hardwareMap, String name, float gain, boolean lightOn) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, name);
        sensor.setGain(gain);
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(lightOn);
        }
    }

    public void update() {
        colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsv);
    }

    public NormalizedRGBA getColors() {
        return colors;
    }

    public float getRed() {
        return colors.red;
    }

    public float getGreen() {
        return colors.green;
    }

    public float getBlue() {
        return colors.blue;
    }

    public float getAlpha() {
        return colors.alpha;
    }

    public float getHue() {
        return hsv[0];
    }

    public float getSaturation() {
        return hsv[1];
    }

    public float getValue() {
        return hsv[2];
    }

    public float[] getHsv() {
        return hsv;
    }

    public void setGain(float gain) {
        sensor.setGain(gain);
    }

    public boolean hasLight() {
        return sensor instanceof SwitchableLight;
    }

    public void setLight(boolean on) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(on);
        }
    }

    public boolean isLightOn() {
        return sensor instanceof SwitchableLight && ((SwitchableLight) sensor).isLightOn();
    }

    public boolean hasDistance() {
        return sensor instanceof DistanceSensor;
    }

    public double getDistance(DistanceUnit unit) {
        return sensor instanceof DistanceSensor ? ((DistanceSensor) sensor).getDistance(unit) : Double.NaN;
    }
}
