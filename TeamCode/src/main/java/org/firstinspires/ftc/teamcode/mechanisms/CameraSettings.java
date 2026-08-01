package org.firstinspires.ftc.teamcode.mechanisms;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Shared camera settings used by all TeleOp programs.
 * Persists across robot reboots using Android SharedPreferences.
 *
 * Usage:
 *   In init():  CameraSettings.load(hardwareMap);
 *               long exp = CameraSettings.exposure;  // reads saved value
 *
 *   To save:    CameraSettings.exposure = newValue;
 *               CameraSettings.save(hardwareMap);     // persists to disk
 */
public class CameraSettings {

    private static final String PREFS_NAME = "CameraSettings";
    private static final String KEY_EXPOSURE = "exposure";
    private static final String KEY_GAIN = "gain";

    /** Camera exposure in milliseconds. */
    public static long exposure = 6;

    /** Camera gain. */
    public static int gain = 250;

    /**
     * Load saved settings from disk. Call in init() of every program
     * that uses the camera. If no saved values exist, defaults are kept.
     */
    public static void load(HardwareMap hardwareMap) {
        SharedPreferences prefs = hardwareMap.appContext
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        exposure = prefs.getLong(KEY_EXPOSURE, exposure);
        gain = prefs.getInt(KEY_GAIN, gain);
    }

    /**
     * Save current settings to disk. Call after tuning
     * (e.g., from ExposureAutoTune or GP1 bumper adjustments).
     * Survives robot reboot.
     */
    public static void save(HardwareMap hardwareMap) {
        SharedPreferences prefs = hardwareMap.appContext
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        prefs.edit()
                .putLong(KEY_EXPOSURE, exposure)
                .putInt(KEY_GAIN, gain)
                .apply();
    }
}
