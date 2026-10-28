package core;

import android.os.Environment;

import java.io.File;

/** Resolves the folder used for Apex Pathing's persisted tuning data. */
public final class ApexStorage {
    private ApexStorage() { }

    public static File getDirectory() {
        String override = System.getProperty("apexpathing.storageDirectory");
        if (override != null && !override.trim().isEmpty()) { return new File(override); }
        return new File(Environment.getExternalStorageDirectory(), "FIRST/ApexPathing");
    }

    public static File getConstantsFile() {
        return new File(getDirectory(), "constants.json");
    }

    public static File getLogDirectory() { return new File(getDirectory(), "logs"); }
}
