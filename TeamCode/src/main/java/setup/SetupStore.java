package setup;

import java.io.*;
import java.nio.charset.StandardCharsets;
import org.json.JSONObject;

/** Backup-backed profile replacement. A canceled or failed wizard never replaces the last setup. */
public final class SetupStore {
    private final File file;
    public SetupStore(File directory) { file = new File(directory, "robot-setup.json"); }
    public File getFile() { return file; }

    public RobotProfile load() throws IOException {
        File source = file.exists() ? file : new File(file.getPath() + ".bak");
        return read(source);
    }

    public RobotProfile loadPrevious() throws IOException { return read(new File(file.getPath() + ".bak")); }

    private RobotProfile read(File source) throws IOException {
        if (!source.exists()) { return null; }
        try (Reader reader = new InputStreamReader(new FileInputStream(source), StandardCharsets.UTF_8)) {
            StringBuilder text = new StringBuilder();
            char[] buffer = new char[4096];
            int read;
            while ((read = reader.read(buffer)) != -1) { text.append(buffer, 0, read); }
            return RobotProfile.fromJson(new JSONObject(text.toString()));
        } catch (Exception e) { throw new IOException("Saved robot setup could not be read; the existing file was kept", e); }
    }

    public synchronized void save(RobotProfile profile) throws IOException {
        profile.validate();
        if (!file.getParentFile().isDirectory() && !file.getParentFile().mkdirs()) {
            throw new IOException("Could not create the robot setup folder");
        }
        File pending = new File(file.getPath() + ".tmp"), backup = new File(file.getPath() + ".bak");
        try (FileOutputStream output = new FileOutputStream(pending)) {
            output.write(profile.toJson().toString().getBytes(StandardCharsets.UTF_8));
            output.getFD().sync();
        }
        if (file.exists()) {
            boolean validPrimary;
            try { read(file); validPrimary = true; } catch (IOException invalid) { validPrimary = false; }
            if (!validPrimary && backup.isFile()) {
                File invalid = new File(file.getPath() + ".invalid-" + System.currentTimeMillis());
                if (!file.renameTo(invalid)) { throw new IOException("Could not retain the unreadable setup"); }
            } else {
            if (backup.exists() && !backup.delete()) { throw new IOException("Could not retain the previous setup"); }
            if (!file.renameTo(backup)) { throw new IOException("Could not back up the previous setup"); }
            }
        }
        if (!pending.renameTo(file)) {
            if (backup.exists() && !backup.renameTo(file)) { throw new IOException("Save interrupted; previous setup is in robot-setup.json.bak"); }
            throw new IOException("Could not save robot setup; previous setup was restored");
        }
    }
}
