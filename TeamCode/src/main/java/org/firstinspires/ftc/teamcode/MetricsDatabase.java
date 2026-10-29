package org.firstinspires.ftc.teamcode;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.DatabaseUtils;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Single shared SQLite database for recording robot metrics.
 * Use MetricsDatabase.getInstance() to access it from any OpMode.
 */
public class MetricsDatabase extends SQLiteOpenHelper implements IPerformanceLog {

    private static final String DATABASE_NAME = "metrics.db";
    private static final int DATABASE_VERSION = 1;

    // Metrics table: one row per recorded value
    public static final String TABLE_METRICS = "metrics";
    public static final String COLUMN_ID = "_id";
    public static final String COLUMN_TIMESTAMP = "timestamp";  // milliseconds since epoch
    public static final String COLUMN_OP_MODE = "op_mode";
    public static final String COLUMN_NAME = "name";
    public static final String COLUMN_VALUE = "value";

    private static final String CREATE_TABLE_METRICS =
            "CREATE TABLE " + TABLE_METRICS + " ("
                    + COLUMN_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, "
                    + COLUMN_TIMESTAMP + " INTEGER NOT NULL, "
                    + COLUMN_OP_MODE + " TEXT NOT NULL, "
                    + COLUMN_NAME + " TEXT NOT NULL, "
                    + COLUMN_VALUE + " REAL NOT NULL"
                    + ")";

    // volatile so the double-checked lock in getInstance() is safe
    private static volatile MetricsDatabase instance;

    @Override
    public void logMetric(String opMode, String name, double value) {
        insertMetric(opMode, name, value);
    }

    private MetricsDatabase(Context context) {
        // Application context avoids leaking an Activity across OpMode runs
        super(context.getApplicationContext(), DATABASE_NAME, null, DATABASE_VERSION);
    }

    public static MetricsDatabase getInstance() {
        if (instance == null) {
            synchronized (MetricsDatabase.class) {
                if (instance == null) {
                    instance = new MetricsDatabase(AppUtil.getInstance().getApplication());
                }
            }
        }
        return instance;
    }

    /**
     * Records one metric value, timestamped with the current time.
     *
     * @return the new row's ID, or -1 if the insert failed
     */
    public long insertMetric(String opMode, String name, double value) {
        ContentValues row = new ContentValues();
        row.put(COLUMN_TIMESTAMP, System.currentTimeMillis());
        row.put(COLUMN_OP_MODE, opMode);
        row.put(COLUMN_NAME, name);
        row.put(COLUMN_VALUE, value);
        return getWritableDatabase().insert(TABLE_METRICS, null, row);
    }

    /**
     * Returns every recorded metric, oldest first.
     */
    public List<Metric> getAllMetrics() {
        return queryMetrics(null, null);
    }

    /**
     * Returns every metric recorded by the given OpMode, oldest first.
     */
    public List<Metric> getMetricsByOpMode(String opMode) {
        return queryMetrics(COLUMN_OP_MODE + " = ?", new String[] {opMode});
    }

    /**
     * Returns how many metrics are stored, without loading them.
     */
    public long getMetricCount() {
        return DatabaseUtils.queryNumEntries(getReadableDatabase(), TABLE_METRICS);
    }

    /**
     * Runs a query on the metrics table and converts each row to a Metric.
     * A null selection returns every row.
     */
    private List<Metric> queryMetrics(String selection, String[] selectionArgs) {
        List<Metric> metrics = new ArrayList<>();

        try (Cursor cursor = getReadableDatabase().query(
                TABLE_METRICS, null, selection, selectionArgs, null, null, COLUMN_ID + " ASC")) {
            int idIndex = cursor.getColumnIndexOrThrow(COLUMN_ID);
            int timestampIndex = cursor.getColumnIndexOrThrow(COLUMN_TIMESTAMP);
            int opModeIndex = cursor.getColumnIndexOrThrow(COLUMN_OP_MODE);
            int nameIndex = cursor.getColumnIndexOrThrow(COLUMN_NAME);
            int valueIndex = cursor.getColumnIndexOrThrow(COLUMN_VALUE);

            while (cursor.moveToNext()) {
                metrics.add(new Metric(
                        cursor.getLong(idIndex),
                        cursor.getLong(timestampIndex),
                        cursor.getString(opModeIndex),
                        cursor.getString(nameIndex),
                        cursor.getDouble(valueIndex)));
            }
        }

        return metrics;
    }

    /**
     * Deletes every recorded metric.
     *
     * @return the number of rows deleted
     */
    public int clearAllMetrics() {
        // "1" as the where clause makes delete() return the row count (null returns 0)
        return getWritableDatabase().delete(TABLE_METRICS, "1", null);
    }

    /**
     * Writes every recorded metric to a CSV file that opens in a spreadsheet.
     * Overwrites the file if it already exists.
     *
     * @return the number of metrics written
     */
    public int exportToCsv(File file) throws IOException {
        List<Metric> metrics = getAllMetrics();
        SimpleDateFormat timeFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US);

        File folder = file.getParentFile();
        if (folder != null && !folder.isDirectory() && !folder.mkdirs()) {
            throw new IOException("Could not create folder " + folder);
        }

        try (BufferedWriter out = new BufferedWriter(new FileWriter(file))) {
            out.write("id,timestamp_ms,time,op_mode,name,value\n");
            for (Metric m : metrics) {
                out.write(m.id + "," + m.timestamp + "," + timeFormat.format(new Date(m.timestamp))
                        + "," + csvQuote(m.opMode) + "," + csvQuote(m.name) + "," + m.value + "\n");
            }
        }

        return metrics.size();
    }

    // Quotes a CSV text field so commas or quotes inside it don't split the column
    private static String csvQuote(String text) {
        return "\"" + text.replace("\"", "\"\"") + "\"";
    }

    @Override
    public void onCreate(SQLiteDatabase db) {
        db.execSQL(CREATE_TABLE_METRICS);
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        // Migrate schema here when DATABASE_VERSION is bumped
    }

    /**
     * One row of the metrics table.
     */
    public static class Metric {
        public final long id;
        public final long timestamp;  // milliseconds since epoch
        public final String opMode;
        public final String name;
        public final double value;

        public Metric(long id, long timestamp, String opMode, String name, double value) {
            this.id = id;
            this.timestamp = timestamp;
            this.opMode = opMode;
            this.name = name;
            this.value = value;
        }
    }
}
