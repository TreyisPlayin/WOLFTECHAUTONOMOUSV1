package org.firstinspires.ftc.teamcode.ui;

import org.firstinspires.ftc.teamcode.R;
import android.app.AlertDialog;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import org.firstinspires.ftc.teamcode.ZoneMap;
import org.firstinspires.ftc.teamcode.PresetManager;
import org.firstinspires.ftc.teamcode.Checkpoint;
import org.firstinspires.ftc.teamcode.ui.ZoneMapView;
import java.util.Arrays;

/**
 * PresetEditorActivity
 *
 * Single‐screen UI to draw zones and create/edit 2×2 checkpoint boxes.
 * Tap a cell to add or edit a checkpoint’s action.
 */
public class PresetEditorActivity extends AppCompatActivity {
    private ZoneMap       zoneMap;
    private PresetManager preset;
    private ZoneMapView   mapView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_preset_editor);

        // 1) Initialize models
        zoneMap = new ZoneMap();
        preset  = new PresetManager();

        // 2) Wire up custom view
        mapView = findViewById(R.id.zoneMapView);
        mapView.setZoneMap(zoneMap);
        mapView.setZoneListener(this::onZoneCreated);
        mapView.setCellTapListener(this::onCellTapped);
    }

    /**
     * Received when the user drags out a rectangular zone.
     */
    private void onZoneCreated(int x0, int y0, int x1, int y1) {
        String[] zoneTypes = { "START_ZONE", "SCORING_ZONE", "FORBIDDEN" };
        new AlertDialog.Builder(this)
                .setTitle("Select Zone Type")
                .setItems(zoneTypes, (dialog, which) -> {
                    int type;
                    switch (which) {
                        case 0: type = ZoneMap.START_ZONE;   break;
                        case 1: type = ZoneMap.SCORING_ZONE; break;
                        case 2: type = ZoneMap.FORBIDDEN;    break;
                        default: type = ZoneMap.EMPTY;       break;
                    }
                    zoneMap.markZone(x0, y0, x1, y1, type);
                    mapView.invalidate();
                })
                .show();
    }

    /**
     * Received when the user taps a cell (to add or edit a checkpoint).
     */
    private void onCellTapped(int cellX, int cellY) {
        // Check if a checkpoint already exists at this cell
        int idx = preset.findCheckpointIndexAt(cellX, cellY);
        double initHeading = 0;
        String initAction  = "pickup";

        if (idx >= 0) {
            Checkpoint old = preset.getCheckpoints().get(idx);
            initHeading = old.heading;
            initAction  = old.action;
        }

        showCheckpointDialog(cellX, cellY, initHeading, initAction, idx);
    }

    /**
     * Shows a dialog to choose or edit a checkpoint’s action.
     * If editIndex >= 0, updates existing; otherwise adds new.
     */
    private void showCheckpointDialog(int cellX,
                                      int cellY,
                                      double initHeading,
                                      String initAction,
                                      int editIndex) {
        String[] actions = { "start", "pickup", "score", "human", "stop:5" };
        final String[] selected = { initAction };
        int checked = Arrays.asList(actions).indexOf(initAction);
        if (checked < 0) checked = 0;

        new AlertDialog.Builder(this)
                .setTitle(editIndex >= 0 ? "Edit Checkpoint" : "New Checkpoint")
                .setSingleChoiceItems(actions, checked, (dialog, which) -> {
                    selected[0] = actions[which];
                })
                .setPositiveButton("OK", (dialog, which) -> {
                    // Mark a 2×2 box on the grid
                    zoneMap.markZone(cellX, cellY, cellX + 1, cellY + 1, ZoneMap.CHECKPOINT);

                    // Add or update the preset
                    if (editIndex >= 0) {
                        preset.updateCheckpoint(
                                editIndex,
                                cellX,
                                cellY,
                                initHeading,
                                selected[0]
                        );
                    } else {
                        preset.addCheckpoint(
                                cellX,
                                cellY,
                                initHeading,
                                selected[0]
                        );
                    }

                    mapView.invalidate();
                })
                .setNegativeButton("Cancel", null)
                .show();
    }
}
