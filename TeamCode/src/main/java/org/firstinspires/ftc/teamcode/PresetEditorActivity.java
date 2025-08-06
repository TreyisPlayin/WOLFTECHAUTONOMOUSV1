package org.firstinspires.ftc.teamcode.ui;

import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;

import androidx.appcompat.app.AppCompatActivity;

import org.firstinspires.ftc.teamcode.PresetManager;
import org.firstinspires.ftc.teamcode.ZoneMap;

public class PresetEditorActivity extends AppCompatActivity {
    private ZoneMapView  mapView;
    private PresetManager preset;
    private ZoneMap      zoneMap;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_preset_editor);

        // Initialize data models
        zoneMap = new ZoneMap();
        preset  = new PresetManager();

        // Find UI elements
        mapView = findViewById(R.id.zoneMapView);
        mapView.setZoneMap(zoneMap);
        mapView.setPresetManager(preset);

        Button btnZone = findViewById(R.id.btnZoneTool);
        Button btnCP   = findViewById(R.id.btnCP);
        Spinner spinner = findViewById(R.id.spinnerZoneType);
        EditText etAction = findViewById(R.id.etAction);
        EditText etHeading = findViewById(R.id.etHeading);
        Button btnSave = findViewById(R.id.btnSave);

        btnZone.setOnClickListener(v -> mapView.setTool(ZoneMapView.Tool.ZONE));
        btnCP  .setOnClickListener(v -> mapView.setTool(ZoneMapView.Tool.CHECKPOINT));

        spinner.setOnItemSelectedListener((parent, view, pos, id) -> {
            int type = ZoneMap.START_ZONE;
            switch(pos) {
                case 0: type = ZoneMap.START_ZONE; break;
                case 1: type = ZoneMap.SCORING_ZONE; break;
                case 2: type = ZoneMap.FORBIDDEN; break;
            }
            mapView.setZoneType(type);
        });

        btnSave.setOnClickListener(v -> {
            String action = etAction.getText().toString();
            double heading = Double.parseDouble(etHeading.getText().toString());
            mapView.setCheckpointAction(action);
            mapView.setCheckpointHeading(heading);
            // Could persist the preset or finish
            finish();
        });
    }
}
