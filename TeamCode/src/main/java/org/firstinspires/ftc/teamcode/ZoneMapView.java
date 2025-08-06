package org.firstinspires.ftc.teamcode.ui;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.MotionEvent;
import android.view.View;
import org.firstinspires.ftc.teamcode.ZoneMap;
import org.firstinspires.ftc.teamcode.Checkpoint;
import org.firstinspires.ftc.teamcode.PresetManager;

import java.util.List;

/**
 * ZoneMapView renders a 144×144 grid and allows the user to:
 *  - Tap-and-drag to draw rectangular zones (of the currently selected type)
 *  - Tap to add checkpoints (with current heading & action)
 */
public class ZoneMapView extends View {
    private static final int CELLS = ZoneMap.SIZE;
    private final Paint gridPaint = new Paint();
    private final Paint zonePaint = new Paint();
    private final Paint cpPaint = new Paint();

    private final ZoneMap zoneMap;
    private final PresetManager preset;

    // Current tool state
    public enum Tool { ZONE, CHECKPOINT }
    private Tool currentTool = Tool.ZONE;
    private int currentZoneType = ZoneMap.START_ZONE;
    private String currentAction = "pickup";
    private double currentHeading = 0;

    // Touch-drag coords for zone drawing
    private int dragStartX, dragStartY;
    private boolean dragging = false;

    public ZoneMapView(Context ctx, ZoneMap zoneMap, PresetManager preset) {
        super(ctx);
        this.zoneMap = zoneMap;
        this.preset = preset;

        gridPaint.setColor(Color.LTGRAY);
        gridPaint.setStrokeWidth(1);

        zonePaint.setColor(Color.argb(100, 0, 0, 255));  // semi-transparent blue
        cpPaint.setColor(Color.RED);
        cpPaint.setTextSize(24);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        int w = getWidth(), h = getHeight();
        float cellW = w / (float)CELLS, cellH = h / (float)CELLS;

        // 1) Draw grid
        for (int i = 0; i <= CELLS; i++) {
            canvas.drawLine(i*cellW, 0, i*cellW, h, gridPaint);
            canvas.drawLine(0, i*cellH, w, i*cellH, gridPaint);
        }

        // 2) Draw zones
        int[][] grid = zoneMap.getGrid();
        for (int y = 0; y < CELLS; y++) {
            for (int x = 0; x < CELLS; x++) {
                int type = grid[y][x];
                if (type != ZoneMap.EMPTY) {
                    zonePaint.setColor(getColorForType(type));
                    canvas.drawRect(x*cellW, y*cellH,
                            (x+1)*cellW, (y+1)*cellH,
                            zonePaint);
                }
            }
        }

        // 3) Draw checkpoints
        List<Checkpoint> cps = preset.getCheckpoints();
        for (Checkpoint cp : cps) {
            float cx = (float)(cp.x / ZoneMap.SIZE * w);
            float cy = (float)(cp.y / ZoneMap.SIZE * h);
            canvas.drawCircle(cx, cy, Math.min(cellW,cellH)*0.4f, cpPaint);
            canvas.drawText(cp.action, cx + 4, cy - 4, cpPaint);
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent e) {
        float xf = e.getX(), yf = e.getY();
        int x = (int)(xf / getWidth() * CELLS);
        int y = (int)(yf / getHeight()* CELLS);

        switch (e.getAction()) {
            case MotionEvent.ACTION_DOWN:
                if (currentTool == Tool.ZONE) {
                    dragStartX = x; dragStartY = y; dragging = true;
                } else {
                    zoneMap.markCheckpoint(x, y);
                    preset.addCheckpoint(x, y, currentHeading, currentAction);
                }
                break;

            case MotionEvent.ACTION_MOVE:
                // Optional: show rubber-band preview
                break;

            case MotionEvent.ACTION_UP:
                if (currentTool == Tool.ZONE && dragging) {
                    int x0 = Math.min(dragStartX, x);
                    int y0 = Math.min(dragStartY, y);
                    int x1 = Math.max(dragStartX, x);
                    int y1 = Math.max(dragStartY, y);
                    zoneMap.markZone(x0, y0, x1, y1, currentZoneType);
                    dragging = false;
                }
                break;
        }
        invalidate();
        return true;
    }

    /** Change between zone-draw or checkpoint mode */
    public void setTool(Tool t) { currentTool = t; }

    /** Choose which zone type to draw (e.g. START_ZONE, SCORING_ZONE) */
    public void setZoneType(int zoneType) { currentZoneType = zoneType; }

    /** When adding checkpoints, set their action label */
    public void setCheckpointAction(String action) { currentAction = action; }

    /** When adding checkpoints, set their desired heading */
    public void setCheckpointHeading(double h) { currentHeading = h; }

    /** Utility to pick a consistent color for each zone type */
    private int getColorForType(int type) {
        switch(type) {
            case ZoneMap.START_ZONE:   return Color.argb(100,0,255,0);
            case ZoneMap.SCORING_ZONE: return Color.argb(100,255,0,0);
            case ZoneMap.FORBIDDEN:    return Color.argb(100,128,0,128);
            default:                   return Color.argb(100,0,0,255);
        }
    }
}
