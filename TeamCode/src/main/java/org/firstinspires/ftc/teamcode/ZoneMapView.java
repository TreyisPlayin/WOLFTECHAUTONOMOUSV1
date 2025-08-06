package org.firstinspires.ftc.teamcode.ui;

import android.content.Context;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.teamcode.ZoneMap;

/**
 * Custom View for drawing a 144×144 grid of the FTC field.
 *  • Drag to create rectangular zones.
 *  • Tap a cell to trigger a 2×2 checkpoint placement/edit.
 */
public class ZoneMapView extends View {

    /** Callback when a zone rectangle is created */
    public interface ZoneListener {
        void onZoneCreated(int x0, int y0, int x1, int y1);
    }

    /** Callback when a cell is tapped */
    public interface CellTapListener {
        void onCellTapped(int x, int y);
    }

    private static final int CELLS = ZoneMap.SIZE;

    private Paint gridPaint;
    private Paint zonePaint;

    private ZoneMap zoneMap;                     // set via setter in Activity
    private ZoneListener zoneListener;           // ditto
    private CellTapListener tapListener;         // ditto

    // Interaction state for dragging zones
    private boolean draggingZone = false;
    private int zx0, zy0;
    private float ax, ay;

    /** Constructor when creating from code */
    public ZoneMapView(Context context) {
        super(context);
        initPaints();
    }

    /** Constructor used when inflating from XML */
    public ZoneMapView(Context context, AttributeSet attrs) {
        super(context, attrs);
        initPaints();
    }

    /** Common paint setup */
    private void initPaints() {
        gridPaint = new Paint();
        gridPaint.setColor(Color.LTGRAY);
        gridPaint.setStrokeWidth(1f);

        zonePaint = new Paint();
        zonePaint.setColor(Color.argb(80, 0, 0, 255));
    }

    /** Must be called by your Activity after inflation */
    public void setZoneMap(ZoneMap zm) {
        this.zoneMap = zm;
    }

    /** Must be called by your Activity after inflation */
    public void setZoneListener(ZoneListener l) {
        this.zoneListener = l;
    }

    /** Must be called by your Activity after inflation */
    public void setCellTapListener(CellTapListener l) {
        this.tapListener = l;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        int w = getWidth(), h = getHeight();
        float cw = w / (float)CELLS, ch = h / (float)CELLS;

        // 1) Draw grid lines
        for (int i = 0; i <= CELLS; i++) {
            canvas.drawLine(i*cw, 0, i*cw, h, gridPaint);
            canvas.drawLine(0, i*ch, w, i*ch, gridPaint);
        }

        // 2) Draw all marked zones (including 2×2 CP boxes)
        if (zoneMap != null) {
            int[][] grid = zoneMap.getGrid();
            for (int y = 0; y < CELLS; y++) {
                for (int x = 0; x < CELLS; x++) {
                    if (grid[y][x] != ZoneMap.EMPTY) {
                        canvas.drawRect(
                                x*cw, y*ch,
                                (x+1)*cw, (y+1)*ch,
                                zonePaint
                        );
                    }
                }
            }
        }

        // 3) Preview drag rectangle
        if (draggingZone) {
            canvas.drawRect(
                    zx0*cw, zy0*ch,
                    ax*cw,  ay*ch,
                    zonePaint
            );
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent e) {
        int w = getWidth(), h = getHeight();
        float xf = e.getX(), yf = e.getY();
        int cellX = (int)(xf / w * CELLS);
        int cellY = (int)(yf / h * CELLS);

        switch (e.getAction()) {
            case MotionEvent.ACTION_DOWN:
                // start dragging for a zone
                zx0 = cellX;
                zy0 = cellY;
                draggingZone = true;
                break;

            case MotionEvent.ACTION_MOVE:
                if (draggingZone) {
                    ax = xf / w * CELLS;
                    ay = yf / h * CELLS;
                }
                break;

            case MotionEvent.ACTION_UP:
                if (draggingZone) {
                    // finish zone drag
                    draggingZone = false;
                    int x1 = (int)ax;
                    int y1 = (int)ay;
                    if (zoneListener != null) {
                        zoneListener.onZoneCreated(zx0, zy0, x1, y1);
                    }
                }
                // detect quick tap for checkpoint
                if (tapListener != null
                        && e.getEventTime() - e.getDownTime() < 200) {
                    tapListener.onCellTapped(cellX, cellY);
                }
                break;
        }
        invalidate();
        return true;
    }
}
