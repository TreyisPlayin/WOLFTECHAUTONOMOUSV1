package org.firstinspires.ftc.teamcode;

/**
 * Simple (x,y) coordinate on a grid or field.
 */
public class Position {
    public int x, y;
    public Position(int x, int y) {
        this.x = x;
        this.y = y;
    }
    @Override public boolean equals(Object o) {
        if (!(o instanceof Position)) return false;
        Position p = (Position)o;
        return this.x == p.x && this.y == p.y;
    }
}
