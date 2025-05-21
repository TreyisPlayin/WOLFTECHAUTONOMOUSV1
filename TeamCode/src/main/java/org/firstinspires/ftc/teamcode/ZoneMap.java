package org.firstinspires.ftc.teamcode;

/**
 * ZoneMap.java
 *
 * Holds your 24×24 alliance maps.
 * You can edit the RED_ZONE_MAP and BLUE_ZONE_MAP arrays directly.
 */
public class ZoneMap {
    // 24×24 grid for Red alliance
    private static final int[][] RED_ZONE_MAP   = new int[24][24];
    // 24×24 grid for Blue alliance
    private static final int[][] BLUE_ZONE_MAP  = new int[24][24];

    // Zone constants (match FieldMapper expectations)
    public static final int EMPTY           = 0;
    public static final int OBSTACLE        = -1;
    public static final int START_ZONE      = 1;
    public static final int COLLECTION_ZONE = 2;
    public static final int SCORING_ZONE    = 3;
    public static final int HUMAN_PLAYER    = 4;
    public static final int FORBIDDEN       = 5;

    /** Getter so you can fill the array manually in RED_ZONE_MAP. */
    public static int[][] getRedZoneMap()  {
        return RED_ZONE_MAP;
    }
    /** Getter so you can fill the array manually in BLUE_ZONE_MAP. */
    public static int[][] getBlueZoneMap() {
        return BLUE_ZONE_MAP;
    }
}
