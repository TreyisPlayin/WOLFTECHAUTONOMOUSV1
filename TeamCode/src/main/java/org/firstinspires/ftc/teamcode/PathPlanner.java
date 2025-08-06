package org.firstinspires.ftc.teamcode;

import java.util.*;

/**
 * A* pathfinding on the 144×144 grid. Avoids any non-EMPTY cell.
 */
public class PathPlanner {
    private final ZoneMap zoneMap;

    public PathPlanner(ZoneMap zoneMap) {
        this.zoneMap = zoneMap;
    }

    public List<Position> aStar(Position start, Position goal) {
        int sx = (int)start.getX(), sy = (int)start.getY();
        int gx = (int)goal.getX(), gy = (int)goal.getY();

        boolean[][] closed = new boolean[ZoneMap.SIZE][ZoneMap.SIZE];
        Node[][] nodes     = new Node[ZoneMap.SIZE][ZoneMap.SIZE];
        PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingDouble(n->n.f));

        Node s = new Node(sx, sy, 0, h(sx,sy,gx,gy), null);
        nodes[sy][sx] = s;
        open.add(s);

        int[][] dirs = {{1,0},{-1,0},{0,1},{0,-1}};
        while (!open.isEmpty()) {
            Node cur = open.poll();
            if (cur.x==gx && cur.y==gy) return buildPath(cur);

            closed[cur.y][cur.x] = true;
            for (int[] d : dirs) {
                int nx=cur.x+d[0], ny=cur.y+d[1];
                if (nx<0||ny<0||nx>=ZoneMap.SIZE||ny>=ZoneMap.SIZE) continue;
                if (closed[ny][nx] || zoneMap.getCell(nx,ny)!=ZoneMap.EMPTY) continue;

                double ng = cur.g + 1;
                Node nb = nodes[ny][nx];
                if (nb==null) {
                    nb = new Node(nx, ny, ng, h(nx,ny,gx,gy), cur);
                    nodes[ny][nx] = nb;
                    open.add(nb);
                } else if (ng < nb.g) {
                    nb.g = ng;
                    nb.f = ng + nb.h;
                    nb.parent = cur;
                }
            }
        }
        return Collections.emptyList();
    }

    private double h(int x,int y,int gx,int gy) {
        return Math.abs(gx-x) + Math.abs(gy-y);
    }

    private List<Position> buildPath(Node n) {
        LinkedList<Position> path = new LinkedList<>();
        while (n != null) {
            path.addFirst(new Position(n.x, n.y, 0));
            n = n.parent;
        }
        return path;
    }

    private static class Node {
        int x,y; double g,h,f; Node parent;
        Node(int x,int y,double g,double h,Node p) {
            this.x = x; this.y=y; this.g=g; this.h=h; this.f=g+h; this.parent=p;
        }
    }

    /** Stub: drive along path. Replace with your drivetrain logic. */
    public void followPath(List<Position> path, HardwareConfig robot) {
        for (Position wp : path) {
            // e.g. driveTo(wp.getX(), wp.getY());
        }
    }
}
