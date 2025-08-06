package org.firstinspires.ftc.teamcode;

import java.util.*;

/**
 * PathPlanner runs A* on the 144×144 grid from ZoneMap to generate waypoints.
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
        Node[][] nodes = new Node[ZoneMap.SIZE][ZoneMap.SIZE];
        PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingDouble(n->n.f));

        Node startNode = new Node(sx, sy, 0, heuristic(sx,sy,gx,gy), null);
        nodes[sy][sx] = startNode;
        open.add(startNode);

        int[][] dirs = {{1,0},{-1,0},{0,1},{0,-1}};

        while (!open.isEmpty()) {
            Node cur = open.poll();
            if (cur.x==gx && cur.y==gy) return reconstruct(cur);

            closed[cur.y][cur.x] = true;
            for (int[] d : dirs) {
                int nx=cur.x+d[0], ny=cur.y+d[1];
                if (nx<0||ny<0||nx>=ZoneMap.SIZE||ny>=ZoneMap.SIZE) continue;
                if (closed[ny][nx] || zoneMap.getCell(nx,ny)!=ZoneMap.EMPTY) continue;

                double ng = cur.g + 1;
                Node neighbor = nodes[ny][nx];
                if (neighbor==null) {
                    neighbor = new Node(nx,ny,ng,heuristic(nx,ny,gx,gy),cur);
                    nodes[ny][nx] = neighbor;
                    open.add(neighbor);
                } else if (ng < neighbor.g) {
                    neighbor.g = ng;
                    neighbor.f = ng + neighbor.h;
                    neighbor.parent = cur;
                }
            }
        }
        return Collections.emptyList();
    }

    private double heuristic(int x, int y, int gx, int gy) {
        return Math.abs(gx-x)+Math.abs(gy-y);
    }

    private List<Position> reconstruct(Node node) {
        LinkedList<Position> path = new LinkedList<>();
        while (node!=null) {
            path.addFirst(new Position(node.x, node.y, 0));
            node = node.parent;
        }
        return path;
    }

    private static class Node {
        int x,y;
        double g,h,f;
        Node parent;
        Node(int x,int y,double g,double h,Node p) {
            this.x=x;this.y=y;this.g=g;this.h=h;this.f=g+h;this.parent=p;
        }
    }

    /** Drives the robot along the path; replace with actual drivetrain logic. */
    public void followPath(List<Position> path, HardwareConfig robot) {
        for (Position p : path) {
            // robot.driveTo(p.x, p.y);
        }
    }
}
