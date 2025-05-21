package org.firstinspires.ftc.teamcode;

import java.util.*;

/**
 * PathPlanner.java
 *
 * A* pathfinding on a grid; avoids obstacles (cells<0).
 * Then follows the path by commanding the robot.
 */
public class PathPlanner {
    /**
     * Compute path from start→goal.
     */
    public List<Position> aStar(int[][] grid, Position start, Position goal) {
        int w = grid.length, h = grid[0].length;
        boolean[][] closed = new boolean[w][h];
        Node[][] nodes = new Node[w][h];
        PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingDouble(n->n.f));

        open.add(new Node(start,0,heuristic(start,goal),null));
        int[][] dirs = {{1,0},{-1,0},{0,1},{0,-1}};

        while (!open.isEmpty()) {
            Node cur = open.poll();
            if (cur.pos.equals(goal)) return reconstruct(cur);
            closed[cur.pos.x][cur.pos.y] = true;
            for (int[] d: dirs) {
                int nx=cur.pos.x+d[0], ny=cur.pos.y+d[1];
                if (nx<0||ny<0||nx>=w||ny>=h) continue;
                if (closed[nx][ny]||grid[nx][ny]<0) continue;
                double g = cur.g+1;
                if (nodes[nx][ny]==null||g<nodes[nx][ny].g) {
                    nodes[nx][ny] = new Node(new Position(nx,ny),g,heuristic(new Position(nx,ny),goal),cur);
                    open.add(nodes[nx][ny]);
                }
            }
        }
        return Collections.emptyList();
    }

    /** Drive the robot along the path. */
    public void followPath(List<Position> path, HardwareConfig robot, Odometry odo) {
        for (Position p: path) {
            robot.driveTo(p.x, p.y, odo);
        }
    }

    private double heuristic(Position a, Position b) {
        return Math.abs(a.x-b.x)+Math.abs(a.y-b.y);
    }

    private List<Position> reconstruct(Node n) {
        LinkedList<Position> path = new LinkedList<>();
        for (Node cur=n; cur!=null; cur=cur.parent) path.addFirst(cur.pos);
        return path;
    }

    private static class Node {
        Position pos; double g,f; Node parent;
        Node(Position p,double g,double h,Node par){pos=p;this.g=g;this.f=g+h;parent=par;}
    }
}
