package org.firstinspires.ftc.teamcode;

import java.util.*;

/**
 * A* pathfinder on ZoneMap plus "string-pull" smoothing (line-of-sight corner cutting).
 * If you draw "no-go lines" as 1-cell-wide FORBIDDEN stripes on the map, A* will avoid them.
 */
public class PathPlanner {
    private final ZoneMap map;

    public PathPlanner(ZoneMap map) { this.map = map; }

    /** A* grid search (4-neighbor). */
    public List<Position> aStar(Position start, Position goal) {
        int sx=(int)start.getX(), sy=(int)start.getY();
        int gx=(int)goal.getX(),  gy=(int)goal.getY();

        PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingDouble(n->n.f));
        Node[][] nodes = new Node[ZoneMap.SIZE][ZoneMap.SIZE];
        boolean[][] closed = new boolean[ZoneMap.SIZE][ZoneMap.SIZE];

        Node s = new Node(sx,sy,0,heur(sx,sy,gx,gy),null);
        nodes[sy][sx]=s; open.add(s);

        int[][] dirs={{1,0},{-1,0},{0,1},{0,-1}};
        while(!open.isEmpty()){
            Node c=open.poll();
            if (c.x==gx && c.y==gy) return smooth(reconstruct(c));
            closed[c.y][c.x]=true;
            for(int[] d:dirs){
                int nx=c.x+d[0], ny=c.y+d[1];
                if (nx<0||ny<0||nx>=ZoneMap.SIZE||ny>=ZoneMap.SIZE) continue;
                if (closed[ny][nx] || map.getCell(nx,ny)!=ZoneMap.EMPTY) continue;
                double ng=c.g+1;
                Node nb = nodes[ny][nx];
                if (nb==null){ nb=new Node(nx,ny,ng,heur(nx,ny,gx,gy),c); nodes[ny][nx]=nb; open.add(nb); }
                else if (ng<nb.g){ nb.g=ng; nb.f=ng+nb.h; nb.parent=c; }
            }
        }
        return Collections.emptyList();
    }

    private double heur(int x,int y,int gx,int gy){ return Math.abs(gx-x)+Math.abs(gy-y); }

    private List<Position> reconstruct(Node n){
        LinkedList<Position> path=new LinkedList<>();
        while(n!=null){ path.addFirst(new Position(n.x,n.y,0)); n=n.parent; }
        return path;
    }

    /** "String-pull" smoothing: keep only points where straight line would hit an obstacle. */
    public List<Position> smooth(List<Position> raw){
        if (raw.isEmpty()) return raw;
        List<Position> out=new ArrayList<>();
        int i=0;
        out.add(raw.get(0));
        while(i<raw.size()-1){
            int j=raw.size()-1;
            // Walk backward from the end until we find first that is *not* visible
            for(; j>i+1; j--){
                if (lineOfSight(raw.get(i), raw.get(j))) break;
            }
            out.add(raw.get(j));
            i=j;
        }
        return out;
    }

    /** Bresenham-like check along the segment for any FORBIDDEN cell. */
    private boolean lineOfSight(Position a, Position b){
        int x0=(int)a.getX(), y0=(int)a.getY(), x1=(int)b.getX(), y1=(int)b.getY();
        int dx=Math.abs(x1-x0), dy=Math.abs(y1-y0);
        int sx=x0<x1?1:-1, sy=y0<y1?1:-1, err=dx-dy;
        int x=x0,y=y0;
        while(true){
            if (map.getCell(x,y)!=ZoneMap.EMPTY) return false;
            if (x==x1 && y==y1) break;
            int e2=2*err;
            if (e2> -dy){ err-=dy; x+=sx; }
            if (e2<  dx){ err+=dx; y+=sy; }
        }
        return true;
    }

    private static class Node{ int x,y; double g,h,f; Node parent;
        Node(int x,int y,double g,double h,Node p){this.x=x;this.y=y;this.g=g;this.h=h;this.f=g+h;this.parent=p;}
    }
}
