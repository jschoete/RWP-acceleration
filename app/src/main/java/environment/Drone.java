package environment;

import io.jbotsim.core.Node;
import io.jbotsim.core.Point;
import io.jbotsim.ui.icons.Icons;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by A. Casteigts before 16/02/17, modified by J. Schoeters afterwards.
 * This class extends the VectorNode class, instantiating a representation of a drone.
 */
public class Drone extends VectorNode {
    Point nextPoint;
    Point lastPoint;
    public List points_visited;
    List<Point> trajectory_locations = new ArrayList<Point>();
    int iterator_trajectory_locations = 0;
    int delay = 0;
    List visit_order;

    public Drone() {
        super();
        setSensingRange(0);
        points_visited = new ArrayList();
        setIcon(Icons.DRONE);
        setIconSize(40);
    }

    @Override
    public void onStart() {
        super.onStart();
        onPointReached(this.getLocation());
    }

    @Override
    public void onClock() {
        super.onClock();
        for (int i = 0; i < this.getTopology().getNodes().size(); i++) {
            Node n = this.getTopology().getNodes().get(i);
            if (n instanceof Target && passedBy(this.lastPoint, this.nextPoint, n.getLocation())) {
                Target t = (Target) n;
                if (t.isVisible()) {
                    t.makeInvisible();
                    makeNextTargetVisible(t);
                }
            }
        }
    }

    private void makeNextTargetVisible(Target t) {
        for (int i=0; i<visit_order.size()-2; i++){
            if (t.getLocation().equals(visit_order.get(i))){
                for (Node n : this.getTopology().getNodes()){
                    if (n instanceof Target && n.getLocation().equals(visit_order.get(i+2))){
                        Target t2 = (Target) n;
                        t2.makeVisible();
                    }
                }
            }
        }
    }

    /**
     * checks if checkpoint lies between l0 and l1
     */
    public static boolean passedBy(io.jbotsim.core.Point l0, io.jbotsim.core.Point l1, io.jbotsim.core.Point checkpoint) {

        if (distanceToLine(checkpoint, l0, l1) <= 1)
            return true;
        return false;
    }

    public static double distanceToLine(io.jbotsim.core.Point p, io.jbotsim.core.Point p1, io.jbotsim.core.Point p2) {

        double x = p.getX();
        double y = p.getY();
        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();

        double A = x - x1;
        double B = y - y1;
        double C = x2 - x1;
        double D = y2 - y1;

        double dot = A * C + B * D;
        double len_sq = C * C + D *D;
        double param = -1;
        if (len_sq != 0) // in case of length line being 0
            param = dot / len_sq;

        double xx, yy;

        if (param < 0){
            xx = x1;
            yy = y1;
        }
        else if (param > 1){
            xx = x2;
            yy = y2;
        }
        else{
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        double dx = x - xx;
        double dy = y - yy;
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public void onPointReached(Point point) {
        if (delay == 0) {
            points_visited.add(lastPoint);
            lastPoint = this.getLocation();
            delay++;
            nextPoint = this.getLocation();
            travelTo(nextPoint);
        }
        else{
            if (delay == 1) {
                trajectory_locations = environment.Main.trajectory_locations;
                visit_order = environment.Main.visit_order;
                delay++;
            }
            if (iterator_trajectory_locations < trajectory_locations.size()) {
                nextPoint = trajectory_locations.get(iterator_trajectory_locations);
                iterator_trajectory_locations++;
            }
            points_visited.add(lastPoint);
            lastPoint = this.getLocation();
            travelTo(nextPoint);
        }
    }
}