package environment;

import io.jbotsim.core.Node;
import io.jbotsim.core.Point;

/**
 * Created by A. Casteigts before 16/02/17, modified by J. Schoeters afterwards.
 * This class represents an entity with the "Racetrack" acceleration constraints (such as a Drone).
 */
public abstract class VectorNode extends Node {
    public static final double DEVIATION = 20.0;
    public Point vector = new Point(0, 0);
    private Point nextPoint;
    private double speed;

    public VectorNode() {
        super();
    }

    public void travelTo(Point requestedTarget) {
        Point projection = add(getLocation(), vector);
        Point requestedChange = sub(requestedTarget, projection);
        if (len(requestedChange) < DEVIATION) {
            nextPoint = requestedTarget;
        }else{
            Point allowedChange = mul(requestedChange, DEVIATION / len(requestedChange));
            nextPoint = add(projection, allowedChange);
        }
        if(!nextPoint.equals(this.getLocation()))
            setDirection(nextPoint);
        vector = sub(nextPoint, getLocation());
        speed = len(vector)/12.0; //smooth rendering
    }

    @Override
    public void onClock() {
        if (nextPoint != null){
            if (distance(nextPoint) > speed) {
                move(speed);
            }else{
                setLocation(nextPoint);
                nextPoint = null;
                onPointReached(getLocation());
            }
        }
    }

    /* Called when the next point is reached (to be overridden) */
    public abstract void onPointReached(Point point);

    /* Vector operations */
    private static double len(Point p) {
        return (new Point(0,0)).distance(p);
    }
    private static Point mul(Point p, double a) {
        return (new Point(p.getX()*a, p.getY()*a));
    }
    private static Point sub(Point p1, Point p2) {
        return (new Point(p1.getX()-p2.getX(), p1.getY()-p2.getY()));
    }
    private static Point add(Point p1, Point p2) {
        return (new Point(p1.getX()+p2.getX(), p1.getY()+p2.getY()));
    }
}
