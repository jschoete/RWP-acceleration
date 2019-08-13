package algorithm;

import io.jbotsim.core.Point;

import java.util.Set;

/**
 * Created by J. Schoeters on 29/03/17.
 * A vertex in a phase space represents a precise configuration in which the drone can find itself.
 * The phase space itself is a directed network of phase space vertices.
 * A phase space vertex has an arc to another phase space vertex if it can change to this configuration in one time unit.
 */
public class PhaseSpaceVertex {
    public short x;
    public short y;
    public short dx;
    public short dy;
    public PhaseSpaceVertex[] nextTransitions;
    public PhaseSpaceVertex previousTransition;

    public PhaseSpaceVertex(int x, int y, int dx, int dy){
        this.x = (short) x;
        this.y = (short) y;
        this.dx = (short) dx;
        this.dy = (short) dy;
    }

    public PhaseSpaceVertex(Point location, Point speed){
        this.x = (short) location.getX();
        this.y = (short) location.getY();
        this.dx = (short) speed.getX();
        this.dy = (short) speed.getY();
    }

    public Point getLocation(){
        return new Point(x, y);
    }

    public Point getSpeed(){
        return new Point(dx, dy);
    }

    public void setLocation(int x, int y){
        this.x = (short) x;
        this.y = (short) y;
    }

    public void setLocation(Point location){
        this.x = (short) location.getX();
        this.y = (short) location.getY();
    }

    public void setSpeed(int dx, int dy){
        this.dx = (short) dx;
        this.dy = (short) dy;
    }

    public void setSpeed(Point speed){
        this.dx = (short) speed.getX();
        this.dy = (short) speed.getY();
    }

    @Override
    public boolean equals(Object e) {
        if ( e instanceof PhaseSpaceVertex &&
                this.x == ((PhaseSpaceVertex)e).x && this.y == ((PhaseSpaceVertex)e).y && this.dx == ((PhaseSpaceVertex)e).dx && this.dy == ((PhaseSpaceVertex)e).dy )
                //this.location.equals(((algorithm.PhaseSpaceVertex)e).location) &&
                //this.speedvector.equals(((algorithm.PhaseSpaceVertex)e).speedvector))
            return true;
        else
            return false;
    }

    @Override
    public int hashCode() {
        int result = (this.x + 10*this.y + 100*this.dx + 1000*this.dy);
        return result;
    }

    @Override
    public String toString(){
        return "Location : (" + this.x + ", " + this.y + ") ; Speed : (" + this.dx + ", " + this.dy + ")";
    }

    public boolean sameLocation(PhaseSpaceVertex psn){
        return (this.x == psn.getLocation().getX() && this.y == psn.getLocation().getY());
    }

    public boolean sameLocation(Point p){
        return (this.x == p.getX() && this.y == p.getY());
    }

    public boolean sameLocation(short x, short y){
        return (this.x == x && this.y == y);
    }

    public boolean sameSpeed(PhaseSpaceVertex psn){
        return (this.dx == psn.getSpeed().getX() && this.dy == psn.getSpeed().getY());
    }

    public boolean sameSpeed(Point p){
        return (this.dx == p.getX() && this.dy == p.getY());
    }

    public boolean sameSpeed(short dx, short dy){
        return (this.dx == dx && this.dy == dy);
    }

    public PhaseSpaceVertex clone(){
        return new PhaseSpaceVertex(this.x, this.y, this.dx, this.dy);
    }

    public void nextTransitions(double resolution, Set<PhaseSpaceVertex> visited, int nb_transitions){
        if (nb_transitions != 5 && nb_transitions != 9)
            System.out.println("please input 5 or 9 as nb_transitions");
        else{
            if (nb_transitions == 5)
                next5Transitions(resolution, visited);
            else if (nb_transitions == 9)
                next9Transitions(resolution, visited);
        }
    }

    public void next5Transitions(double resolution, Set<PhaseSpaceVertex> visited) {
        this.nextTransitions = new PhaseSpaceVertex[5];
        int iterator = 0;
        for (double i = -resolution; i <= resolution; i += resolution) {
            if (i == 0) {
                for (double j = -resolution; j <= resolution; j += resolution) {
                    Point newLocation = new Point(this.x + this.dx, this.y + this.dy + j);
                    Point newSpeed = new Point(this.dx, this.dy + j);
                    if (!visited.contains(new PhaseSpaceVertex(newLocation, newSpeed))) {
                        this.nextTransitions[iterator] = new PhaseSpaceVertex(newLocation, newSpeed);
                        this.nextTransitions[iterator].previousTransition = this;
                    } else
                        this.nextTransitions[iterator] = (new PhaseSpaceVertex(newLocation, newSpeed)).getExistingPhaseSpaceNode(visited);
                    iterator++;
                }
            } else {
                Point newLocation = new Point(this.x + this.dx + i, this.y + this.dy);
                Point newSpeed = new Point(this.dx + i, this.dy);
                if (!visited.contains(new PhaseSpaceVertex(newLocation, newSpeed))) {
                    this.nextTransitions[iterator] = new PhaseSpaceVertex(newLocation, newSpeed);
                    this.nextTransitions[iterator].previousTransition = this;
                } else
                    this.nextTransitions[iterator] = (new PhaseSpaceVertex(newLocation, newSpeed)).getExistingPhaseSpaceNode(visited);
                iterator++;
            }
        }
    }

    public void next9Transitions(double resolution, Set<PhaseSpaceVertex> visited){
        this.nextTransitions = new PhaseSpaceVertex[9];
        int iterator = 0;
        for (double i = -resolution; i <= resolution; i += resolution){
            for (double j = -resolution; j <= resolution; j += resolution){
                Point newLocation = new Point(this.x + this.dx + i, this.y + this.dy + j);
                Point newSpeed = new Point(this.dx + i, this.dy + j);
                if (!visited.contains(new PhaseSpaceVertex(newLocation, newSpeed))) {
                    this.nextTransitions[iterator] = new PhaseSpaceVertex(newLocation, newSpeed);
                    this.nextTransitions[iterator].previousTransition = this;
                } else
                    this.nextTransitions[iterator] = (new PhaseSpaceVertex(newLocation, newSpeed)).getExistingPhaseSpaceNode(visited);
                iterator++;
            }
        }
    }

    public boolean PhaseSpaceNodeExists(Set<PhaseSpaceVertex> visited) {
        if (visited.contains(this))
            return true;
        else
            return false;
    }

    public PhaseSpaceVertex getExistingPhaseSpaceNode(Set<PhaseSpaceVertex> visited) {
        for (PhaseSpaceVertex e : visited) {
            if (e.equals(this))
                return e;
        }
        System.out.println("ERROR could not return existing algorithm.PhaseSpaceVertex");
        return this;
    }

    public boolean contains_reversedNextTransition(Set<PhaseSpaceVertex> set, double resolution, int nb_transitions){
        if (nb_transitions != 5 && nb_transitions != 9)
            System.out.println("please input 5 or 9 as nb_transitions");
        else{
            if (nb_transitions == 5)
                return contains_reversedNext5Transition(set, resolution);
            else if (nb_transitions == 9)
                return contains_reversedNext9Transition(set, resolution);
        }
        return false;
    }

    public boolean contains_reversedNext5Transition(Set<PhaseSpaceVertex> set, double resolution){

        PhaseSpaceVertex first = this.clone();
        first.dx = (short) -first.dx;
        first.dy = (short) -first.dy;

        PhaseSpaceVertex second = this.clone();
        second.dx = (short) (-second.dx + resolution);
        second.dy = (short) -second.dy;

        PhaseSpaceVertex third = this.clone();
        third.dx = (short) (-third.dx - resolution);
        third.dy = (short) -third.dy;

        PhaseSpaceVertex fourth = this.clone();
        fourth.dx = (short) -fourth.dx;
        fourth.dy = (short) (-fourth.dy + resolution);

        PhaseSpaceVertex fifth = this.clone();
        fifth.dx = (short) -fifth.dx;
        fifth.dy = (short) (-fifth.dy - resolution);

        if (set.contains(first) || set.contains(second) || set.contains(third) || set.contains(fourth) || set.contains(fifth))
            return true;
        return false;
    }

    public boolean contains_reversedNext9Transition(Set<PhaseSpaceVertex> set, double resolution){

        PhaseSpaceVertex first = this.clone();
        first.dx = (short) -first.dx;
        first.dy = (short) -first.dy;

        PhaseSpaceVertex second = this.clone();
        second.dx = (short) (-second.dx + resolution);
        second.dy = (short) -second.dy;

        PhaseSpaceVertex third = this.clone();
        third.dx = (short) (-third.dx - resolution);
        third.dy = (short) -third.dy;

        PhaseSpaceVertex fourth = this.clone();
        fourth.dx = (short) -fourth.dx;
        fourth.dy = (short) (-fourth.dy + resolution);

        PhaseSpaceVertex fifth = this.clone();
        fifth.dx = (short) -fifth.dx;
        fifth.dy = (short) (-fifth.dy - resolution);

        PhaseSpaceVertex sixth = this.clone();
        sixth.dx = (short) (-sixth.dx + resolution);
        sixth.dy = (short) (-sixth.dy + resolution);

        PhaseSpaceVertex seventh = this.clone();
        seventh.dx = (short) (-seventh.dx + resolution);
        seventh.dy = (short) (-seventh.dy - resolution);

        PhaseSpaceVertex eighth = this.clone();
        eighth.dx = (short) (-eighth.dx - resolution);
        eighth.dy = (short) (-eighth.dy + resolution);

        PhaseSpaceVertex ninth = this.clone();
        ninth.dx = (short) (-ninth.dx - resolution);
        ninth.dy = (short) (-ninth.dy - resolution);

        if (set.contains(first) || set.contains(second) || set.contains(third) || set.contains(fourth) || set.contains(fifth)
                || set.contains(sixth) || set.contains(seventh) || set.contains(eighth) || set.contains(ninth))
            return true;
        return false;
    }

    public PhaseSpaceVertex find_reversedNextTransition(double resolution, Set visited, int nb_transitions){
        if (nb_transitions != 5 && nb_transitions != 9)
            System.out.println("please input 5 or 9 as nb_transitions");
        else{
            if (nb_transitions == 5)
                return find_reversedNext5Transition(resolution, visited);
            else if (nb_transitions == 9)
                return find_reversedNext9Transition(resolution, visited);
        }
        return this;
    }

    public PhaseSpaceVertex find_reversedNext5Transition(double resolution, Set visited){

        PhaseSpaceVertex psn;

        psn = new PhaseSpaceVertex(this.x, this.y, -this.dx, -this.dy);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, (int) (-this.dx + resolution), -this.dy);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, (int) (-this.dx - resolution), -this.dy);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, -this.dx, (int) (-this.dy + resolution));
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, -this.dx, (int) (-this.dy - resolution));
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        return null;
    }

    public PhaseSpaceVertex find_reversedNext9Transition(double resolution, Set visited){

        PhaseSpaceVertex psn;

        psn = new PhaseSpaceVertex(this.x, this.y, -this.dx, -this.dy);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, (int) (-this.dx + resolution), -this.dy);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, (int) (-this.dx - resolution), -this.dy);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, -this.dx, (int) (-this.dy + resolution));
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, -this.dx, (int) (-this.dy - resolution));
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, (int) (-this.dx + resolution), (int) (-this.dy + resolution));
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, (int) (-this.dx + resolution), (int) (-this.dy - resolution));
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, (int) (-this.dx - resolution), (int) (-this.dy + resolution));
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        psn = new PhaseSpaceVertex(this.x, this.y, (int) (-this.dx - resolution), (int) (-this.dy - resolution));
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode(visited);

        return null;
    }

    public boolean passedBy(Point checkpoint, double sensingrange) {
        if (this.previousTransition ==null)
            return false;
        Point t1 = this.getLocation();
        Point t0 = this.previousTransition.getLocation();

        if (distanceToLine(checkpoint, t0, t1) <= sensingrange)
            return true;
        return false;
    }

    public static double distanceToLine(Point p, Point p1, Point p2) {

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
}
