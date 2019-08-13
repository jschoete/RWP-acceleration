package algorithm;

import io.jbotsim.core.Point;

import java.util.Set;

/**
 * Created by J. Schoeters on 29/03/17.
 * A phase space vertex, but with an extra parameter : visitedCheckpoint.
 * This is used to store the information needed for the Inter_check_2BFS class' algorithm.
 */
public class PhaseSpaceVertex2 extends PhaseSpaceVertex {

    public int visitedCheckpoint; //we chose to use an int instead of a boolean, in case multiple checkpoints were to be visited

    public PhaseSpaceVertex2(int x, int y, int dx, int dy, int visitedCheckpoint){
        super(x, y, dx, dy);
        this.visitedCheckpoint = visitedCheckpoint;
    }

    public PhaseSpaceVertex2(Point location, Point speed, int visitedCheckpoint){
        super(location, speed);
        this.visitedCheckpoint = visitedCheckpoint;
    }

    @Override
    public boolean equals(Object e) {
        if ( e instanceof PhaseSpaceVertex2 &&
                this.x == ((PhaseSpaceVertex2)e).x && this.y == ((PhaseSpaceVertex2)e).y &&
                this.dx == ((PhaseSpaceVertex2)e).dx && this.dy == ((PhaseSpaceVertex2)e).dy &&
                this.visitedCheckpoint == ((PhaseSpaceVertex2)e).visitedCheckpoint)
            return true;
        else
            return false;
    }

    @Override
    public int hashCode() {
        return super.hashCode() + this.visitedCheckpoint;
    }

    public String toString(){
        String result = super.toString();
        if (this.visitedCheckpoint == 1) {
            result = result.concat(" ; visited checkpoint ");
        }
        else
            result = result.concat(" ; checkpoint(s) not visited ");
        return result;
    }

    public PhaseSpaceVertex2 clone(){
        return new PhaseSpaceVertex2(this.x, this.y, this.dx, this.dy, this.visitedCheckpoint);
    }

    public void nextTransitions2(double resolution, int nb_transitions){
        if (nb_transitions == 5)
            next5Transitions2(resolution);
        else if (nb_transitions == 9)
            next9Transitions2(resolution);
        else
            System.out.println("please input 5 or 9 as nb_transitions");
    }

    public void next5Transitions2(double resolution){
        this.nextTransitions = new PhaseSpaceVertex[5];
        int iterator = 0;
        for (double i = -resolution; i <= resolution; i += resolution) {
            if (i == 0) {
                for (double j = -resolution; j <= resolution; j += resolution) {
                    Point newLocation = new Point(this.x + this.dx, this.y + this.dy + j);
                    Point newSpeed = new Point(this.dx, this.dy + j);
                    PhaseSpaceVertex2 son = new PhaseSpaceVertex2(newLocation, newSpeed, this.visitedCheckpoint);
                    this.nextTransitions[iterator] = son;
                    this.nextTransitions[iterator].previousTransition = this;
                    iterator++;
                }
            } else {
                Point newLocation = new Point(this.x + this.dx + i, this.y + this.dy);
                Point newSpeed = new Point(this.dx + i, this.dy);
                PhaseSpaceVertex2 son = new PhaseSpaceVertex2(newLocation, newSpeed, this.visitedCheckpoint);
                this.nextTransitions[iterator] = son;
                this.nextTransitions[iterator].previousTransition = this;
                iterator++;
            }
        }
    }

    public void next9Transitions2(double resolution){
        this.nextTransitions = new PhaseSpaceVertex[9];
        int iterator = 0;
        for (double i = -resolution; i <= resolution; i += resolution){
            for (double j = -resolution; j <= resolution; j += resolution){
                Point newLocation = new Point(this.x + this.dx + i, this.y + this.dy + j);
                Point newSpeed = new Point(this.dx + i, this.dy + j);
                PhaseSpaceVertex2 son = new PhaseSpaceVertex2(newLocation, newSpeed, this.visitedCheckpoint);
                this.nextTransitions[iterator] = son;
                this.nextTransitions[iterator].previousTransition = this;
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

    public PhaseSpaceVertex2 getExistingPhaseSpaceNode2(Set<PhaseSpaceVertex> visited) {
        for (PhaseSpaceVertex e : visited) {
            if (e.equals(this))
                return (PhaseSpaceVertex2) e;
        }
        System.out.println("ERROR could not return existing algorithm.PhaseSpaceVertex2");
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
        //We check the special case in which we have our psn on the checkpoint.
        //In this case visitedCheckpoint will be true, and the inversed PSN in other BFS will have visitedCheckpoint true also.
        if (this.visitedCheckpoint == 1){

            PhaseSpaceVertex2 first = this.clone();
            first.dx = (short) -first.dx;
            first.dy = (short) -first.dy;

            PhaseSpaceVertex2 second = this.clone();
            second.dx = (short) (-second.dx + resolution);
            second.dy = (short) -second.dy;

            PhaseSpaceVertex2 third = this.clone();
            third.dx = (short) (-third.dx - resolution);
            third.dy = (short) -third.dy;

            PhaseSpaceVertex2 fourth = this.clone();
            fourth.dx = (short) -fourth.dx;
            fourth.dy = (short) (-fourth.dy + resolution);

            PhaseSpaceVertex2 fifth = this.clone();
            fifth.dx = (short) -fifth.dx;
            fifth.dy = (short) (-fifth.dy - resolution);

            //
            PhaseSpaceVertex2 first0 = this.clone();
            first0.visitedCheckpoint = 0;
            first0.dx = (short) -first0.dx;
            first0.dy = (short) -first0.dy;

            PhaseSpaceVertex2 second0 = this.clone();
            second0.visitedCheckpoint = 0;
            second0.dx = (short) (-second0.dx + resolution);
            second0.dy = (short) -second0.dy;

            PhaseSpaceVertex2 third0 = this.clone();
            third0.visitedCheckpoint = 0;
            third0.dx = (short) (-third0.dx - resolution);
            third0.dy = (short) -third0.dy;

            PhaseSpaceVertex2 fourth0 = this.clone();
            fourth0.visitedCheckpoint = 0;
            fourth0.dx = (short) -fourth0.dx;
            fourth0.dy = (short) (-fourth0.dy + resolution);

            PhaseSpaceVertex2 fifth0 = this.clone();
            fifth0.visitedCheckpoint = 0;
            fifth0.dx = (short) -fifth0.dx;
            fifth0.dy = (short) (-fifth0.dy - resolution);

            //
            if (set.contains(first) || set.contains(second) || set.contains(third) || set.contains(fourth) || set.contains(fifth)
                    || set.contains(first0) || set.contains(second0) || set.contains(third0) || set.contains(fourth0) || set.contains(fifth0)){
                return true;
            }
            return false;
        }
        else{

            int b = 1;

            PhaseSpaceVertex2 first = this.clone();
            first.visitedCheckpoint = b;
            first.dx = (short) -first.dx;
            first.dy = (short) -first.dy;

            PhaseSpaceVertex2 second = this.clone();
            second.visitedCheckpoint = b;
            second.dx = (short) (-second.dx + resolution);
            second.dy = (short) -second.dy;

            PhaseSpaceVertex2 third = this.clone();
            third.visitedCheckpoint = b;
            third.dx = (short) (-third.dx - resolution);
            third.dy = (short) -third.dy;

            PhaseSpaceVertex2 fourth = this.clone();
            fourth.visitedCheckpoint = b;
            fourth.dx = (short) -fourth.dx;
            fourth.dy = (short) (-fourth.dy + resolution);

            PhaseSpaceVertex2 fifth = this.clone();
            fifth.visitedCheckpoint = b;
            fifth.dx = (short) -fifth.dx;
            fifth.dy = (short) (-fifth.dy - resolution);

            if (set.contains(first) || set.contains(second) || set.contains(third) || set.contains(fourth) || set.contains(fifth)){
                return true;
            }
            return false;
        }
    }

    public boolean contains_reversedNext9Transition(Set<PhaseSpaceVertex> set, double resolution){
        //We check the special case in which we have our psn on the checkpoint.
        //In this case visitedCheckpoint will be true, and the inversed PSN in other BFS will have visitedCheckpoint true also.
        if (this.visitedCheckpoint == 1){

            PhaseSpaceVertex2 first = this.clone();
            first.dx = (short) -first.dx;
            first.dy = (short) -first.dy;

            PhaseSpaceVertex2 second = this.clone();
            second.dx = (short) (-second.dx + resolution);
            second.dy = (short) -second.dy;

            PhaseSpaceVertex2 third = this.clone();
            third.dx = (short) (-third.dx - resolution);
            third.dy = (short) -third.dy;

            PhaseSpaceVertex2 fourth = this.clone();
            fourth.dx = (short) -fourth.dx;
            fourth.dy = (short) (-fourth.dy + resolution);

            PhaseSpaceVertex2 fifth = this.clone();
            fifth.dx = (short) -fifth.dx;
            fifth.dy = (short) (-fifth.dy - resolution);

            PhaseSpaceVertex2 sixth = this.clone();
            sixth.dx = (short) (-sixth.dx + resolution);
            sixth.dy = (short) (-sixth.dy + resolution);

            PhaseSpaceVertex2 seventh = this.clone();
            seventh.dx = (short) (-seventh.dx + resolution);
            seventh.dy = (short) (-seventh.dy - resolution);

            PhaseSpaceVertex2 eighth = this.clone();
            eighth.dx = (short) (-eighth.dx - resolution);
            eighth.dy = (short) (-eighth.dy + resolution);

            PhaseSpaceVertex2 ninth = this.clone();
            ninth.dx = (short) (-ninth.dx - resolution);
            ninth.dy = (short) (-ninth.dy - resolution);

            PhaseSpaceVertex2 first0 = this.clone();
            first0.visitedCheckpoint = 0;
            first0.dx = (short) -first0.dx;
            first0.dy = (short) -first0.dy;

            PhaseSpaceVertex2 second0 = this.clone();
            second0.visitedCheckpoint = 0;
            second0.dx = (short) (-second0.dx + resolution);
            second0.dy = (short) -second0.dy;

            PhaseSpaceVertex2 third0 = this.clone();
            third0.visitedCheckpoint = 0;
            third0.dx = (short) (-third0.dx - resolution);
            third0.dy = (short) -third0.dy;

            PhaseSpaceVertex2 fourth0 = this.clone();
            fourth0.visitedCheckpoint = 0;
            fourth0.dx = (short) -fourth0.dx;
            fourth0.dy = (short) (-fourth0.dy + resolution);

            PhaseSpaceVertex2 fifth0 = this.clone();
            fifth0.visitedCheckpoint = 0;
            fifth0.dx = (short) -fifth0.dx;
            fifth0.dy = (short) (-fifth0.dy - resolution);

            PhaseSpaceVertex2 sixth0 = this.clone();
            sixth0.visitedCheckpoint = 0;
            sixth0.dx = (short) (-sixth0.dx + resolution);
            sixth0.dy = (short) (-sixth0.dy + resolution);

            PhaseSpaceVertex2 seventh0 = this.clone();
            seventh0.visitedCheckpoint = 0;
            seventh0.dx = (short) (-seventh0.dx + resolution);
            seventh0.dy = (short) (-seventh0.dy - resolution);

            PhaseSpaceVertex2 eighth0 = this.clone();
            eighth0.visitedCheckpoint = 0;
            eighth0.dx = (short) (-eighth0.dx - resolution);
            eighth0.dy = (short) (-eighth0.dy + resolution);

            PhaseSpaceVertex2 ninth0 = this.clone();
            ninth0.visitedCheckpoint = 0;
            ninth0.dx = (short) (-ninth0.dx - resolution);
            ninth0.dy = (short) (-ninth0.dy - resolution);

            if (set.contains(first) || set.contains(second) || set.contains(third) || set.contains(fourth) || set.contains(fifth)
                    || set.contains(sixth) || set.contains(seventh) || set.contains(eighth) || set.contains(ninth)
                    || set.contains(first0) || set.contains(second0) || set.contains(third0) || set.contains(fourth0) || set.contains(fifth0)
                    || set.contains(sixth0) || set.contains(seventh0) || set.contains(eighth0) || set.contains(ninth0)) {
                return true;
            }
            return false;
        }
        else{

            int b = 1;

            PhaseSpaceVertex2 first = this.clone();
            first.visitedCheckpoint = b;
            first.dx = (short) -first.dx;
            first.dy = (short) -first.dy;

            PhaseSpaceVertex2 second = this.clone();
            second.visitedCheckpoint = b;
            second.dx = (short) (-second.dx + resolution);
            second.dy = (short) -second.dy;

            PhaseSpaceVertex2 third = this.clone();
            third.visitedCheckpoint = b;
            third.dx = (short) (-third.dx - resolution);
            third.dy = (short) -third.dy;

            PhaseSpaceVertex2 fourth = this.clone();
            fourth.visitedCheckpoint = b;
            fourth.dx = (short) -fourth.dx;
            fourth.dy = (short) (-fourth.dy + resolution);

            PhaseSpaceVertex2 fifth = this.clone();
            fifth.visitedCheckpoint = b;
            fifth.dx = (short) -fifth.dx;
            fifth.dy = (short) (-fifth.dy - resolution);

            PhaseSpaceVertex2 sixth = this.clone();
            sixth.visitedCheckpoint = b;
            sixth.dx = (short) (-sixth.dx + resolution);
            sixth.dy = (short) (-sixth.dy + resolution);

            PhaseSpaceVertex2 seventh = this.clone();
            seventh.visitedCheckpoint = b;
            seventh.dx = (short) (-seventh.dx + resolution);
            seventh.dy = (short) (-seventh.dy - resolution);

            PhaseSpaceVertex2 eighth = this.clone();
            eighth.visitedCheckpoint = b;
            eighth.dx = (short) (-eighth.dx - resolution);
            eighth.dy = (short) (-eighth.dy + resolution);

            PhaseSpaceVertex2 ninth = this.clone();
            ninth.visitedCheckpoint = b;
            ninth.dx = (short) (-ninth.dx - resolution);
            ninth.dy = (short) (-ninth.dy - resolution);

            if (set.contains(first) || set.contains(second) || set.contains(third) || set.contains(fourth) || set.contains(fifth)
                    || set.contains(sixth) || set.contains(seventh) || set.contains(eighth) || set.contains(ninth)) {
                return true;
            }
            return false;
        }
    }

    public PhaseSpaceVertex2 find_reversedNextTransition(double deviation, int b, Set visited, int nb_transitions){
        if (nb_transitions != 5 && nb_transitions != 9)
            System.out.println("Please input 5 or 9 as nb_transitions");
        else{
            if (nb_transitions == 5){
                return find_reversedNext5Transition(deviation, b, visited);
            }

            else if (nb_transitions == 9){
                return find_reversedNext9Transition(deviation, b, visited);
            }

        }
        return this;
    }

    public PhaseSpaceVertex2 find_reversedNext5Transition(double deviation, int b, Set visited){

        PhaseSpaceVertex2 psn;

        psn = new PhaseSpaceVertex2(this.x, this.y, -this.dx, -this.dy, b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, (int) (-this.dx + deviation), -this.dy, b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, (int) (-this.dx - deviation), -this.dy, b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, -this.dx, (int) (-this.dy + deviation), b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, -this.dx, (int) (-this.dy - deviation), b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        return null;
    }

    public PhaseSpaceVertex2 find_reversedNext9Transition(double deviation, int b, Set visited){

        PhaseSpaceVertex2 psn;

        psn = new PhaseSpaceVertex2(this.x, this.y, -this.dx, -this.dy, b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, (int) (-this.dx + deviation), -this.dy, b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, (int) (-this.dx - deviation), -this.dy, b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, -this.dx, (int) (-this.dy + deviation), b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, -this.dx, (int) (-this.dy - deviation), b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, (int) (-this.dx + deviation), (int) (-this.dy + deviation), b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, (int) (-this.dx + deviation), (int) (-this.dy - deviation), b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, (int) (-this.dx - deviation), (int) (-this.dy + deviation), b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        psn = new PhaseSpaceVertex2(this.x, this.y, (int) (-this.dx - deviation), (int) (-this.dy - deviation), b);
        if (psn.PhaseSpaceNodeExists(visited))
            return psn.getExistingPhaseSpaceNode2(visited);

        return null;
    }
}
