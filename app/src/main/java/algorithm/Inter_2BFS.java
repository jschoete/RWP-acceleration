package algorithm;

import environment.VectorNode;
import io.jbotsim.core.Point;
import io.jbotsim.core.Topology;

import java.util.*;

/**
 * Created by J. Schoeters on 03/04/17.
 * This algorithm uses 2 BFS to search for a valid trajectory in the phase space,
 * between 2 phase space vertices representing the visiting entity's current configuration and the next point to visit.
 */
public class Inter_2BFS {

    public static List<Point> getTrajectory(List<Point> nodelist, Point start_location, Topology tp, int nb_transitions){

        //Trajectory from drone to first cherry
        int iterator = 0;
        for (int i=0; i<nodelist.size(); i++)
            if (nodelist.get(i).equals(start_location))
                iterator = i;

        Point firstCherry_location = nodelist.get((iterator + 1) % nodelist.size());
        Point zero_speed = new Point(0, 0);
        PhaseSpaceVertex psn_start = new PhaseSpaceVertex(start_location, zero_speed);
        PhaseSpaceVertex psn_firstCherry = new PhaseSpaceVertex(firstCherry_location, zero_speed);
        List<PhaseSpaceVertex> trajectory = intersection2BFS_Trajectory(psn_start, psn_firstCherry, VectorNode.DEVIATION, nb_transitions);

        //Trajectory from cherry to cherry
        for (int i = iterator + 2; i < iterator + nodelist.size(); i++) {
            PhaseSpaceVertex psn_cherry = trajectory.get(trajectory.size() - 1);
            Point nextCherry_location = nodelist.get(i % nodelist.size());
            PhaseSpaceVertex psn_nextCherry = new PhaseSpaceVertex(nextCherry_location, zero_speed);
            List<PhaseSpaceVertex> moreTrajectory = intersection2BFS_Trajectory(psn_cherry, psn_nextCherry, VectorNode.DEVIATION, nb_transitions);
            trajectory.addAll(moreTrajectory);
        }

        //Trajectory from last cherry to drone
        PhaseSpaceVertex psn_lastCherry = trajectory.get(trajectory.size() - 1);
        List<PhaseSpaceVertex> moreTrajectory = intersection2BFS_Trajectory(psn_lastCherry, psn_start, VectorNode.DEVIATION, nb_transitions);
        trajectory.addAll(moreTrajectory);

        //From quad_BFS.PhaseSpaceVertex(2)s to Point
        List<Point> result = new ArrayList<>();
        for (PhaseSpaceVertex psn : trajectory){
            Point p = new Point(psn.x, psn.y);
            result.add(p);
        }
        return result;
    }


    public static List<PhaseSpaceVertex> intersection2BFS_Trajectory(PhaseSpaceVertex psn_start, PhaseSpaceVertex psn_goal, double deviation, int nb_transitions) {
        //Initialization
        List<PhaseSpaceVertex> result = new ArrayList<>();
        Set<PhaseSpaceVertex> visited_start = new HashSet<PhaseSpaceVertex>();
        Set<PhaseSpaceVertex> visited_goal = new HashSet<PhaseSpaceVertex>();
        PhaseSpaceVertex psn_intersection = psn_start;
        Queue queue_start = new LinkedList<PhaseSpaceVertex>();
        Queue queue_goal = new LinkedList<PhaseSpaceVertex>();
        queue_start.add(psn_start);
        queue_goal.add(psn_goal);
        boolean intersectionFound_start = false;
        boolean intersectionFound_goal = false;

        //Find intersection of two BFS' that also passed checkpoint
        while (!(intersectionFound_start || intersectionFound_goal)) {

            //First BFS
            PhaseSpaceVertex psn_in_queue_start = (PhaseSpaceVertex) queue_start.remove();
            psn_in_queue_start.nextTransitions(deviation, visited_start, nb_transitions);

            for (int i = 0; i < psn_in_queue_start.nextTransitions.length; i++) {

                PhaseSpaceVertex psn_current = psn_in_queue_start.nextTransitions[i];
                if (!visited_start.contains(psn_current)) {

                    if (psn_current.contains_reversedNextTransition(visited_goal, deviation, nb_transitions)) {
                        psn_intersection = psn_current;
                        intersectionFound_start = true;
                        break;
                    }
                    if (!visited_start.contains(psn_current)) {
                        queue_start.add(psn_current);
                        visited_start.add(psn_current);
                    }
                }
            }
            if (intersectionFound_start)
                break;

            //Second BFS
            PhaseSpaceVertex psn_in_queue_goal = (PhaseSpaceVertex) queue_goal.remove();
            psn_in_queue_goal.nextTransitions(deviation, visited_goal, nb_transitions);

            for (int i = 0; i < psn_in_queue_goal.nextTransitions.length; i++) {

                PhaseSpaceVertex psn_current = psn_in_queue_goal.nextTransitions[i];
                if (!visited_goal.contains(psn_current)) {

                    if (psn_current.contains_reversedNextTransition(visited_start, deviation, nb_transitions)) {
                        psn_intersection = psn_current;
                        intersectionFound_goal = true;
                        break;
                    }
                    if (!visited_goal.contains(psn_current)) {
                        queue_goal.add(psn_current);
                        visited_goal.add(psn_current);
                    }
                }

            }
        }

        //Reconstructing trajectory
        if(intersectionFound_start) {

            boolean startFound = false;
            PhaseSpaceVertex psn = psn_intersection;
            while (!startFound) {
                if (psn.sameLocation(psn_start) && psn.sameSpeed(psn_start))
                    startFound = true;
                else {
                    result.add(psn);
                    PhaseSpaceVertex debugging = psn.clone();
                    psn = psn.previousTransition;
                    if (psn.equals(debugging))
                        break;
                }
            }
            Collections.reverse(result);
            result.remove(result.size() - 1);
            boolean goalFound = false;
            PhaseSpaceVertex psn_other_BFS = psn_intersection.find_reversedNextTransition(deviation, visited_goal, nb_transitions);

            while (!goalFound) {
                if (psn_other_BFS.sameLocation(psn_goal)) {
                    goalFound = true;
                    PhaseSpaceVertex newResult = psn_other_BFS.clone();
                    newResult.dx = (short) -newResult.dx;
                    newResult.dy = (short) -newResult.dy;
                    result.add(newResult);
                } else {
                    PhaseSpaceVertex newResult = psn_other_BFS.clone();
                    newResult.dx = (short) -newResult.dx;
                    newResult.dy = (short) -newResult.dy;
                    result.add(newResult);
                    psn_other_BFS = psn_other_BFS.previousTransition;
                }
            }
            return result;
        }
        else{ //If intersection was found by goal side.

            boolean startFound = false;
            PhaseSpaceVertex psn_other_BFS = psn_intersection.find_reversedNextTransition(deviation, visited_start, nb_transitions);
            while (!startFound){
                if (psn_other_BFS.sameLocation(psn_start) && psn_other_BFS.sameSpeed(psn_start)) {
                    startFound = true;
                    result.add(psn_other_BFS);
                }
                else {
                    result.add(psn_other_BFS);
                    PhaseSpaceVertex debugging = psn_other_BFS.clone();
                    psn_other_BFS = psn_other_BFS.previousTransition;
                    if (psn_other_BFS.equals(debugging))
                        break;
                }
            }
            result.remove(result.size() - 1);
            Collections.reverse(result);
            if (result.size() != 0)
                result.remove(result.size() - 1);

            boolean goalFound = false;
            while (!goalFound) {
                if (psn_intersection.equals(psn_goal)) {
                    goalFound = true;
                    result.add(psn_intersection);
                }
                else {
                    result.add(psn_intersection);
                    PhaseSpaceVertex debugging = psn_intersection.clone();
                    psn_intersection = psn_intersection.previousTransition;
                    if (psn_intersection.equals(debugging))
                        break;
                }
            }
            return result;
        }
    }
}
