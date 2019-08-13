package algorithm;

import io.jbotsim.core.Point;
import io.jbotsim.core.Topology;

import java.util.*;

import static environment.Main.resolution;

/**
 * Created by J. Schoeters on 16/02/17.
 * Similar to Inter_2BFS.java, but with a checkpoint added.
 * One has to pass the checkpoint in either one of the 2 BFS' before the trajectory can be accepted.
 */
public class Inter_check_2BFS {

    public static List<Point> getTrajectory(List<Point> nodelist, Point startPoint, Topology tp, double resolution, int nb_transitions) {
        double sensingrange = 0;

        //Trajectory from drone to first cherry
        int iterator = 0;
        for (int i = 0; i < nodelist.size(); i++)
            if (nodelist.get(i).equals(startPoint))
                iterator = i;

        Point firstCherry_location = nodelist.get((iterator + 1) % nodelist.size());
        Point secondCherry_location = nodelist.get((iterator + 2) % nodelist.size());

        Point drone_location = startPoint;
        Point drone_speed = new Point(0, 0);
        PhaseSpaceVertex2 psn_drone = new PhaseSpaceVertex2(drone_location, drone_speed, 0);

        System.out.println("searching first from : " + psn_drone.getLocation().toString() + " to : " + firstCherry_location.toString() + " (goal : " + secondCherry_location.toString() + ")");
        List<PhaseSpaceVertex2> trajectory = checkpoint2BFS_Trajectory(psn_drone, firstCherry_location, secondCherry_location, resolution, nb_transitions, sensingrange, tp);
        List<PhaseSpaceVertex2> moreTrajectory;
        trajectory.get(trajectory.size() - 1).setSpeed(trajectory.get(trajectory.size() - 1).x - trajectory.get(trajectory.size() - 2).x, trajectory.get(trajectory.size() - 1).y - trajectory.get(trajectory.size() - 2).y);

        //Trajectory from cherry to cherry
        for (int i = iterator + 2; i < iterator + nodelist.size() - 1; i++) {

            PhaseSpaceVertex2 psn_cherry = trajectory.get(trajectory.size() - 1);
            psn_cherry.visitedCheckpoint = 0;
            Point nextCherry_location = nodelist.get(i % nodelist.size());
            Point nextNextCherry_location = nodelist.get((i + 1) % nodelist.size());

            System.out.println("searching next from : " + psn_cherry.getLocation().toString() + " to : " + nextCherry_location.toString() + " (goal : " + nextNextCherry_location.toString() + ")");
            moreTrajectory = checkpoint2BFS_Trajectory(psn_cherry, nextCherry_location, nextNextCherry_location, resolution, nb_transitions, sensingrange, tp);
            trajectory.addAll(moreTrajectory);
            trajectory.get(trajectory.size() - 1).setSpeed(trajectory.get(trajectory.size() - 1).x - trajectory.get(trajectory.size() - 2).x, trajectory.get(trajectory.size() - 1).y - trajectory.get(trajectory.size() - 2).y);
        }

        //Trajectory from second last cherry to last cherry
        Point lastCherry_location = nodelist.get((iterator + nodelist.size() - 1) % nodelist.size());
        PhaseSpaceVertex2 secondLastCherry = trajectory.get(trajectory.size() - 1);
        secondLastCherry.visitedCheckpoint = 0;
        System.out.println("searching second last from : " + secondLastCherry.getLocation().toString() + " to : " + lastCherry_location.toString() + " (goal : " + startPoint.toString() + ")");
        moreTrajectory = checkpoint2BFS_Trajectory(secondLastCherry, lastCherry_location, startPoint, resolution, nb_transitions, sensingrange, tp);
        trajectory.addAll(moreTrajectory);
        trajectory.get(trajectory.size() - 1).setSpeed(trajectory.get(trajectory.size() - 1).x - trajectory.get(trajectory.size() - 2).x, trajectory.get(trajectory.size() - 1).y - trajectory.get(trajectory.size() - 2).y);

        //Trajectory from last cherry to drone
        PhaseSpaceVertex psn_lastCherry = trajectory.get(trajectory.size() - 1);
        Point zero_speed = new Point(0, 0);
        PhaseSpaceVertex psn_start = new PhaseSpaceVertex(startPoint, zero_speed);
        System.out.println("searching last from : " + psn_lastCherry.getLocation().toString() + " to : " + psn_start.getLocation().toString());
        List<PhaseSpaceVertex> lastTrajectory = Inter_2BFS.intersection2BFS_Trajectory(psn_lastCherry, psn_start, resolution, nb_transitions);

        //From PhaseSpaceNodes to Points
        List<Point> result = new ArrayList<>();
        for (PhaseSpaceVertex psn : trajectory) {
            Point p = new Point(psn.x, psn.y);
            result.add(p);
        }
        for (PhaseSpaceVertex psn : lastTrajectory) {
            Point p = new Point(psn.x, psn.y);
            result.add(p);
        }
        return result;
    }

    public static List<PhaseSpaceVertex2> checkpoint2BFS_Trajectory(PhaseSpaceVertex2 psn_start, Point checkpoint, Point goal, double resolution, int nb_transitions,
                                                                    double sensingrange, Topology tp) {
        //Initialization
        List<Point> copy_nodelist = new ArrayList<>();
        copy_nodelist.add(psn_start.getLocation());
        copy_nodelist.add(checkpoint);
        copy_nodelist.add(goal);
        Set<PhaseSpaceVertex> visited_start = new HashSet<>();
        Set<PhaseSpaceVertex> visited_goal = new HashSet<>();
        Point goal_speed = new Point(0, 0);
        PhaseSpaceVertex2 psn_goal = new PhaseSpaceVertex2(goal, goal_speed, 0);
        PhaseSpaceVertex2 psn_intersection = psn_start;
        Queue queue_start = new LinkedList<PhaseSpaceVertex2>();
        Queue queue_goal = new LinkedList<PhaseSpaceVertex2>();
        queue_start.add(psn_start);
        queue_goal.add(psn_goal);
        boolean intersectionFound_start = false;
        boolean intersectionFound_goal = false;

        //Find intersection of two BFS'
        while (!(intersectionFound_start || intersectionFound_goal)) {

            //First BFS
            PhaseSpaceVertex2 psn_in_queue_start = (PhaseSpaceVertex2) queue_start.remove();
            psn_in_queue_start.nextTransitions2(resolution, nb_transitions);
            delete_unwanted_children(psn_in_queue_start.nextTransitions, copy_nodelist, visited_start, tp.getWidth(), tp.getHeight());
            for (int i = 0; i < psn_in_queue_start.nextTransitions.length; i++) {
                PhaseSpaceVertex2 psn_current = (PhaseSpaceVertex2) psn_in_queue_start.nextTransitions[i];
                if (psn_current != null) {
                    if (!visited_start.contains(psn_current)) {
                        if (psn_current.passedBy(checkpoint, sensingrange)) {
                            psn_current.visitedCheckpoint = 1;
                        }
                        if (psn_current.contains_reversedNextTransition(visited_goal, resolution, nb_transitions)) {
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
            }

            if (intersectionFound_start) //No need to go in second BFS if an intersection is already found
                break;

            //Second BFS
            PhaseSpaceVertex2 psn_in_queue_goal = (PhaseSpaceVertex2) queue_goal.remove();
            psn_in_queue_goal.nextTransitions2(resolution, nb_transitions);
            delete_unwanted_children(psn_in_queue_goal.nextTransitions, copy_nodelist, visited_goal, tp.getWidth(), tp.getHeight());
            for (int i = 0; i < psn_in_queue_goal.nextTransitions.length; i++) {
                PhaseSpaceVertex2 psn_current = (PhaseSpaceVertex2) psn_in_queue_goal.nextTransitions[i];
                if (psn_current != null) {
                    if (!visited_start.contains(psn_current)) {
                        if (psn_current.passedBy(checkpoint, sensingrange)) {
                            psn_current.visitedCheckpoint = 1;
                        }
                        if (psn_current.contains_reversedNextTransition(visited_start, resolution, nb_transitions)) {
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
        }

        //Finding and returning the correct trajectory
        return reconstruct_trajectory(intersectionFound_start, intersectionFound_goal, psn_intersection, psn_start, psn_goal, checkpoint, visited_start, visited_goal, nb_transitions);
    }

    private static List<PhaseSpaceVertex2> reconstruct_trajectory(boolean intersectionFound_start, boolean intersectionFound_goal,
                                                                  PhaseSpaceVertex2 psn_intersection, PhaseSpaceVertex2 psn_start,
                                                                  PhaseSpaceVertex2 psn_goal, Point checkpoint, Set<PhaseSpaceVertex> visited_start,
                                                                  Set<PhaseSpaceVertex> visited_goal, int nb_transitions) {
        if (intersectionFound_start) {
            if (psn_intersection.visitedCheckpoint == 1)  //if the start side BFS found the intersection, and it did pass by checkpoint
                return reconstruct_trajectory_start_B(psn_intersection, psn_start);
            else  //if the start side BFS found the intersection, and it did not pass by checkpoint
                return reconstruct_trajectory_start_notB(psn_intersection, checkpoint, visited_goal, nb_transitions, psn_start);
        } else {
            if (psn_intersection.visitedCheckpoint == 1)  //if the goal side BFS found the intersection, and it did pass by checkpoint
                return reconstruct_trajectory_goal_B(psn_intersection, checkpoint, visited_start, nb_transitions, psn_start);
            else //if the goal side BFS found the intersection, and it did not pass by checkpoint
                return reconstruct_trajectory_goal_notB(psn_intersection, visited_start, nb_transitions, psn_start);
        }
    }

    private static List<PhaseSpaceVertex2> reconstruct_trajectory_start_B(PhaseSpaceVertex2 psn_intersection, PhaseSpaceVertex2 psn_start) {
        List<PhaseSpaceVertex2> result = new ArrayList<>();
        boolean checkpointFound = false;
        while (!checkpointFound) {
            if (((PhaseSpaceVertex2) (psn_intersection.previousTransition)).visitedCheckpoint == 0)
                checkpointFound = true;
            else
                psn_intersection = (PhaseSpaceVertex2) psn_intersection.previousTransition;
        }

        boolean startFound = false;
        while (!startFound) {
            if (psn_intersection.sameLocation(psn_start) && psn_intersection.sameSpeed(psn_start))
                startFound = true;
            else {
                result.add(psn_intersection);
                PhaseSpaceVertex2 debugging = psn_intersection.clone();
                psn_intersection = (PhaseSpaceVertex2) psn_intersection.previousTransition;
                if (debugging.equals(psn_intersection))
                    break;
            }
        }
        Collections.reverse(result);
        return result;
    }

    private static List<PhaseSpaceVertex2> reconstruct_trajectory_start_notB(PhaseSpaceVertex2 psn_intersection, Point checkpoint,
                                                                             Set<PhaseSpaceVertex> visited_goal, int nb_transitions, PhaseSpaceVertex2 psn_start) {
        List<PhaseSpaceVertex2> result = new ArrayList<>();
        boolean startFound = false;
        PhaseSpaceVertex2 psn = psn_intersection;
        while (!startFound) {
            if (psn.sameLocation(psn_start) && psn.sameSpeed(psn_start))
                startFound = true;
            else {
                result.add(psn);
                PhaseSpaceVertex2 debugging = psn.clone();
                psn = (PhaseSpaceVertex2) psn.previousTransition;
                if (psn.equals(debugging))
                    break;
            }
        }
        Collections.reverse(result);
        result.remove(result.size() - 1);
        boolean checkpointFound = false;
        PhaseSpaceVertex2 psn_other_BFS = psn_intersection.find_reversedNextTransition(resolution, 1, visited_goal, nb_transitions);

        while (!checkpointFound) {
            if (psn_other_BFS.sameLocation(checkpoint)) {
                checkpointFound = true;
                PhaseSpaceVertex2 newResult = psn_other_BFS.clone();
                newResult.dx = (short) -newResult.dx;
                newResult.dy = (short) -newResult.dy;
                result.add(newResult);
            } else if (((PhaseSpaceVertex2) (psn_other_BFS.previousTransition)).visitedCheckpoint == 0) {
                checkpointFound = true;
                PhaseSpaceVertex2 newResult = psn_other_BFS.clone();
                newResult.dx = (short) -newResult.dx;
                newResult.dy = (short) -newResult.dy;
                result.add(newResult);
                PhaseSpaceVertex2 other_checkpoint = ((PhaseSpaceVertex2) (psn_other_BFS.previousTransition)).clone();
                other_checkpoint.dx = (short) -other_checkpoint.dx;
                other_checkpoint.dy = (short) -other_checkpoint.dy;
                result.add(other_checkpoint);
            } else {
                PhaseSpaceVertex2 newResult = psn_other_BFS.clone();
                newResult.dx = (short) -newResult.dx;
                newResult.dy = (short) -newResult.dy;
                result.add(newResult);
                psn_other_BFS = (PhaseSpaceVertex2) psn_other_BFS.previousTransition;
            }
        }
        return result;
    }
    private static List<PhaseSpaceVertex2> reconstruct_trajectory_goal_B(PhaseSpaceVertex2 psn_intersection, Point checkpoint,
                                                                         Set<PhaseSpaceVertex> visited_start, int nb_transitions,
                                                                         PhaseSpaceVertex2 psn_start) {
        List<PhaseSpaceVertex2> result = new ArrayList<>();
        PhaseSpaceVertex2 psn_other_BFS;

        if (psn_intersection.sameLocation(checkpoint)) {
            psn_other_BFS = psn_intersection.find_reversedNextTransition(resolution, 1, visited_start, nb_transitions);
        } else {
            psn_other_BFS = psn_intersection.find_reversedNextTransition(resolution, 0, visited_start, nb_transitions);
            if (psn_other_BFS == null)
                psn_other_BFS = psn_intersection.find_reversedNextTransition(resolution, 1, visited_start, nb_transitions);
        }
        boolean startFound = false;
        while (!startFound) {
            if (psn_other_BFS.sameLocation(psn_start) && psn_other_BFS.sameSpeed(psn_start))
                startFound = true;
            else {
                result.add(psn_other_BFS);
                PhaseSpaceVertex2 debugging = psn_other_BFS.clone();
                psn_other_BFS = (PhaseSpaceVertex2) psn_other_BFS.previousTransition;
                if (debugging.equals(psn_other_BFS))
                    break;
            }
        }
        Collections.reverse(result);
        result.remove(result.size() - 1);

        boolean checkpointFound = false;
        while (!checkpointFound) {
            if (psn_intersection.sameLocation(checkpoint)) {
                checkpointFound = true;
                PhaseSpaceVertex2 newResult = psn_intersection.clone();
                newResult.dx = (short) -newResult.dx;
                newResult.dy = (short) -newResult.dy;
                result.add(newResult);
            } else if (((PhaseSpaceVertex2) (psn_intersection.previousTransition)).visitedCheckpoint == 0) {
                checkpointFound = true;
                PhaseSpaceVertex2 newResult = psn_intersection.clone();
                newResult.dx = (short) -newResult.dx;
                newResult.dy = (short) -newResult.dy;
                result.add(newResult);
                PhaseSpaceVertex2 other_checkpoint = ((PhaseSpaceVertex2) (psn_intersection.previousTransition)).clone();
                other_checkpoint.dx = (short) -other_checkpoint.dx;
                other_checkpoint.dy = (short) -other_checkpoint.dy;
                result.add(other_checkpoint);
            } else {
                PhaseSpaceVertex2 newResult = psn_intersection.clone();
                newResult.dx = (short) -newResult.dx;
                newResult.dy = (short) -newResult.dy;
                result.add(newResult);
                psn_intersection = (PhaseSpaceVertex2) psn_intersection.previousTransition;
            }
        }
        return result;
    }

    private static List<PhaseSpaceVertex2> reconstruct_trajectory_goal_notB(PhaseSpaceVertex2 psn_intersection, Set<PhaseSpaceVertex> visited_start,
                                                                            int nb_transitions, PhaseSpaceVertex2 psn_start) {
        List<PhaseSpaceVertex2> result = new ArrayList<>();
        PhaseSpaceVertex2 psn_other_BFS = psn_intersection.find_reversedNextTransition(resolution, 1, visited_start, nb_transitions);
        boolean checkpointFound = false;

        while (!checkpointFound) {
            if (((PhaseSpaceVertex2) (psn_other_BFS.previousTransition)).visitedCheckpoint == 0) {
                checkpointFound = true;

            } else {
                psn_other_BFS = (PhaseSpaceVertex2) psn_other_BFS.previousTransition;
            }
        }

        boolean startFound = false;
        while (!startFound) {
            if (psn_other_BFS.sameLocation(psn_start) && psn_other_BFS.sameSpeed(psn_start)) {
                startFound = true;
            } else {
                result.add(psn_other_BFS);
                PhaseSpaceVertex2 debugging = psn_other_BFS.clone();
                psn_other_BFS = (PhaseSpaceVertex2) psn_other_BFS.previousTransition;
                if (psn_other_BFS.equals(debugging))
                    break;
            }
        }
        Collections.reverse(result);
        return result;
    }

    public static void delete_unwanted_children(PhaseSpaceVertex[] children, List<Point> nodelist, Set<PhaseSpaceVertex> visited, int bbox_width, int bbox_height) {
        delete_already_visited_children(children, visited);
        delete_children_out_of_bounding_box(children, bbox_width, bbox_height);
        delete_fleeing_children(children, nodelist);
    }

    public static void delete_already_visited_children(PhaseSpaceVertex[] children, Set<PhaseSpaceVertex> visited) {
        for (int i = 0; i < children.length; i++) {
            if (children[i] != null)
                if (visited.contains(children[i]))
                    children[i] = null; //can be put to the already visited psn in the set visited, but not necessary for BFS, and is costly.
        }
    }

    public static void delete_children_out_of_bounding_box(PhaseSpaceVertex[] children, int bbox_width, int bbox_height) {
        for (int i = 0; i < children.length; i++) {
            if (children[i] != null)
                if (children[i].getLocation().getX() < 0 || children[i].getLocation().getX() > bbox_width ||
                        children[i].getLocation().getY() < 0 || children[i].getLocation().getY() > bbox_height)
                    children[i] = null;
        }
    }

    public static void delete_fleeing_children(PhaseSpaceVertex[] children, List nodelist) {
        int central_psn_id = children.length / 2;
        PhaseSpaceVertex central_psn = children[central_psn_id];
        if (central_psn != null)
            for (int i = 0; i < children.length; i++) {
                if (children[i] != null)
                    if (fleeing(children[i], central_psn, nodelist))
                        children[i] = null;
            }
    }

    public static boolean fleeing(PhaseSpaceVertex psn, PhaseSpaceVertex central_psn, List<Point> nodelist) {
        PhaseSpaceVertex parent = psn.previousTransition;
        for (Point p : nodelist) {
            double distance = psn.getLocation().distance(p);
            double central_distance = central_psn.getLocation().distance(p);
            double parent_distance = parent.getLocation().distance(p);
            if (distance <= central_distance || distance <= parent_distance)
                return false;
        }
        return true;
    }
}
