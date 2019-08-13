package environment;

import algorithm.Inter_check_2BFS;
import io.jbotsim.core.Node;
import io.jbotsim.core.Topology;
import io.jbotsim.core.event.ClockListener;
import io.jbotsim.core.event.CommandListener;
import io.jbotsim.ui.JViewer;
import io.jbotsim.ui.painting.BackgroundPainter;
import io.jbotsim.ui.painting.UIComponent;
//import video.JBackgroundPainterHD;
//import video.VideoHelper;

import java.awt.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Created by A. Casteigts before 16/02/17, modified by J. Schoeters afterwards.
 * In this class, we create a field with target nodes to visit,
 * and a realistic drone trying to visit these nodes according to some acceleration constraints.
 */
public class Main implements CommandListener, ClockListener, BackgroundPainter {
    Topology tp;
//    private RenderingHints rh ;
    io.jbotsim.core.Point startPoint = new io.jbotsim.core.Point(500,400);
    static List<io.jbotsim.core.Point> visit_order = new ArrayList<>();
    static List<io.jbotsim.core.Point> trajectory_locations = new ArrayList<>();
    public static double resolution;
    Drone drone;
    String command_precompute = "Start precomputation";

    public static void main(String[] args) {
        new Main();
    }

    public Main() {
        //Display
//        rh = new RenderingHints(
//                RenderingHints.KEY_ANTIALIASING,
//                RenderingHints.VALUE_ANTIALIAS_ON);
//        rh.add(new RenderingHints(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_SPEED));
//        VideoHelper.useOpenGL();
//        tp = VideoHelper.generateTopology();
        tp.setTimeUnit(5);
        tp.addClockListener(this);
        JViewer jv = new JViewer(tp);
//        jv.getJTopology().setDefaultBackgroundPainter(new JBackgroundPainterHD());

        //Initialization
        resolution = 0.35 * VectorNode.DEVIATION;
        placeTargets(tp);
        drone = (Drone) tp.getNodes().get(tp.getNodes().size() - 1);
        List<io.jbotsim.core.Point> POIs = new ArrayList<>();
        for (int i = 0; i < tp.getNodes().size() - 1; i++)
            POIs.add(tp.getNodes().get(i).getLocation());
        Collections.shuffle(POIs);
        POIs.add(0, drone.getLocation());
        visit_order = POIs;
        for (int i = 0; i < visit_order.size(); i++) {
            if (i < 3) {
                for (Node n : tp.getNodes()) {
                    if (n.getLocation().equals(visit_order.get(i))) {
                        if (n instanceof Target) {
                            Target t = (Target) n;
                            t.makeVisible();
                        }
                    }
                }
            }
        }

        tp.addCommandListener(this);
        tp.addCommand(command_precompute);
        System.out.println("Select '" + command_precompute + "' (right click in JBotSim window) !");
    }

    /**
     * One can remove or add targets if needed here.
     * For random placement of targets, one can use coordinates x: -1, y: -1
     */
    public void placeTargets(Topology tp){
        tp.addNode(104.0, 523.0, new Target());
        tp.addNode(842.0, 577.0, new Target());
        tp.addNode(108.0, 120.0, new Target());
        tp.addNode(443.0, 551.0, new Target());
        tp.addNode(356.0, 289.0, new Target());
//        tp.addNode(629.0, 661.0, new Target());
//        tp.addNode(429.0, 115.0, new Target());
//        tp.addNode(668.0, 399.0, new Target());
//        tp.addNode(368.0, 579.0, new Target());
//        tp.addNode(862.0, 267.0, new Target());
//        tp.addNode(529.0, 656.0, new Target());
//        tp.addNode(646.0, 423.0, new Target());
//        tp.addNode(241.0, 619.0, new Target());
//        tp.addNode(661.0, 176.0, new Target());
//        tp.addNode(116.0, 442.0, new Target());
//        tp.addNode(476.0, 350.0, new Target());
//        tp.addNode(448.0, 130.0, new Target());
//        tp.addNode(437.0, 444.0, new Target());
//        tp.addNode(736.0, 420.0, new Target());
//        tp.addNode(638.0, 575.0, new Target());

        tp.addNode(500, 400, new Drone());
        relocateTargets(tp.getNodes(), (int)resolution);
        relocate(startPoint, (int) resolution);
    }

    public static void relocateTargets(List<Node> nodelist, int deviation){
        for (Node n : nodelist){
            if (n.getX() % deviation < deviation/2){
                n.setLocation(n.getX() - (n.getX() % deviation), n.getY());
            }
            else
                n.setLocation(n.getX() - (n.getX() % deviation) + deviation, n.getY());
            if (n.getY() % deviation < deviation/2){
                n.setLocation(n.getX(), n.getY() - (n.getY() % deviation));
            }
            else
                n.setLocation(n.getX(), n.getY() - (n.getY() % deviation) + deviation);
        }
    }

    public static void relocate(io.jbotsim.core.Point point, int resolution){
        if (point.getX() % resolution < resolution / 2) {
            point.setLocation(point.getX() - (point.getX() % resolution), point.getY());
        } else
            point.setLocation(point.getX() - (point.getX() % resolution) + resolution, point.getY());
        if (point.getY() % resolution < resolution / 2) {
            point.setLocation(point.getX(), point.getY() - (point.getY() % resolution));
        } else
            point.setLocation(point.getX(), point.getY() - (point.getY() % resolution) + resolution);
    }

    public boolean hasReturned(Drone drone){
        if (drone.getLocation().equals(startPoint) &&
                drone.vector.distance(new io.jbotsim.core.Point(0,0))<resolution &&
                (int) Math.ceil(tp.getTime() / 10.0) != 0){
            return true;
        }else
            return false;
    }

    @Override
    public void onClock() {
        if (hasReturned(drone)) {
            tp.pause();
        }
    }

    @Override
    public void paintBackground(UIComponent uiComponent, Topology topology) {
        Graphics2D g2d = (Graphics2D) uiComponent.getComponent();
//        g2d.setRenderingHints(rh);
        try {
            Color c = new Color(0, 0, 0);  //black color
            int g = 0;
            io.jbotsim.core.Point l0 = drone.getLocation();
            io.jbotsim.core.Point l1 = drone.getLocation();
            if (drone.points_visited.size() > 0) {
                l1 = (io.jbotsim.core.Point) drone.points_visited.get(drone.points_visited.size() - 1);
            }
            g2d.setColor(c);
            g2d.drawLine((int) l0.getX(), (int) l0.getY(), (int) l1.getX(), (int) l1.getY());
            for (int i = drone.points_visited.size() - 1; i > 0; i--) {
                while (c.getGreen() < 200 && i != 0) {
                    l0 = (io.jbotsim.core.Point) drone.points_visited.get(i);
                    l1 = (io.jbotsim.core.Point) drone.points_visited.get(i - 1);
                    g2d.setColor(c);
                    g2d.drawLine((int) l0.getX(), (int) l0.getY(), (int) l1.getX(), (int) l1.getY());
                    g += 25;
                    c = new Color(g, g, g);
                    i--;
                }
            }
        } catch (Exception e){
            //nothing
        }
    }

    @Override
    public void onCommand(String command) {
        if (command.equals(command_precompute)){
            //Precomputation trajectory
            System.out.println("Precomputing trajectory, please wait...");
            trajectory_locations = Inter_check_2BFS.getTrajectory(visit_order, startPoint, tp, resolution, 9);
            System.out.println("Done ! click 'Start execution' in JBotSim window !");

        }
    }
}