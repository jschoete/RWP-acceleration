package video;

import io.jbotsim.core.Topology;

/**
 * Added by R. Laplace on 01/05/2019 (approx.).
 */
public class VideoHelper {
    public static Topology generateTopology() {
        return generateTopology720();
    }
    public static Topology generateTopology720() {
        return new Topology(1280,720);
    }

    public static String useOpenGL() {
        return System.setProperty("sun.java2d.opengl", "true");
    }
}
