package video;

import io.jbotsim.core.Node;
import io.jbotsim.core.Topology;
import io.jbotsim.ui.painting.BackgroundPainter;
import io.jbotsim.ui.painting.UIComponent;

import java.awt.*;

/**
 * Added by R. Laplace on 01/05/2019 (approx.).
 */
public class JBackgroundPainterHD implements BackgroundPainter {


    private RenderingHints rh ;
    private  BasicStroke stroke;

    protected static final int GRAY_LEVEL = 5;
    protected static Color color;

    public JBackgroundPainterHD() {
        rh = new RenderingHints(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);
        rh.add(new RenderingHints(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_SPEED));
        stroke = new BasicStroke(1);
        color = new Color(GRAY_LEVEL, GRAY_LEVEL, GRAY_LEVEL);
    }
    @Override
    public void paintBackground(UIComponent uiComponent, Topology tp) {
        Graphics2D g2d = (Graphics2D) uiComponent.getComponent();
        setStroke(g2d, tp);
        setRenderingHints(g2d, tp);
        setColor(g2d, tp);

        for (Node n : tp.getNodes()) {
            drawSensingRange(g2d, n);
        }
    }

    protected void drawSensingRange(Graphics2D g2d, Node n) {
        double sR = n.getSensingRange();
        if (sR > 0) {
            g2d.setColor(Color.gray);
            g2d.drawOval((int) n.getX() - (int) sR, (int) n.getY() - (int) sR, 2 * (int) sR, 2 * (int) sR);
        }
    }


    protected void setColor(Graphics2D g2d, Topology topology) {
        g2d.setColor(Color.gray);
    }

    protected void setStroke(Graphics2D g2d, Topology topology) {
        g2d.setStroke(new BasicStroke(1));
    }

    protected void setRenderingHints(Graphics2D g2d, Topology topology) {
        // default does nothing
        g2d.setRenderingHints(rh);
    }
}
