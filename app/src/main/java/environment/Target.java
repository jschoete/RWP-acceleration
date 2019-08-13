package environment;

import io.jbotsim.core.Node;
import io.jbotsim.ui.icons.Icons;

/**
 * Created by J. Schoeters on 15/04/19 (approx.)
 * This class represents target nodes for the drone to visit.
 */
public class Target extends Node {

    private boolean visible;

    public Target() {
        super();
        this.visible = false;
        setIcon("/transparent.png");
        setIconSize(30);
        disableWireless();
    }

    public boolean isVisible(){
        return this.visible;
    }

    public void makeVisible() {
        setIcon(Icons.FLAG);
//        setIcon("/target.png");
        this.visible = true;
    }

    public void makeInvisible() {
        setIcon("/transparent.png");
        this.visible = false;
    }
}

