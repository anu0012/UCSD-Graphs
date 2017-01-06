package roadgraph;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapEdge {
    private GeographicPoint start;
    private GeographicPoint end;
    private String streetname;
    private String roadtype;
    private double distance;

    /**public MapEdge(){
     start = null;
     end = null;
     streetname = null;
     distance = 0;

     }*/

    public MapEdge(GeographicPoint start, GeographicPoint end, String streetname, String roadtype, double distance){
        this.start = start;
        this.end = end;
        this.streetname = streetname;
        this.roadtype = roadtype;
        this.distance = distance;
    }


    public GeographicPoint getStart() {
        return start;
    }
    public void setStart(GeographicPoint start) {
        this.start = start;
    }
    public GeographicPoint getEnd() {
        return end;
    }
    public void setEnd(GeographicPoint end) {
        this.end = end;
    }
    public String getStreetname() {
        return streetname;
    }
    public void setStreetname(String streetname) {
        this.streetname = streetname;
    }
    public double getDistance() {
        return distance;
    }
    public void setDistance(double distance) {
        this.distance = distance;
    }


}