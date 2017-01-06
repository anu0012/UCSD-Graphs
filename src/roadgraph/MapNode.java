package roadgraph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode> {
    private GeographicPoint location;
    private double length = 0;
    private double predictedLength = 0;
    private HashSet<MapEdge> edgeList; // = new ArrayList<>();



    public MapNode(GeographicPoint location){
        this.location = location;
        edgeList = new HashSet<>();
    }

    public GeographicPoint getLocation(){
        return location;
    }

    public HashSet<MapEdge> getEdgeList() {
        return edgeList;
    }

    public void addEdge(MapEdge newEdge){
        edgeList.add(newEdge);
    }

    public void addEdge(GeographicPoint endloc, String street, String roadtype, double distance ){
        MapEdge newEdge = new MapEdge(location,endloc,street,roadtype, distance);
        edgeList.add(newEdge);
    }

    public List<GeographicPoint> getNeighbors(){
        List<GeographicPoint> neighbors = new ArrayList<>();
        for(MapEdge e : edgeList){
            neighbors.add(e.getEnd());
        }


        return neighbors;
    }

    public int compareTo(MapNode node) {
        // TODO Auto-generated method stub
        //return (this.getLength()).compareTo(node.getLength());
        return Double.compare(this.getLength(),node.getLength());

    }


    public double  getLength() {
        // TODO Auto-generated method stub
        return length;
    }
    public void setLength(double len){
        length = len;
    }

    public double getPredictedLength() {
        return predictedLength;
    }

    public void setPredictedLength(double predictedLength) {
        this.predictedLength = predictedLength;
    }


    //public Set<> getNeighbors


}