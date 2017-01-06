package roadgraph;

import geography.GeographicPoint;

import java.util.Comparator;

/**
 * Created by anuragsharma on 31/10/16.
 */
public class DistanceComparator implements Comparator<GeographicPoint> {
    @Override
    public int compare(GeographicPoint x, GeographicPoint y)
    {
        /*if(x.distance(MapGraph.st)>y.distance(MapGraph.st))
            return 1;
        else if(x.distance(MapGraph.st)<y.distance(MapGraph.st))
            return -1;
        */
         //   return 0;
        return Double.compare(MapGraph.distMap.get(x),MapGraph.distMap.get(y));
    }
}
