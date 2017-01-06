/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 *
 */
public class MapGraph{
	private int numVertices;
	private int numEdges;
	private Set<GeographicPoint> setVertices = new HashSet<>();
	private Map<GeographicPoint,ArrayList<GeographicPoint>> edgeMap = new HashMap<>();
	public static GeographicPoint st = null;
	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph()
	{
		numVertices = 0;
		numEdges = 0;
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return setVertices;
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}



	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		numVertices++;
		return setVertices.add(location);
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2.
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
						String roadType, double length) throws IllegalArgumentException {

		if (from == null || to == null || roadName == null || roadType == null || (Double)length == null) {
			throw new IllegalArgumentException();
		}
		if (length < 0) {
			throw new IllegalArgumentException();
		}
		numEdges++;
		if(edgeMap.containsKey(from)) {
			ArrayList<GeographicPoint> l = edgeMap.get(from);
			l.add(to);
			edgeMap.put(from, l);
		}
		else
		{
			ArrayList<GeographicPoint> l = new ArrayList<>();
			l.add(to);
			edgeMap.put(from, l);
		}
	}

	/** Find the path from start to goal using breadth first search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return bfs(start, goal, temp);
	}

	/** Find the path from start to goal using breadth first search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
									 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if(start == null || goal==null){
			return null;
		}
		Queue<GeographicPoint> queue = new LinkedList<>();
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint,GeographicPoint> parent = new HashMap<>();
		LinkedList<GeographicPoint> path = new LinkedList<>();
		queue.add(start);
		visited.add(start);

		while(!queue.isEmpty())
		{
			GeographicPoint curr = queue.poll();
			if(curr.getX() == goal.getX() && curr.getY() == goal.getY()) {
				GeographicPoint temp = goal;
				path.addFirst(temp);
				while(!temp.equals(start))
				{
					path.addFirst(parent.get(temp));
					temp = parent.get(temp);
				}
				return path;
			}
			if(edgeMap.get(curr) == null)
				continue;
			ArrayList<GeographicPoint> l = edgeMap.get(curr);
			for(int i=0;i<l.size();i++)
			{
				if(!visited.contains(l.get(i)))
				{
					visited.add(l.get(i));
					parent.put(l.get(i),curr);
					queue.add(l.get(i));
				}
			}
		}
		return null;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
		return dijkstra(start, goal, temp);
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	static HashMap<GeographicPoint,Double> distMap = new HashMap<>();
	public List<GeographicPoint> dijkstra(GeographicPoint start,
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			System.out.println("No path exists.");
			return null;
		}
		Comparator<GeographicPoint> comparator = new DistanceComparator();
		PriorityQueue<GeographicPoint> priorityQueue = new PriorityQueue<>(10,comparator);
		HashSet<GeographicPoint> visited = new HashSet<>();
		HashMap<GeographicPoint,GeographicPoint> parent = new HashMap<>();

		LinkedList<GeographicPoint> path = new LinkedList<>();
		Iterator iter = setVertices.iterator();
		int visitedNodes = 0;
		st = start;
		while(iter.hasNext())
		{
			distMap.put((GeographicPoint) iter.next(),Double.POSITIVE_INFINITY);
		}
		distMap.put(start,0.0);
		priorityQueue.add(start);
		while(!priorityQueue.isEmpty())
		{
			GeographicPoint curr = priorityQueue.poll();
			visitedNodes++;
			if(!visited.contains(curr))
			{
				visited.add(curr);
				if(curr.getX() == goal.getX() && curr.getY() == goal.getY())
				{
					System.out.println("Dijkstra: "+visitedNodes);
					GeographicPoint temp = goal;
					path.addFirst(temp);
					while(!temp.equals(start))
					{
						path.addFirst(parent.get(temp));
						temp = parent.get(temp);
					}
					return path;
				}
				if(edgeMap.get(curr) == null)
					continue;
				ArrayList<GeographicPoint> l = edgeMap.get(curr);
				for(int i=0;i<l.size();i++)
				{
					GeographicPoint n = l.get(i);
					if(!visited.contains(n))
					{
						if(distMap.get(curr)+curr.distance(n) < distMap.get(n))
						{
							//update n's distance
							distMap.put(n,distMap.get(curr)+curr.distance(n));
							parent.put(n,curr);
							priorityQueue.add(n);
						}
					}
				}
			}
		}
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return aStarSearch(start, goal, temp);
	}

	/** Find the path from start to goal using A-Star search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start,
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			System.out.println("No path exists.");
			return null;
		}
		Comparator<GeographicPoint> comparator = new DistanceComparator();
		PriorityQueue<GeographicPoint> priorityQueue = new PriorityQueue<>(10,comparator);
		HashSet<GeographicPoint> visited = new HashSet<>();
		HashMap<GeographicPoint,GeographicPoint> parent = new HashMap<>();
		//HashMap<GeographicPoint,Double> distMap = new HashMap<>();
		HashMap<GeographicPoint,Double> projDist = new HashMap<>();
		LinkedList<GeographicPoint> path = new LinkedList<>();
		Iterator iter = setVertices.iterator();
		int visitedNodes = 0;
		st = start;
		while(iter.hasNext())
		{
			GeographicPoint x = (GeographicPoint) iter.next();
			projDist.put(x,Double.POSITIVE_INFINITY);
			distMap.put(x,Double.POSITIVE_INFINITY);
		}
		projDist.put(start,0.0);
		distMap.put(start,0.0);
		priorityQueue.add(start);
		while(!priorityQueue.isEmpty())
		{
			GeographicPoint curr = priorityQueue.poll();
			visitedNodes++;
			if(!visited.contains(curr))
			{
				visited.add(curr);
				if(curr.getX() == goal.getX() && curr.getY() == goal.getY())
				{
					System.out.println("Astar: "+visitedNodes);
					GeographicPoint temp = goal;
					path.addFirst(temp);
					while(!temp.equals(start))
					{
						path.addFirst(parent.get(temp));
						temp = parent.get(temp);
					}
					return path;
				}
				if(edgeMap.get(curr) == null)
					continue;
				ArrayList<GeographicPoint> l = edgeMap.get(curr);
				for(int i=0;i<l.size();i++)
				{
					GeographicPoint n = l.get(i);
					if(!visited.contains(n))
					{
						double distance = Math.abs(n.distance(goal));
						double tentDist = distMap.get(curr)+Math.abs(curr.distance(n));
						if(tentDist+distance < projDist.get(n))
						{
							//update n's distance
							distMap.put(n,tentDist);
							projDist.put(n,tentDist+distance);
							parent.put(n,curr);
							priorityQueue.add(n);
						}
					}
				}
			}
		}
		return null;
	}



	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		GeographicPoint g1 = new GeographicPoint(1.0, 1.0);
		GeographicPoint g2 = new GeographicPoint(8.0, -1.0);
		//System.out.println(firstMap.bfs(g1,g2));
		// You can use this method for testing.
/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);


		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		/* Here are some test cases you should try before you attempt
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);


		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/


		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);



	}

}
