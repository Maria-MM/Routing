import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.jgrapht.alg.interfaces.AStarAdmissibleHeuristic;
import org.jgrapht.alg.shortestpath.AStarShortestPath;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleGraph;
import org.jgrapht.graph.SimpleWeightedGraph;
import org.psjava.algo.graph.shortestpath.DijkstraAlgorithm;
import org.psjava.algo.graph.shortestpath.SingleSourceShortestPathResult;
import org.psjava.ds.graph.DirectedWeightedEdge;
import org.psjava.ds.graph.MutableDirectedWeightedGraph;
import org.psjava.ds.numbersystrem.DoubleNumberSystem;
import org.psjava.ds.numbersystrem.IntegerNumberSystem;
import org.psjava.goods.GoodDijkstraAlgorithm;


import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.PathWrapper;
import com.graphhopper.config.CHProfile;
import com.graphhopper.config.Profile;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.util.CarFlagEncoder;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.storage.GraphStorage;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.Instruction;
import com.graphhopper.util.InstructionList;
import com.graphhopper.util.PointList;
import com.graphhopper.util.gpx.GPXEntry;

import es.usc.citius.hipster.algorithm.Hipster;
import es.usc.citius.hipster.graph.GraphBuilder;
import es.usc.citius.hipster.graph.GraphSearchProblem;
import es.usc.citius.hipster.graph.HipsterDirectedGraph;
import es.usc.citius.hipster.graph.HipsterGraph;
import es.usc.citius.hipster.model.problem.SearchProblem;

public class Main {
	

	public static void graphHopper() {
		// create one GraphHopper instance
		GraphHopper hopper = new GraphHopperOSM().forServer();
		String osmFile = "C://Users//manke//Documents//TMC projekt//map.osm";
		hopper.setDataReaderFile(osmFile);
		// where to store graphhopper files?
		String graphFolder = "C://Users//manke//Documents//TMC projekt//graph";
		hopper.setGraphHopperLocation(graphFolder);
		hopper.setEncodingManager(EncodingManager.create("car"));

		// see docs/core/profiles.md to learn more about profiles
		hopper.setProfiles(
		    new Profile("car").setVehicle("car").setWeighting("fastest")
		);
		// this enables speed mode for the profile we call "car" here
		hopper.getCHPreparationHandler().setCHProfiles(
		    new CHProfile("car")
		);

		// now this can take minutes if it imports or a few seconds for loading
		// of course this is dependent on the area you import
		hopper.importOrLoad();

		// simple configuration of the request object
		double lonFrom=18.602856, lonTo=18.607562, latFrom=54.320462, latTo=54.323349;
		GHRequest req = new GHRequest(latFrom, lonFrom, latTo, lonTo).
		    // note that we have to specify which profile we are using even when there is only one like here
		    setProfile("car").
		    setLocale(Locale.US);
		GHResponse rsp = hopper.route(req);

		// first check for errors
		if(rsp.hasErrors()) {
		   // handle them!
		   // rsp.getErrors()
		   return;
		}

		// use the best path, see the GHResponse class for more possibilities.
		PathWrapper path = rsp.getBest();

		// points, distance in meters and time in millis of the full path
		PointList pointList = path.getPoints();
		double distance = path.getDistance();
		long timeInMs = path.getTime();

		InstructionList il = path.getInstructions();
		// iterate over every turn instruction
		for(Instruction instruction : il) {
			instruction.getDistance();
		}
	
	}
	
	/*public static void graphHopper2() {
		FlagEncoder encoder = new CarFlagEncoder();
		EncodingManager em = EncodingManager.create(encoder);
		GraphBuilder gb = new GraphBuilder(em);
				//.setLocation("graphhopper_folder").setStore(true);
		GraphStorage graph = gb.create();
		// Make a weighted edge between two nodes.
		EdgeIteratorState edge = graph.edge(fromId, toId);
		edge.setDistance(distance);
		edge.setFlags(encoder.setProperties(speed, true, true));
		// Flush to disc
		graph.flush();
		GraphStorage graph = gb.load();
	}*/
	
	public static void psjava_dijkstra(Graph g, int VERTICES_COUNT) {
		DoubleNumberSystem NS = DoubleNumberSystem.getInstance();
		MutableDirectedWeightedGraph<Integer, Double> graph = MutableDirectedWeightedGraph.create();
		for (Vertex vertex : g.vertices) {
			graph.insertVertex(vertex.id);
		}
		for (Edge edge : g.edges) {
			graph.addEdge(edge.firstId, edge.secondId, edge.weight);
			graph.addEdge(edge.secondId, edge.firstId, edge.weight);
		}

		// Calculate distances from a single source 'A'

		DijkstraAlgorithm dijkstra = GoodDijkstraAlgorithm.getInstance();
		SingleSourceShortestPathResult<Integer, Double, DirectedWeightedEdge<Integer, Double>> result = dijkstra
				.calc(graph, 0, NS);
		double distanceToC = result.getDistance(VERTICES_COUNT-1); 
		Iterable<DirectedWeightedEdge<Integer, Double>> way = result.getPath(VERTICES_COUNT-1);
		System.out.println(way.toString());
		System.out.println("The distance to " + Integer.toString(VERTICES_COUNT-1)+" is: " + distanceToC);
	}
	
	public static void hipster4j_astar(Graph g, int VERTICES_COUNT) {

		GraphBuilder<Integer, Double> gb = GraphBuilder.<Integer, Double>create();

		int count = 0;
		System.out.println("Hipster4j started to create graph");
		for (Edge edge : g.edges) {
			gb.connect(edge.firstId).to(edge.secondId).withEdge(edge.weight);
		}
		System.out.println("Hipster4j finished to create graph");
		HipsterGraph<Integer, Double> graph2 = gb.createUndirectedGraph();

		// Create the search problem. For graph problems, just use
		// the GraphSearchProblem util class to generate the problem with ease.
		
		SearchProblem p = GraphSearchProblem.startingFrom(0).in(graph2).takeCostsFromEdges().build();

		System.out.println(Hipster.createAStar(p).search(Integer.toString(VERTICES_COUNT-1)));
	}
	
	public static void jgrapht(final Graph g, int VERTICES_COUNT) {
		SimpleWeightedGraph<Integer, DefaultWeightedEdge> graph 
		  = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);
		for (Vertex vertex : g.vertices) {
			graph.addVertex(vertex.id);
		}
		for (Edge edge : g.edges) {
			DefaultWeightedEdge dwe = graph.addEdge(edge.firstId, edge.secondId);
			graph.setEdgeWeight(dwe, edge.weight);
		}
		AStarAdmissibleHeuristic<Integer> heuristic = new AStarAdmissibleHeuristic<Integer>() {
	        @Override
	        public double getCostEstimate(Integer sourceVertex, Integer targetVertex) {
	        	//return g.vertices.get(sourceVertex).findDistance(g.vertices.get(targetVertex));
	        	return 0.0;
	        }
	    };
		AStarShortestPath<Integer, DefaultWeightedEdge> astarShortestPath 
	      = new AStarShortestPath<Integer, DefaultWeightedEdge>(graph, heuristic);
	    List<Integer> shortestPath = astarShortestPath
	      .getPath(0,VERTICES_COUNT-1).getVertexList();
	    System.out.println(shortestPath.toString());

	}
	
	public static void main(String[] args) throws FileNotFoundException, IOException {

		// TODO Auto-generated method stub
		
		long startTime, endTime, timeElapsed;
		int VERTICES_COUNT=1000000;
		// Let's construct a simple graph.
		String path = "C:/Users/manke/Documents/TMC projekt/triangular/";
		Graph g = Parser.parse(path+"data"+VERTICES_COUNT+".txt", path+"vertices"+VERTICES_COUNT+".txt");
		startTime = System.nanoTime();
		psjava_dijkstra(g, VERTICES_COUNT);
		endTime = System.nanoTime();
		timeElapsed = endTime - startTime;

		System.out.println("psjava execution time in milliseconds : " + timeElapsed / 1000000);

		startTime = System.nanoTime();
		jgrapht(g, VERTICES_COUNT);
		endTime = System.nanoTime();
		timeElapsed = endTime - startTime;

		System.out.println("jgrapht execution time in milliseconds : " + timeElapsed / 1000000);
		
		startTime = System.nanoTime();
		hipster4j_astar(g, VERTICES_COUNT);
		endTime = System.nanoTime();
		timeElapsed = endTime - startTime;

		System.out.println("hipster4j execution time in milliseconds : " + timeElapsed / 1000000);
		//graphHopper();
	}

}
