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

import com.carrotsearch.hppc.IntIndexedContainer;
import com.graphhopper.routing.AStar;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.CarFlagEncoder;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.FastestWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Directory;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.storage.GraphStorage;
import com.graphhopper.util.EdgeIteratorState;

import es.usc.citius.hipster.algorithm.Hipster;
import es.usc.citius.hipster.graph.GraphBuilder;
import es.usc.citius.hipster.graph.GraphSearchProblem;
import es.usc.citius.hipster.graph.HipsterDirectedGraph;
import es.usc.citius.hipster.graph.HipsterGraph;
import es.usc.citius.hipster.model.problem.SearchProblem;

public class Main {

	public static void graphHopper(Graph g, int VERTICES_COUNT) {
		FlagEncoder encoder = new CarFlagEncoder();
		EncodingManager em = EncodingManager.create(encoder);
		com.graphhopper.storage.GraphBuilder gb = new com.graphhopper.storage.GraphBuilder(em);
		GraphHopperStorage graph = gb.create();

		for (Vertex vertex : g.vertices) {
			graph.getNodeAccess().setNode(vertex.id, 0, 0);
		}

		for (Edge e : g.edges) {
			EdgeIteratorState edge = graph.edge(e.firstId, e.secondId, e.weight/1000, true);
		}

	    System.out.println(graph.getEdges());
		Weighting weighting = new FastestWeighting(encoder);
		AStar a = new AStar(graph, weighting, TraversalMode.EDGE_BASED);
		Path p = a.calcPath(0, VERTICES_COUNT-1);
		System.out.println(p.toString());
	}

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
		double distanceToC = result.getDistance(VERTICES_COUNT - 1);
		Iterable<DirectedWeightedEdge<Integer, Double>> way = result.getPath(VERTICES_COUNT - 1);
		System.out.println(way.toString());
		System.out.println("The distance to " + Integer.toString(VERTICES_COUNT - 1) + " is: " + distanceToC);
	}

	public static void hipster4j_astar(Graph g, int VERTICES_COUNT) {

		es.usc.citius.hipster.graph.GraphBuilder<Integer, Double> gb = es.usc.citius.hipster.graph.GraphBuilder
				.<Integer, Double>create();

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

		System.out.println(Hipster.createAStar(p).search(VERTICES_COUNT - 1));
	}

	public static void jgrapht(final Graph g, int VERTICES_COUNT, Boolean isDijkstra) {
		SimpleWeightedGraph<Integer, DefaultWeightedEdge> graph = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);
		for (Vertex vertex : g.vertices) {
			graph.addVertex(vertex.id);
		}
		for (Edge edge : g.edges) {
			DefaultWeightedEdge dwe = graph.addEdge(edge.firstId, edge.secondId);
			graph.setEdgeWeight(dwe, edge.weight);
		}
		
		if(isDijkstra) {
			DijkstraShortestPath<Integer, DefaultWeightedEdge> dijkstraShortestPath = new DijkstraShortestPath<Integer, DefaultWeightedEdge>(
					graph);
			List<Integer> shortestPath = dijkstraShortestPath.getPath(0, VERTICES_COUNT - 1).getVertexList();
			System.out.println(shortestPath.toString());
		}
		else {
			AStarAdmissibleHeuristic<Integer> heuristic = new AStarAdmissibleHeuristic<Integer>() {
				@Override
				public double getCostEstimate(Integer sourceVertex, Integer targetVertex) {
					//return 	g.vertices.get(sourceVertex).findDistance(g.vertices.get(targetVertex));
					return 0.0;
				}
			};
			AStarShortestPath<Integer, DefaultWeightedEdge> astarShortestPath = new AStarShortestPath<Integer, DefaultWeightedEdge>(
					graph, heuristic);
			List<Integer> shortestPath = astarShortestPath.getPath(0, VERTICES_COUNT - 1).getVertexList();
			System.out.println(shortestPath.toString());
		}
		

	}

	public static void main(String[] args) throws FileNotFoundException, IOException {

		// TODO Auto-generated method stub

		long startTime, endTime, timeElapsed;
		int VERTICES_COUNT = 1000000;
		// Let's construct a simple graph.
		String path = "C:/Users/manke/Documents/TMC projekt/triangular/";
		Graph g = Parser.parse(path + "data" + VERTICES_COUNT + ".txt", path + "vertices" + VERTICES_COUNT + ".txt");
		
		startTime = System.nanoTime();
		psjava_dijkstra(g, VERTICES_COUNT);
		endTime = System.nanoTime();
		timeElapsed = endTime - startTime;

		System.out.println("psjava execution time in milliseconds : " + timeElapsed / 1000000);

		/*startTime = System.nanoTime();
		jgrapht(g, VERTICES_COUNT, true);
		endTime = System.nanoTime();
		timeElapsed = endTime - startTime;

		System.out.println("jgrapht Dijkstra execution time in milliseconds : " + timeElapsed / 1000000);*/
		
		startTime = System.nanoTime();
		jgrapht(g, VERTICES_COUNT, false);
		endTime = System.nanoTime();
		timeElapsed = endTime - startTime;

		System.out.println("jgrapht A* execution time in milliseconds : " + timeElapsed / 1000000);
		
		startTime = System.nanoTime();
		graphHopper(g, VERTICES_COUNT);
		endTime = System.nanoTime();
		timeElapsed = endTime - startTime;

		System.out.println("graphHopper execution time in milliseconds : " + timeElapsed / 1000000);

		startTime = System.nanoTime();
		hipster4j_astar(g, VERTICES_COUNT);
		endTime = System.nanoTime();
		timeElapsed = endTime - startTime;

		System.out.println("hipster4j execution time in milliseconds : " + timeElapsed / 1000000);
		// graphHopper();
	}

}
