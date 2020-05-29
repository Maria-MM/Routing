import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;

public class Parser {
	static public Graph parse(String file_name, String file_name2) throws FileNotFoundException, IOException {
		Graph graph = new Graph();
		graph.vertices = new ArrayList<Vertex>();
		graph.edges = new ArrayList<Edge>();
		try (BufferedReader br = new BufferedReader(new FileReader(file_name2))) {
			String line;
			HashSet<Integer> set = new HashSet<Integer>();
			line = br.readLine();
			int count = 0;
			while ((line = br.readLine()) != null) {
				// process the line.
				String[] parts = line.split(" ");
				//if(count%1000 == 0)
					//System.out.println(count + " " + line);
				//System.out.println(parts.length);
				Integer a = Integer.parseInt(parts[0]);
				Integer b = Integer.parseInt(parts[1]);
				Vertex v = new Vertex(count);
				v.x = a;
				v.y = b;
				graph.vertices.add(v);
				count++;
			}
		}
		try (BufferedReader br = new BufferedReader(new FileReader(file_name))) {
			String line;
			HashSet<Integer> set = new HashSet<Integer>();
			line = br.readLine();
			while ((line = br.readLine()) != null) {
				// process the line.
				String[] parts = line.split(" ");
				//System.out.println(line);
				//System.out.println(parts.length);
				Integer a = Integer.parseInt(parts[0]);
				Integer b = Integer.parseInt(parts[1]);
				Double w = Double.parseDouble(parts[2]); 
				/*if(!set.contains(a)) {
					set.add(a);
					graph.vertices.add(new Vertex(a));
				}
				if(!set.contains(b)) {
					set.add(b);
					graph.vertices.add(new Vertex(b));
				}*/
				graph.edges.add(new Edge(a, b, w));
			}
		}
		
		System.out.println(graph.edges.size()+" edges found");
		return graph;
	}
	
}
