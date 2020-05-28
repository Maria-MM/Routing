
public class Vertex {
	public Vertex(int id_) {
		// TODO Auto-generated constructor stub
		id = id_;
		x = y = 0;
	}

	public int id;
	public double x, y;
	double findDistance(Vertex v) {
		return (x-v.x)*(x-v.x)+(y-v.y)*(y-v.y);
	}
}

