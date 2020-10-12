package undirectedgraph;

import java.util.*;
import searchalgorithm.*;
import searchproblem.SearchProblem;
import searchproblem.State;

public class Graph {
	private HashMap<String,Vertex> vertices;
	private HashMap<Integer,Edge> edges;	
	private ArrayList<VertexSet> vSets;
	
	private long expansions;
	private long generated;
	private long repeated;
	private double time;
	
	public Graph() {

		this.vertices = new HashMap<String,Vertex>();
		this.edges = new HashMap<Integer,Edge>();
		this.vSets = new ArrayList<VertexSet>();
		this.expansions = 0;
		this.generated = 0;
		this.repeated = 0;
		this.time = 0;
	}
	
	
	
	public Node searchSeqSolution(String initLabel, String goalLabel, Algorithms algID, String[] region) {
		Node node_init_reg, node_reg_goal,solution = null;
		Graph g = new Graph();
		Vertex init_city = this.getVertice(initLabel);
		Vertex goal_city = this.getVertice(goalLabel);
		g.addVertice(initLabel, init_city.getLatitude(), goal_city.getLongitude());
		g.addVertice(initLabel, init_city.getLatitude(), goal_city.getLongitude());
		for (int i = 0; i < region.length; i++) {
			VertexSet vertex_set = this.getVerticeSet(region[i]);
			HashSet<Vertex> region_cities = vertex_set.getVertices();
			Iterator<Vertex> it = region_cities.iterator();
			while(it.hasNext()) {
				Vertex v = it.next();
				g.addVertice(v.getLabel(), v.getLatitude(), v.getLongitude());
				node_init_reg = this.searchSolution(initLabel, v.getLabel(), algID);
				node_reg_goal = this.searchSolution(v.getLabel(), goalLabel, algID);
				g.addEdge(initLabel, v.getLabel(), node_init_reg.getPathCost());
				g.addEdge(v.getLabel(), goalLabel, node_reg_goal.getPathCost());
			}
		}		
		return solution;
	}

	public void addVertice(String label, double lat, double lng) {
		Vertex v =  new Vertex(label);
		this.vertices.put(label, v);
		v.setCoordinates(lat, lng);
	}
	
	public Vertex getVertice(String label) {
		return this.vertices.get(label);
	}
	
	public void addVerticeSet(String label) {
		VertexSet vSet =  new VertexSet(label);
		this.vSets.add(vSet);
	}
	
	public VertexSet getVerticeSet(String setLabel) {
		for (VertexSet vSet : vSets) {
			if (vSet.getLabel()==setLabel) 
				return vSet;
		}
		return null;
	}
	
	public void addVerticeToSet(String labelSet,String labelVertex) {
		Vertex v = this.vertices.get(labelVertex);
		for (VertexSet vSet : vSets) {
			if (vSet.getLabel()==labelSet) {
				vSet.addVertice(v);
				break;
			}
		}
	}
	
	public boolean addEdge(Vertex one, Vertex two, double weight) {
		if (one.equals(two)) return false;
		Edge e = new Edge(one,two,weight);
		if (edges.containsKey(e.hashcode())) return false;
		if (one.containsNeighbor(e) || two.containsNeighbor(e))  return false;	
		edges.put(e.hashcode(), e);
		one.addNeighbor(e);
		two.addNeighbor(e);
		return true;
	} 
	
	public boolean addEdge(String oneLabel, String twoLabel, double weight) {
		Vertex one = getVertice(oneLabel);
		Vertex two = getVertice(twoLabel);
		return addEdge(one,two,weight);
	}
	
	public boolean addEdge(String oneLabel, String twoLabel) {
		Vertex one = getVertice(oneLabel);
		Vertex two = getVertice(twoLabel);
		return addEdge(one,two,one.straightLineDistance(two));
	}

	public void showLinks() {
		System.out.println("********************* LINKS *********************");
		for (Vertex current: vertices.values()) {		
			System.out.print(current + ": ");
			for (Edge e: current.getNeighbors()) {
				System.out.print(e.getNeighbor(current) + " (" + e.getWeight() + "); ");
			}
			System.out.println();
		}
		System.out.println("*************************************************");
	}
	
	public void showSets() {
		System.out.println("********************* SETS *********************");
		for (VertexSet vSet: vSets) {
			System.out.println(vSet);
		}
		System.out.println("*************************************************");
	}
	
	public Node searchSolution(String initLabel, String goalLabel, Algorithms algID) {
		State init = new State(this.getVertice(initLabel));
		State goal = new State(this.getVertice(goalLabel));
		SearchProblem prob = new SearchProblem(init,goal);
		SearchAlgorithm alg = null;
		switch(algID) {
			case BreadthFirstSearch :
				alg = new BreadthFirstSearch(prob);
				break;
			case DepthFirstSearch :
				alg = new DepthFirstSearch(prob);
				break; 
			case UniformCostSearch :
				alg = new UniformCostSearch(prob);
				break; 
			case GreedySearch :
				alg = new GreedySearch(prob);
				break; 
			case AStarSearch :
				alg = new AStarSearch(prob);
				break; 
		   default : 
			  System.out.println("ERROR: algorithm not implemented!");
		}
		Node n = alg.searchSolution();	
		Map<String,Number> m = alg.getMetrics();
		this.expansions += (long)m.get("Node Expansions");
		this.generated += (long)m.get("Nodes Generated");
		this.repeated += (long)m.get("State repetitions");
		this.time += (double)m.get("Runtime (ms)");
		return n;
	}
	
	

	public void showSolution(Node n) {
		System.out.println("******************* SOLUTION ********************");
		System.out.println("Node Expansions: " + this.expansions);
		System.out.println("Nodes Generated: " + this.generated);
		System.out.println("State Repetitions: " + this.repeated);
		System.out.printf("Runtime (ms): %6.3f \n",this.time);
		Node ni = null;
		List<Object> solution = n.getPath();
		double dist = 0;
		for (int i = 0; i<solution.size()-1;i++) {
			System.out.printf("| %-9s | %4.0f | ",solution.get(i), dist);
			ni = searchSolution(solution.get(i).toString(), solution.get(i+1).toString(), Algorithms.AStarSearch);
			System.out.print(ni.getPath());	
			System.out.println(" -> " + (int)ni.getPathCost());
			dist += ni.getPathCost();
		}
		System.out.printf("| %-9s | %4.0f | \n",solution.get(solution.size()-1), dist);
		System.out.println("*************************************************");
	}
	
	
}
