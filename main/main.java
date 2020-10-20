package main;

import searchalgorithm.Algorithms;
import searchalgorithm.Node;
import undirectedgraph.Graph;
import undirectedgraph.Romenia;

public class main {

	public static void main(String[] args) {
        Graph graph = Romenia.defineGraph();
        graph.showLinks();
        graph.showSets();
        Node n;
        n = graph.searchSolution("Timisoara", "Neamt", Algorithms.GreedySearch);
        graph.showSolution(n); 
	}

}
