package undirectedgraph;

public class Romenia {

    public static Graph defineGraph() {
        
        Graph g = new Graph();
        // Define cities:
        g.addVertice("Arad", 46.18333, 21.31667);
        g.addVertice("Bucharest", 44.43225, 26.10626);
        g.addVertice("Craiova", 44.33018, 23.79488);
        g.addVertice("Dobreta", 44.63692, 22.65973);
        g.addVertice("Eforie", 44.05842, 28.63361);
        g.addVertice("Fagaras", 45.84164, 24.97310);
        g.addVertice("Giurgiu", 43.90371, 25.96993);
        g.addVertice("Hirsova", 44.68935, 27.94566);
        g.addVertice("Iasi", 47.15845, 27.60144);
        g.addVertice("Lugoj", 45.69099, 21.90346);
        g.addVertice("Mehadia", 44.90411, 22.36452);
        g.addVertice("Neamt", 46.97587, 26.38188);
        g.addVertice("Oradea", 47.04650, 21.91894);
        g.addVertice("Pitesti", 44.85648, 24.86918);
        g.addVertice("R. Vilcea", 45.09968, 24.36932);
        g.addVertice("Sibiu", 45.79833, 24.12558);
        g.addVertice("Timisoara", 45.74887, 21.20868);
        g.addVertice("Urziceni", 44.71653, 26.64112);
        g.addVertice("Vaslui", 46.64069, 27.72765);
        g.addVertice("Zerind", 46.62251, 21.51742);
        // Define routes:
        g.addEdge("Arad","Sibiu");
        g.addEdge("Arad","Timisoara");
        g.addEdge("Arad","Zerind");
        g.addEdge("Bucharest","Fagaras");
        g.addEdge("Bucharest","Giurgiu");
        g.addEdge("Bucharest","Pitesti");
        g.addEdge("Bucharest","Urziceni");
        g.addEdge("Craiova","Dobreta");
        g.addEdge("Craiova","Pitesti");
        g.addEdge("Craiova","R. Vilcea");
        g.addEdge("Dobreta","Mehadia");
        g.addEdge("Eforie","Hirsova");
        g.addEdge("Fagaras","Sibiu");
        g.addEdge("Hirsova","Urziceni");
        g.addEdge("Iasi","Neamt");
        g.addEdge("Iasi","Vaslui");
        g.addEdge("Lugoj","Mehadia");
        g.addEdge("Lugoj","Timisoara");
        g.addEdge("Oradea","Sibiu");
        g.addEdge("Oradea","Zerind");
        g.addEdge("Pitesti","R. Vilcea");
        g.addEdge("R. Vilcea","Sibiu");
        g.addEdge("Urziceni","Vaslui");
        // Define regions:
        g.addVerticeSet("Banat");
        g.addVerticeToSet("Banat","Lugoj");
        g.addVerticeToSet("Banat","Mehadia");
        g.addVerticeToSet("Banat","Timisoara");
        g.addVerticeSet("Crisana");
        g.addVerticeToSet("Crisana","Arad");
        g.addVerticeToSet("Crisana","Oradea");
        g.addVerticeToSet("Crisana","Zerind");
        g.addVerticeSet("Dobrogea");
        g.addVerticeToSet("Dobrogea","Eforie");
        g.addVerticeToSet("Dobrogea","Hirsova");
        g.addVerticeSet("Moldova");
        g.addVerticeToSet("Moldova","Iasi");
        g.addVerticeToSet("Moldova","Neamt");
        g.addVerticeToSet("Moldova","Vaslui");
        g.addVerticeSet("Muntenia");
        g.addVerticeToSet("Muntenia","Bucharest");
        g.addVerticeToSet("Muntenia","Giurgiu");
        g.addVerticeToSet("Muntenia","Pitesti");
        g.addVerticeToSet("Muntenia","Urziceni");
        g.addVerticeSet("Oltenia");
        g.addVerticeToSet("Oltenia","Craiova");
        g.addVerticeToSet("Oltenia","Dobreta");
        g.addVerticeToSet("Oltenia","R. Vilcea");
        g.addVerticeSet("Transilvania");
        g.addVerticeToSet("Transilvania","Fagaras");
        g.addVerticeToSet("Transilvania","Sibiu");

        return g;
    }

}
