package algo.project;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.graphstream.ui.view.Viewer;

public class Main {

  public static void main(String[] args) {
    // Configure the GraphStream to use Swing for rendering
    System.setProperty("org.graphstream.ui", "swing");

    // Specify the path to the CSV file for positive graph
    String csvFilePath = "trimmed_file.csv"; // Update with actual path

    // Initialize PositiveGraph and generate a positive graph from the CSV file
    PositiveGraph positiveGraphGenerator = new PositiveGraph();
    Graph positiveGraph = positiveGraphGenerator.generatePositiveGraphFromCSV(csvFilePath);

    // Print the positive graph
    System.out.println("Positive Graph:");
    positiveGraphGenerator.printGraph(positiveGraph);

    // Convert to GraphStream graph and display the positive graph
    org.graphstream.graph.Graph gsPositiveGraph = positiveGraphGenerator.toGraphStreamGraph();
    Viewer positiveViewer = gsPositiveGraph.display();
    positiveViewer.setCloseFramePolicy(Viewer.CloseFramePolicy.EXIT);

    // Specify the path to the CSV file for negative graph
    String negativeCsvFilePath = "negative_file.csv"; // Update with actual path provided by the user

    // Create a negative graph from the CSV file
    Graph negativeGraph = NegativeGraph.createNegativeGraphFromCSV(negativeCsvFilePath, positiveGraph);

    // Print the negative graph
    System.out.println("Negative Graph:");
    NegativeGraph.printGraph(negativeGraph);

    // Convert to GraphStream graph and display the negative graph
    org.graphstream.graph.Graph gsNegativeGraph = NegativeGraph.toGraphStreamGraph(negativeGraph);
    Viewer negativeViewer = gsNegativeGraph.display();
    negativeViewer.setCloseFramePolicy(Viewer.CloseFramePolicy.EXIT);

    // Run Bellman-Ford algorithm to find negative weight cycle
    List<Vertex> cycle = new ArrayList<Vertex>();
    BellmanFord bellmanFord = new BellmanFord(negativeGraph);
    boolean hasNegativeCycle = bellmanFord.run(negativeGraph.getVertices().get(0)); // Assume the first vertex as source

    if (hasNegativeCycle) {
      System.out.println("Negative weight cycle detected:");
      cycle = bellmanFord.getNegativeCycle();
      for (Vertex v : cycle) {
        System.out.println(v);
      }
    } else {
      System.out.println("No negative weight cycle detected.");
      return;
    }

    // Execute the Dijkstra algorithm to find paths from the cycle's Pickup to the nearest Garage and within the cycle
    Vertex cycleStart = cycle.get(0);
    Dijkstra dijkstra = new Dijkstra(positiveGraph);
    dijkstra.printPathFromPickupToCycleAndGarage(cycleStart, cycle);

  }
}
