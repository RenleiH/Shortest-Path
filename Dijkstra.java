package algo.project;

import java.util.*;

public class Dijkstra {
  private Graph graph;
  private Map<Vertex, Integer> distances; // Distance from the source vertex to each vertex
  private Map<Vertex, Vertex> predecessors; // Predecessor of each vertex in the shortest path
  private PriorityQueue<Vertex> priorityQueue; // Priority queue for selecting the vertex with the smallest distance

  public Dijkstra(Graph graph) {
    this.graph = graph;
    distances = new HashMap<>();
    predecessors = new HashMap<>();
    priorityQueue = new PriorityQueue<>(Comparator.comparingInt(distances::get));
  }

  // Runs the Dijkstra algorithm from the given source vertex
  public void runDijkstra(Vertex source) {
    initialize(source);

    while (!priorityQueue.isEmpty()) {
      Vertex u = priorityQueue.poll();

      for (Edge e : graph.getEdges(u)) {
        relax(u, e.getToVertex(), e.getWeight());
      }
    }
  }

  // Initializes the distances and predecessors for the Dijkstra algorithm
  private void initialize(Vertex source) {
    for (Vertex v : graph.getVertices()) {
      distances.put(v, Integer.MAX_VALUE); // Set initial distances to infinity
      predecessors.put(v, null); // Set initial predecessors to null
    }
    distances.put(source, 0); // Distance to source is 0
    priorityQueue.add(source); // Add source to the priority queue
  }

  // Relaxes the edge (u, v) with weight
  private void relax(Vertex u, Vertex v, int weight) {
    if (distances.get(u) != Integer.MAX_VALUE && distances.get(v) > distances.get(u) + weight) {
      distances.put(v, distances.get(u) + weight); // Update the distance to v
      predecessors.put(v, u); // Update the predecessor of v
      priorityQueue.add(v); // Add v to the priority queue
    }
  }

  // Returns the shortest path from the source to the given target vertex
  public List<Vertex> getShortestPath(Vertex target) {
    List<Vertex> path = new ArrayList<>();
    for (Vertex at = target; at != null; at = predecessors.get(at)) {
      path.add(at);
    }
    Collections.reverse(path);
    return path;
  }

  // Prints the shortest path from the source to the given target vertex
  public void printShortestPath(Vertex source, Vertex target) {
    List<Vertex> path = getShortestPath(target);
    if (path.size() == 1 && path.get(0) == target) {
      System.out.println("No path from " + source + " to " + target);
    } else {
      System.out.println("Shortest path from " + source + " to " + target + ": " + path);
    }
  }

  // Finds the nearest vertex of a given type (e.g., "Garage", "Pickup", "Dropoff") within a specified list of vertices
  public Vertex findNearestVertex(Vertex source, String vertexType, List<Vertex> vertices) {
    runDijkstra(source);
    int minDistance = Integer.MAX_VALUE;
    Vertex nearestVertex = null;
    for (Vertex vertex : vertices) {
      if (vertex.getType().equals(vertexType) && distances.get(vertex) < minDistance) {
        minDistance = distances.get(vertex);
        nearestVertex = vertex;
      }
    }
    return nearestVertex;
  }

  // Executes the sequence of finding paths from a Garage to all Pickups and Dropoffs and back to a Garage
  public void executePickupDropoffCycle(Vertex cycleStart, List<Vertex> cycle) {
    Vertex nearestGarage = findNearestVertex(cycleStart, "Garage", graph.getVertices());
    printShortestPath(cycleStart, nearestGarage);

    List<Vertex> visitedVertices = new ArrayList<>();
    visitedVertices.add(nearestGarage);

    Vertex current = nearestGarage;
    while (visitedVertices.size() < cycle.size() + 1) {
      Vertex nearestPickup = findNearestVertex(current, "Pickup", cycle);
      if (nearestPickup != null && !visitedVertices.contains(nearestPickup)) {
        printShortestPath(current, nearestPickup);
        visitedVertices.add(nearestPickup);
        current = nearestPickup;
      }

      Vertex nearestDropoff = findNearestVertex(current, "Dropoff", cycle);
      if (nearestDropoff != null && !visitedVertices.contains(nearestDropoff)) {
        printShortestPath(current, nearestDropoff);
        visitedVertices.add(nearestDropoff);
        current = nearestDropoff;
      }
    }

    Vertex returnGarage = findNearestVertex(current, "Garage", cycle);
    printShortestPath(current, returnGarage);
  }

  // Print the path from a cycle to the nearest garage and then between each point in the cycle
  public void printPathFromPickupToCycleAndGarage(Vertex cycleStart, List<Vertex> cycle) {
    // Find the nearest garage from the given cycle start (Pickup)
    Vertex nearestGarage = findNearestVertex(cycleStart, "Garage", graph.getVertices());
    System.out.println("Path from nearest Garage to Pickup:");
    printShortestPath(nearestGarage, cycleStart);

    // Print the path between each point in the cycle
    System.out.println("Paths between each point in the cycle:");
    Vertex current = cycleStart;
    for (Vertex next : cycle) {
      if (!current.equals(next)) {
        printShortestPath(current, next);
        current = next;
      }
    }

    // Print the path from the last point in the cycle back to the nearest garage
    Vertex lastPoint = cycle.get(cycle.size() - 1);
    Vertex returnGarage = findNearestVertex(lastPoint, "Garage", graph.getVertices());
    System.out.println("Path from last point in cycle back to nearest Garage:");
    printShortestPath(lastPoint, returnGarage);
  }
}
