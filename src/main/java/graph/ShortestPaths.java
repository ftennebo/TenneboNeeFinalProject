package graph;
import java.util.*;
import java.io.File;
import java.io.FileNotFoundException;

/** Provides an implementation of Dijkstra's single-source shortest paths
 * algorithm.
 * Sample usage:
 *   Graph g = // create your graph
 *   ShortestPaths sp = new ShortestPaths();
 *   Node a = g.getNode("A");
 *   sp.compute(a);
 *   Node b = g.getNode("B");
 *   LinkedList<Node> abPath = sp.getShortestPath(b);
 *   double abPathLength = sp.getShortestPathLength(b);
 *   */
public class ShortestPaths {
    // stores auxiliary data associated with each node for the shortest
    // paths computation:
    private HashMap<Node,PathData> paths;

    /** Compute the shortest path to all nodes from origin using Dijkstra's
     * algorithm. Fill in the paths field, which associates each Node with its
     * PathData record, storing total distance from the source, and the
     * back pointer to the previous node on the shortest path.
     * Precondition: origin is a node in the Graph.*/
    public void compute(Node origin) {
        paths = new HashMap<Node,PathData>();

        // TODO 1: implement Dijkstra's algorithm to fill paths with
        // shortest-path data for each Node reachable from origin.

        // create a priority queue to store nodes while running Dijkstra's alg
        // the queue will keep the node with the smallest known distance at the front, so we can process it first
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingDouble(n -> paths.get(n).distance)); // Comparator.comparingDouble() sorts nodes based on their shortest known distance

        // Initialize distances
        paths.put(origin, new PathData(0, null));
        pq.add(origin);

        // continue algorithm while priority queue contains elements
        while (!pq.isEmpty()) {
            Node current = pq.poll(); // get node with the smallest distance
            double currentDistance = paths.get(current).distance;

            // iterate through all the neighbors of the current node in the graph
                // current.getNeighbors() gets a HashMap where the keys are the node objects, values are the edge weights
                // entrySet() returns a set of key value pairs from the HashMap
                // for loop iterates through each neighbor and its distance
            for (Map.Entry<Node, Double> neighborEntry : current.getNeighbors().entrySet()) {
                // checking if there's a shorter path to each neighbor
                Node neighbor = neighborEntry.getKey(); // get the neighbor node
                double edgeWeight = neighborEntry.getValue(); // get the edge weight (the distance)
                double newDistance = currentDistance + edgeWeight;

                // check if a shorter path to a neighboring node has been found,
                // update when necessary:
                if (newDistance < paths.get(neighbor).distance || !paths.containsKey(neighbor)){
                    // !paths.containsKey(neighbor) -> paths is a HashMap storing shortest known distance to each node, and the prev node on the shortest path
                    // so, if neighbor is not yet in paths, it means we haven't processed it yet, and we ust add it.
                    // (newDistance < paths.get(neighbor).distance -> newDistance is calculated distance from neighbur to current node, paths.get(neighbor).distance is the previously recorded shortest distance to neighbor.
                    // so, if newDistance is smaller, we have found a shorter route, need to update.
                    paths.put(neighbor, new PathData(newDistance, current)); // update paths (HM) to store new shortest distance
                    pq.add(neighbor); // add neighbor to the priority queue (which always sorts by shortest distance)
                }

            }

        }

    }

    /** Returns the length of the shortest path from the origin to destination.
     * If no path exists, return Double.POSITIVE_INFINITY.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called. */
    public double shortestPathLength(Node destination) {
        // TODO 2 - implement this method to fetch the shortest path length
        // from the paths data computed by Dijkstra's algorithm.
        throw new UnsupportedOperationException();
    }

    /** Returns a LinkedList of the nodes along the shortest path from origin
     * to destination. This path includes the origin and destination. If origin
     * and destination are the same node, it is included only once.
     * If no path to it exists, return null.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called. */
    public LinkedList<Node> shortestPath(Node destination) {
        // TODO 3 - implement this method to reconstruct sequence of Nodes
        // along the shortest path from the origin to destination using the
        // paths data computed by Dijkstra's algorithm.
        throw new UnsupportedOperationException();
    }


    /** Inner class representing data used by Dijkstra's algorithm in the
     * process of computing shortest paths from a given source node. */
    class PathData {
        double distance; // distance of the shortest path from source
        Node previous; // previous node in the path from the source

        /** constructor: initialize distance and previous node */
        public PathData(double dist, Node prev) {
            distance = dist;
            previous = prev;
        }
    }


    /** Static helper method to open and parse a file containing graph
     * information. Can parse either a basic file or a CSV file with
     * sidewalk data. See GraphParser, BasicParser, and DBParser for more.*/
    protected static Graph parseGraph(String fileType, String fileName) throws
        FileNotFoundException {
        // create an appropriate parser for the given file type
        GraphParser parser;
        if (fileType.equals("basic")) {
            parser = new BasicParser();
        } else if (fileType.equals("db")) {
            parser = new DBParser();
        } else {
            throw new IllegalArgumentException(
                    "Unsupported file type: " + fileType);
        }

        // open the given file
        parser.open(new File(fileName));

        // parse the file and return the graph
        return parser.parse();
    }

    public static void main(String[] args) {
      // read command line args
      String fileType = args[0];
      String fileName = args[1];
      String SidewalkOrigCode = args[2];

      String SidewalkDestCode = null;
      if (args.length == 4) {
        SidewalkDestCode = args[3];
      }

      // parse a graph with the given type and filename
      Graph graph;
      try {
          graph = parseGraph(fileType, fileName);
      } catch (FileNotFoundException e) {
          System.out.println("Could not open file " + fileName);
          return;
      }
      graph.report();


      // TODO 4: create a ShortestPaths object, use it to compute shortest
      // paths data from the origin node given by origCode.

      // TODO 5:
      // If destCode was not given, print each reachable node followed by the
      // length of the shortest path to it from the origin.

      // TODO 6:
      // If destCode was given, print the nodes in the path from
      // origCode to destCode, followed by the total path length
      // If no path exists, print a message saying so.
    }
}
