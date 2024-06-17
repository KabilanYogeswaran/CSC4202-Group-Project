import java.util.*;

public class TreasureHunt {

    public static void main(String[] args) {
        Graph graph = new Graph();
        graph.addEdge("A", "B", 4);
        graph.addEdge("A", "C", 2);
        graph.addEdge("B", "C", 5);
        graph.addEdge("B", "D", 10);
        graph.addEdge("B", "F", 7);
        graph.addEdge("C", "D", 3);
        graph.addEdge("C", "E", 8);
        graph.addEdge("D", "E", 2);
        graph.addEdge("D", "F", 6);
        graph.addEdge("E", "F", 1);
        graph.addEdge("E", "G", 4);
        graph.addEdge("F", "G", 2);
        graph.addEdge("G", "H", 3);

        String[] startNodes = {"A", "B", "C"};

        for (String start : startNodes) {
            long startTimeBFS = System.nanoTime();
            List<List<String>> paths = getAllPathsBFS(graph, start, "H");
            long endTimeBFS = System.nanoTime();

            System.out.println("All paths from " + start + " to H:");
            for (List<String> path : paths) {
                System.out.println(path + " with distance " + getPathDistance(graph, path));
            }

            long startTimeDijkstra = System.nanoTime();
            List<String> shortestPath = getShortestPathDijkstra(graph, start, "H");
            long endTimeDijkstra = System.nanoTime();

            System.out.println("Shortest path from " + start + " to H: " + shortestPath + " with distance " + getPathDistance(graph, shortestPath));
            System.out.println("Dijkstra Execution time: " + (endTimeDijkstra - startTimeDijkstra) / 1e6 + " ms");
            System.out.println();
        }
    }

    private static List<List<String>> getAllPathsBFS(Graph graph, String start, String end) {
        List<List<String>> paths = new ArrayList<>();
        Queue<List<String>> queue = new LinkedList<>();
        queue.add(Arrays.asList(start));

        while (!queue.isEmpty()) {
            List<String> path = queue.poll();
            String last = path.get(path.size() - 1);

            if (last.equals(end)) {
                paths.add(new ArrayList<>(path));
            } else {
                for (Edge edge : graph.getNeighbors(last)) {
                    if (!path.contains(edge.destination)) {
                        List<String> newPath = new ArrayList<>(path);
                        newPath.add(edge.destination);
                        queue.add(newPath);
                    }
                }
            }
        }

        return paths;
    }

    private static List<String> getShortestPathDijkstra(Graph graph, String start, String end) {
        Map<String, Integer> distances = new HashMap<>();
        Map<String, String> previousNodes = new HashMap<>();
        PriorityQueue<Edge> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(e -> e.weight));

        for (String node : graph.getNodes()) {
            distances.put(node, Integer.MAX_VALUE);
        }
        distances.put(start, 0);
        priorityQueue.add(new Edge(start, 0));

        while (!priorityQueue.isEmpty()) {
            Edge edge = priorityQueue.poll();
            String currentNode = edge.destination;

            for (Edge neighbor : graph.getNeighbors(currentNode)) {
                int newDist = distances.get(currentNode) + neighbor.weight;
                if (newDist < distances.get(neighbor.destination)) {
                    distances.put(neighbor.destination, newDist);
                    priorityQueue.add(new Edge(neighbor.destination, newDist));
                    previousNodes.put(neighbor.destination, currentNode);
                }
            }
        }

        List<String> path = new ArrayList<>();
        for (String at = end; at != null; at = previousNodes.get(at)) {
            path.add(at);
        }
        Collections.reverse(path);
        return path;
    }

    private static int getPathDistance(Graph graph, List<String> path) {
        int distance = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            String from = path.get(i);
            String to = path.get(i + 1);
            for (Edge edge : graph.getNeighbors(from)) {
                if (edge.destination.equals(to)) {
                    distance += edge.weight;
                    break;
                }
            }
        }
        return distance;
    }
}

class Graph {
    private final Map<String, List<Edge>> adjList = new HashMap<>();
    private final Map<String, Integer> nodeIndexMap = new HashMap<>();
    private int index = 0;

    public void addEdge(String source, String destination, int weight) {
        adjList.putIfAbsent(source, new ArrayList<>());
        adjList.putIfAbsent(destination, new ArrayList<>());
        adjList.get(source).add(new Edge(destination, weight));
        if (!nodeIndexMap.containsKey(source)) {
            nodeIndexMap.put(source, index++);
        }
        if (!nodeIndexMap.containsKey(destination)) {
            nodeIndexMap.put(destination, index++);
        }
    }

    public List<Edge> getNeighbors(String node) {
        return adjList.get(node);
    }

    public Set<String> getNodes() {
        return adjList.keySet();
    }

    public int getNodeIndex(String node) {
        return nodeIndexMap.get(node);
    }
}

class Edge {
    String destination;
    int weight;

    Edge(String destination, int weight) {
        this.destination = destination;
        this.weight = weight;
    }
}
