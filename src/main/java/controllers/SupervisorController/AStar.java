import java.util.*;

public class AStar {
  private static class Node {
    int x, y;
    Node parent;
    double g, h;

    public Node(int x, int y, Node parent, double g, double h) {
      this.x = x;
      this.y = y;
      this.parent = parent;
      this.g = g;
      this.h = h;
    }

    public double getF() {
      return g + h;
    }
  }

  public static List<Node> findPath(int[][] grid, int startX, int startY, int targetX, int targetY) {
    PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(Node::getF));
    Set<Node> closedSet = new HashSet<>();
    Node startNode = new Node(startX, startY, null, 0, heuristic(startX, startY, targetX, targetY));
    openSet.add(startNode);

    while (!openSet.isEmpty()) {
      Node current = openSet.poll();

      if (current.x == targetX && current.y == targetY) {
        List<Node> path = new ArrayList<>();
        while (current != null) {
          path.add(current);
          current = current.parent;
        }
        Collections.reverse(path);
        return path;
      }

      closedSet.add(current);

      for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
          if (i == 0 && j == 0) continue;

          int newX = current.x + i;
          int newY = current.y + j;
          if (newX >= 0 && newX < grid.length && newY >= 0 && newY < grid[0].length && grid[newX][newY] == 0) {
            Node neighbor = new Node(newX, newY, current, current.g + 1, heuristic(newX, newY, targetX, targetY));
            if (closedSet.contains(neighbor)) continue;

            double tentativeG = current.g + 1;
            if (!openSet.contains(neighbor) || tentativeG < neighbor.g) {
              neighbor.g = tentativeG;
              neighbor.h = heuristic(newX, newY, targetX, targetY);
              if (!openSet.contains(neighbor)) openSet.add(neighbor);
            }
          }
        }
      }
    }

    return null;
  }

  private static double heuristic(int x1, int y1, int x2, int y2) {
    // Euclidean distance heuristic
    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
  }
}
