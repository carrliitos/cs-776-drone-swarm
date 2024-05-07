import java.util.*;

/**
 * AStar class contains a static nested Node class and a static method for finding a path using the A* algorithm.
 * @author Benzon Carlitos Salazar
 */
public class AStar {

  /**
   * Represents a node in the A* algorithm.
   */
  public static class Node {
    int x, y, z;
    Node parent;
    double g, h;

    /**
     * Constructs a new Node.
     *
     * @param x       The x-coordinate of the node.
     * @param y       The y-coordinate of the node.
     * @param z       The z-coordinate of the node.
     * @param parent  The parent node of this node.
     * @param g       The cost from the start node to this node.
     * @param h       The heuristic cost from this node to the target node.
     */
    public Node(int x, int y, int z, Node parent, double g, double h) {
      this.x = x;
      this.y = y;
      this.z = z;
      this.parent = parent;
      this.g = g;
      this.h = h;
    }

    /**
     * Calculates the total cost of the node (g + h).
     *
     * @return The total cost of the node.
     */
    public double getF() {
      return g + h;
    }
  }

  /**
   * Finds a path using the A* algorithm.
   *
   * @param grid      The 3D grid representing the environment.
   * @param startX    The x-coordinate of the start position.
   * @param startY    The y-coordinate of the start position.
   * @param startZ    The z-coordinate of the start position.
   * @param targetX   The x-coordinate of the target position.
   * @param targetY   The y-coordinate of the target position.
   * @param targetZ   The z-coordinate of the target position.
   * @return          The list of nodes representing the path from start to target, or null if no path is found.
   */
  public static List<Node> findPath(int[][][] grid, int startX, int startY, int startZ, int targetX, int targetY, int targetZ) {
    PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(Node::getF));
    Set<Node> closedSet = new HashSet<>();
    Node startNode = new Node(startX, startY, startZ, null, 0, heuristic(startX, startY, startZ, targetX, targetY, targetZ));
    openSet.add(startNode);

    while (!openSet.isEmpty()) {
      Node current = openSet.poll();

      if (current.x == targetX && current.y == targetY && current.z == targetZ) {
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
          for (int k = -1; k <= 1; k++) {
            if (i == 0 && j == 0 && k == 0) continue;

            int newX = current.x + i;
            int newY = current.y + j;
            int newZ = current.z + k;
            if (newX >= 0 && newX < grid.length && newY >= 0 && newY < grid[0].length && newZ >= 0 && newZ < grid[0][0].length && grid[newX][newY][newZ] == 0) {
              Node neighbor = new Node(newX, newY, newZ, current, current.g + 1, heuristic(newX, newY, newZ, targetX, targetY, targetZ));
              if (closedSet.contains(neighbor)) continue;

              double tentativeG = current.g + 1;
              if (!openSet.contains(neighbor) || tentativeG < neighbor.g) {
                neighbor.g = tentativeG;
                neighbor.h = heuristic(newX, newY, newZ, targetX, targetY, targetZ);
                if (!openSet.contains(neighbor)) openSet.add(neighbor);
              }
            }
          }
        }
      }
    }

    return null;
  }

  /**
   * Calculates the heuristic value (Euclidean distance) between two points in 3D space.
   *
   * @param x1  The x-coordinate of the first point.
   * @param y1  The y-coordinate of the first point.
   * @param z1  The z-coordinate of the first point.
   * @param x2  The x-coordinate of the second point.
   * @param y2  The y-coordinate of the second point.
   * @param z2  The z-coordinate of the second point.
   * @return    The heuristic value between the two points.
   */
  private static double heuristic(int x1, int y1, int z1, int x2, int y2, int z2) {
    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2) + Math.pow(z2 - z1, 2));
  }
}
