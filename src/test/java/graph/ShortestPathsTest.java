package graph;

import static org.junit.Assert.*;
import org.junit.FixMethodOrder;

import org.junit.Test;
import org.junit.runners.MethodSorters;

import java.io.File;
import java.net.URL;
import java.io.FileNotFoundException;

import java.util.LinkedList;

@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class ShortestPathsTest {


    /* Returns the Graph loaded from the file with filename fn. */
    private Graph loadBasicGraph(String fn) {
        Graph result = null;
        try {
            // Load file from resources
            URL resource = getClass().getClassLoader().getResource(fn);
            if (resource == null) {
                fail("Could not find graph " + fn);
            }
            File file = new File(resource.toURI());
            result = ShortestPaths.parseGraph("basic", file.getAbsolutePath());
        } catch (Exception e) {
            fail("Could not find graph " + fn);
        }
        return result;
    }
    /** Dummy test case demonstrating syntax to create a graph from scratch.
     * Write your own tests below. */
    @Test
    public void test00Nothing() {
        Graph g = new Graph();
        Node a = g.getNode("A");
        Node b = g.getNode("B");
        g.addEdge(a, b, 1);

        // sample assertion statements:
        assertTrue(true);
        assertEquals(2+2, 4);
    }

    /** Minimal test case to check the path from A to B in Simple0.txt */
    @Test
    public void test01Simple0() {
        Graph g = loadBasicGraph("Simple0.txt");
        g.report();
        ShortestPaths sp = new ShortestPaths();
        Node a = g.getNode("A");
        sp.compute(a);
        Node b = g.getNode("B");
        LinkedList<Node> abPath = sp.shortestPath(b);
        assertEquals(abPath.size(), 2);
        assertEquals(abPath.getFirst(), a);
        assertEquals(abPath.getLast(),  b);
        assertEquals(sp.shortestPathLength(b), 1.0, 1e-6);
    }

    /* Pro tip: unless you include @Test on the line above your method header,
     * gradle test will not run it! This gets me every time. */
    @Test
    public void test02MultiplePaths () {
        Graph g = new Graph();
        Node a = g.getNode("A");
        Node b = g.getNode("B");
        Node c = g.getNode("C");
        Node d = g.getNode("D");

        g.addEdge(a, b, 4);
        g.addEdge(a, c, 2);
        g.addEdge(c, b, 1);
        g.addEdge(c, d, 3);
        g.addEdge(b, d, 1);

        ShortestPaths sp = new ShortestPaths();
        sp.compute(a);

        // Shortest path A -> C -> B (2 + 1 = 3)
        assertEquals(3.0, sp.shortestPathLength(b), 1e-6);
        LinkedList<Node> path = sp.shortestPath(b);
        assertEquals(3, path.size());
        assertEquals(a, path.getFirst());
        assertEquals(b, path.getLast());
    }

    /** Test case where some nodes are disconnected */
    @Test
    public void test03DisconnectedGraph() {
        Graph g = new Graph();
        Node a = g.getNode("A");
        Node b = g.getNode("B");
        Node c = g.getNode("C");

        g.addEdge(a, b, 5);
        // No edge between A or B to C

        ShortestPaths sp = new ShortestPaths();
        sp.compute(a);

        // C is unreachable from A
        assertEquals(Double.POSITIVE_INFINITY, sp.shortestPathLength(c), 1e-6);
        assertNull(sp.shortestPath(c));
    }

}
