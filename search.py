# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import sys

import util
from node import Node


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state
        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state
        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take
        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.
    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    return blindGraphSearch(problem, util.Stack())


def blindTreeSearch(problem, fringe):
    from node import Node
    import sys
    fringe.push(Node(problem.getStartState()))
    while True:
        if fringe.isEmpty():
            print("ERROR: Map has no solution")
            sys.exit(-1)
        n = fringe.pop()
        if problem.isGoalState(n.state):
            return n.total_path()
        for state, action, cost in problem.getSuccessors(n.state):
            fringe.push(Node(state, n, action, cost))


def blindGraphSearch(problem, fringe):
    from node import Node
    import sys
    fringe.push(Node(problem.getStartState()))
    expandedStates = []
    while True:
        if fringe.isEmpty():
            print("ERROR: Map has no solution")
            sys.exit(-1)
        n = fringe.pop()
        expandedStates.append(n.state)
        if problem.isGoalState(n.state):
            return n.total_path()
        for state, action, cost in problem.getSuccessors(n.state):
            if state not in expandedStates:
                fringe.push(Node(state, n, action, cost))


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    return blindGraphSearch(problem, util.Queue())


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    generated = {}
    fringe = util.PriorityQueue()
    n = Node(problem.getStartState())
    fringe.push(n, n.cost)
    generated[n.state] = [n, 'F'] #'F' indica que esta en el fringe
    while True:
        if fringe.isEmpty():
            print("ERROR: Map has no solution")
            sys.exit(-1)
        n = fringe.pop()
        if problem.isGoalState(n.state):
            return n.total_path()
        if generated[n.state][1] == 'E':
            continue
        generated[n.state] = [n, 'E'] #'E' significa que esta expandido
        for state, action, cost, in problem.getSuccessors(n.state):
            ns = Node(state, n, action, ns.cost + cost)
            if not ns.state in generated:
                fringe.push(ns, ns.cost)
                generated[ns.state] = [ns, 'F']
            elif ns.cost < generated[ns.state][0].cost: #Para que la condicion sea cierta el nodo debe estar en el fringe
                fringe.push(ns, ns.cost)
                generated[ns.state] = [ns, 'F']


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    problem.getStartState()
    print state, heuristic(state, problem) #Printamos el valor de la heurística


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
