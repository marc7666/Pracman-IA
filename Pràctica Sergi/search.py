# -*- coding: utf-8 -*-

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
import node


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
    return blindSearch(problem, util.Stack())


def breadthFirstSearch(problem):
    return blindSearch(problem, util.Queue())


def blindSearch(problem, fringe):
    """Generic blind search algorithm"""
    fringeset = set()
    expanded = set()

    fringe.push(node.Node(problem.getStartState()))
    fringeset.add(problem.getStartState())

    while True:
        if fringe.isEmpty():
            sys.exit("No solution")

        n = fringe.pop()
        expanded.add(n.state)
        fringeset.remove(n.state)

        for s, a, c in problem.getSuccessors(n.state):
            ns = node.Node(s, n, a, n.cost + c)
            if problem.isGoalState(ns.state):
                return ns.get_path_actions()
            if ns.state not in fringeset and ns.state not in expanded:
                fringe.push(ns)
                fringeset.add(ns.state)


def treeDepthFirstSearch(problem):
    return treeBlindSearch(problem, util.Stack())


def treeBreadthFirstSearch(problem):
    return treeBlindSearch(problem, util.Queue())


def treeBlindSearch(problem, fringe):
    """Generic blind search algorithm"""
    fringeset = set()

    fringe.push(node.Node(problem.getStartState()))
    fringeset.add(problem.getStartState())

    while True:
        if fringe.isEmpty():
            sys.exit("No solution")

        n = fringe.pop()

        fringeset.remove(n.state)

        for s, a, c in problem.getSuccessors(n.state):
            ns = node.Node(s, n, a, n.cost + c)
            if problem.isGoalState(ns.state):
                return ns.get_path_actions()
            #  ns not in fringe No està bé
            if ns.state not in fringeset:
                fringe.push(ns)
                fringeset.add(ns.state)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    return blindSearch1(problem, nullHeuristic)


def depthLimitedSearch(problem, k):
    fringe = util.Stack()
    fringe.push(node.Node(problem.getStartState()))

    cut = False
    while True:
        if fringe.isEmpty():
            if cut:
                return None
            else:
                return sys.exit("No solution")

        n = fringe.pop()

        if n.cost == k:
            cut = True

        else:
            for s, a, c in problem.getSuccessors(n.state):
                ns = node.Node(s, n, a, n.cost + c)
                if problem.isGoalState(ns.state):
                    return ns.get_path_actions()
                fringe.push(ns)


def treeIterativeDeepeningSearch(problem):
    depth = 0
    while True:
        result = depthLimitedSearch(problem, depth)
        if result is not None:
            return result
        depth += 1


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state
    to the nearest goal in the provided SearchProblem.
    This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    return blindSearch1(problem, heuristic)


def greedyBestFirstSearch(problem, heuristic):
    fringe = util.PriorityQueue()
    fringeset = set()
    expanded = set()

    fringe.push(node.Node(problem.getStartState()), heuristic(problem.getStartState(), problem))
    fringeset.add(problem.getStartState())

    while True:
        if fringe.isEmpty():
            return sys.exit("No solution")

        n = fringe.pop()
        fringeset.remove(n.state)

        if problem.isGoalState(n.state):
            return n.get_path_actions()

        expanded.add(n.state)

        for s, a, c in problem.getSuccessors(n.state):
            ns = node.Node(s, n, a, heuristic(n.state, problem))
            if ns.state not in fringeset and ns.state not in expanded:
                fringe.push(ns, heuristic(ns.state, problem))
                fringeset.add(ns.state)
            elif (ns.state in fringeset) and (heuristic(ns.state, problem) < heuristic(n.state, problem)):
                fringe.push(ns, heuristic(ns.state, problem))
                fringeset.push(ns.state)


def blindSearch1(problem, heuristic):
    fringe = util.PriorityQueue()
    fringeset = set()
    expanded = set()

    fringe.push(node.Node(problem.getStartState()), 0)
    fringeset.add(problem.getStartState())

    while True:
        if fringe.isEmpty():
            return sys.exit("No solution")

        n = fringe.pop()
        fringeset.remove(n.state)

        if problem.isGoalState(n.state):
            return n.get_path_actions()

        expanded.add(n.state)

        for s, a, c in problem.getSuccessors(n.state):
            ns = node.Node(s, n, a, n.cost + c)
            pathMaxCost = pathMax(n, ns, heuristic, problem)
            if ns.state not in fringeset and ns.state not in expanded:
                fringe.push(ns, pathMaxCost)
                fringeset.add(ns.state)
            elif (ns.state in fringeset) and (ns.cost < n.cost):
                fringe.push(ns, pathMaxCost)
                fringeset.push(ns.state)


def manhattanHeuristic(position, problem):
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])


def euclideanHeuristic(position, problem):
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0] ) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5


def customHeuristic(position, problem):
    return 0


def pathMax(n, ns, heuristic, problem):
    aux = ns.cost + heuristic(ns.state, problem)
    aux1 = n.cost + heuristic(n.state, problem)
    if max(aux1, aux) == aux1:
        return n.cost
    else:
        return ns.cost


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
tbfs = treeBreadthFirstSearch
tdfs = treeDepthFirstSearch
dls = depthLimitedSearch
tids = treeIterativeDeepeningSearch
ucs = uniformCostSearch
bfsh = greedyBestFirstSearch
mandH = manhattanHeuristic
eucdH = euclideanHeuristic
custH = customHeuristic
