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

import util

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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))





def iterativeDeepeningSearch(problem):
    """Search the deepest node in an iterative manner."""
    "*** YOUR CODE HERE FOR TASK 1 ***"

    "Intiialize the stack and set the initial Depth Limit to 1"
    depthLimit = 1;
    stack = util.Stack();

    while True:

        "Run this section every time we increment our depth to set up the Depth Limiting Search"
        "Initializes visited states to null, pushes the starting node to stack"
        "and adds the starting state to the visited states list "
        "also accesses the key data from the node"

        visitedStates = []
        stack.push((problem.getStartState(), [], 0))

        node = stack.pop()
        state = node[0]
        directionsToThisNode = node[1]
        costToThisNode = node[2]

        visitedStates.append((state))

        "This while loop runs the Depth Limiting Search by implementing the stack to search for the deepest node"
        "Every time it runs, it checks if currently inspected state is the goal state"
        "If the goal state, while loop breaks and skips to final check that returns the directions"
        "Otherwise, it adds all the successors to the stack and inspects them too"
        while (problem.isGoalState(state) != True):

            succesorNodes = problem.getSuccessors(state)

            "Inspecting a successor node"
            for successorNode in succesorNodes:

                "Parse key data from succesorNodes"
                succesorState = successorNode[0]
                succesorDirection = successorNode[1]
                succesorCost = successorNode[2]
                totalDepth = costToThisNode + succesorCost

                previouslyVisited = succesorState in visitedStates
                inDepth = totalDepth <= depthLimit

                "Check if the successor state has been previously visited and within this iterations depthLimit"
                "If both checks are true, the state is a valid candidate for Goal State and added to the stack/visited list"
                if ((previouslyVisited!= True) and inDepth):
                    stack.push( (succesorState, directionsToThisNode + [succesorDirection], totalDepth))
                    visitedStates.append(succesorState)

            "The loop generating the successors and adding to the stack is finished"
            "If the stack isnt empty, pop it to inspect if has a goal state when the while performs its isGoalState check"
            "Otherwise break the goal state test as there are no more nodes to inspect at this depthLimit, so increment depthLimit"
            if (stack.isEmpty()):
                break;
            else:
                node = stack.pop()
                state = node[0]
                directionsToThisNode = node[1]
                costToThisNode = node[2]
        "final check to see if state is goal state, if so, returns its associated set of directions for Pacman"
        if (problem.isGoalState(state)):
            return directionsToThisNode

        depthLimit += 1


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    util.raiseNotDefined()



def waStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has has the weighted (x 2) lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE FOR TASK 2 ***"


    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iterativeDeepeningSearch
wastar = waStarSearch
