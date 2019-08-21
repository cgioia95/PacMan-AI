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



def DepthLimitingSearch(stack, depthLimit, problem):

        visitedStates = []
        stack.push((problem.getStartState(), [], 0))

        node = stack.pop()
        state = node[0]
        directionsToThisNode = node[1]
        costToThisNode = node[2]

        visitedStates.append((state))

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
                break
            else:
                node = stack.pop()
                state = node[0]
                directionsToThisNode = node[1]
                costToThisNode = node[2]

        if (problem.isGoalState(state)):
            return directionsToThisNode
        else:
            return []



def iterativeDeepeningSearch(problem):
    """Search the deepest node in an iterative manner."""

    depthLimit = 1;
    stack = util.Stack();

    while (DepthLimitingSearch(stack, depthLimit, problem) == []):
        depthLimit = depthLimit + 1

    return DepthLimitingSearch(stack, depthLimit, problem)


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

def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    position, foodGrid = state

    print(position)
    print(foodGrid)
    "Useses the largest Manhtannan distance of the distances between Pacman's position"
    "the food on the grid, using the max ensures ad admissable, and thus consistent heuristic"

    foodList = foodGrid.asList()

    maxDistance = 0

    for food in foodList:

        distance = manhattan_distance(position, food)

        if (distance > maxDistance):
            maxDistance = distance

    return maxDistance


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    p_inf = float("inf")

    pQueue = util.PriorityQueue();
    closed = set()

    bestG = {}

    initState = problem.getStartState();
    initDirections = []
    initCost = 0

    initNode = (initState, initDirections, initCost)

    pQueue.push(initNode, heuristic(initState, problem))

    while (pQueue.isEmpty() != True ):

        minNodePopped = pQueue.pop()
        state = minNodePopped[0]
        directions = minNodePopped[1]
        cost = minNodePopped[2]


        if (state not in closed or cost < bestG[state]):

            closed.add(state)
            bestG[state] = cost

            if (problem.isGoalState(state)):
                return directions


            succesorNodes = problem.getSuccessors(state)

            for successorNode in succesorNodes:

                "Parse key data from succesorNodes"
                succesorState = successorNode[0]
                succesorDirection = successorNode[1]
                succesorCost = successorNode[2]

                cost = cost + succesorCost
                totalCost = cost + heuristic(succesorState, problem)

                totalDirection = directions + [succesorDirection]

                if (heuristic(succesorState, problem) < p_inf):

                    pQueue.push((succesorState, totalDirection, totalCost), totalCost)

    return []




def waStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has has the weighted (x 2) lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE FOR TASK 2 ***"

    p_inf = float("inf")

    pQueue = util.PriorityQueue();
    closed = set()

    bestG = {}

    initState = problem.getStartState();
    initDirections = []
    initCost = 0

    initNode = (initState, initDirections, initCost)

    pQueue.push(initNode, 2*heuristic(initState, problem))

    while (pQueue.isEmpty() != True ):

        minNodePopped = pQueue.pop()
        state = minNodePopped[0]
        directions = minNodePopped[1]
        cost = minNodePopped[2]


        if (state not in closed or cost < bestG[state]):

            closed.add(state)
            bestG[state] = cost

            if (problem.isGoalState(state)):
                return directions


            succesorNodes = problem.getSuccessors(state)

            for successorNode in succesorNodes:

                "Parse key data from succesorNodes"
                succesorState = successorNode[0]
                succesorDirection = successorNode[1]
                succesorCost = successorNode[2]

                cost = cost + succesorCost
                totalCost = cost + 2*heuristic(succesorState, problem)

                totalDirection = directions + [succesorDirection]

                if (2*heuristic(succesorState, problem) < p_inf):

                    pQueue.push((succesorState, totalDirection, totalCost), totalCost)

    return []


def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    position, foodGrid = state


    "Useses the largest Manhtannan distance of the distances between Pacman's position"
    "the food on the grid, using the max ensures ad admissable, and thus consistent heuristic"

    foodList = foodGrid.asList()

    maxDistance = 0

    for food in foodList:

        distance = manhattan_distance(position, food)

        if (distance > maxDistance):
            maxDistance = distance

    return maxDistance

def manhattan_distance(a,b):

    xy1 = a
    xy2 = b
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iterativeDeepeningSearch
wastar = waStarSearch
