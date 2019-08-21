    p_inf = float("inf")

    pQueue = util.PriorityQueue();
    closed = ()

    initState = problem.getStartState();
    initDirections = []
    initCost = 0

    initNode = (initState, initDirections, initCost)

    pQueue.push(initNode, 2*heuristic(initState, problem))

    while ( not pQueue.isEmpty() ):

        minNodePopped = pQueue.pop()
        state = minNodePopped[0]
        directions = minNodePopped[1]
        cost = minNodePopped[2]




        if (state not in closed):

            if (problem.isGoalState(state)):
                return directions

            closed = closed + (state)

            succesorNodes = problem.getSuccessors(state)

            for successorNode in succesorNodes:

                "Parse key data from succesorNodes"
                succesorState = successorNode[0]
                succesorDirection = successorNode[1]
                succesorCost = successorNode[2]

                cost = cost + succesorCost
                totalCost = cost + heuristic(succesorState, problem)

                totalDirection = directions + [succesorDirection]

                pQueue.push((succesorState, totalDirection, cost ),totalCost)
