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
