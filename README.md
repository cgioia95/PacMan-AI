# PacMan-AI

Demonstration of various algorithms to navigate Pacman levels as a learning exercise in Artificial Intelligence.

Before proceeding, have `python` installed. 

To run Pacman, enter commands from the `comp90054-a1-2019-master` directory as follows: `python pacman.y -l ${maze} -p ${agent} -a fn=${algorithm}, heuristic={heuristic}. 

`maze` is the level in which Pacman operates, 

`agent` specifies what AI agent is implemented, e.g. GoWestAgent = An agent that goes West until it can't., SearchAgent = General search agent finds a path using a supplied search algorithm. 

`algorithm` specifies what algorith is implemnted, e.g. A*, Iterative Depth First Search, etc.

`heuristic` is an evaluation formula of Pacman's's position to determine the next move. 
