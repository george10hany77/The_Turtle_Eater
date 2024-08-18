import sys

from AStarNode import AStarNode
from PathFinding import PathFinding


def main(args):
    #ob1 = AStarNode(4,3)
    #ob2 = AStarNode(5,5)
    #ob3 = AStarNode(4,4)
    #ob4 = AStarNode(3,3)
    #ob5 = AStarNode(2,0)
    #ob6 = AStarNode(2,1)
    #ob7 = AStarNode(2,2)
    #ob8 = AStarNode(2,3)
    
    ob1 = AStarNode(4,3)
    ob2 = AStarNode(5,5)
    ob3 = AStarNode(4,4)
    ob4 = AStarNode(3,3)
    ob5 = AStarNode(2,0)
    ob6 = AStarNode(2,1)
    ob7 = AStarNode(2,2)
    ob8 = AStarNode(2,3)
    
    obstacles = [ob1, ob2, ob3, ob4, ob5, ob6, ob7, ob8]

    start = AStarNode(5, 5)
    end = AStarNode(11, 11)
    path = PathFinding(11, 11)
    path.add_obstacles(obstacles)
    print(path.a_star(start, end))


if __name__ == '__main__':
    main(sys.argv)
