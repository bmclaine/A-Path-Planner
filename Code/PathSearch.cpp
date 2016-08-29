#include "PathSearch.h"

namespace fullsail_ai {
	namespace algorithms {

		bool isGreater(PathSearch::PlannerNode* const& lhs, PathSearch::PlannerNode* const & rhs)
		{
			return lhs->finalCost > rhs->finalCost;
		}

		bool isLesser(PathSearch::Edge const lhs, PathSearch::Edge const rhs)
		{
			return lhs.cost < rhs.cost;
		}

		PathSearch::PathSearch() : openQueue(isGreater)
		{

		}

		PathSearch::~PathSearch()
		{

		}

		void PathSearch::initialize(TileMap* _tileMap)
		{
			tileMap = _tileMap;

			unsigned int numRows = _tileMap->getRowCount();
			unsigned int numColumns = _tileMap->getColumnCount();

			for (unsigned int row = 0; row < numRows; row++)
			{
				for (unsigned int column = 0; column < numColumns; column++)
				{
					SearchNode* newSearchNode = new SearchNode();
					newSearchNode->data = _tileMap->getTile(row, column);
					searchMap[_tileMap->getTile(row, column)] = newSearchNode;
				}
			}

			auto iter = searchMap.begin();
			for (; iter != searchMap.end(); ++iter)
			{
				double xCoord = iter->first->getXCoordinate(), yCoord = iter->first->getYCoordinate();
				int row = iter->first->getRow(), column = iter->first->getColumn();

				for (int rhsRow = -1; rhsRow <= 1; rhsRow++)
				{
					for (int rhsColumn = -1; rhsColumn <= 1; rhsColumn++)
					{
						if (rhsRow == 0 && rhsColumn == 0)
							continue;

						Tile* rhs = _tileMap->getTile(row + rhsRow, column + rhsColumn);

						if (rhs == nullptr || rhs->getWeight() == 0)
						{
							continue;
						}

						if (areAdjacent(iter->first, rhs) == true)
						{
							Edge newEdge;
							newEdge.endPoint = searchMap[rhs];
							float deltaX = (float)(rhs->getXCoordinate() - xCoord);
							float deltaY = (float)(rhs->getYCoordinate() - yCoord);

							newEdge.cost = sqrt((deltaY * deltaY) + (deltaX * deltaX)) * rhs->getWeight();
							iter->second->edges.push_back(newEdge);
						}
					}
				}

				std::sort(iter->second->edges.begin(), iter->second->edges.end(), isLesser);
			}
		}

		void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
		{
			done = false;
			lastNode = nullptr;

			// Keep track of the start and goal node for further use
			startNode = searchMap[tileMap->getTile(startRow, startColumn)];
			goalNode = searchMap[tileMap->getTile(goalRow, goalColumn)];

			//	Find the graph vertex that represents the start tile
			//	Create a new PlannerNode at the vertex
			PlannerNode* plNode = new PlannerNode();
			plNode->parent = nullptr;
			plNode->vertex = searchMap[startNode->data];

			//	Enqueue this node into the open container
			openQueue.push(plNode);
			openQueue.front()->givenCost = 0.0f;

			// Associate the start location with this node in the visited map
			visitedMap[startNode->data] = plNode;

			//	Set its heuristicCost equal to the distance between the start and goal tiles
			visitedMap[startNode->data]->heuristicCost = EstimateHeuristicCost(startNode, goalNode);
			visitedMap[startNode->data]->finalCost = EstimateHeuristicCost(startNode, goalNode);
		}

		void PathSearch::update(long timeslice)
		{
			long startTime = (long)GetTickCount();

			//	update line drawling
			if (lastNode != nullptr)
			{
				PlannerNode* tempNode = lastNode;

				while (tempNode->parent != nullptr)
				{
					tempNode->vertex->data->clearLines();
					tempNode = tempNode->parent;
				}
			}

			//	While the open container possesses nodes do the following..
			while (openQueue.empty() == false)
			{
				//	Dequeue the current node from the open container
				PlannerNode* currNode = openQueue.front();
				openQueue.pop();

				//	If this node is the goal..
				if (currNode->vertex == goalNode)
				{
					done = true;

					//	Build the solution list..
					while (currNode->parent != nullptr)
					{
						currNode->vertex->data->addLineTo(currNode->parent->vertex->data, 0xFFFF0000);
						solutionList.push_back(currNode->vertex->data);
						currNode = currNode->parent;
					}

					solutionList.push_back(currNode->vertex->data);

					//	And exit!
					return;
				}

				//	Otherwise, for each successor of current..
				unsigned int numNodes = currNode->vertex->edges.size();
				for (unsigned int edge = 0; edge < numNodes; edge++)
				{
					SearchNode* successor = currNode->vertex->edges[edge].endPoint;

					//	Compute the newGivenCost of the successor
					float tempGivenCost = currNode->givenCost + currNode->vertex->edges[edge].cost;

					//	If successor is a duplicate..
					if (visitedMap[successor->data] != nullptr)
					{
						//	but newGivenCost is cheaper than its givenCost
						if (tempGivenCost < visitedMap[successor->data]->givenCost)
						{
							PlannerNode* successorNode = visitedMap[successor->data];
							openQueue.remove(successorNode);
							//	 Set its givenCost equal to the newGivenCost
							successorNode->givenCost = tempGivenCost;
							successorNode->finalCost = successorNode->heuristicCost + successorNode->givenCost;
							successorNode->parent = currNode;
							successor->data->setFill(0x7F00FF00);
							openQueue.push(successorNode);
						}
					}
					else //	If successor is NOT a duplicate
					{
						//	Enqueue a PlannerNode at the successor into the open container
						PlannerNode* successorNode = new PlannerNode();
						successorNode->vertex = successor;
						successorNode->parent = currNode;
						successorNode->heuristicCost = EstimateHeuristicCost(successor, goalNode);
						successorNode->givenCost = tempGivenCost;
						successorNode->finalCost = successorNode->heuristicCost + successorNode->givenCost;

						//	Set its heuristicCost equal to the distance between its tile and the goal tile
							successorNode->heuristicCost = EstimateHeuristicCost(successor, goalNode);

						//	Enqueue this node into the open container
						successor->data->setFill(0x7F0000FF);
						openQueue.push(successorNode);

						//	Associate the successor's location with this node in the visited map
						visitedMap[successor->data] = successorNode;
					}
				}

				lastNode = currNode;

				long currTime = (long)GetTickCount();
				long timeTaken = currTime - startTime;

				if (timeslice == 0 || timeTaken > timeslice)
				{
					while (currNode->parent != nullptr)
					{
						currNode->vertex->data->addLineTo(currNode->parent->vertex->data, 0xFFFF0000);
						currNode = currNode->parent;
					}

					break;
				}
			}
		}

		void PathSearch::exit()
		{
			auto iter = visitedMap.begin();
			for (; iter != visitedMap.end();)
			{
				if (iter->second != nullptr)
				{
					delete iter->second;
					iter->second = nullptr;
					iter = visitedMap.erase(iter);
				}
				else
					++iter;
			}

			openQueue.clear();
			solutionList.clear();

			goalNode = nullptr;
			startNode = nullptr;
			lastNode = nullptr;
		}

		void PathSearch::shutdown()
		{
			tileMap = nullptr;

			auto iter = searchMap.begin();
			for (; iter != searchMap.end();)
			{
				if (iter->second != nullptr)
				{
					delete iter->second;
					iter->second = nullptr;
					iter = searchMap.erase(iter);
				}
				else
					++iter;
			}
		}

		bool PathSearch::isDone() const
		{
			return done;
		}

		std::vector<Tile const*> const PathSearch::getSolution() const
		{
			return solutionList;
		}

		bool PathSearch::areAdjacent(Tile const* lhs, Tile const* rhs)
		{
			if (abs(lhs->getRow() - rhs->getRow()) > 1 || abs(lhs->getColumn() - rhs->getColumn()) > 1)
			{
				return false;
			}

			if (lhs->getRow() % 2 == 0)
			{
				if (lhs->getColumn() - rhs->getColumn() == -1 && lhs->getRow() != rhs->getRow())
				{
					return false;
				}
			}
			else
			{
				if (lhs->getColumn() - rhs->getColumn() == 1 && lhs->getRow() != rhs->getRow())
				{
					return false;
				}
			}

			return true;
		}

		float PathSearch::EstimateHeuristicCost(const SearchNode* lhs, const SearchNode* rhs)
		{
			float ret = 0.0f;
			float deltaX = (float)abs((rhs->data->getXCoordinate() - lhs->data->getXCoordinate()));
			float deltaY = (float)abs((rhs->data->getYCoordinate() - lhs->data->getYCoordinate()));

			//	Manhattan Distance..
			//	ret = deltaX + deltaY;

			//	Max Distance.. I forget what this is called.. xD
			ret = max(deltaX, deltaY);

			//	Straight Line Distance
			//	ret = sqrt((deltaY * deltaY) + (deltaX * deltaX));

			return ret;
		}
	}
}  // namespace fullsail_ai::algorithms

