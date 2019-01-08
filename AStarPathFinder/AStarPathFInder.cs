using System;
using System.Collections.Generic;
using System.Linq;

namespace Raptorpan.AStarPathFinder
{
    /// <summary>
    /// An implementation of the A* pathfinding algorith. Uses a helper to calculate paths.
    /// </summary>
    /// <typeparam name="TKey">The type to key the search off of.</typeparam>
    public class AStarPathFinder<TKey> where TKey : IEquatable<TKey>
    {
        private readonly int maxAttempts;

        /// <summary>
        /// Creates a new instance of the A* path finder.
        /// </summary>
        public AStarPathFinder()
        {
        }

        /// <summary>
        /// Creates a new instance of the A* path finder.
        /// </summary>
        /// <param name="maxRetries">The number of times to attempt a search before quitting.</param>
        public AStarPathFinder(int maxRetries)
        {
            this.maxAttempts = maxRetries;
        }

        /// <summary>
        /// Finds the best possible path between the start location and goal location.
        /// </summary>
        /// <param name="start">Start location to begin the search from.</param>
        /// <param name="goal">The goal location.</param>
        /// <param name="aStarHelper">The helper object for the A* search. Gets the values.</param>
        /// <returns>The ordered collection of locations to get from the start to the end.</returns>
        /// Based on the A* from: https://en.wikipedia.org/wiki/A*_search_algorithm
        public IList<TKey> FindPath(TKey start, TKey goal, IAStarHelper<TKey> aStarHelper)
        {
            // The set of nodes already evaluated
            var closedSet = new HashSet<TKey>();

            // The set of currently discovered nodes that are not evaluated yet.
            // Initially, only the start node is known.
            var openSet = new HashSet<TKey>
            {
                start
            };

            // For each node, which node it can most efficiently be reached from.
            // If a node can be reached from many nodes, cameFrom will eventually contain the
            // most efficient previous step.
            var cameFrom = new Dictionary<TKey, TKey>();

            // For each node, the cost of getting from the start node to that node.
            Dictionary<TKey, int> gScore = aStarHelper.GetKeysMap();

            if (!gScore.ContainsKey(start))
            {
                throw new ArgumentException("The starting key is invalid..");
            }

            if (!gScore.ContainsKey(goal))
            {
                throw new ArgumentException("The ending key is invalid.");
            }

            // For each node, the total cost of getting from the start node to the goal
            // by passing by that node. That value is partly known, partly heuristic.
            Dictionary<TKey, int> fScore = aStarHelper.GetKeysMap();

            // The cost of going from start to start is zero.
            gScore[start] = 0;

            // For the first node, that value is completely heuristic.
            fScore[start] = aStarHelper.FindPathHeuristicEstimate(start, goal);
            
            int currentAttempt = 0;
            bool infiniteAttempts = this.maxAttempts == default(int);

            while (openSet.Count > 0 && (infiniteAttempts || currentAttempt < this.maxAttempts))
            {
                TKey current = openSet.Aggregate((l, r) => fScore[l] < fScore[r] ? l : r);

                if (current.Equals(goal))
                {
                    return aStarHelper.RecontructPath(cameFrom, current);
                }

                openSet.Remove(current);
                closedSet.Add(current);

                foreach (TKey neighbor in aStarHelper.GetNeighbors(current))
                {
                    if (closedSet.Contains(neighbor))
                    {
                        // Ignore the neighbor which is already evaluated.
                        continue;
                    }

                    // The distance from start to a neighbor
                    int tentativeGScore = gScore[current] + aStarHelper.DistanceBetween(current, neighbor);

                    if (!openSet.Contains(neighbor))
                    {
                        // Discover a new node
                        openSet.Add(neighbor);
                    }
                    else if (tentativeGScore >= gScore[neighbor])
                    {
                        continue;
                    }

                    // This path is the best until now.
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + aStarHelper.FindPathHeuristicEstimate(neighbor, goal);
                }

                currentAttempt++;
            }

            return null;
        }
    }
}
