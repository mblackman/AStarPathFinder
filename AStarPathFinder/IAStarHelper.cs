using System.Collections.Generic;

namespace Raptorpan.AStarPathFinder
{
    /// <summary>
    /// Functions to facilitate an A* pathfinding algorithm.
    /// </summary>
    /// <typeparam name="T">The type the path is keyed by.</typeparam>
    public interface IAStarHelper<T>
    {
        /// <summary>
        /// Gets all the keys mapped to int.Max.
        /// </summary>
        /// <returns>A mapping between all the keys and int.Max</returns>
        Dictionary<T, int> GetKeysMap();

        /// <summary>
        /// Gets the heuristic path cost estimate between the keys in the path.
        /// </summary>
        /// <param name="start">They starting of the path.</param>
        /// <param name="goal">The end location of the path.</param>
        /// <returns>The heuristic cost of the path between the keys.</returns>
        int FindPathHeuristicEstimate(T start, T goal);

        /// <summary>
        /// Creates an in order <see cref="List{T}"/> of the sortest evaluated path.
        /// </summary>
        /// <param name="cameFrom">Gets a mapping of the nodes traveled towards the goal.</param>
        /// <param name="goal">The ending location of the path.</param>
        /// <returns>Gets the found path from the start to the goal.</returns>
        IList<T> RecontructPath(Dictionary<T, T> cameFrom, T goal);

        /// <summary>
        /// Gets all the neighboring keys to this key.
        /// </summary>
        /// <param name="current">THe key to get the neighbors of.</param>
        /// <returns>All the neighbors to this key in no particular order.</returns>
        IEnumerable<T> GetNeighbors(T current);

        /// <summary>
        /// Gets the distance between the given keys.
        /// </summary>
        /// <param name="start">The starting key.</param>
        /// <param name="goal">The goal key.</param>
        /// <returns>The distance.</returns>
        int DistanceBetween(T start, T goal);
    }
}
