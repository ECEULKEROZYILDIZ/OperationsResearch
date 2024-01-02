// Copyright 2010-2022 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// [START program]
// [START import]
using System;
using System.Collections.Generic;
using Google.OrTools.ConstraintSolver;
using Google.Protobuf.WellKnownTypes; // Duration
// [END import]

/// <summary>
///   Minimal TSP using distance matrix.
/// </summary>
public class VrpCapacity
{
    // [START data_model]
    class DataModel
    {
        public long[,] DistanceMatrix = {
            { 0, 10, 20, 30, 45, 6, 7, 21, 23, 25 },
            { 23, 0, 12, 32, 3, 54, 5, 6, 7, 8 },
            { 24, 15, 0, 16, 37, 65, 2, 16, 21, 22 },
            { 25, 30, 17, 0, 16, 23, 34, 26, 23, 25 },
            { 43, 2, 32, 20, 0, 54, 14, 19, 23, 25 },
            { 6, 56, 63, 21, 52, 0, 9, 8, 21, 32 },
            { 6, 5, 3, 32, 12, 9, 0, 12, 7, 27 },
            { 22, 7, 14, 27, 18, 9, 14, 0, 44, 45 },
            { 24, 9, 21, 23, 21, 25, 6, 43, 0, 14 },
            { 25, 8, 22, 24, 24, 30, 22, 46, 13, 0 },
        };
        // [START demands_capacities]

        public long[] Demands = { 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000 };

        public long[] VehicleCapacities = { 5000, 5000, 5000, 5000 };
        // [END demands_capacities]
        public int VehicleNumber = 4;
        public int Depot = 0;
    };
    // [END data_model]

    // [START solution_printer]
    /// <summary>
    ///   Print the solution.
    /// </summary>
    static void PrintSolution(in DataModel data, in RoutingModel routing, in RoutingIndexManager manager,
                              in Assignment solution)
    {
        Console.WriteLine($"Objective {solution.ObjectiveValue()}:");

        // Inspect solution.
        long totalDistance = 0;
        long totalLoad = 0;
        for (int i = 0; i < data.VehicleNumber; ++i)
        {
            Console.WriteLine("Route for Vehicle {0}:", i);
            long routeDistance = 0;
            long routeLoad = 0;
            var index = routing.Start(i);
            while (routing.IsEnd(index) == false)
            {
                long nodeIndex = manager.IndexToNode(index);
                routeLoad += data.Demands[nodeIndex];
                Console.Write("{0} Load({1}) -> ", nodeIndex, routeLoad);
                var previousIndex = index;
                index = solution.Value(routing.NextVar(index));
                routeDistance += routing.GetArcCostForVehicle(previousIndex, index, 0);
            }
            Console.WriteLine("{0}", manager.IndexToNode((int)index));
            Console.WriteLine("Distance of the route: {0}m", routeDistance);
            totalDistance += routeDistance;
            totalLoad += routeLoad;
        }
        Console.WriteLine("Total distance of all routes: {0}m", totalDistance);
        Console.WriteLine("Total load of all routes: {0}m", totalLoad);
    }
    // [END solution_printer]

    public static void Main(String[] args)
    {
        // Instantiate the data problem.
        // [START data]
        DataModel data = new DataModel();
        // [END data]

        // Create Routing Index Manager
        // [START index_manager]
        RoutingIndexManager manager =
            new RoutingIndexManager(data.DistanceMatrix.GetLength(0), data.VehicleNumber, data.Depot);
        // [END index_manager]

        // Create Routing Model.
        // [START routing_model]
        RoutingModel routing = new RoutingModel(manager);
        // [END routing_model]

        // Create and register a transit callback.
        // [START transit_callback]
        int transitCallbackIndex = routing.RegisterTransitCallback((long fromIndex, long toIndex) =>
        {
            // Convert from routing variable Index to
            // distance matrix NodeIndex.
            var fromNode = manager.IndexToNode(fromIndex);
            var toNode = manager.IndexToNode(toIndex);
            return data.DistanceMatrix[fromNode, toNode];
        });
        // [END transit_callback]

        // Define cost of each arc.
        // [START arc_cost]
        routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);
        // [END arc_cost]

        // Add Capacity constraint.
        // [START capacity_constraint]
        int demandCallbackIndex = routing.RegisterUnaryTransitCallback((long fromIndex) =>
        {
            // Convert from routing variable Index to
            // demand NodeIndex.
            var fromNode =
                manager.IndexToNode(fromIndex);
            return data.Demands[fromNode];
        });
        routing.AddDimensionWithVehicleCapacity(demandCallbackIndex, 0, // null capacity slack
                                                data.VehicleCapacities, // vehicle maximum capacities
                                                true,                   // start cumul to zero
                                                "Capacity");
        // [END capacity_constraint]

        // Setting first solution heuristic.
        // [START parameters]
        RoutingSearchParameters searchParameters =
            operations_research_constraint_solver.DefaultRoutingSearchParameters();
        searchParameters.FirstSolutionStrategy = FirstSolutionStrategy.Types.Value.PathCheapestArc;
        searchParameters.LocalSearchMetaheuristic = LocalSearchMetaheuristic.Types.Value.GuidedLocalSearch;
        searchParameters.TimeLimit = new Duration { Seconds = 1 };
        // [END parameters]

        // Solve the problem.
        // [START solve]
        Assignment solution = routing.SolveWithParameters(searchParameters);
        // [END solve]

        // Print solution on console.
        // [START print_solution]
        PrintSolution(data, routing, manager, solution);
        // [END print_solution]
    }
}
// [END program]