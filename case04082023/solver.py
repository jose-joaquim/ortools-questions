#!/usr/bin/env python3

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import sys, math, json

inf = 10007


def print_solution(data, manager, routing, solution, print_formatted=False):
    print(f"Objective: {solution.ObjectiveValue()}")

    time_global = routing.GetDimensionOrDie("GlobalTime")
    time_reload = routing.GetDimensionOrDie("TimeReload")
    distance_global = routing.GetDimensionOrDie("GlobalDistance")
    distance_reload = routing.GetDimensionOrDie("DistanceReload")
    landing_dimension = routing.GetDimensionOrDie("LandingReload")
    load_dimension = routing.GetDimensionOrDie("Load")

    dimensions = [
        time_global,
        distance_global,
        time_reload,
        distance_reload,
        load_dimension,
        landing_dimension,
    ]

    indexes = data["AllIndexes"]
    visitedLocations = []

    total_time = 0
    for vehicle_id in range(data["num_vehicles"]):
        solution_rows = [
            [
                "flight",
                "vehicle",
                "node",
                "node_id",
                "global time",
                "global distance",
                "reload time",
                "reload distance",
                "load",
                "landing",
            ]
        ]

        flight_count = 1

        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)
        route_makespan = 0
        route = []
        arr = [dim.CumulVar(index) for dim in dimensions]
        while not routing.IsEnd(index):
            row = [solution.Value(dim) for dim in arr]
            inside = "{}".format(row)

            next_node = manager.IndexToNode(index)
            node_now = indexes[next_node]
            visitedLocations.append(node_now)

            if next_node > data["FirstCheckPoint"]:
                flight_count += 1

            plan_output += f"TDLL{inside} {node_now}, {next_node} -> "

            previous_index = index
            index = solution.Value(routing.NextVar(index))

            row = [flight_count, vehicle_id, node_now, next_node] + row
            solution_rows.append(row)

            route_makespan += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )

            arr = [dim.CumulVar(index) for dim in dimensions]
            route.append(previous_index)

        row = [solution.Value(dim) for dim in arr]
        arr = [dim.CumulVar(index) for dim in dimensions]
        next_node = manager.IndexToNode(index)
        route.append(next_node)

        node_now = indexes[next_node]

        previous_node = manager.IndexToNode(previous_index)
        node = manager.IndexToNode(index)

        row = [flight_count, vehicle_id, node_now, next_node] + row
        solution_rows.append(row)

        inside = "{}".format([solution.Value(arr[i]) for i in range(len(arr))])
        plan_output += f"TDL{inside} {node_now}, {next_node}\n"
        plan_output += "Makespan of the route: {}m\n".format(route_makespan)
        print(plan_output)
        total_time += route_makespan

        if print_formatted == True:
            for row in solution_rows:
                print("\t".join(map(str, row)))

    notVisitedLocations = [x for x in indexes if x not in visitedLocations]
    print("Total makespan of all routes: {}m".format(total_time))


def location_capacity_constraint(data, manager, routing):
    time_dimension = routing.GetDimensionOrDie("TimeReload")
    load_dimension = routing.GetDimensionOrDie("Load")

    full_tank = data["MaxTimeWithoutSupply"] * data["Consumption"]
    for i in range(1, len(data["TimeMatrix"])):
        i_index = manager.NodeToIndex(i)
        time_so_far = time_dimension.CumulVar(i_index)
        used_fuel = time_so_far * data["Consumption"]
        delta_pax = load_dimension.CumulVar(i_index) - min(
            0, data["PassengerWeight"] * data["NodeBalance"][i]
        )
        payload = (
            data["MaxTakeoffWeight"]
            - data["BasicOperationalWeight"]
            - full_tank
            - delta_pax
            + used_fuel
        )

        load_dimension.SetCumulVarSoftUpperBound(
            i_index,
            min(payload, data["MaxPassengersCardinality"] * data["PassengerWeight"]),
            inf * 2,
        )


def dimensions(data, manager, routing):
    indexes_l = data["AllIndexes"]

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        return data["DistanceMatrix"][from_node][to_node]

    def distance_callback_reload(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        # From a checkpoint
        if from_node > data["FirstCheckPoint"]:
            return -500

        return distance_callback(from_index, to_index)

    transit_callback_index_reload = routing.RegisterTransitCallback(
        distance_callback_reload
    )
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.AddDimension(transit_callback_index, 0, inf, True, "GlobalDistance")
    routing.AddDimension(
        transit_callback_index_reload, inf, inf, True, "DistanceReload"
    )
    distance_dimension = routing.GetDimensionOrDie("GlobalDistance")
    distance_dimension.SetGlobalSpanCostCoefficient(10)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        time = data["TimeMatrix"][from_node][to_node]
        service = data["service_time"][from_node]

        if indexes_l[from_node][0] == indexes_l[to_node][0] or from_node == 0:
            service = 0

        return time + service

    def time_callback_reload(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        # From a checkpoint
        if from_node > data["FirstCheckPoint"]:
            return -500

        return time_callback(from_index, to_index)

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    time_callback_index_reload = routing.RegisterTransitCallback(time_callback_reload)
    routing.AddDimension(time_callback_index_reload, inf, inf, True, "TimeReload")
    routing.AddDimension(time_callback_index, 0, inf, True, "GlobalTime")
    routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index)

    def load_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["NodeBalance"][from_node] * data["PassengerWeight"]

    veh_cap = [inf for _ in range(data["num_vehicles"])]
    load_callback_index = routing.RegisterUnaryTransitCallback(load_callback)
    routing.AddDimensionWithVehicleCapacity(
        load_callback_index, 0, veh_cap, True, "Load"
    )

    def landing_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        # From a checkpoint OR from depot
        if from_node > data["FirstCheckPoint"] or from_node == 0:
            return -500

        if indexes_l[from_node][0] != indexes_l[to_node][0]:
            return 1

        return 0

    landing_callback_index = routing.RegisterTransitCallback(landing_callback)
    routing.AddDimension(
        landing_callback_index,
        inf,
        data["MaxLandingPerRoute"],
        True,
        "LandingReload",
    )

    routing.AddConstantDimension(1, 100_000_00, True, "__visited__")


def dropping(data, manager, routing):
    for node in range(1, len(data["TimeMatrix"])):
        penalty = 0
        if data["NodeBalance"][node] != 0:
            penalty = inf * 5000

        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)


def pickups_deliveries(data, manager, routing):
    distance_dimension = routing.GetDimensionOrDie("GlobalTime")

    unique_dems = []
    for dem in data["Demands"]:
        if (dem[0], dem[1]) not in unique_dems:
            unique_dems.append((dem[0], dem[1]))

    for cnt, request in enumerate(unique_dems):
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])

        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index)
            <= distance_dimension.CumulVar(delivery_index)
        )


def adjust_reload_stuff(data, manager, routing):
    time_dim = routing.GetDimensionOrDie("TimeReload")
    distance_dim = routing.GetDimensionOrDie("DistanceReload")
    landing_dim = routing.GetDimensionOrDie("LandingReload")
    load_dim = routing.GetDimensionOrDie("Load")
    for i in range(data["FirstCheckPoint"]):
        i_index = manager.NodeToIndex(i)
        time_dim.SlackVar(i_index).SetValue(0)
        distance_dim.SlackVar(i_index).SetValue(0)
        load_dim.SlackVar(i_index).SetValue(0)

    for l in range(data["FirstCheckPoint"], len(data["TimeMatrix"])):
        l_index = manager.NodeToIndex(l)
        # routing.solver().Add(load_dim.CumulVar(l_index) == 0)
        load_dim.SetCumulVarSoftUpperBound(l_index, 0, 100_000)

    load_dim.SetCumulVarSoftUpperBound(routing.End(0), 0, inf)


def main():
    data = {}

    with open(sys.argv[1], "r") as f:
        data = json.load(f)

    manager = pywrapcp.RoutingIndexManager(
        len(data["TimeMatrix"]), data["num_vehicles"], data["depot"]
    )

    routing_parameters = pywrapcp.DefaultRoutingModelParameters()
    routing_parameters.solver_parameters.trace_propagation = False
    routing_parameters.solver_parameters.trace_search = False
    routing = pywrapcp.RoutingModel(manager, routing_parameters)

    dimensions(data, manager, routing)
    pickups_deliveries(data, manager, routing)
    dropping(data, manager, routing)
    adjust_reload_stuff(data, manager, routing)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )

    search_parameters.log_search = False
    search_parameters.time_limit.seconds = 600
    # search_parameters.use_cp_sat = True

    initial = [
        [1, 21, 17, 37, 42, 4, 24, 16, 36, 45, 7, 27, 13, 33],
        [2, 22, 18, 38, 43, 5, 25, 11, 31, 46, 8, 28, 12, 32],
        [3, 23, 19, 39, 44, 6, 26, 15, 35, 47, 9, 29, 14, 34],
    ]

    initial = None

    if initial != None:
        routing.CloseModelWithParameters(search_parameters)
        initial_solution = routing.ReadAssignmentFromRoutes(initial, True)
        print("Initial solution:")
        print_solution(data, manager, routing, initial_solution, True)

        print("\n\n")
        exit()

        solution = routing.SolveFromAssignmentWithParameters(
            initial_solution, search_parameters
        )
    else:
        solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(data, manager, routing, solution, True)


if __name__ == "__main__":
    main()
