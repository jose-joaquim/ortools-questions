def print_solution(data, manager, routing, solution, print_formatted=False):
    print(f"Objective: {solution.ObjectiveValue()}")

    time_global = routing.GetDimensionOrDie("GlobalTime")
    time_reload = routing.GetDimensionOrDie("TimeReload")
    landing_dimension = routing.GetDimensionOrDie("LandingReload")
    load_dimension = routing.GetDimensionOrDie("Load")

    dimensions = [
        time_global,
        time_reload,
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
                "reload time",
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
