# This code is adapted from multiple sources in the OR-Tools repository.
# Source 1: vrp_time_windows.py
# https://github.com/google/or-tools/tree/1d696f9108a0ebfd99feb73b9211e2f5a6b0812b/ortools/constraint_solver/samples/vrp_time_windows.py
# Source 2: vrptw_store_solution_data.py
# https://github.com/google/or-tools/tree/1d696f9108a0ebfd99feb73b9211e2f5a6b0812b/ortools/constraint_solver/samples/vrptw_store_solution_data.py
# License: Apache 2.0
# The code was modified to suit the requirements of the project.

from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from adress_api import adressapi
import pdb;
import numpy as np

early_start =480

class Depot:
    """
    Represents a depot location.
    """
    def __init__(self, id, adress,coordinates=None):
        self.id = id
        self.adress = adress
        self.coordinates=coordinates


class Van:
    """
    Represents a van with specific characteristics.
    """
    def __init__(self, id, depot, emissions_per_km, fuel_consumption_per_km, start_time=480, return_deadline=1080):
        self.id = id
        self.depot = depot
        self.emissions_per_km = emissions_per_km
        self.fuel_consumption_per_km = fuel_consumption_per_km
        self.start_time = start_time - early_start
        self.return_deadline = return_deadline- early_start


class Reservation:
    """
    Represents a reservation with specific location, time, and duration.
    """
    def __init__(self, id, adress, stop_duration, time_window=None, priority=0,coordinates=None):
        self.id = id
        self.adress = adress
        self.stop_duration = stop_duration
        self.time_window = (time_window[0]-early_start,time_window[1]-early_start) or (0, 1440)  # Default: Full day
        self.priority = priority
        self.coordinates=coordinates 




class RouteOptimizer:
    """
    Optimizer for routing problems, accounting for vans, depots, reservations, and constraints.
    """
    def __init__(self, vans, reservations, depots):
        self.vans = vans
        self.reservations = reservations
        self.depots = depots
        self.distance_matrix = None
        self.time_matrix = None
        self.manager = None
        self.routing = None
        self.solution = None
        self.api=adressapi()

    def generate_pseudo_matrix(self):

        all_locations = [depot.adress for depot in self.depots] + [res.adress for res in self.reservations]
        num_locations = len(all_locations)
        print(num_locations)

        self.distance_matrix = [[abs(i - j) * 7 for j in range(num_locations)] for i in range(num_locations)]
        self.time_matrix = [[abs(i - j) * 10 for j in range(num_locations)] for i in range(num_locations)]

    def get_coordinates(self):

        for res in self.depots:
                    if res.coordinates==None:
                        res.coordinates=self.api.get_coordinates(res.adress)
                        if not res.coordinates:
                            print(f"Could not geocode: {res.adress}")

        for res in self.reservations:
            if res.coordinates==None:
                res.coordinates=self.api.get_coordinates(res.adress)
                if not res.coordinates:
                    print(f"Could not geocode: {res.adress}")
                


    def generate_matrices_from_api(self):
        self.get_coordinates()
        all_locations = [depot.coordinates for depot in self.depots] + [res.coordinates for res in self.reservations]
        print(all_locations)
        self.time_matrix= np.ceil(np.array(self.api.get_travel_times(all_locations))/60).astype(int)
        print(self.time_matrix)


    def create_data_model(self):
        


        """
        Creates the data model for the routing problem.
        """

        time_windows = []
        stop_durations = [0]
        
        # Add depot time windows
        # for van in self.vans:
        #     time_windows.append((480, 1080))
        time_windows.append(((480-early_start), (1080-early_start)))
        # Add reservation time windows and stop durations
        for res in self.reservations:
            time_windows.append((res.time_window[0],res.time_window[1]))
            stop_durations.append(res.stop_duration)

        print(time_windows)
        print(self.time_matrix)
        return {
            'distance_matrix': self.distance_matrix,
            'time_matrix': self.time_matrix,
            'time_windows': time_windows,
            'stop_durations': stop_durations,
            'num_vehicles': len(self.vans),
            'depots': 0,

        }

    def get_recommendation(self):
        #temporary 
        coord=[[4.348591, 50.865544], [4.566716, 51.13374], [4.402453, 51.246664], [4.710309, 50.881011], [3.732703, 51.057496], [5.482751, 50.969549],
         [5.334128, 50.935484], [4.371755, 50.843183], [3.275675, 50.837035], [4.398169, 51.918494], [5.554438, 50.635755], [3.217718, 51.208664], 
         [2.924749, 51.220182], [4.471338, 51.025018]]
        for c in range(len(self.depots)):
            self.reservations[c]=coord[c]
        for c in range(1,len(coord)):
            self.reservations[c-len(self.depots)]=coord[c]


        target_node=0
        self.generate_matrices_from_api()
        #optimizer.generate_pseudo_matrix()
        data = self.create_data_model()
        self.initialize_routing(data)
        # Get the time dimension
        time_dimension = self.routing.GetDimensionOrDie("Time")
        index = self.manager.NodeToIndex(target_node)
        # Get the cumulative time variable at the node
        time_var = time_dimension.CumulVar(index)
        # Get the latest possible arrival time at this node
        max_time = self.solution.Max(time_var)


    def initialize_routing(self, data):
        """
        Initializes the routing model with constraints.
        """
        try:
            print("Initializing routing...")
            # Create the RoutingIndexManager
            manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                                        data['num_vehicles'],
                                                        data["depots"],)
            routing = pywrapcp.RoutingModel(manager)
            self.manager=manager
            def time_callback(from_index, to_index):
                """Returns the travel time between the two nodes."""
                # Convert from routing variable Index to time matrix NodeIndex.
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return data["time_matrix"][from_node][to_node]+data["stop_durations"][to_node]

            transit_callback_index = routing.RegisterTransitCallback(time_callback)
            # [END transit_callback]


            # Define cost of each arc.
            # [START arc_cost]
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
            # [END arc_cost]

            # Add Time Windows constraint.
            # [START time_windows_constraint]
            time = "Time"
            routing.AddDimension(
                transit_callback_index,
                6000,  # allow waiting time
                100000000,  # maximum time per vehicle
                False,  # Don't force start cumul to zero.
                time,
            )
            time_dimension = routing.GetDimensionOrDie(time)
            print("Time dimension added.")


            # Add time window constraints for each location except depot.
            for location_idx, time_window in enumerate(data["time_windows"]):
                if location_idx == data["depots"]:
                    continue
                index = manager.NodeToIndex(location_idx)
                time_dimension.CumulVar(index).SetRange(int(time_window[0]), int(time_window[1]))
            print("Time window constraints added for locations.")


            # Add time window constraints for each vehicle start node.
            depot_idx =0
            for vehicle_id in range(data["num_vehicles"]):
                index = routing.Start(vehicle_id)
                time_dimension.CumulVar(index).SetRange(
                    data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1]
                )

            print("Time window constraints added for vehicles.")


            for i in range(data["num_vehicles"]):
                routing.AddVariableMinimizedByFinalizer(
                    time_dimension.CumulVar(routing.Start(i))
                )
                routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))
            
            penalty = 100000000
            for node in range(1, len(data["time_windows"])):
                routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

            print("Disjunctions added.")

            self.routing=routing

        except Exception as e:
            print(f"An error occurred during initialization: {e}\n")

    def solve(self, search_parameters=None):
        """
        Solves the routing problem with enhanced error handling.
        """
        try:
            if search_parameters is None:
                search_parameters = pywrapcp.DefaultRoutingSearchParameters()
                search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
                #search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
                #search_parameters.time_limit.seconds = 5

            self.solution = self.routing.SolveWithParameters(search_parameters)
            return self.solution is not None
        except Exception as e:
            print(f"Error during solving: {e}")
            return False


    def calculate_metrics(self):
        """
        Calculates key metrics like total distance and emissions.
        """
        total_distance = 0
        total_emissions = 0

        for vehicle_id in range(self.manager.GetNumberOfVehicles()):
            route_distance = 0
            index = self.routing.Start(vehicle_id)
            while not self.routing.IsEnd(index):
                next_index = self.solution.Value(self.routing.NextVar(index))
                from_node = self.manager.IndexToNode(index)
                to_node = self.manager.IndexToNode(next_index)
                route_distance += self.distance_matrix[from_node][to_node]
                index = next_index
            total_distance += route_distance
            total_emissions += route_distance * self.vans[vehicle_id].emissions_per_km

        return {
            'total_distance': total_distance,
            'total_emissions': total_emissions,
        }


def minutes_to_time_full(minutes):
    minutes=minutes+early_start
    hours = minutes // 60
    mins = minutes % 60
    return f"{int(hours):02d}:{int(mins):02d}"

# [START solution_printer]
def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
        # Display dropped nodes.
    dropped_nodes = "Dropped nodes:"
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += f" {manager.IndexToNode(node)}"
    print(dropped_nodes)
    time_dimension = routing.GetDimensionOrDie("Time")
    total_time = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
    
            plan_output += (
                f"{manager.IndexToNode(index)}"
                f" Time({minutes_to_time_full((solution.Max(time_var)-data['stop_durations'][manager.IndexToNode(index)]))},{minutes_to_time_full((solution.Max(time_var)))})"
                " -> "
            )
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += (
            f"{manager.IndexToNode(index)}"
            f" Time({minutes_to_time_full((solution.Max(time_var)-data['stop_durations'][manager.IndexToNode(index)]))},{minutes_to_time_full((solution.Max(time_var)))})\n"
        )
        plan_output += f"Time of the route: {solution.Max(time_var)}min\n"
        print(plan_output)
        total_time += solution.Min(time_var)
    print(f"Total time of all routes: {total_time}min")
    # [END solution_printer]


# Example Usage
def main():
    depots = [Depot(id=0, adress="Havenlaan 86C,  Brussel")]#, Depot(id=1, address="Depot B")]
    vans = [
        Van(id=0, depot=depots[0], emissions_per_km=90, fuel_consumption_per_km=0.1, start_time=480, return_deadline=1080),
        Van(id=1, depot=depots[0], emissions_per_km=200, fuel_consumption_per_km=0.2, start_time=480, return_deadline=750),
        Van(id=2, depot=depots[0], emissions_per_km=200, fuel_consumption_per_km=0.2, start_time=480, return_deadline=1080),
        #Van(id=3, depot=depots[1], emissions_per_km=200, fuel_consumption_per_km=0.2, start_time=480, return_deadline=1080),
    ]

    reservations = [
        Reservation(id=1, adress="Lier , grotemarkt , belgium", stop_duration=10, time_window=(480, 600)),
        Reservation(id=2, adress="antwerpen , grotemarkt , belgie", stop_duration=20, time_window=(540, 660)),
        Reservation(id=3, adress="Leuven , grotemarkt , belgium", stop_duration=30, time_window=(600, 720)),
        Reservation(id=4, adress="Gent , grotemarkt , belgium", stop_duration=40, time_window=(480, 600)),
        Reservation(id=5, adress="Genk , grotemarkt , belgium", stop_duration=20, time_window=(540, 660)),
        Reservation(id=6, adress="Hasselt , grotemarkt , belgium", stop_duration=25, time_window=(600, 720)),
        Reservation(id=7, adress="brussel , grotemarkt , belgium", stop_duration=15, time_window=(700, 800)),
        Reservation(id=8, adress="kortrijk , grotemarkt , belgium", stop_duration=20, time_window=(550, 580)),
        Reservation(id=9, adress="Namen, grotemarkt , belgium", stop_duration=25, time_window=(900, 1080)),
        Reservation(id=10, adress="Luik , grotemarkt , belgium", stop_duration=50, time_window=(480, 600)),
        Reservation(id=11, adress="brugge , grotemarkt , belgium", stop_duration=20, time_window=(540, 660)),
        Reservation(id=12, adress="oostende , grotemarkt , belgium", stop_duration=10, time_window=(600, 720)),
        Reservation(id=13, adress="mechelen , grotemarkt , belgium", stop_duration=15, time_window=(480, 900)),

    ]

    optimizer = RouteOptimizer(vans, reservations,depots)
    
    optimizer.generate_matrices_from_api()
    #optimizer.generate_pseudo_matrix()
    data = optimizer.create_data_model()
    optimizer.initialize_routing(data)



    if optimizer.solve():
        print_solution(data, optimizer.manager, optimizer.routing, optimizer.solution)
        #print("Metrics:", optimizer.calculate_metrics())
    else:

        print("No solution found.")


if __name__ == "__main__":
    main()
