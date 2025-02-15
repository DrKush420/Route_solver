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
import pandas as pd
import os
from ingest import load_and_process_dataframe


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



class Reservation:
    """
    Represents a reservation with specific location, time, and duration.
    """
    def __init__(self, id, adress, stop_duration, time_window=None, priority=0,coordinates=None):
        self.id = id
        self.adress = adress
        self.stop_duration = stop_duration
        self.time_window = (time_window[0],time_window[1]) or (0, 1440)  # Default: Full day
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
        self.stop_durations =None
        self.manager = None
        self.routing = None
        self.solution = None
        self.depot=0
        self.start_hour=480
        self.api=adressapi()

    def generate_pseudo_matrix(self):

        all_locations = [depot.adress for depot in self.depots] + [res.adress for res in self.reservations]
        num_locations = len(all_locations)

        self.distance_matrix = [[abs(i - j) * 7 for j in range(num_locations)] for i in range(num_locations)]
        self.time_matrix = [[abs(i - j) * 10 for j in range(num_locations)] for i in range(num_locations)]

    def get_coordinates(self):

        for res in self.depots:
            if res.coordinates is None:
                res.coordinates=self.api.get_coordinates(res.adress)
                if not res.coordinates:
                    print(f"Could not geocode: {res.adress}")

        for res in self.reservations:
            if res.coordinates is None:
                res.coordinates=self.api.get_coordinates(res.adress)
                if not res.coordinates:
                    print(f"Could not geocode: {res.adress}")
                


    def generate_matrices_from_api(self):
        self.get_coordinates()
        all_locations = [depot.coordinates for depot in self.depots] + [res.coordinates for res in self.reservations]
        #print(all_locations)
        self.time_matrix= np.ceil(np.array(self.api.get_travel_times(all_locations))/60).astype(int)
        with open("matrix.txt", "w") as file:
            for row in self.time_matrix:
                file.write(" ".join(map(str, row)) + "\n")


    def create_data_model(self):
        """
        Creates the data model for the routing problem.
        """

        time_windows = []
        stop_durations = [0]
        
        # Add depot time windows
        # for van in self.vans:
        #     time_windows.append((480, 1080))
        time_windows.append(((480-self.start_hour), (1080-self.start_hour)))
        # Add reservation time windows and stop durations
        for res in self.reservations:
            time_windows.append((res.time_window[0]-self.start_hour,res.time_window[1]-self.start_hour))
            stop_durations.append(res.stop_duration)
        self.stop_durations=stop_durations
        self.time_windows=time_windows
        #print(time_windows)
        #print(self.time_matrix)

    def get_recommendation(self,new_reservation):
        #temporary 
        coord=[[4.348591, 50.865544], [4.566716, 51.13374], [4.402453, 51.246664], [4.710309, 50.881011], [3.732703, 51.057496], [5.482751, 50.969549],
         [5.334128, 50.935484], [4.371755, 50.843183], [3.275675, 50.837035], [4.398169, 51.918494], [5.554438, 50.635755], [3.217718, 51.208664], 
         [2.924749, 51.220182], [4.471338, 51.025018]]
        for c in range(len(self.depots)):
            self.reservations[c].coordinates=coord[c]
        for c in range(1,len(coord)):
            self.reservations[c-len(self.depots)].coordinates=coord[c]

        self.reservations.append(new_reservation)
        target_node=len(self.reservations)+len(self.depots)-1

        """
        Model execution for a new reservation.
        """
        self.generate_matrices_from_api()
        #optimizer.generate_pseudo_matrix()
        self.create_data_model()
        self.initialize_routing()
        self.solve()

        # Get the time dimension
        time_dimension = self.routing.GetDimensionOrDie("Time")
        index = self.manager.NodeToIndex(target_node)
        # Get the cumulative time variable at the node
        time_var = time_dimension.CumulVar(index)
        # Get the latest possible arrival time at this node
        max_time = self.solution.Max(time_var)
        print(f"Latest possible arrival time at node {target_node}: {self.minutes_to_time_full(max_time-new_reservation.stop_duration)}")
        return max_time



    def initialize_routing(self):
        """
        Initializes the routing model with constraints.
        """
        try:
            print("Initializing routing...")
            # Create the RoutingIndexManager
            manager = pywrapcp.RoutingIndexManager(len(self.time_matrix),
                                                        len(self.vans),
                                                        self.depot,)
            routing = pywrapcp.RoutingModel(manager)
            self.manager=manager
            def time_callback(from_index, to_index):
                """Returns the travel time between the two nodes."""
                # Convert from routing variable Index to time matrix NodeIndex.
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return self.time_matrix[from_node][to_node]+self.stop_durations[to_node]

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
                0,  # allow waiting time
                600,  # maximum time per vehicle
                False,  # Don't force start cumul to zero.
                time,
            )
            time_dimension = routing.GetDimensionOrDie(time)
            print("Time dimension added.")

            #     # Add service time at each node
            # for node in range(len(self.stop_durations)):
            #     index = manager.NodeToIndex(node)
            #     time_dimension.CumulVar(index).SetValue(self.stop_durations[node])
            # print("Service time added for each node.")

            # def service_time_callback(from_index):
            #     """Returns the service time for a given node."""
            #     from_node = manager.IndexToNode(from_index)
            #     return self.stop_durations[from_node]

            # service_callback_index = routing.RegisterUnaryTransitCallback(service_time_callback)
            # time_dimension.SetCumulVarSoftUpperBound(service_callback_index, 0, 1)

            # # Add service time as part of the travel time
            # routing.AddTransitEvaluator(service_callback_index, time)

            # Add time window constraints for each location except depot.
            for location_idx, time_window in enumerate(self.time_windows):
                if location_idx == self.depot:
                    continue
                index = manager.NodeToIndex(location_idx)
                time_dimension.CumulVar(index).SetRange(int(time_window[0]), int(time_window[1]))
            print("Time window constraints added for locations.")


            # Add time window constraints for each vehicle start node.
            depot_idx =0
            for vehicle_id in range(len(self.vans)):
                index = routing.Start(vehicle_id)
                time_dimension.CumulVar(index).SetRange(
                    self.time_windows[depot_idx][0], self.time_windows[depot_idx][1]
                )
            print("Time window constraints added for vehicles.")


            for i in range(len(self.vans)):
                routing.AddVariableMinimizedByFinalizer(
                    time_dimension.CumulVar(routing.Start(i))
                )
                routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))
            


            # penalty = 100000000
            # for node in range(1, len(self.time_windows)):
            #     routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

            # print("Disjunctions added.")

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
                search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
                search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
                search_parameters.time_limit.seconds = 50

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


    def minutes_to_time_full(self,minutes):
        minutes=minutes+self.start_hour
        hours = minutes // 60
        mins = minutes % 60
        return f"{int(hours):02d}:{int(mins):02d}"


    # [START solution_printer]
    def print_sol(self):
        """Prints solution on console."""
        print(f"Objective: {self.solution.ObjectiveValue()}")
            # Display dropped nodes.
        dropped_nodes = "Dropped nodes:"
        for node in range(self.routing.Size()):
            if self.routing.IsStart(node) or self.routing.IsEnd(node):
                continue
            if self.solution.Value(self.routing.NextVar(node)) == node:
                dropped_nodes += f" {self.manager.IndexToNode(node)}"
        print(dropped_nodes)
        time_dimension = self.routing.GetDimensionOrDie("Time")
        total_time = 0
        for vehicle_id in range(len(self.vans)):
            index = self.routing.Start(vehicle_id)
            plan_output = f"Route for vehicle {vehicle_id}:\n"
            while not self.routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
        
                plan_output += (
                    f"{self.manager.IndexToNode(index)}"
                    f" Time({self.minutes_to_time_full((self.solution.Max(time_var)-self.stop_durations[self.manager.IndexToNode(index)]))},{self.minutes_to_time_full((self.solution.Max(time_var)))})"
                    " -> "
                )
                index = self.solution.Value(self.routing.NextVar(index))
            time_var = time_dimension.CumulVar(index)
            plan_output += (
                f"{self.manager.IndexToNode(index)}"
                f" Time({self.minutes_to_time_full((self.solution.Max(time_var)-self.stop_durations[self.manager.IndexToNode(index)]))},{self.minutes_to_time_full((self.solution.Max(time_var)))})\n"
            )
            plan_output += f"Time of the route: {self.solution.Max(time_var)}min\n"
            print(plan_output)
            total_time += self.solution.Min(time_var)
        print(f"Total time of all routes: {total_time}min")
        # [END solution_printer]


    # [START solution_printer]
    def print_solution(self):
        """Prints solution on console."""
        print(f"Objective: {self.solution.ObjectiveValue()}")
            # Display dropped nodes.
        dropped_nodes = "Dropped nodes:"
        for node in range(self.routing.Size()):
            if self.routing.IsStart(node) or self.routing.IsEnd(node):
                continue
            if self.solution.Value(self.routing.NextVar(node)) == node:
                dropped_nodes += f" {self.manager.IndexToNode(node)}"
        print(dropped_nodes)
        time_dimension = self.routing.GetDimensionOrDie("Time")
        total_time = 0
        for vehicle_id in range(len(self.vans)):
            index = self.routing.Start(vehicle_id)
            plan_output = f"Route for vehicle {vehicle_id}:\n"
            while not self.routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
        
                plan_output += (
                    f"{self.depots[self.manager.IndexToNode(index)].adress if self.manager.IndexToNode(index) < len(self.depots) else self.reservations[self.manager.IndexToNode(index) - len(self.depots)].adress}"
                    f" Time({self.minutes_to_time_full((self.solution.Max(time_var)-self.stop_durations[self.manager.IndexToNode(index)]))},{self.minutes_to_time_full((self.solution.Max(time_var)))})"
                    " -> "
                )
                index = self.solution.Value(self.routing.NextVar(index))
            time_var = time_dimension.CumulVar(index)
            plan_output += (
                f"{self.depots[self.manager.IndexToNode(index)].adress if self.manager.IndexToNode(index) < len(self.depots) else self.reservations[self.manager.IndexToNode(index) - len(self.depots)].adress}"
                f" Time({self.minutes_to_time_full((self.solution.Max(time_var)-self.stop_durations[self.manager.IndexToNode(index)]))},{self.minutes_to_time_full((self.solution.Max(time_var)))})\n"
            )
            plan_output += f"Time of the route: {self.solution.Max(time_var)}min\n"
            print(plan_output)
            total_time += self.solution.Min(time_var)
        print(f"Total time of all routes: {total_time}min")
        # [END solution_printer]


# Example Usage
def main():
    depots = [Depot(id=0, adress="Trawoollaan 1A/2 1830 Machelen")]#, Depot(id=1, address="Depot B")]
    vans = [
        Van(id=0, depot=depots[0], emissions_per_km=90, fuel_consumption_per_km=0.1, start_time=480, return_deadline=1080),
        Van(id=1, depot=depots[0], emissions_per_km=200, fuel_consumption_per_km=0.2, start_time=480, return_deadline=750),
        Van(id=2, depot=depots[0], emissions_per_km=200, fuel_consumption_per_km=0.2, start_time=480, return_deadline=1080),
        Van(id=3, depot=depots[0], emissions_per_km=200, fuel_consumption_per_km=0.2, start_time=480, return_deadline=1080),
        Van(id=4, depot=depots[0], emissions_per_km=200, fuel_consumption_per_km=0.2, start_time=480, return_deadline=1080),
        Van(id=5, depot=depots[0], emissions_per_km=200, fuel_consumption_per_km=0.2, start_time=480, return_deadline=1080),
    ]

    file_path = os.path.join("data", "history.csv")
    time_window = (480,1200)
    reservations = []
    tup= load_and_process_dataframe(file_path)
    for res in range(len(tup)):
        print(tup[res][1])
        reservations.append(Reservation(id=res, adress=tup[res][0], stop_duration=int(tup[res][1]), time_window=time_window))

    optimizer = RouteOptimizer(vans, reservations,depots)
    
    optimizer.generate_matrices_from_api()
    #optimizer.generate_pseudo_matrix()
    optimizer.create_data_model()
    optimizer.initialize_routing()
    if optimizer.solve():
        optimizer.print_solution()
        #print("Metrics:", optimizer.calculate_metrics())
    else:
        print("No solution found.")
    
    #optimizer.get_recommendation(Reservation(id=14, adress="Halle , grotemarkt , belgium", stop_duration=15, time_window=(480, 1400)))
    #optimizer.print_solution()


if __name__ == "__main__":
    main()
