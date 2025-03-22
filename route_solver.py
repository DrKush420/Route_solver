"""
Vehicle Routing Problem (VRP) with Time Windows solution using Google OR-Tools.
Handles multiple vehicles, depots, and reservations with time constraints.
"""

from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from adress_api import adressapi
import numpy as np
import os
from ingest import load_and_process_dataframe


class Depot:
    """
    Represents a depot location where vehicles start and end their routes.

    Attributes:
        id: Unique identifier for the depot
        address: Physical address of the depot
        coordinates: (latitude, longitude) tuple from geocoding
    """

    def __init__(self, id, address, start_time, return_deadline, coordinates=None):
        self.id = id
        self.address = address
        self.coordinates = coordinates
        self.start_time = start_time
        self.return_deadline = return_deadline


class Van:
    """
    Represents a delivery vehicle with operational constraints.

    Attributes:
        id: Unique vehicle identifier
        depot: Home depot for the vehicle
        start_time: Earliest departure time (minutes since midnight)
        return_deadline: Latest return time (minutes since midnight)
    """

    def __init__(self, id, depot, start_time=480, return_deadline=1080):
        self.id = id
        self.depot = depot
        self.start_time = start_time
        self.return_deadline = return_deadline


class Reservation:
    """
    Represents a delivery stop with time constraints.

    Attributes:
        id: Unique stop identifier
        address: Physical address for delivery
        stop_duration: Service time required at stop (minutes)
        time_window: (earliest, latest) arrival times (minutes since midnight)
        priority: Priority level for mandatory stops
        coordinates: (latitude, longitude) tuple from geocoding
    """

    def __init__(self, id, address, stop_duration, time_window=None, priority=0, coordinates=None):
        self.id = id
        self.index=None
        self.address = address
        self.stop_duration = stop_duration
        self.time_window = time_window or (0, 1440)  # Default: Full day window
        self.priority = priority
        self.coordinates = coordinates


class RouteOptimizer:
    """
    Core optimizer class that handles routing logic and constraints.

    Key Functionality:
    - Geocoding addresses to coordinates
    - Matrix generation for travel times
    - OR-Tools model configuration
    - Solution finding and visualization
    """

    def __init__(self, vans, reservations, depots):
        """
        Initialize optimizer with problem entities.

        Args:
            vans: List of Van objects
            reservations: List of Reservation objects
            depots: List of Depot objects (currently single depot supported)
        """
        self.vans = vans
        self.reservations = reservations
        self.depots = depots
        self.distance_matrix = None
        self.time_matrix = None
        self.stop_durations = None
        self.manager = None  # OR-Tools index manager
        self.routing = None  # OR-Tools routing model
        self.solution = None  # Computed solution
        self.start_hour = 480  # Default start time (8:00 AM in minutes)
        self.api = adressapi()  # Geocoding API client

    # Matrix Generation Methods ------------------------------------------------

    def get_coordinates(self):
        """Geocode all addresses to coordinates using the API."""
        # Process depots
        for depot in self.depots:
            if not depot.coordinates:
                depot.coordinates = self.api.get_coordinates(depot.address)
               
                if not depot.coordinates:
                    print(f"Geocoding failed for depot: {depot.address}")

        # Process reservations
        for res in self.reservations:
            if not res.coordinates:
                res.coordinates = self.api.get_coordinates(res.address)
                if not res.coordinates:
                    print(f"Geocoding failed for reservation: {res.address}")

    def generate_time_matrix_from_api(self):
        """
        Generate time matrix using actual travel times from geocoding API.
        Stores results in self.time_matrix.
        """
        self.get_coordinates()

        # Combine all locations: depots first, then reservations
        all_locations = [d.coordinates for d in self.depots]
        for i in range(len(self.depots)):
            self.depots[i].index=i
        all_locations += [r.coordinates for r in self.reservations]

        # Get travel times in minutes from API
        self.time_matrix = np.ceil(
            np.array(self.api.get_travel_times(all_locations)) / 60)
        self.time_matrix = self.time_matrix.astype(int)

        # Model Setup Methods ------------------------------------------------------

    def add_time_windows_and_stop_durations_from_reservations(self):
        """Prepare time windows and service durations for OR-Tools model."""
        time_windows = []
        stop_durations =  [0 for _ in range(len(self.depots))]  # Depot has no service time

        # Add depot time window (converted to relative time)
        for dep in self.depots:
            depot_window = (
            dep.start_time - self.start_hour,
            dep.return_deadline - self.start_hour
            )
            time_windows.append(depot_window)

        # Add reservation time windows and service durations
        for res in self.reservations:
            time_windows.append((
                res.time_window[0] - self.start_hour,
                res.time_window[1] - self.start_hour
            ))
            stop_durations.append(res.stop_duration)

        self.time_windows = time_windows
        self.stop_durations = stop_durations




    def initialize_routing(self):
        """Configure OR-Tools routing model with constraints."""
        try:
            # Create routing index manager (nodes, vehicles, depot)
            self.manager = pywrapcp.RoutingIndexManager(
                len(self.time_matrix),
                len(self.vans),
                [van.depot.index for van in self.vans],  # index of start location
                [van.depot.index for van in self.vans]   # index of stopping location     
            )  

            # Create routing model instance
            self.routing = pywrapcp.RoutingModel(self.manager)

            # Register transit callback
            def time_callback(from_index, to_index):
                """Calculate total time between nodes including service time."""
                from_node = self.manager.IndexToNode(from_index)
                to_node = self.manager.IndexToNode(to_index)
                return (self.time_matrix[from_node][to_node]
                        + self.stop_durations[to_node])

            transit_callback_index = self.routing.RegisterTransitCallback(time_callback)

            # Set arc cost evaluator (travel time as cost)
            self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

            # Add time dimension constraint
            time_dimension_name = "Time"
            self.routing.AddDimension(
                transit_callback_index,
                0,  # Allow waiting
                600,  # Maximum route duration (10 hours)
                False,  # Don't force start time to zero
                time_dimension_name
            )
            time_dimension = self.routing.GetDimensionOrDie(time_dimension_name)

            # Apply time window constraints for all locations except depot
            for location_idx, (start, end) in enumerate(self.time_windows):
                if location_idx < len(self.depots):
                    continue
                # Convert location index to routing model's node index
                node_index = self.manager.NodeToIndex(location_idx)
                time_dimension.CumulVar(node_index).SetRange(int(start), int(end))

            # Apply time window constraints for all vehicles
            for vehicle_id, van in enumerate(self.vans):
                start_index = self.routing.Start(vehicle_id)
                end_index = self.routing.End(vehicle_id)

                time_dimension.CumulVar(start_index).SetRange(
                    self.time_windows[0][0], self.time_windows[0][1]
                )

                self.routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(start_index))
                self.routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(end_index))
            
            # penalty = 100000000   
            # for node in range(len(self.depots), len(self.time_windows)):
            #     routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

            # print("Disjunctions added.")

        except Exception as e:
            print(f"Routing initialization failed: {str(e)}")
            raise

    # Solution Methods ---------------------------------------------------------

    def solve(self, search_params=None):
        """Execute the routing optimization with given parameters."""
        try:
            # Configure search parameters
            if not search_params:
                search_params = pywrapcp.DefaultRoutingSearchParameters()
                search_params.first_solution_strategy = (
                    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
                )
                search_params.local_search_metaheuristic = (
                    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
                )
                search_params.time_limit.seconds = 5  # Balance quality vs speed

            self.solution = self.routing.SolveWithParameters(search_params)
            return self.solution is not None

        except Exception as e:
            print(f"Solving failed: {str(e)}")
            return False
        


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
                    f"{self._get_location_address(self.manager.IndexToNode(index))}"
                    f" Time({self._minutes_to_time((self.solution.Max(time_var)-self.stop_durations[self.manager.IndexToNode(index)]))},{self._minutes_to_time((self.solution.Max(time_var)))})"
                    " -> "
                )
                index = self.solution.Value(self.routing.NextVar(index))
            time_var = time_dimension.CumulVar(index)
            plan_output += (
                f"{self._get_location_address(self.manager.IndexToNode(index))}"
                f" Time({self._minutes_to_time((self.solution.Max(time_var)-self.stop_durations[self.manager.IndexToNode(index)]))},{self._minutes_to_time((self.solution.Max(time_var)))})\n"
            )
            plan_output += f"Time of the route: {self.solution.Max(time_var)}min\n"
            print(plan_output)
            total_time += self.solution.Min(time_var)
        print(f"Total time of all routes: {total_time}min")
        # [END solution_printer]



    # Helper Methods -----------------------------------------------------------

    def _minutes_to_time(self, minutes):
        """Convert minutes since midnight to HH:MM format."""
        adjusted = minutes + self.start_hour
        return f"{adjusted // 60:02d}:{adjusted % 60:02d}"

    def _get_location_address(self, node_index):
        """Resolve node index to physical address."""
        if node_index < len(self.depots):
            return self.depots[node_index].address
        return self.reservations[node_index - len(self.depots)].address

    def _get_coord_adress(self, node_index):
        """Resolve node index to physical address."""
        if node_index < len(self.depots):
            return [self.depots[node_index].coordinates[1],self.depots[node_index].coordinates[0]]
        return [self.reservations[node_index - len(self.depots)].coordinates[1] , self.reservations[node_index - len(self.depots)].coordinates[0]]
    
    def get_routes(self):
        routes = []
        for vehicle_id in range(self.routing.vehicles()):
            index = self.routing.Start(vehicle_id)
            route = []
            while not self.routing.IsEnd(index):
                node_index = self.manager.IndexToNode(index)
                route.append(self._get_coord_adress(node_index))
                index = self.solution.Value(self.routing.NextVar(index))
            if node_index==self.manager.IndexToNode(index): #skips empty routes
                continue
            node_index = self.manager.IndexToNode(index)
            route.append(self._get_coord_adress(node_index))
            routes.append(route)

        return routes



def test():
    depot = [Depot(
        id=0,
        address="Trawoollaan 1A/2 1830 Machelen",
        start_time=480,  # 8:00 AM
        return_deadline=1080  
    ),
    Depot(
        id=1,
        address="Grote markt , Lier, belgie",
        start_time=480,  # 8:00 AM
        return_deadline=1080  
    )]

    # Create vehicle fleet
    vans = [
        Van(id=0, depot=depot[0]),
        Van(id=1, depot=depot[0]),
        Van(id=2, depot=depot[0]),
        Van(id=3, depot=depot[0]),
        Van(id=4, depot=depot[0]),
        Van(id=5, depot=depot[0]),
        Van(id=6, depot=depot[1]),
    ]

    # Load reservations from data file
    reservations = []
    data_path = os.path.join("data", "history.csv")
    for address, duration in load_and_process_dataframe(data_path):
        print(" adres " + address + " duration " + str(duration))
        reservations.append(
            Reservation(
                id=len(reservations),
                address=address,
                stop_duration=int(duration),
                time_window=(480, 1020)  # 8AM-8PM window
            )
        )
        # Configure and run optimizer
    optimizer = RouteOptimizer(vans, reservations, depot)
    optimizer.generate_time_matrix_from_api()
    optimizer.add_time_windows_and_stop_durations_from_reservations()
    optimizer.initialize_routing()
    optimizer.solve()
    return optimizer


# Example Usage ----------------------------------------------------------------

def format_routes_for_json(routes):
    """Format routes into JSON serializable structure."""
    formatted_routes = []
    for i, route in enumerate(routes):
        route_dict = {
            "name": f"Route {i + 1}",
            "coordinates": route
        }
        formatted_routes.append(route_dict)
    return formatted_routes

def get_routes():
    """Get vehicle routes from a solution and convert to a list of routes."""
    routes=test()
    return format_routes_for_json(routes.get_routes())

def main():
    """Example problem setup and execution."""
    # Configure depot
    depot = [Depot(
        id=0,
        address="Trawoollaan 1A/2 1830 Machelen",
        start_time=480,  # 8:00 AM
        return_deadline=1080  
    ),
    Depot(
        id=1,
        address="Grote markt , Lier, belgie",
        start_time=480,  # 8:00 AM
        return_deadline=1080  
    )]

    # Create vehicle fleet
    vans = [
        Van(id=0, depot=depot[0]),
        Van(id=1, depot=depot[0]),
        Van(id=2, depot=depot[0]),
        Van(id=3, depot=depot[0]),
        Van(id=4, depot=depot[0]),
        Van(id=5, depot=depot[0]),
        Van(id=6, depot=depot[1]),
    ]

    # Load reservations from data file
    reservations = []
    data_path = os.path.join("data", "history.csv")
    for address, duration in load_and_process_dataframe(data_path):
        print(" adres " + address + " duration " + str(duration))
        reservations.append(
            Reservation(
                id=len(reservations),
                address=address,
                stop_duration=int(duration),
                time_window=(480, 1020)  # 8AM-8PM window
            )
        )
        # Configure and run optimizer
    optimizer = RouteOptimizer(vans, reservations, depot)
    optimizer.generate_time_matrix_from_api()
    optimizer.add_time_windows_and_stop_durations_from_reservations()
    optimizer.initialize_routing()

    if optimizer.solve():
        optimizer.print_solution()
    else:
        print("No feasible solution found")


if __name__ == "__main__":
    main()
