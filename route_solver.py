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
        self.depot_index = 0  # Currently only supports single depot
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
        all_locations += [r.coordinates for r in self.reservations]

        # Get travel times in minutes from API
        self.time_matrix = np.ceil(
            np.array(self.api.get_travel_times(all_locations)) / 60)
        self.time_matrix = self.time_matrix.astype(int)

        # Model Setup Methods ------------------------------------------------------

    def add_time_windows_and_stop_durations_from_reservations(self):
        """Prepare time windows and service durations for OR-Tools model."""
        time_windows = []
        stop_durations = [0]  # Depot has no service time

        # Add depot time window (converted to relative time)
        depot_window = (
            self.depots[0].start_time - self.start_hour,
            self.depots[0].return_deadline - self.start_hour
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
                self.depot_index
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
                # if location_idx == self.depot_index:
                #     continue
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
                search_params.time_limit.seconds = 100  # Balance quality vs speed

            self.solution = self.routing.SolveWithParameters(search_params)
            return self.solution is not None

        except Exception as e:
            print(f"Solving failed: {str(e)}")
            return False

    def print_solution(self):
        """Format and print the optimized routes with timing information."""
        if not self.solution:
            print("No solution to print")
            return

        time_dimension = self.routing.GetDimensionOrDie("Time")
        total_time = 0

        for vehicle_id in range(len(self.vans)):
            index = self.routing.Start(vehicle_id)
            route = []

            while not self.routing.IsEnd(index):
                node = self.manager.IndexToNode(index)
                time_var = time_dimension.CumulVar(index)

                # Format arrival and departure times
                arrival = self._minutes_to_time(self.solution.Min(time_var))
                departure = self._minutes_to_time(self.solution.Max(time_var))
                address = self._get_location_address(node)

                route.append(
                    f"{address} (Arrive: {arrival}, Depart: {departure})"
                )
                index = self.solution.Value(self.routing.NextVar(index))

            # Add final depot node
            node = self.manager.IndexToNode(index)
            time_var = time_dimension.CumulVar(index)
            total_time += self.solution.Min(time_var)
            route.append(
                f"{self._get_location_address(node)}"
                f" (Arrive: {self._minutes_to_time(self.solution.Min(time_var))})"
            )

            # Print formatted route
            print(f"\nVehicle {vehicle_id} Route:")
            print("\n -> ".join(route))
            print(f"Total Route Time: {self.solution.Min(time_var)} minutes\n")

        print(f"Combined Total Time: {total_time} minutes")

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


# Example Usage ----------------------------------------------------------------

def main():
    """Example problem setup and execution."""
    # Configure depot
    depot = Depot(
        id=0,
        address="Trawoollaan 1A/2 1830 Machelen",
        start_time=480,  # 8:00 AM
        return_deadline=500  # 6:00 PM
    )

    # Create vehicle fleet
    vans = [
        Van(id=0, depot=depot),
        Van(id=1, depot=depot),
        Van(id=2, depot=depot),
        Van(id=3, depot=depot),
        Van(id=4, depot=depot),
    ]

    # Load reservations from data file
    reservations = []
    data_path = os.path.join("data", "history.csv")
    for address, duration in load_and_process_dataframe(data_path):
        print("adres " + address + "duration " + str(duration))
        reservations.append(
            Reservation(
                id=len(reservations),
                address=address,
                stop_duration=int(duration),
                time_window=(480, 1020)  # 8AM-8PM window
            )
        )
        # Configure and run optimizer
    optimizer = RouteOptimizer(vans, reservations, [depot])
    optimizer.generate_time_matrix_from_api()
    optimizer.add_time_windows_and_stop_durations_from_reservations()
    optimizer.initialize_routing()

    if optimizer.solve():
        optimizer.print_solution()
    else:
        print("No feasible solution found")


if __name__ == "__main__":
    main()
