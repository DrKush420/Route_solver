import openrouteservice
import numpy as np
import os
from dotenv import load_dotenv


class adressapi:

    def __init__(self):
        # Load variables from .env file
        load_dotenv()
        self.API_KEY = os.getenv("API_KEY")
        self.client = openrouteservice.Client(key=self.API_KEY)
        self.locations = []

    # Function to get coordinates from an address
    def get_coordinates(self, address):
        response = self.client.pelias_search(text=address)
        if response and response['features']:
            return response['features'][0]['geometry']['coordinates']
        return None

    # Function to get travel times for a new location
    def get_travel_times(self, locations):

        response = self.client.distance_matrix(
            locations=locations,
            profile='driving-car',
            metrics=['duration']
        )

        return response['durations']

    # Function to add a new address to the matrix
    def add_location(self, address):

        # Get coordinates
        new_location = self.get_coordinates(address)
        if not new_location:
            print(f"Could not geocode: {address}")
            return

        # If first location, just initialize
        if not self.locations:
            self.locations.append(new_location)
            distance_matrix = np.array([[0]])  # 1x1 zero matrix for self-travel
            return

        # Get new travel times
        distance_matrix = np.array(self.get_travel_times(self.locations + [new_location]))
        self.locations.append(new_location)

        print(distance_matrix)
        return distance_matrix, self.locations

# api=adressapi()
# # Example usage
# api.add_location("Lier , grotemarkt , belgium")
# api.add_location("antwerpen, grotemarkt,belgie")
# api.add_location("London, UK")
# api.add_location("Madrid, Spain")  # This will only fetch data for Madrid vs others
