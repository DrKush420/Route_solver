from flask import Flask, jsonify, render_template
from route_solver import get_routes  # Import the function from route_solver.py

application = Flask(__name__)

@application.route('/')
def index():
    # Serve the map page (index.html)
    return render_template('index.html')

# @app.route('/get_routes', methods=['GET'])
# def get_routes():
#     # Simulated route data (replace with your actual logic)
#     routes = [{'name': 'Route 1', 'coordinates': [[50.917015, 4.434706], [50.884271, 3.440323], [51.337839, 3.343351], [51.226154, 4.114086]]}, {'name': 'Route 2', 'coordinates': [[50.917015, 4.434706], [50.95394, 4.358771], [50.764677, 4.312432], [50.842989, 4.329696]]}, {'name': 'Route 3', 'coordinates': [[50.917015, 4.434706], [50.92662, 4.624117], [50.896995, 4.636335], [50.852428, 4.78407], [50.776471, 5.114653]]}, {'name': 'Route 4', 'coordinates': [[50.917015, 4.434706], [51.137404, 4.458756], [51.200501, 4.470047], [51.091613, 4.222682], [51.003314, 4.12241]]}, {'name': 'Route 5', 'coordinates': [[51.130249, 4.57099], [51.0487, 4.632142], [51.080732, 4.953324], [51.304634, 4.736368], [51.143007, 4.563523]]}]
    
#     # Return the JSON response
#     return jsonify({"routes": routes})

@application.route('/get_routes')
def get_routes_api():
    # Call the get_routes function and return the data as JSON
    routes = get_routes()  # Get routes from the imported function

    return jsonify({"routes": routes})  # Return routes as JSON

if __name__ == '__main__':
    application.run(debug=True)
