from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import math
import sys
import json

# Specify the JSON data file
with open('3_days_60_locations.json') as file:
    event = json.load(file)

Locations = event['Locations']
Matrix = event['Matrix']
Windows = event['Windows']
ServiceCosts = event['ServiceCosts']
Penalties = event['Penalties']
NUM_DAYS = event['NumberOfDays']
NUM_EVENTS = event['NumberOfEvents']
DURATION = event['Duration']
ONE_DAY = 86400

def transit_callback(from_index, to_index):
    # Returns the travel time plus service time between the two nodes.
    # Convert from routing variable Index to time matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return Matrix[from_node][to_node] + ServiceCosts[from_node]

# Create the routing index manager.
# Start is the start location of the first day
# End is the end location of the last day
manager = pywrapcp.RoutingIndexManager(len(Matrix), 1, [0], [2*NUM_DAYS-1])

# Create Routing Model.
routing = pywrapcp.RoutingModel(manager)

# Register the Transit Callback.
transit_callback_index = routing.RegisterTransitCallback(transit_callback)

# Set the arc cost evaluator for all vehicles
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Add Time Windows constraint.
routing.AddDimension(
    transit_callback_index,
    ONE_DAY * NUM_DAYS, # An upper bound for slack (the wait times at the locations).
    ONE_DAY * NUM_DAYS, # An upper bound for the total time over each vehicle's route.
    False, # Determine whether the cumulative variable is set to zero at the start of the vehicle's route.
    'Time')
time_dimension = routing.GetDimensionOrDie('Time')

# Allow locations to be droppable.
# Do not allow the start location of the first day to be droppable.
# Do not allow the end location of the last day to be droppable.
for node in range(0, len(Matrix)):
    if node != 0 and node != (2 * NUM_DAYS) - 1:
        routing.AddDisjunction([manager.NodeToIndex(node)], Penalties[node])

# Add time window constraints for all start and end locations.
for i in range(0, NUM_DAYS*2):
    if i == 0:
        index = routing.Start(0)
    elif i == NUM_DAYS*2 - 1:
        index = routing.End(0)
    else:
        index = manager.NodeToIndex(i)
    time_dimension.CumulVar(index).SetRange(Windows[i][0],Windows[i][1])

# Add time window constraints for all additional event locations.
for location_index, time_window in enumerate(Windows):
    if location_index in range(NUM_DAYS * 2, NUM_DAYS * 2 + NUM_EVENTS):
        index = manager.NodeToIndex(location_index)

        # Add the range between the start of the first day and the end of the last day
        time_dimension.CumulVar(index).SetRange(Windows[location_index][0],Windows[location_index][1])

# Add time window constraints for all regular locations.
for location_index, time_window in enumerate(Windows):
    if location_index in range(NUM_DAYS * 2 + NUM_EVENTS, len(Matrix)):
        index = manager.NodeToIndex(location_index)

        # Add the range between the start of the first day and the end of the last day
        time_dimension.CumulVar(index).SetRange(0, 86400 * NUM_DAYS)

        for Day in range(NUM_DAYS):

            Day_Start = Day * ONE_DAY
            Day_End = Day_Start + ONE_DAY
            Working_Start = Windows[Day * 2][0]
            Working_End =	Windows[Day * 2 + 1][0]

            # Remove the range between the start of the day and the start of work
            time_dimension.CumulVar(index).RemoveInterval(Day_Start, Working_Start)

            # Remove the range between the start of the day and the start of location
            time_dimension.CumulVar(index).RemoveInterval(Day_Start, Day_Start + time_window[0])

            # Remove the range between the end of work and the end of the day
            time_dimension.CumulVar(index).RemoveInterval(Working_End, Day_End)

            # Remove the range between the end of location and the end of the day
            time_dimension.CumulVar(index).RemoveInterval(Day_Start + time_window[1], Day_End)

# Instantiate route start and end times to produce feasible times
routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(0)))
routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(0)))

# Setting first solution heuristic. 
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.ALL_UNPERFORMED)

# Setting local search metaheuristics:
search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
search_parameters.time_limit.seconds = DURATION
search_parameters.log_search = False

# Solve the problem
solution = routing.SolveWithParameters(search_parameters)

if not solution:
    print("No solution found !")
    sys.exit(1)

result = {
    'DroppedLocations': [],
    'ScheduledLocations': []
}

# Return dropped locations
for node in range(routing.Size()):
    if routing.IsStart(node) or routing.IsEnd(node):
        continue
    if solution.Value(routing.NextVar(node)) == node:
        index = manager.IndexToNode(node)
        result['DroppedLocations'].append(Locations[index])

# Return scheduled locations
index = routing.Start(0)
while not routing.IsEnd(index):
    time = time_dimension.CumulVar(index)
    Location = Locations[manager.IndexToNode(index)]
    Location['earliestArrivalTime'] = solution.Min(time);
    Location['latestArrivalTime'] = solution.Max(time);
    result['ScheduledLocations'].append(Location)
    index = solution.Value(routing.NextVar(index))
time = time_dimension.CumulVar(index)
Location = Locations[manager.IndexToNode(index)]
Location['earliestArrivalTime'] = solution.Min(time);
Location['latestArrivalTime'] = solution.Max(time);
result['ScheduledLocations'].append(Location)

print(result)