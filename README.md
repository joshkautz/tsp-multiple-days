# tsp-multiple-days
Modeling a real-world scenarios for the traveling salesperson problem over the span of multiple days using Google OR-Tools library in Python.

## Constraints
- Single vehicle.
- Be able to specify an arbitrarily large number of locations with fixed time windows for the vehicle to visit.
- Be able to specify an arbitrarily large number of locations with non-fixed time windowsfor the vehicle to visit.
- Be able to specify time windows for which the vehicle may visit each location.
- Be able to specify service costs to represent the duration for which the vehicle must stay at each location.
- Be able to specify drop penalties to allow for the omission (dropping) of locations in order to yielf a feasible solution. This will also allow for influencing the severity of the omission of certain locations.
- Be able to specify the number of days over which to create the solution.
- Be able to specify the start location and end location for each day.
- Be able to specify the start time and end time for each day.
- Be able to specify the time for how long Google OR-Tools may search for a solution.


## Data Sets
### 3_days_60_locations.json
One vehicle, Three days, different starting and ending times and locations each day, 15 minute service costs, open time windows for each non-fixed location, equal drop penalties for each non-fixed location, 10 second solve time.

### *In Progress...*