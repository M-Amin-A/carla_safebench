import json
import copy

sample_scenario = {
    "data_id": 0,
    "scenario_folder": "dilema",
    "scenario_id": 1,
    "route_id": 12,
    "risk_level": None,
    "parameters": []
}
scenarios = []

num_scenarios = 100
scenario_folder = "dilema"
scenario_id = 1
route_ids = list(range(4, 14))

for i in range(num_scenarios):
    scenario = copy.deepcopy(sample_scenario)
    scenario["data_id"] = i
    scenario["scenario_folder"] = scenario_folder
    scenario["scenario_id"] = scenario_id
    scenario["route_id"] = route_ids[i % len(route_ids)]
    scenarios.append(scenario)

with open('./out.json', 'w') as file:
    json.dump(scenarios, file)