# model.py
#
#    Copyright 2018 Jeff Kessler
#    This program is distributed under the terms of the GNU General Public License
#
#    The EVCharging Model is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This model is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with the EVCharging Model.  If not, see <http://www.gnu.org/licenses/>.

from ortools.linear_solver import pywraplp
from dateutil.parser import parse as date_parse
import weakref
import pandas as pd
import time
import argparse

#TODO:
# Make Data Processing Class

class DataInputError(Exception):
    pass

class ModelSetupError(Exception):
    pass


class EVOptimizer():
    def __init__(self, carbon_price=0):
        """ carbon price is set in $/ton of CO2e for LCA of grid electricity
        """

        self.solver          = pywraplp.Solver('Vehicle Charging',
                                pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)
        self.status          = "Not Run"
        self.objective       = self.solver.Objective()
        self.constraints     = {}
        self.variables       = {}
        self.driving         = {}
        self.charger_levels  = []
        self.vehicles        = []
        self.electricity     = []
        self.carbon_price    = carbon_price


    def add_electric_grid(self, grid_profile):
        self.electricity = grid_profile


    def get_charging_profile(self,filename="charging_profile.csv"):
        start_time = time.time()

        if self.status == "Not Run":
            raise ModelSetupError("The Model has not been solved")

        df = pd.DataFrame()
        for variable in self.variables:
            energy_demand = self.variables[variable].solution_value()
            if energy_demand > 0:
                charger_details = variable.split("_")
                charging = [energy_demand] + charger_details
                df = df.append([charging])

        df.to_csv(filename, header=False, mode="a")

        print("It took %f seconds to export results" % (time.time()-start_time))

    def get_driving_profile(self):
        if self.status == "Not Run":
            raise ModelSetupError("The Model has not been solved")

        for variable in self.driving:
            energy_demand = self.driving[variable].solution_value()
            if energy_demand > 0:
                charger_details = variable.split("_")
                print(energy_demand, charger_details)



    def add_vehicle_constraints(self):
        ''' Adds hourly charging constraints for each vehicle to the linear program
            and adds costs for charging to the objective function
        '''
        for ref in self.vehicles:
            vehicle = ref()
            self.generate_hourly_charging(vehicle)
            self.generate_energy_demand(vehicle)
            self.state_of_charge(vehicle)

    def is_driving(self,vehicle, hour):
        ''' Used to constrain charging variables.
            Returns true if the vehicle is driving
        '''
        return vehicle.drive_cycle[hour] is not 0


    def add_chargers(self, chargers):
        # Defined limits for charging power in kW
        # Duple of charging power and cost to charge in $/kWh
        for charger in chargers:
            self.charger_levels.append(charger)

    def generate_energy_demand(self, vehicle):
        hours           = len(vehicle.drive_cycle)

        for hour in range(hours):
            driving_name  = str("driving_%s_%i") % (id(vehicle), hour)
            energy_demand = vehicle.drive_cycle[hour]/vehicle.energy_use
            self.driving[driving_name] = self.solver.NumVar(energy_demand, energy_demand, driving_name)

    def generate_hourly_charging(self, vehicle):
        hours           = len(vehicle.drive_cycle)
        vehicle_id      = id(vehicle)
        SetCoefficient  = self.objective.SetCoefficient
        charger_levels  = self.charger_levels

        # Get Charging Levels
        charging_types = len(charger_levels)

        if not self.charger_levels:
            raise ModelSetupError("Charging Stations have not been added")

        if not self.electricity:
            raise ModelSetupError("Electric Grid Profile has not been added")

        for hour in range(hours):
            is_driving       = self.is_driving(vehicle, hour)
            electricity_rate = self.electricity.rate(hour)
            carbon_cost      = self.electricity.carbon(hour)*self.carbon_price

            for i in range(charging_types):
                level = charger_levels[i].level
                identifier = str("level%s_%s_%s") % (level,vehicle_id,hour)

                if not is_driving  or level == "slack":
                    # Allows "slack" charging so that model can still converge
                    charging_rate = charger_levels[i].power
                    charging_cost = charger_levels[i].cost
                else:
                # Vehicle can't drive and charge simultaneously
                    charging_rate  = 0
                    charging_cost  = 9999


                # Define the Charging Variable and adds it to the Objective Function
                charging_var = self.variables[identifier] = self.solver.NumVar(0, charging_rate, identifier)
                SetCoefficient(charging_var, charging_cost + electricity_rate + carbon_cost)


    def state_of_charge(self,vehicle):
        # Sets up the constraints such that the state of charge is
        # never below a minimum threshold in a given hour
        # SOC = Initial State of Charge + sum(charging) - sum(driving)
        low             = vehicle.battery_capacity*0.10
        high            = vehicle.battery_capacity
        hours           = len(vehicle.drive_cycle)
        constraints     = self.constraints
        vehicle_id      = id(vehicle)
        charging        = self.variables
        charger_levels  = self.charger_levels
        charging_types  = len(charger_levels)
        initial_state   = str("Init_%s") % vehicle_id

        # Optimize so that only variables for relevant driving hours are added
        # May want to randomize the state of charge?
        init = self.variables[initial_state] = self.solver.NumVar(low, low, initial_state)

        for hour in range(hours):
            constraint_name = str("SOC_%s_%i") % (id(vehicle), hour)
            constraints[constraint_name] = self.solver.Constraint(low, high)

            # State of charge at Time = 0
            constraints[constraint_name].SetCoefficient(init, 1)

            for interval in range(hour+1):
                try:
                    driving_energy = str("driving_%s_%i") % (vehicle_id, interval)
                    constraints[constraint_name].SetCoefficient(self.driving[driving_energy], -1)
                except:
                    raise ModelSetupError("Error adding driving constraint %s for interval %i" % (constraint_name, constraint_name))

                for i in range(charging_types):
                    try:
                        # Energy added to battery by charging
                        var_name = str("level%s_%s_%s") % (charger_levels[i].level, vehicle_id, interval)
                        constraints[constraint_name].SetCoefficient(charging[var_name], 1)
                    except:
                        raise ModelSetupError("Error adding charging constraint %s for interval %i" % (constraint_name, interval))


    def add_vehicle(self, vehicle):
        self.vehicles.append(weakref.ref(vehicle))

    def add_all_vehicles(self, vehicles):
        start_time = time.time()
        for vehicle in vehicles:

            self.add_vehicle(vehicle)

        print("Adding all Vehicles took %f seconds" % (time.time()-start_time))

    def clear(self):
        self.solver.Clear()
        self.status = "Not Run"

    def solve(self, clear=False):
        if clear:
            self.clear()

        if self.status != "Not Run":
            return "Model has already returned status:%s" % self.status

        if not self.vehicles:
            raise ModelSetupError("Vehicle fleet has not been added")

        self.add_vehicle_constraints()
        solver_result = self.solver.Solve()
        self.status = self.solver_status(solver_result)

        return self.status

    def solver_status(self, status):
        if status is self.solver.OPTIMAL:
            return "OPTIMAL"

        if status is self.solver.INFEASIBLE:
            return "INFEASIBLE"

        if status is self.solver.ABNORMAL:
            return "ABNORMAL"

        if status is self.solver.NOT_SOLVED:
            return "NOT_SOLVED"

class ElectricGrid():
    def __init__(self, hours=24, carbon_price=125):
        '''
        Setup an electric grid with carbon intensity and
        the cost for charging ($/kWh) in a given hour
        '''
        if hours<24:
            raise DataInputError("The grid must have at least 24 hours defined")

        self.rates = [0]*hours
        self.CI = [0]*hours
        self.carbon_price = carbon_price

    def normalize_hour(self, hour):
        normalized = hour - int(hour/24)*24
        try:
            self.rates[hour]
            return hour
        except:
            self.rates[normalized]
            return normalized
        else:
            raise ModelSetupError("Error with finding Grid Data")

    def rate(self, hour):
        normal_hour = self.normalize_hour(hour)
        return self.rates[normal_hour]

    def carbon(self, hour):
        hour = self.normalize_hour(hour)
        return self.CI[hour]

    def define_hour(self, hour, cost, CI):
        self.rates[hour] = cost
        self.CI[hour] = CI

    def load_grid_rates(self, filename="electricity_pricing.csv"):
        '''
        Will load rate and CI from a CSV file
        '''
        # Configured to use specific default file.  Will improve later
        MJ_per_kWh = 3.6
        tons_per_grams = 1e-6
        df = pd.read_csv(filename)
        for index, row in df.iterrows():
            # Convert from gCO2e/MJ to tons CO2e/kWh
            CI = row["CI"] * MJ_per_kWh * tons_per_grams
            try:
                self.define_hour(int(row["hour"]), row["rate"], CI)
            except:
                raise DataInputError("The electric grid data columns must be labeled as 'hour', 'rate', 'CI'")


class ChargingStation():
    _instances = []
    def __init__(self, power, cost, level):
        self.power = power
        self.cost  = cost
        self.level = level
        self._instances.append(weakref.ref(self))

    @classmethod
    def charger_list(cls):
        charger_list = []
        for ref in cls._instances:
            obj = ref()
            if obj is None:
                cls._instances.remove(ref)
            else:
                charger_list.append(obj)
        return charger_list


class Vehicle():
    _instances   = []
    _vehicle_ids = {}
    def __init__(self, battery_capacity = 75, energy_use = 3.5, driving_days = 7, vehicle_id = "default"):
        # Energy Use is in miles/kWh
        # Battery Capacity is in kWh
        # Drive cycle is the number of days of driving behavior to optimize over

        self._instances.append(weakref.ref(self))

        self.battery_capacity         = battery_capacity
        self.energy_use               = energy_use
        self.drive_cycle              = [0]*driving_days*24
        self.driving_days             = driving_days
        self._vehicle_ids[vehicle_id] = weakref.ref(self)

    @classmethod
    def get_vehicle(cls, vehicle_id):
        if vehicle_id not in cls._vehicle_ids:
            use_vehicle = Vehicle(vehicle_id = vehicle_id)
        else:
            use_vehicle = cls._vehicle_ids[vehicle_id]()

        return use_vehicle

    @classmethod
    def load_drive_cycle(cls, filename="vehicle_trips.csv", limit=100, starting_index=0):
        # Configured to use specific default file
        # Should not be part of the vehicle class
        vehicle_list = []
        vehicle_id   = None
        df           = pd.read_csv(filename)
        df           = df.sort_values(["VehNum"], ascending=[1])
        df           = df.reset_index()
        df           = df[df.index >= starting_index]

        for index, row in df.iterrows():
            if len(vehicle_list) >= limit and row["VehNum"] != vehicle_id:
                return index+1, vehicle_list
                print("Left off at: %i" % (index+1))

            # Convert from gCO2e/MJ to tons CO2e/kWh
            duration       = row["duration"]
            trip_miles     = row["trpmiles"]
            miles_per_hour = trip_miles/duration
            date           = row["tdaydat2"]
            vehicle_id     = row["VehNum"]
            vehicle_type   = row["vehtype"]
            vehicle        = cls.get_vehicle(vehicle_id)



            if vehicle not in vehicle_list:
                vehicle_list.append(vehicle)

                try:
                    # Define Battery and Capacity based on Vehicle Type
                    cls.vehicle_type(vehicle, vehicle_type)
                except:
                    raise DataInputError("Vehicle type not specified in input file")

            for hours in range(duration):
                hour = row["strthr"] + hours
                if hour >= 24:
                    hour = hour - 24
                try:
                    vehicle.add_mileage(hour, date, miles_per_hour)

                except:
                    print(hour, date, miles_per_hour)
                    raise DataInputError("The trip data columns must be labeled as 'VehNum', 'duration', 'trpmiles', 'tdaydat2")

        return index+1, vehicle_list

    def vehicle_type(self, veh_type):
        """
        Takes the vehicle type as identified by the NHTS data, and creates battery
        size and efficiency parameters
        """
        if veh_type == 1:
            # Automobile/Car/Station Wagon
            self.battery_capacity = 65
            self.energy_use       = 4

        if veh_type == 2:
            # Van (Mini/Cargo/Passenger)
            self.battery_capacity = 100
            self.energy_use       = 2.5

        if veh_type == 3:
            # SUV
            self.battery_capacity = 100
            self.energy_use       = 2.9

        if veh_type == 4:
            # Pickup Truck
            self.battery_capacity = 120
            self.energy_use       = 2.5

        if veh_type == 5:
            # Other Truck
            self.battery_capacity = 160
            self.energy_use       = 2

        if veh_type == 6:
            # RV/Rereational
            self.battery_capacity = 200
            self.energy_use       = 1.7

        if veh_type == 7:
            # Motorcycle
            self.battery_capacity = 14.4
            self.energy_use       = 15.5

    def add_mileage(self, hour, date, miles):
        '''
        adds mileage for a specific vehicle
        '''
        # Date provided in MM/DD/YY convention
        # Can be used to add a specific number of miles driven to a given day
        # Weekday starts on Monday at Midnight(Hour 0)
        # Drive Cycle is duplicated for weekend driving and weekday driving

        weekday                          = date_parse(date).weekday()
        if hour < 0 or hour >= 7*24:
            raise DataInputError("Input day exceeds defined matrix space for hour: %i" % hour)
            
        try:
            if weekday >= 5:
                # Weekend Driving
                for i in range(5,7):
                    driving_hour = 24*i+hour
                    self.drive_cycle[driving_hour] = miles
            else:
                for i in range(5):
                    driving_hour = 24*i+hour
                    self.drive_cycle[driving_hour] = miles            
        except:
            raise DataInputError("Input day exceeds defined matrix space for hour: %i on %s" % (hour, date))

    @classmethod
    def vehicle_list(cls):
        vehicle_list=[]
        for ref in cls._instances:
            obj = ref()
            if obj is None:
                cls._instances.remove(ref)
            else:
                vehicle_list.append(obj)
        return vehicle_list


def find_unique_trips(filename="vehicle_trips.csv"):
        df = pd.read_csv(filename)
        return float(df.index.max())

def iterative_solver(vehicle_limit, chargers, starting_index, trips="vehicle_trips.csv", export_file="output.csv", carbon_price=0, grid_file="electricity_pricing.csv"):
    end_veh, vehicles = Vehicle.load_drive_cycle(filename=trips, limit = vehicle_limit, starting_index = starting_index)
    # UGHH sooo slow to solve all vehicles simultaneously

    # Defining Electric Grid
    caiso = ElectricGrid(carbon_price = carbon_price)
    caiso.load_grid_rates(grid_file)

    # Defining Linear Program


    model = EVOptimizer(carbon_price = caiso.carbon_price)
    model.add_electric_grid(caiso)
    model.add_chargers(chargers)
    model.add_all_vehicles(vehicles)

    # Define State of Charge Constraints
    start_time = time.time()
    model.solve()
    print("Model took %f seconds to solve" % (time.time()-start_time))

    # Exporting Results
    print("Exporting charging results for %i trips..." % end_veh)
    model.get_charging_profile(export_file)
    return float(end_veh)

def parse_arguments():
    """
    Get a set of useful and changeable arguments from the commandline
    and the associated default values if no argument is passed
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', help='Carbon Price')
    parser.add_argument('-trips', help='csv file containing trips')
    parser.add_argument('-export', help='file to export results to')
    parser.add_argument('-grid', help='the grid csv file to import')
    parser.add_argument('-vehicles', help='Number of vehicles per convergence. Default is 25')
    args = parser.parse_args()
    
    if args.c:
        carbon_price = float(args.c)
    else:
        carbon_price = 0
    
    if args.grid:
        grid_file = args.grid
    else:
        grid_file = "electricity_pricing.csv"

    if args.trips:
        trip_filename = args.trips
    else:
        trip_filename   = "vehicle_trips.csv"
        
    if args.export:
        export_file = args.export
    else:
        export_file     = "default_results.csv"
    
    if args.vehicles:
        vehicle_limit = float(args.vehicles)
    else:
        vehicle_limit = 25
    
    return(carbon_price, trip_filename, export_file, vehicle_limit, grid_file)
    

if __name__ == "__main__":

    # Retrieve the values passed from the command line
    carbon_price, trip_filename, export_file, vehicle_limit, grid_file = parse_arguments()
     
    print("Program Running...")

    # Setting up Charging Levels
    level1          = ChargingStation(1.9, 0, 1)
    level2          = ChargingStation(6.8, .02, 2)
    level3          = ChargingStation(120, .45, 3)
    level3p         = ChargingStation(350, .65, "3p")
    slack_charger   = ChargingStation(9999, 1000, "slack")
    chargers        = ChargingStation.charger_list()

    # Setting up export and iteration
    unique_trips    = find_unique_trips(filename = trip_filename)
    #unique_trips     = 2023

    print("%i unique vehicle trips identified" % unique_trips)


    # Start with trip index
    index           = 0
    i               = 0

    print("loading vehicle drive cycle...")
    start_time = time.time()
    while index < unique_trips:
        i = i +1

        index = iterative_solver(vehicle_limit, chargers = chargers, starting_index = index,
                                    trips=trip_filename, export_file=export_file, 
                                    carbon_price=carbon_price, grid_file = grid_file)
        pct_complete = index/unique_trips*100
        print ("Modeling charging behavior for %i vehicles... (%.2f pct complete)" % (i*vehicle_limit ,pct_complete) )

    print("Complete model run in %f seconds" % (time.time()- start_time))



    # print("Energy Demand")
    # model.get_driving_profile()
