# engine_interp_gasturb.py
# 
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020 
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by: 
#   Murilo Augusto Gallani, murilo.gallani@embraer.com.br, Embraer S.A.
# 
# Created:  Feb 2020, M. Gallani
# Modified: 


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import SUAVE

from SUAVE.Core import Units, Data
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm

from scipy import interpolate, interp
import os



## @ingroup Methods-Propulsion
def engine_interp_gasturb(engine, conditions): 
    """ The gas turbine output shaft power, fuel flow and residual thrust
        Based on interpolation of data from GasTurb

        Inputs:
            throttle
            Engine:
                sea-level power (datum)
                data tables (shp,ff,resfn) for each rating
                    1: APR
                    2: NTO
                    3: MCL
                    4: MCR
                    5: FIDLE
                    6: MCT
                bucket curve (non-dimensional)
                power scaling factor
                rating
            Freestream conditions:
                altitude
                true airspeed
                delta isa
                
        Outputs:
            Brake power (or Shaft power)
            Power (brake) specific fuel consumption
            Fuel flow
            Residual thrust

        """
    
    
    # Unpack
    Hpkft            = conditions.freestream.altitude / Units['kft']
    KTAS             = conditions.freestream.velocity / Units.kts
    delta_isa        = conditions.freestream.delta_ISA
    JREGM            = engine.gas_turbine_rating
    DPDEC            = engine.power_scaling_factor
    bucket           = engine.bucket
    power_extraction = engine.power_extraction

    throttle  = engine.inputs.throttle
    
    if engine.tables == None:
        engine.tables = load_data(engine.data_file)

    # interpolating available power from data tables
    x = engine.tables.ratings[JREGM].delta_isa
    y = engine.tables.ratings[JREGM].altitude
    z = np.array(engine.tables.ratings[JREGM].airspeed)
    dataHP = engine.tables.ratings[JREGM].shp
    dataFF = engine.tables.ratings[JREGM].ff
    dataRT = engine.tables.ratings[JREGM].resfn

    f1 = interpolate.RegularGridInterpolator((x,y,z), dataHP, bounds_error = False, fill_value = None)
    f2 = interpolate.RegularGridInterpolator((x,y,z), dataFF, bounds_error = False, fill_value = None)
    f3 = interpolate.RegularGridInterpolator((x,y,z), dataRT, bounds_error = False, fill_value = None)

    Pavailable = np.zeros_like(Hpkft)
    fuel_flow_rate = np.zeros_like(Hpkft)
    residual_thrust = np.zeros_like(Hpkft)
    BSFC0 = np.zeros_like(Hpkft)

    Hpkft[Hpkft < 0.0] = 0.0
    for idx,hp in enumerate(Hpkft):
        Pavailable[idx]      = f1([delta_isa[:,0][idx],Hpkft[:,0][idx],KTAS[:,0][idx]])
        fuel_flow_rate[idx]  = f2([delta_isa[:,0][idx],Hpkft[:,0][idx],KTAS[:,0][idx]])
        residual_thrust[idx] = f3([delta_isa[:,0][idx],Hpkft[:,0][idx],KTAS[:,0][idx]])
        
    Pavailable[Pavailable < 0.0] = 0.1
    residual_thrust[residual_thrust < 0.0] = 0.1
    
    Pavailable = Pavailable * DPDEC
    fuel_flow_rate = fuel_flow_rate * DPDEC
    residual_thrust = residual_thrust * DPDEC
    
    BSFC0 = fuel_flow_rate / Pavailable
    output_power = Pavailable * throttle

    # applying bucket curve
    a = np.array([0.])
    RSFC = interp(throttle, bucket.RMTR, bucket.RSFC)
    BSFC1 = BSFC0 * RSFC #* 1.61#1.72 1.54
    fuel_flow_rate   = np.fmax(output_power*BSFC1,a)
    
    # store to outputs
    outputs = Data()
    
    # -------- for the warnings function ----------------------
    outputs.warnings = Data
    
    outputs.warnings.delta_isa_outofbound = False  
    outputs.warnings.Hpkft_outofbound = False
    outputs.warnings.KTAS_outofbound = False
    outputs.warnings.power_unavailable = False
    
    if np.any(delta_isa > x[-1])  or np.any(delta_isa < x[0]):
        outputs.warnings.delta_isa_outofbound = True
    if np.any(Hpkft > y[-1])  or np.any(Hpkft < y[0]):
        outputs.warnings.Hpkft_outofbound = True
    if np.any(KTAS > z[-1])  or np.any(KTAS < z[0]):
        outputs.warnings.KTAS_outofbound = True
    if np.any(output_power > Pavailable):
        outputs.warnings.power_unavailable = True
    # ---------------------------------------------------------

    outputs.available_power                 = Pavailable * Units['hp'] 
    outputs.power                           = output_power * Units['hp']  # [Watts]
    outputs.specific_fuel_consumption       = BSFC1 * Units['lb/hp/h']    # [lb/h/shp]
    outputs.fuel_flow                       = fuel_flow_rate * Units['lb/h']
    
    outputs.residual_thrust                 = residual_thrust * Units['lbf']   # [N]
    outputs.RSFC                            = RSFC # [-]

    if any(outputs.power == 0.0):
        print('erro')

    return outputs

def load_data(file_name):
    
    file_to_read = os.path.join(SUAVE.__path__[0], 'Data_Files\\Gas_Turbine\\', file_name)
    
    f = open(file_to_read,'r')
    data = f.readlines()
    f.close()

    tables = Data()
    tables.ratings = dict()

    for idx,line in enumerate(data):
        if line.find('JREGM') >= 0:
            JREGM = float(line[8])
            key = line[10:13]
            tables.ratings[key] = Data()
            nalt = int(data[idx+1][7])
            nisa = int(data[idx+2][7])
            ntas = int(data[idx+3][7])
            tables.ratings[key].shp = np.zeros( (nisa,nalt,ntas) )
            tables.ratings[key].ff = np.zeros( (nisa,nalt,ntas) )
            tables.ratings[key].resfn = np.zeros( (nisa,nalt,ntas) )

        if line.find('ALTITUDE') >= 0:
            tables.ratings[key].altitude = list(map(float,data[idx+1].split()))
        if line.find('STANDARD') >= 0:
            tables.ratings[key].delta_isa = list(map(float,data[idx+1].split()))
        if line.find('AIRSPEED') >= 0:
            tables.ratings[key].airspeed = list(map(float,data[idx+1].split()))
        if line.find('SHP') >= 0:
            ndlim = data[idx+1].find('!')
            count = 0
            for i in range(nalt):
                for j in range(nisa):
                    count += 1
                    tables.ratings[key].shp[j,i,:] = list(map(float,data[idx+count][1:ndlim].split()))
        if line.find('DFC') >= 0:
            ndlim = data[idx+1].find('!')
            count = 0
            for i in range(nalt):
                for j in range(nisa):
                    count += 1
                    tables.ratings[key].ff[j,i,:] = list(map(float,data[idx+count][1:ndlim].split()))
        if line.find('DRES') >= 0:
            ndlim = data[idx+1].find('!')
            count = 0
            for i in range(nalt):
                for j in range(nisa):
                    count += 1
                    tables.ratings[key].resfn[j,i,:] = list(map(float,data[idx+count][1:ndlim].split()))

    
    return tables

    


