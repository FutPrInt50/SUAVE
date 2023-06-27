## @ingroup Methods-Figures_of_Merit
# direct_operating_costs.py
#
# Created:  Apr 2022, J. Frank


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
import copy
from SUAVE.Core import Units
from SUAVE.Core import Data
from SUAVE.Methods.Figures_of_Merit.global_warming_potential import global_warming_potential
from SUAVE.Methods.Figures_of_Merit.Supporting_Functions.material_costs import material_costs
from scipy.interpolate import interp1d

# ----------------------------------------------------------------------
#  Method
# ----------------------------------------------------------------------
## @ingroup Methods-Figures_of_Merit
def direct_operating_costs(results, vehicle, noise, settings, LH2 = False,hybrid = False, doc_landing_method = 'Blase'):
    """ This method computes the emissions of carbon dioxide
    """

    doc = Data()
    gwp = global_warming_potential(results, vehicle, settings, LH2)
    m_costs = material_costs(vehicle,hybrid)

    # STANDARD VALUES

    noise_fudge = settings.noise_fudge
    dollar_to_euro = settings.dollar_to_euro
    price_A1 = settings.price_A1
    price_LH2 = settings.price_LH2
    price_elec = settings.price_elec
    n_crew_complements = settings.n_crew_complements
    salary_flight_attendant = settings.salary_flight_attendant
    salary_cockpit_crew = settings.salary_cockpit_crew
    maintenance_labor_per_hour = settings.maintenance_labor_per_hour
    k_af = settings.k_af
    k_gt = settings.k_gt
    k_p = settings.k_p
    k_fc = settings.k_fc
    k_em = settings.k_em
    k_pms = settings.k_pms
    f_ins = settings.f_ins
    k_landing = settings.k_landing
    k_nav = settings.k_nav
    f_growth = settings.f_growth
    price_co2 = settings.price_co2
    consumer_price_index = settings.consumer_price_index
    IR = settings.IR
    DP_ac = settings.DP_ac
    N_bat_cycl = settings.N_bat_cycl
    N_fc_cycl = settings.N_fc_cycl
    f_rv_ac = settings.f_rv_ac
    f_rv_bat = settings.f_rv_bat
    f_rv_fc = settings.f_rv_fc

    # UNPACK VALUES

    segments = [key for key in results.segments.keys() if ('reserve' not in key) and ('hold' not in key)]
    first_segment = segments[0]
    last_segment = segments[-1]

    block_time = (results.segments[last_segment].conditions.frames.inertial.time[-1] - \
                  results.segments[first_segment].conditions.frames.inertial.time[0]) / Units.hours
    block_time_supplement = 1.83 #Quelle Scholz 14.4 (aus AEA 1989a für Kurz- und Mittelstrecke)

    flight_cycles_per_year = (8760 - 2749)/(block_time + block_time_supplement) # TODO

    if LH2 == True:
        FB_LH2 = results.segments[first_segment].conditions.weights.total_mass[0] - \
                 results.segments[last_segment].conditions.weights.total_mass[-1]
        FB_A1 = 0.
    else:
        FB_A1 = results.segments[first_segment].conditions.weights.total_mass[0] - \
                results.segments[last_segment].conditions.weights.total_mass[-1]
        FB_LH2 = 0.
    if 'battery' in vehicle.propulsors.network.keys():
        E_bat = (results.segments[first_segment].conditions.propulsion.battery_energy[0] - \
                results.segments[last_segment].conditions.propulsion.battery_energy[-1]) / Units.kWh
        structural_weight = vehicle.mass_properties.operating_empty - vehicle.mass_properties.propulsion - vehicle.propulsors.network.battery.mass_properties.mass
    else:
        E_bat = 0.
        structural_weight = vehicle.mass_properties.operating_empty - vehicle.mass_properties.propulsion

    n_passengers = vehicle.passengers
    n_flight_attendant = np.ceil(n_passengers / 50)

    mtom = vehicle.mass_properties.max_takeoff
    total_range = results.segments[last_segment].conditions.frames.inertial.position_vector[-1][0] / Units.km

    span = vehicle.wings.main_wing.spans.projected
    length = vehicle.fuselages['fuselage'].lengths.total

    c_af = m_costs.airframe
    c_gt = m_costs.gas_turbine
    c_p = m_costs.propeller
    c_lh2 = m_costs.hydrogen_tank
    c_em = m_costs.electrical_motors
    c_pms = m_costs.power_management_system
    c_bat = m_costs.battery
    c_fc = m_costs.fuel_cell

    weight_payload = results.segments[0].analyses.weights.mass_properties.payload
    
    P_total_max = vehicle.propulsors.network.sea_level_power * vehicle.propulsors.network.number_of_engines / Units.kW
    if 'emotor' in vehicle.propulsors.network.keys():
        P_total_max += vehicle.propulsors.network.emotor.rated_power * vehicle.propulsors.network.number_of_engines / Units.kW
    if 'emotorWTP' in vehicle.propulsors.network.keys():
        P_total_max += vehicle.propulsors.network.emotorWTP.rated_power * 2


    # CALCULATE COSTS
    # ENERGY
    doc.energy = dollar_to_euro * flight_cycles_per_year * (FB_A1 * price_A1 + FB_LH2 * price_LH2 + E_bat * price_elec)
    # CREW
    doc.crew = n_crew_complements * (salary_flight_attendant * n_flight_attendant + salary_cockpit_crew)

    # MAINTENANCE
    # AIRFRAME MAINTENANCE
    # weight_lh2_tank = 0
    doc.af_mat = structural_weight/1000 * (0.21 * block_time + 13.7) + 57.5 # Entspricht Strohmayer Skript, nur verwendet Strohmayer nicht structural weight sondern OME
    # AIRFRAME MAINTENANCE PERSONNEL
    # SIMON BLASE METHODE
    doc.af_per = maintenance_labor_per_hour * ((structural_weight * 10**-4 + 0.5) * block_time + (structural_weight * 10**-4 + 0.25))
    # STROHMAYER METHODE
    # B = 0
    # doc.af_per = maintenance_labor_per_hour * (1+B)*((0.655+0.01*structural_weight*block_time)+0.254+0.01*structural_weight)
    # ENGINE MAINTENANCE
    doc.eng = 1.5242 * 10**-3 * 0.64545 * P_total_max / 54.121 + 30.5 * block_time + 10.6
    # Strohmayer Methode benötigt hier einen statischen Schub bei H=0, was wir nicht als Wert haben
    # GENERAL TECHNOLOGY COSTS
    doc.tec = 5000 * (span * length)**0.75
    # SUM UP TO GET TOTAL MAINTENANCE COSTS
    doc.maintenance = (doc.af_mat + doc.af_per + doc.eng) * flight_cycles_per_year + doc.tec

    if settings.DOC_or_COC == 'COC' and hybrid == True:
        doc.maintenance += c_bat * 12 / vehicle.propulsors.network.battery.durability_target_in_months

    # CAPITAL COSTS
    # Thorbeck Methode:
    # k_ome = 1150
    # k_eng = 2500
    # doc.capital_aircraft = (k_ome*(structural_weight-vehicle.propulsors.network.total_weight*vehicle.propulsors.network.number_of_engines)+k_eng*vehicle.propulsors.network.total_weight*vehicle.propulsors.network.number_of_engines)*(a_ac+f_ins)
    # DP_bat = N_bat_cycl / flight_cycles_per_year

    DP_fc = N_fc_cycl / flight_cycles_per_year

    a_ac = IR * (1 - f_rv_ac * (1 / (1 + IR)) ** DP_ac) / (1 - (1 / (1 + IR)) ** DP_ac) # 0.077218
    a_fc = IR * (1 - f_rv_fc * (1 / (1 + IR)) ** DP_fc) / (1 - (1 / (1 + IR)) ** DP_fc)

    doc.capital_aircraft = (c_af * (1 + k_af) + c_gt * (1 + k_gt) + c_p * (1 + k_p) + c_lh2 + c_em * (1 + k_em) + c_pms * (1 + k_pms)) * (a_ac + f_ins)
    doc.capital_fuel_cell = c_fc * (1 + k_fc) * (a_fc + f_ins)

    doc.capital = (doc.capital_aircraft + doc.capital_fuel_cell) * dollar_to_euro

    if hybrid == True:
        DP_bat = vehicle.propulsors.network.battery.durability_target_in_months / 12
        a_bat = IR * (1 - f_rv_bat * (1 / (1 + IR)) ** DP_bat) / (1 - (1 / (1 + IR)) ** DP_bat)
        doc.capital_battery = c_bat * (a_bat + f_ins)
        doc.capital += doc.capital_battery

    if settings.DOC_or_COC == 'COC':
        doc.capital = 0.
    # FEES

    dB_A_data = [0., 77., 78.6, 80.2, 81.8, 83.4, 85., 86.6, 88.2, 89.8, 91.4, 93.]
    charge_data = [25., 30., 60., 90., 120., 150., 180., 300., 500., 700., 900., 1400.]

    p_noise = interp1d(dB_A_data, charge_data, kind='previous')

    if doc_landing_method == 'Blase':
        doc.landing = (9.5 * 10 ** -3 - 1 * 10 ** -3 * np.log(block_time)) * mtom + p_noise(noise.approach.total + noise_fudge)
    # STROHMAYER (THORBECK)
    else:
        doc.landing = k_landing * mtom # ERSATZFUNKTION OHNE NOISE

    # GROUND OPERATING COSTS
    doc.ground = 0.11 * weight_payload - 5 * 10**-7 * weight_payload**2

    # NAVIGATION COSTS
    # SIMON BLASE VERSION
    doc.nav = k_nav * total_range/1000 * np.sqrt(mtom/50000)
    # STROHMAYER VERSION
    # k_nav = 100 #intra-EU Wert
    # total_range = results.segments[last_segment].conditions.frames.inertial.position_vector[-1][0] / Units.km
    # mtom = vehicle.mass_properties.max_takeoff /1000
    #
    # doc.nav = k_nav * total_range / 100 * np.sqrt(mtom / 50)

    # DOC EMISSIONS
    doc.co2 = (gwp.co2 + gwp.nox) * total_range * n_passengers * (1 + f_growth) * price_co2

    # SUM UP TO GET TOTAL FEES
    doc.fees = (doc.landing + doc.ground + doc.nav + doc.co2) * flight_cycles_per_year

    # SUM UP TO GET TOTAL DIRECT OPERATING COSTS
    #doc.total = (doc.energy + doc.crew + doc.maintenance + doc.capital + doc.fees) * consumer_price_index / (flight_cycles_per_year * n_passengers * total_range /100)
    doc.total = (doc.energy + doc.crew + doc.maintenance + doc.capital + doc.fees) * consumer_price_index / (flight_cycles_per_year * weight_payload / 100 * total_range / 100) # DOC in €/(100km * 100kg_PL)


    # doc.total = euro/pax/100km
    return doc