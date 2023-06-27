## @ingroup Methods-Figures_of_Merit
# Fidelity_Zero.py
#
# Created:  J. Frank, June 2022

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUAVE imports

import SUAVE

from SUAVE.Core import Data, Units

# local imports


# import SUAVE methods
from SUAVE.Analyses import Analysis
# package imports
import numpy as np
import pylab as plt


# ----------------------------------------------------------------------
#  Class
# ----------------------------------------------------------------------

## @ingroup Methods-Figures_of_Merit
class Figure_of_Merit(Analysis):

    def __defaults__(self):
        """This sets the default values and methods for the analysis.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        N/A
        """
        # Initialize quantities

        self.configs = Data()
        self.configs.base = Data()
        self.results = Data()

        settings = self.settings

        # SETTINGS GENERAL
        settings.scaling_minus1_to_one = False # = True bedeutet: Figure of Merit zwischen -1 und 1, = False bedeutet Figure of Merit zwischen 0 und 1
        settings.minmax_co2 = [0, 0.3] * Units.kg # Minimal- und Maximalwerte für die Figure of Merit Normierung (Minimalwert bedeutet SFOM=1.0, Maximalwert bedeutet SFOM=0.0)
        settings.minmax_nox = [0, 0.3] * Units.kg
        settings.minmax_doc = [0, 20]
        settings.minmax_dev = [0, 10 ** 9]
        settings.minmax_cer = [0, 10 ** 8]
        settings.minmax_prod = [0, 10 ** 9]
        settings.minmax_noise_sone = [0, 50]
        settings.weighting_factors_co2_nox_noise = [1.05, 1.05, 0.9] # Gewichtungsfaktorsets der einzelnen Single Figures of Merits
        settings.weighting_factors_dev_cer_prod = [1, 1, 1]
        settings.weighting_factors_EI_ADI_HEAI = [1.15, 0.9, 0.95]
        settings.weighting_factors_sideline_flyover_approach = [1., 1., 1.]

        settings.weighting_factors_EI_ADI_HEAI_conv = [1.15, 0.9, 0.95]
        settings.weighting_factors_EI_ADI_HEAI_env = [1.15, 0.9, 0.95]

        settings.trl = Data() # Technology Readiness Levels der verwendeten Technologien werden definiert durch beispielsweise 'settings.trl.wing_tip_propulsion = 6.'
        settings.hybrid = False # = True für Hybridflugzeuge
        settings.lh2 = False # = True für Flugzeuge die Flüssigwasserstoff als Kraftstoff verwenden
        settings.print_flag = True # = True bedeutet Ausgabe detaillierter Teilergebnisse und SFOMs in der Python-Konsole
        settings.plot_flag = True # = True bedeutet Ausgabe eines Direct Operating Costs Plots
        settings.vehicle_tag = 'ATR42' # Lediglich relevant für den Titel des Direct Operating Cost Plot

        # PRINT NEW MINMAX VALUES
        settings.print_new_minmax_values = False  # = True bedeutet Ausgabe neuer Minmax-Werte in Referenz auf das aktuelle Flugzeug
        settings.relative_deviation = 1.  # Wird verwendet, wenn neue MinMax-Werte berechnet werden sollen und beschreibt um wieviel Prozent die neuen Werte von der Referenz abweichen

        # SETTINGS AIRLINE DESIRABILITY INDEX (DIRECT OPERATING COSTS)
        settings.DOC_or_COC = 'COC'
        settings.noise_fudge = -11.4 # Fudge Factor zur Berücksichtigung unterschiedlicher Messmethoden für Approach und die Airport Gebühren (entspricht Differenz aus Messungen einer ATR 42)
        settings.dollar_to_euro = 0.98 # Wechselkurs Dollar -> Euro
        settings.price_A1 = 1  # 1 according BLASE, 0.5845 is for 2015 # Preis eines Kilogramms Kerosin
        settings.price_LH2 = 4 # Preis eines Kilogramms Flüssigwasserstoff
        settings.price_elec = 0.114 # Preis einer Kilowattstunde elektrischer Energie
        settings.n_crew_complements = 5  # nach unterschiedlichen Quellen auf 5 fixiert
        settings.salary_flight_attendant = 60000  # According SB # Jahreslohn eines Flugbegleiters
        settings.salary_cockpit_crew = 2 * 80000  # 300000 Value from Strohmayer, 2*80000 according SB # Jahreslohn der Cockpit Crew (hier zwei Piloten)
        settings.maintenance_labor_per_hour = 50  # $/h # Stundenlohn eines Wartungsarbeiters
        settings.k_af = 0.1 # Ersatzteilfaktor Airframe
        settings.k_gt = 0.3 # Ersatzteilfaktor Gasturbine
        settings.k_p = 0.1 # Ersatzteilfaktor Propeller
        settings.k_fc = 0.1 # Ersatzteilfaktor Fuel Cell
        settings.k_em = 0.2 # Ersatzteilfaktor Electric Motors
        settings.k_pms = 0.2 # Ersatzteilfaktor Power Management System
        settings.f_ins = 5 * 10 ** -3 #
        settings.k_landing = 0.01 # Nur relevant wenn NICHT doc_landing_method nach Blase verwendet wird
        settings.k_nav = 850 # Kostenkonstante der Navigationsgebühren
        settings.f_growth = 0.043 #
        settings.price_co2 = 0.1 * settings.dollar_to_euro  # 0.1 accoring SB for 2040
        settings.consumer_price_index = 1.6  # 1 for 2015, 1.6 for 2040 according SB
        settings.IR = 0.05
        settings.DP_ac = 20
        settings.N_bat_cycl = 1500
        settings.N_fc_cycl = 15037
        settings.f_rv_ac = 0.1
        settings.f_rv_bat = 0.4
        settings.f_rv_fc = 0.3 #
        settings.doc_landing_method = 'Blase' # Anderer String bedeutet Strohmayer Methode wird verwendet (Kosten nur in Abhängigkeit von MTOM)

        # SETTINGS AIRCRAFT INTRODCUTION INDEX
        settings.f_comp = 0.3  # Assumption 30% Composite Materials
        settings.q = 20  # number of aircraft to be produced in five years. the number 67 results from statistics for ATR42, 20 is used by SB
        settings.n_FTA = 3  # 2-6, according Raymer p.697, 3 used from SB
        settings.Q_total = 4 * settings.q  # Personal Assumption, 20 years of production time with a production rate of q aircraft per 5 years
        settings.percent_learning_curve = 80  # in percent according SB p.36

        # All according Raymer p.698  [$/h]
        settings.labor_rate_tool = 118 # Stundenlohn Werkzeugbau
        settings.labor_rate_mfg = 98  # Stundenlohn generelle Fertigung
        settings.labor_rate_qc = 108 # Stundenlohn Quality Control
        settings.labor_rate_engr = 115 # Stundenlohn Engineering

        # All according Finger Table 1 [-]
        # Korrekturfaktoren nach Finger für die Kostenabschätzung für Flugzeuge mit hohem Faserverbundanteil. Kostenaufschläge gelten für theoretische 100% FVK-Anteil, und werden mit dem tatsächlichen Anteil multipliziert
        settings.F_comp_engr = 2
        settings.F_comp_dev = 1.5
        settings.F_comp_tool = 2
        settings.F_comp_mfg = 1.25
        settings.F_comp_qc = 1.5
        # Korrekturfaktoren nach Finger für die Kostenabschätzungen für hybrid-elektrische Flugzeuge (HEA)
        settings.F_HEA_engr = 1.4  # 1.33-1.66
        settings.F_HEA_dev = 1.05
        settings.F_HEA_tool = 1.1
        settings.F_HEA_mfg = 1.05
        settings.F_HEA_qc = 1.5
        settings.F_HEA_ft = 1.5

        # noise
        settings.number_of_microfones_sideline = 100 # Anzahl der Mikrofone des Sideline-Mikronarrays (Keine Vorgabe in den Quallen gefunden, Ergebnis ist auch wenig sensitiv darauf)
        settings.pos_fist_microfone_sideline = 500 * Units.meter # Position des vordersten Mikrofon des Sideline-Mikrofonarrays (Anpassung verändert das Ergebnis und erfordert einen neuen Korrekturfaktor)
        settings.pos_last_microfone_sideline = 10000 * Units.meter # Position des hintersten Mikrofon des Sideline-Mikrofonarrays (Maximale Lautstärke wird weit vorne gemessen, deshalb verändert eine Anpassung das Ergebnis nicht)
        settings.aircraft_take_off_distance = 1000. * Units.meter # Startweglänge des Flugzeuges, beeinflusst Positionierung des Flyover-Mikrofons
        settings.correction_sideline = -4.37 # Korrekturfaktor basierend auf Literaturwerten der ATR 42-600 aus Janes All the Words Aircraft
        settings.correction_flyover = -9.08 # Korrekturfaktor basierend auf Literaturwerten der ATR 42-600 aus Janes All the Words Aircraft
        settings.correction_approach = +0.9 # Korrekturfaktor basierend auf Literaturwerten der ATR 42-600 aus Janes All the Words Aircraft

        # Composite Parts
        settings.main_wing_comp = False # True wenn Tragfläche aus FVK hergestellt wird
        settings.fuselage_comp = False # True wenn Rumpf aus FVK hergestellt wird
        settings.empennage_comp = False # True wenn Leitwerke aus FVK hergestellt werden
        return

    def calculate(self, results, configs):
        # unpack
        settings = self.settings
        vehicle = configs.base

        # ANALYSES
        gwp = SUAVE.Methods.Figures_of_Merit.global_warming_potential(results,vehicle,settings,LH2=settings.lh2)
        noise = SUAVE.Methods.Figures_of_Merit.noise_certification(results, configs, settings, settings.weighting_factors_sideline_flyover_approach)
        doc = SUAVE.Methods.Figures_of_Merit.direct_operating_costs(results, vehicle, noise, settings, LH2=settings.lh2, hybrid=settings.hybrid, doc_landing_method=settings.doc_landing_method)
        heai = SUAVE.Methods.Figures_of_Merit.hybrid_electric_aircraft_introduction(results, vehicle, settings, hybrid=settings.hybrid)
        trl_total = SUAVE.Methods.Figures_of_Merit.technology_readyness_level(settings.trl)

        # CALCULATION OF SINGLE FIGURES OF MERIT
        sfom_nox = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.minmax(gwp.nox, settings.minmax_nox)
        sfom_co2 = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.minmax(gwp.co2, settings.minmax_co2)
        sfom_noise = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.minmax(noise.total_sone, settings.minmax_noise_sone)
        sfom_doc = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.minmax(doc.total, settings.minmax_doc)
        sfom_dev = (SUAVE.Methods.Figures_of_Merit.Supporting_Functions.minmax(heai.c_dev, settings.minmax_dev) + trl_total) / 2
        sfom_cer = (SUAVE.Methods.Figures_of_Merit.Supporting_Functions.minmax(heai.c_cer, settings.minmax_cer) + trl_total) / 2
        sfom_prod = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.minmax(heai.c_prod, settings.minmax_prod)

        # NEW SCALING: -1 < SFOM < 1:
        if settings.scaling_minus1_to_one == True:
            sfom_nox    = -1. + 2 * sfom_nox
            sfom_co2    = -1. + 2 * sfom_co2
            sfom_noise  = -1. + 2 * sfom_noise
            sfom_doc    = -1. + 2 * sfom_doc
            sfom_dev    = -1. + 2 * sfom_dev
            sfom_cer    = -1. + 2 * sfom_cer
            sfom_prod   = -1. + 2 * sfom_prod

        # CALCULATION OF COMBINED FIGURES OF MERIT
        fom_ei = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.arithmetic_mean([sfom_co2[0], sfom_nox[0], sfom_noise], settings.weighting_factors_co2_nox_noise)
        fom_adi = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.arithmetic_mean(sfom_doc)
        fom_heai = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.arithmetic_mean([sfom_dev[0], sfom_cer[0], sfom_prod[0]], settings.weighting_factors_dev_cer_prod)
        fom = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.arithmetic_mean([fom_ei, fom_adi, fom_heai],
                                                                                       settings.weighting_factors_EI_ADI_HEAI)
        fom_conv = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.arithmetic_mean([fom_ei, fom_adi, fom_heai], settings.weighting_factors_EI_ADI_HEAI_conv)
        fom_env = SUAVE.Methods.Figures_of_Merit.Supporting_Functions.arithmetic_mean([fom_ei, fom_adi, fom_heai], settings.weighting_factors_EI_ADI_HEAI_env)

        self.sfom_nox = sfom_nox
        self.sfom_co2 = sfom_co2
        self.sfom_noise = sfom_noise
        self.sfom_doc = sfom_doc
        self.sfom_dev = sfom_dev
        self.sfom_cer = sfom_cer
        self.sfom_prod = sfom_prod
        self.fom_ei = fom_ei
        self.fom_adi = fom_adi
        self.fom_heai = fom_heai
        self.fom = fom
        self.fom_conv = fom_conv
        self.fom_env = fom_env

        # OUTPUTS
        if settings.print_flag == True:
            print('%s: %.2f €/100kg_PL/100km' % (settings.DOC_or_COC, doc.total))
            print('\n')
            print('C_Tool: %.2e €' % heai.c_tool)
            print('C_MFG: %.2e €' % heai.c_mfg)
            print('C_QC: %.2e €' % heai.c_qc)
            print('C_AF: %.2e €' % heai.m_costs.airframe)
            print('C_GT: %.2e €' % heai.m_costs.gas_turbine)
            print('C_P: %.2e €' % heai.m_costs.propeller)
            print('C_Prod: %.2e €' % heai.c_prod)
            print('\n')
            print('C_DEV: %.2e €' % heai.c_dev)
            print('C_CER: %.2e €' % heai.c_cer)
            print('C_PROD: %.2e €' % heai.c_prod)
            print('TRL_TOTAL: %.5f' % trl_total)
            print('\n')
            print('SFOM_CO2: %.5f' % sfom_co2)
            print('SFOM_NOX: %.5f' % sfom_nox)
            print('SFOM_NOISE: %.5f' % sfom_noise)
            print('SFOM_DOC: %.5f' % sfom_doc)
            print('SFOM_DEV: %.5f' % sfom_dev)
            print('SFOM_CER: %.5f' % sfom_cer)
            print('SFOM_PROD: %.5f' % sfom_prod)
            print('\n')
            print('FOM_EI: %.5f' % fom_ei)
            print('FOM_ADI: %.5f' % fom_adi)
            print('FOM_HEAI: %.5f' % fom_heai)
            print('\n')
            print('FOM: %.5f' % fom)
            print('FOM_conv: %.5f' % fom_conv)
            print('FOM_env: %.5f' % fom_env)

        # DIRECT OPERATING COSTS PLOT
        if settings.plot_flag == True:
            labels = 'energy', 'crew', 'maintenance', 'capital', 'fees'
            sizes = [doc.energy, doc.crew, doc.maintenance, doc.capital, doc.fees]
            fig1, ax1 = plt.subplots()
            ax1.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=90)
            ax1.axis('equal')
            ax1.set(title='Direct Operating Costs ' +  settings.vehicle_tag)
            plt.show()

        # NEW MINMAX VALUES
        if settings.print_new_minmax_values == True:
            SUAVE.Methods.Figures_of_Merit.Supporting_Functions.print_new_minmax(gwp, noise.total_sone, doc.total, heai, trl_total, relative_deviation=settings.relative_deviation, aircraft_name=settings.vehicle_tag)

        return fom