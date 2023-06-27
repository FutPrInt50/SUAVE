## @ingroup Methods-Gemometry-Correlations-Common
# wing_geometry.py
#
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Dominik Eisenhut, eisenhut@ifb.uni-stuttgart.de, University of Stuttgart
#
#
# Created:  Aug 2021, D. Eisenhut
# Modified: Oct 2021, D. Eisenhut



import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_segmented_planform

def create_tapered_wing(wing, aspect_ratio, area, span_limit=None, taper=None):
    # taper only needs specification if single tapered wing without segments, otherwise it can be neglected
    import numpy as np
    from math import sqrt

    span = sqrt(aspect_ratio * area)

    if span_limit is not None and span > span_limit:
        span = span_limit
        aspect_ratio = span ** 2 / area

    if len(wing.Segments) < 2:
        # single tapered wing
        if taper is not None:
            root_chord = (2 * area)/(span * (1+taper))
            tip_chord = taper * root_chord
        else:
            raise AttributeError('Taper must be specified for a single tapered wing without segments!')
    else:
        num_sections = len(wing.Segments)
        local_span = np.array([])
        root_chord_percent = np.array([])
        tip_chord_percent = np.array([])
        for i in range(num_sections-1):
            local_span = np.append([local_span], [span/2 * (wing.Segments[i+1].percent_span_location -
                                                            wing.Segments[i].percent_span_location)])
            root_chord_percent = np.append([root_chord_percent], wing.Segments[i].root_chord_percent)
            tip_chord_percent = np.append([tip_chord_percent], wing.Segments[i+1].root_chord_percent)

        root_chord = area / 4 / sum(0.5 * local_span * (root_chord_percent + tip_chord_percent)) * (1 + wing.symmetric)
        taper = tip_chord_percent[-1]
        tip_chord = taper * root_chord

        for i in range(num_sections-1):
            wing.Segments[i].areas.reference = local_span[i] * root_chord * (root_chord_percent[i] + tip_chord_percent[i])

    wing.aspect_ratio = aspect_ratio
    wing.areas.reference = area
    wing.spans.projected = span
    wing.taper = taper
    wing.chords.root = root_chord
    wing.chords.tip = tip_chord

    return wing



########################################################################################################################
# Implementation
########################################################################################################################

if __name__ == '__main__':
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'tapered_main_wing'
    wing.symmetric = True

    wing.origin = [[13.61, 0, -0.93]]

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'Root'
    segment.percent_span_location = 0.0
    segment.twist = 4. * Units.deg
    segment.root_chord_percent = 1.
    segment.thickness_to_chord = 0.1
    segment.dihedral_outboard = 2.5 * Units.degrees
    segment.sweeps.quarter_chord = 0 * Units.degrees
    segment.thickness_to_chord = .1
    wing.append_segment(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'mid_section1'
    segment.percent_span_location = 0.1
    segment.twist = 0 * Units.deg
    segment.root_chord_percent = 1
    segment.thickness_to_chord = 0.1
    segment.dihedral_outboard = 5.5 * Units.degrees
    segment.sweeps.quarter_chord = 28.225 * Units.degrees
    segment.thickness_to_chord = .1
    wing.append_segment(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'mid_section2'
    segment.percent_span_location = 0.4
    segment.twist = 0.00258 * Units.deg
    segment.root_chord_percent = 0.6
    segment.thickness_to_chord = 0.1
    segment.dihedral_outboard = 5.5 * Units.degrees
    segment.sweeps.quarter_chord = 25. * Units.degrees
    segment.thickness_to_chord = .1
    wing.append_segment(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'mid_section3'
    segment.percent_span_location = 0.8
    segment.twist = 0.00258 * Units.deg
    segment.root_chord_percent = 0.3
    segment.thickness_to_chord = 0.1
    segment.dihedral_outboard = 5.5 * Units.degrees
    segment.sweeps.quarter_chord = 56.75 * Units.degrees
    segment.thickness_to_chord = .1
    wing.append_segment(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'Tip'
    segment.percent_span_location = 1.
    segment.twist = 0. * Units.degrees
    segment.root_chord_percent = 0.1
    segment.thickness_to_chord = 0.1
    segment.dihedral_outboard = 0.
    segment.sweeps.quarter_chord = 0.
    segment.thickness_to_chord = .1
    wing.append_segment(segment)

    del segment

    create_tapered_wing(wing, 11, 100*Units['m**2'], span_limit=36*Units.meter)
    wing_segmented_planform(wing)

    print('abc')
