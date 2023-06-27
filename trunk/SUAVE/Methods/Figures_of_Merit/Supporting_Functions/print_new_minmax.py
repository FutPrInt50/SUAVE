import time
def print_new_minmax(gwp, noise_total, doc_total, heai, trl_total, relative_deviation=1, aircraft_name='Aircraft'):
    NOX = gwp.nox[0]
    CO2 = gwp.co2[0]
    NOISE = noise_total
    DOC = doc_total[0]
    DEV = heai.c_dev[0]
    CER = heai.c_cer[0]
    PROD = heai.c_prod[0]
    TRL = trl_total
    D = relative_deviation

    NOX_min = (1 - D) * NOX
    CO2_min = (1 - D) * CO2
    NOISE_min = (1 - D) * NOISE
    DOC_min = (1 - D) * DOC
    DEV_min = (1 - 2 * D * TRL) * DEV
    CER_min = (1 - 2 * D * TRL) * CER
    PROD_min = (1 - D) * PROD

    NOX_max = (1 + D) * NOX
    CO2_max = (1 + D) * CO2
    NOISE_max = (1 + D) * NOISE
    DOC_max = (1 + D) * DOC
    DEV_max = 2 * D * DEV + DEV_min
    CER_max = 2 * D * CER + CER_min
    PROD_max = (1 + D) * PROD

    print('\n')
    print('--------------------------------------------------------')
    print('# %s from %s (Deviation=%.2f)' % (aircraft_name, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),  D))
    print('settings.minmax_co2 =        [ %.5e, %.5e]' % (CO2_min, CO2_max))
    print('settings.minmax_nox =        [ %.5e, %.5e]' % (NOX_min, NOX_max))
    print('settings.minmax_noise_sone = [ %.5e, %.5e]' % (NOISE_min, NOISE_max))
    print('settings.minmax_doc =        [ %.5e, %.5e]' % (DOC_min, DOC_max))
    print('settings.minmax_dev =        [ %.5e, %.5e]' % (DEV_min, DEV_max))
    print('settings.minmax_cer =        [ %.5e, %.5e]' % (CER_min, CER_max))
    print('settings.minmax_prod =       [ %.5e, %.5e]' % (PROD_min, PROD_max))
    print('--------------------------------------------------------')
    print('\n')



