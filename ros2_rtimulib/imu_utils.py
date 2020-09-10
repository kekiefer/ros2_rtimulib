
G_TO_MPSS = 9.80665


def get_altitude(pressure: float, sea_level_hPa: float = 1013.25) -> float:
    """
    the conversion uses the formula:
    
    h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
    where:
    h  = height above sea level
    T0 = standard temperature at sea level = 288.15
    L0 = standard temperatur elapse rate = -0.0065
    p  = measured pressure
    P0 = static pressure = 1013.25
    g0 = gravitational acceleration = 9.80665
    M  = mloecular mass of earth's air = 0.0289644
    R* = universal gas constant = 8.31432
    Given the constants, this works out to:
    h = 44330.8 * (1 - (p / P0)**0.190263)
    
    Arguments:
        pressure {float} -- current pressure 
        sea_level_hPa {float} -- The current hPa at sea level.
    
    Returns:
        [type] -- [description]
    """
    return 44330.8 * (1 - pow(pressure / sea_level_hPa, 0.190263))

def compute_sea_level(altitude: float, atmospheric: float) -> float:
    """
    Calculates the pressure at sea level (in hPa) from the specified altitude
    (in meters), and atmospheric pressure (in hPa).
    # Equation taken from BMP180 datasheet (page 17):
    # http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    Args:
        altitude    :  Altitude in meters
        atmospheric :  Atmospheric pressure in hPa
    
    Return:
        float The approximate pressure
    """
    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255)
