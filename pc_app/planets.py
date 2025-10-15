"""Very-lightweight planetary target direction utilities.

The routines here implement the analytical approximation published by
NASA/JPL's "Approximate Positions of the Major Planets" note. The goal is to
produce a geocentric direction vector (ignoring parallax) that the wearable can
chase. Accuracy is within a degree or so, which is sufficient for the guidance
game.
"""

from __future__ import annotations

import dataclasses
import math
import random
from datetime import datetime, timezone
from typing import Iterable, Tuple

# Mean obliquity of the ecliptic at J2000 (deg)
_MEAN_OBLIQUITY_DEG = 23.439291

_RNG = random.SystemRandom()

# Observer location as latitude/longitude in degrees.
@dataclasses.dataclass(frozen=True)
class ObserverLocation:
    latitude_deg: float
    longitude_deg: float  # East-positive, range [-180, 180]


@dataclasses.dataclass(frozen=True)
class PlanetCoefficients:
    name: str
    L: Tuple[float, float]  # mean longitude degrees (L0, L1)
    a: Tuple[float, float]  # semi-major axis AU
    e: Tuple[float, float]  # eccentricity
    I: Tuple[float, float]  # inclination degrees
    Omega: Tuple[float, float]  # longitude of ascending node degrees
    pi: Tuple[float, float]  # longitude of perihelion degrees


# Orbital elements approximations (J2000 epoch, T in Julian centuries).
_PLANET_PARAMS = [
    PlanetCoefficients(
        "mercury",
        L=(252.250906, 149472.6746358),
        a=(0.38709831, 0.0),
        e=(0.20563175, 0.000020407),
        I=(7.004986, -0.0059516),
        Omega=(48.330893, -0.125422),
        pi=(77.456119, 0.160476),
    ),
    PlanetCoefficients(
        "venus",
        L=(181.979801, 58517.8156760),
        a=(0.72332982, 0.0),
        e=(0.00677188, -0.000047766),
        I=(3.394662, -0.0008568),
        Omega=(76.679920, -0.278013),
        pi=(131.563707, 0.004799),
    ),
    PlanetCoefficients(
        "mars",
        L=(355.433000, 19140.2993039),
        a=(1.523679, 0.0),
        e=(0.09340062, 0.000090483),
        I=(1.849726, -0.0081479),
        Omega=(49.558093, -0.294984),
        pi=(336.060234, 0.443889),
    ),
    PlanetCoefficients(
        "jupiter",
        L=(34.351484, 3034.9056746),
        a=(5.20260, 0.0),
        e=(0.04849793, 0.000163225),
        I=(1.303270, -0.0019872),
        Omega=(100.464441, 0.176682),
        pi=(14.331309, 0.215552),
    ),
    PlanetCoefficients(
        "saturn",
        L=(50.077471, 1222.1137943),
        a=(9.554909, 0.0),
        e=(0.05550825, -0.000346818),
        I=(2.488878, 0.0025515),
        Omega=(113.665524, -0.256664),
        pi=(93.057237, 0.566541),
    ),
    PlanetCoefficients(
        "uranus",
        L=(314.055005, 428.4669983),
        a=(19.218446, -0.00000037),
        e=(0.04629590, -0.000027337),
        I=(0.773196, -0.0016869),
        Omega=(74.005947, 0.074146),
        pi=(173.005159, 0.089320),
    ),
    PlanetCoefficients(
        "neptune",
        L=(304.348665, 218.4862002),
        a=(30.110386, -0.000000166),
        e=(0.00898809, 0.000006408),
        I=(1.769952, -0.0024971),
        Omega=(131.784057, -0.006165),
        pi=(48.123691, 0.029158),
    ),
]

# Earth coefficients for geocentric conversion.
_EARTH_COEFF = PlanetCoefficients(
    "earth",
    L=(100.466457, 35999.3728565),
    a=(1.000001018, 0.0),
    e=(0.01670863, -0.000042037),
    I=(0.0, 0.0),
    Omega=(0.0, 0.0),
    pi=(102.937348, 0.322555),
)

Vector = Tuple[float, float, float]


def _julian_date(dt: datetime) -> float:
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    dt = dt.astimezone(timezone.utc)
    year = dt.year
    month = dt.month
    day = dt.day + (dt.hour + (dt.minute + dt.second / 60.0) / 60.0) / 24.0
    if month <= 2:
        year -= 1
        month += 12
    A = year // 100
    B = 2 - A + A // 4
    jd = int(365.25 * (year + 4716))
    jd += int(30.6001 * (month + 1))
    jd += day + B - 1524.5
    return jd


def _julian_centuries(dt: datetime) -> float:
    return (_julian_date(dt) - 2451545.0) / 36525.0


def _deg_to_rad(angle_deg: float) -> float:
    return math.radians(angle_deg % 360.0)


def _solve_kepler(M_rad: float, eccentricity: float) -> float:
    E = M_rad
    for _ in range(10):
        f = E - eccentricity * math.sin(E) - M_rad
        f_prime = 1 - eccentricity * math.cos(E)
        delta = f / f_prime
        E -= delta
        if abs(delta) < 1e-6:
            break
    return E


def _orbital_elements(coeff: PlanetCoefficients, T: float) -> Tuple[float, float, float, float, float, float]:
    L = coeff.L[0] + coeff.L[1] * T
    a = coeff.a[0] + coeff.a[1] * T
    e = coeff.e[0] + coeff.e[1] * T
    I = coeff.I[0] + coeff.I[1] * T
    Omega = coeff.Omega[0] + coeff.Omega[1] * T
    pi = coeff.pi[0] + coeff.pi[1] * T
    return L, a, e, I, Omega, pi


def _heliocentric_ecliptic_vector(coeff: PlanetCoefficients, T: float) -> Vector:
    L, a, e, I_deg, Omega_deg, pi = _orbital_elements(coeff, T)
    M_deg = (L - pi) % 360.0
    M_rad = _deg_to_rad(M_deg)
    E = _solve_kepler(M_rad, e)

    # Position in orbital plane.
    x_prime = a * (math.cos(E) - e)
    y_prime = a * math.sqrt(1 - e * e) * math.sin(E)

    # Rotation angles.
    I = _deg_to_rad(I_deg)
    Omega = _deg_to_rad(Omega_deg)
    w = _deg_to_rad(pi - Omega_deg)

    cos_O = math.cos(Omega)
    sin_O = math.sin(Omega)
    cos_I = math.cos(I)
    sin_I = math.sin(I)
    cos_w = math.cos(w)
    sin_w = math.sin(w)

    # 3D rotation from orbital plane to ecliptic coordinates.
    x = (
        (cos_O * cos_w - sin_O * sin_w * cos_I) * x_prime
        + (-cos_O * sin_w - sin_O * cos_w * cos_I) * y_prime
    )
    y = (
        (sin_O * cos_w + cos_O * sin_w * cos_I) * x_prime
        + (-sin_O * sin_w + cos_O * cos_w * cos_I) * y_prime
    )
    z = (sin_w * sin_I) * x_prime + (cos_w * sin_I) * y_prime
    return x, y, z


def _to_equatorial(vec: Vector) -> Vector:
    epsilon = math.radians(_MEAN_OBLIQUITY_DEG)
    sin_e = math.sin(epsilon)
    cos_e = math.cos(epsilon)
    x, y, z = vec
    x_eq = x
    y_eq = y * cos_e - z * sin_e
    z_eq = y * sin_e + z * cos_e
    return x_eq, y_eq, z_eq


def _normalize(vec: Vector) -> Vector:
    mag = math.sqrt(vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2)
    if mag == 0.0:
        return (0.0, 0.0, 0.0)
    return (vec[0] / mag, vec[1] / mag, vec[2] / mag)


def _geocentric_direction(planet: PlanetCoefficients, dt: datetime) -> Vector:
    T = _julian_centuries(dt)
    earth_vec = _heliocentric_ecliptic_vector(_EARTH_COEFF, T)
    planet_vec = _heliocentric_ecliptic_vector(planet, T)
    # Geocentric vector from Earth to planet.
    x = planet_vec[0] - earth_vec[0]
    y = planet_vec[1] - earth_vec[1]
    z = planet_vec[2] - earth_vec[2]
    return _to_equatorial((x, y, z))


def _lst(dt: datetime, longitude_deg: float) -> float:
    jd = _julian_date(dt)
    T = (jd - 2451545.0) / 36525.0
    # Greenwich mean sidereal time in degrees.
    GMST = 280.46061837 + 360.98564736629 * (jd - 2451545.0) + 0.000387933 * T ** 2 - (T ** 3) / 38710000.0
    return (GMST + longitude_deg) % 360.0


def _equatorial_to_horizontal(vec: Vector, location: ObserverLocation, dt: datetime) -> Vector:
    x, y, z = vec
    r = math.sqrt(x * x + y * y + z * z)
    if r == 0.0:
        return (0.0, 0.0, 1.0)
    ra = math.degrees(math.atan2(y, x)) % 360.0
    dec = math.degrees(math.asin(z / r))

    lat = math.radians(location.latitude_deg)
    lst_deg = _lst(dt, location.longitude_deg)
    hour_angle = math.radians((lst_deg - ra + 360.0) % 360.0 - 180.0)
    dec_rad = math.radians(dec)

    sin_alt = math.sin(dec_rad) * math.sin(lat) + math.cos(dec_rad) * math.cos(lat) * math.cos(hour_angle)
    alt = math.asin(max(-1.0, min(1.0, sin_alt)))
    cos_alt = math.cos(alt)
    if abs(cos_alt) < 1e-6:
        az = 0.0
    else:
        sin_az = -math.sin(hour_angle) * math.cos(dec_rad) / cos_alt
        cos_az = (math.sin(dec_rad) - math.sin(lat) * sin_alt) / (math.cos(lat) * cos_alt)
        az = math.atan2(sin_az, cos_az)

    # Convert to ENU unit vector.
    east = math.sin(az) * math.cos(alt)
    north = math.cos(az) * math.cos(alt)
    up = math.sin(alt)
    return _normalize((east, north, up))


def available_planets() -> Iterable[str]:
    return [coeff.name for coeff in _PLANET_PARAMS]


def select_planet(dt: datetime) -> PlanetCoefficients:
    _ = dt  # We rely on hardware RNG; keep signature for future weighting.
    return _RNG.choice(_PLANET_PARAMS)


def target_unit_vector(dt: datetime, location: ObserverLocation, planet: PlanetCoefficients) -> Vector:
    geocentric = _geocentric_direction(planet, dt)
    return _equatorial_to_horizontal(geocentric, location, dt)
