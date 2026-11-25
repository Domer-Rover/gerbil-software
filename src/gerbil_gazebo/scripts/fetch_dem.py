#!/usr/bin/env python3
"""Build a Gazebo Fortress terrain world from a public-domain USGS 3DEP elevation model.

Pipeline, per site:
  1. Ask the USGS 3DEP ImageServer (getSamples) which source dataset is on top at the centre
     coordinate (the 1 m lidar DEM where it exists, otherwise 1/3 arc-second), and record it.
  2. Export a SIZE x SIZE metre patch around the coordinate as float32 GeoTIFF from the 3DEP
     ImageServer (exportImage), in the site's NAD83 UTM zone so pixels are square in metres.
     The server picks the best available source for the requested pixel size (< 1 m here).
  3. Shift the heights so the lowest point is 0 m (so Gazebo's DEM and size.z interpretations
     agree) and write a (2^n)+1 square float32 GeoTIFF. Fortress loads .tif heightmaps through
     GDAL as a DEM, keeping real metres of relief; an 8-bit PNG heightmap would quantise e.g.
     20 m of relief into 8 cm steps, and Fortress' image heightmaps are 8-bit only.
  4. Procedurally generate a tileable ground texture (+ normal map): no imagery is downloaded.
  5. Scatter a few low-poly static obstacles (rotated boxes or logs), pick a flat spawn spot near
     the centre, and write models/<terrain>/ (model + README) and worlds/<site>.sdf.

Requires numpy, tifffile and Pillow (not needed at sim run time, only to regenerate):
    python3 -m venv /tmp/demvenv && /tmp/demvenv/bin/pip install numpy tifffile Pillow
    /tmp/demvenv/bin/python src/gerbil_gazebo/scripts/fetch_dem.py --site mdrs_utah \
        --download-dir /tmp/dem-mdrs

Downloads go into --download-dir (created; must be empty) and are treated as untrusted: the
TIFF is size-capped, parsed as plain numeric data and range-checked before use.

Gazebo heightmap axes: column -> +x (east), the top image row -> +y (north), the patch centre
is the world origin, z = 0 is the lowest point of the patch.
"""

import argparse
import datetime
import json
import math
import os
import sys
import urllib.parse
import urllib.request

try:
    import numpy as np
    import tifffile
    from PIL import Image
except ImportError as exc:  # pragma: no cover
    sys.exit('fetch_dem.py needs numpy, tifffile and Pillow (%s); see the docstring' % exc)

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, '..'))
IMAGE_SERVER = 'https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer'
TNM_API = 'https://tnmaccess.nationalmap.gov/api/v1/products'
MAX_DOWNLOAD = 32 * 1024 * 1024
UA = 'gerbil-gazebo-fetch-dem/1.0'

SITES = {
    'mdrs_utah': dict(
        lat=38.40547138735648, lon=-110.79001630859466,
        title='MDRS / University Rover Challenge site, near Hanksville, Utah',
        model='mdrs_terrain', texture='desert', scatter='rocks', count=14,
        friction=0.9, seed=38),
    'indiana_dunes': dict(
        lat=41.6250, lon=-87.2110,
        title='West Beach, Indiana Dunes National Park, Indiana (beach, foredune and '
              'interdunal flat, with the toe of a large dune at the south edge)',
        model='dunes_terrain', texture='sand', scatter='driftwood', count=8,
        friction=0.7, seed=41),
}


# --------------------------------------------------------------------------- geodesy
def utm_zone(lon):
    return int(math.floor((lon + 180.0) / 6.0)) + 1


def latlon_to_utm(lat, lon, zone):
    """Transverse Mercator forward (Snyder 1987, GRS80); mm-accurate within a zone."""
    a, f = 6378137.0, 1 / 298.257222101
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)
    k0 = 0.9996
    phi, lam = math.radians(lat), math.radians(lon)
    lam0 = math.radians((zone - 1) * 6 - 180 + 3)
    n = a / math.sqrt(1 - e2 * math.sin(phi) ** 2)
    t = math.tan(phi) ** 2
    c = ep2 * math.cos(phi) ** 2
    A = math.cos(phi) * (lam - lam0)
    e4, e6 = e2 * e2, e2 * e2 * e2
    m = a * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
             - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * math.sin(2 * phi)
             + (15 * e4 / 256 + 45 * e6 / 1024) * math.sin(4 * phi)
             - (35 * e6 / 3072) * math.sin(6 * phi))
    x = k0 * n * (A + (1 - t + c) * A ** 3 / 6
                  + (5 - 18 * t + t * t + 72 * c - 58 * ep2) * A ** 5 / 120) + 500000.0
    y = k0 * (m + n * math.tan(phi) * (A * A / 2 + (5 - t + 9 * c + 4 * c * c) * A ** 4 / 24
                                       + (61 - 58 * t + t * t + 600 * c - 330 * ep2) * A ** 6 / 720))
    return x, y


# --------------------------------------------------------------------------- download
def http_get(url, params, limit=MAX_DOWNLOAD):
    req = urllib.request.Request(url + '?' + urllib.parse.urlencode(params), headers={'User-Agent': UA})
    with urllib.request.urlopen(req, timeout=180) as resp:
        data = resp.read(limit + 1)
    if len(data) > limit:
        sys.exit('error: response from %s exceeds %d bytes' % (url, limit))
    return data


def source_info(lat, lon):
    """Top-priority 3DEP source at the point, plus the matching TNM 1 m product if any."""
    geom = json.dumps({'x': lon, 'y': lat, 'spatialReference': {'wkid': 4326}})
    raw = http_get(IMAGE_SERVER + '/getSamples', {
        'geometry': geom, 'geometryType': 'esriGeometryPoint', 'returnFirstValueOnly': 'false',
        'outFields': 'Name,LowPS,AcquisitionDate,URL,Metadata,VerticalDatum', 'f': 'json'}, 2 ** 20)
    samples = json.loads(raw.decode('utf-8')).get('samples') or []
    if not samples:
        sys.exit('error: 3DEP returned no samples at %f, %f' % (lat, lon))
    attrs = samples[0].get('attributes') or {}
    info = {
        'dataset': str(attrs.get('Name', ''))[:200],
        'native_resolution_m': attrs.get('LowPS'),
        'vertical_datum': str(attrs.get('VerticalDatum', ''))[:200],
        'acquired': None,
        'sample_elevation_m': float(samples[0].get('value')),
    }
    if isinstance(attrs.get('AcquisitionDate'), (int, float)):
        info['acquired'] = datetime.datetime.fromtimestamp(attrs['AcquisitionDate'] / 1000, datetime.timezone.utc).strftime('%Y-%m-%d')
    # The TNM API names the downloadable product for the same dataset (for the README).
    d = 0.001
    raw = http_get(TNM_API, {'bbox': '%f,%f,%f,%f' % (lon - d, lat - d, lon + d, lat + d),
                             'datasets': 'Digital Elevation Model (DEM) 1 meter',
                             'prodFormats': 'GeoTIFF'}, 2 ** 22)
    items = json.loads(raw.decode('utf-8')).get('items') or []
    match = [it for it in items if info['dataset'] and info['dataset'] in str(it.get('downloadURL', ''))]
    pick = (match or items or [None])[0]
    if pick:
        info['tnm_product'] = str(pick.get('title', ''))[:200]
        info['tnm_download_url'] = str(pick.get('downloadURL', ''))[:500]
    return info


def export_dem(lat, lon, size_m, px, download_dir):
    zone = utm_zone(lon)
    epsg = 26900 + zone  # NAD83 / UTM zone N (the native CRS of the 1 m 3DEP tiles)
    cx, cy = latlon_to_utm(lat, lon, zone)
    half = size_m / 2.0
    bbox = (cx - half, cy - half, cx + half, cy + half)
    params = {
        'bbox': ','.join('%.3f' % v for v in bbox), 'bboxSR': epsg, 'imageSR': epsg,
        'size': '%d,%d' % (px, px), 'format': 'tiff', 'pixelType': 'F32',
        'interpolation': 'RSP_BilinearInterpolation',
        'renderingRule': json.dumps({'rasterFunction': 'None'}), 'f': 'image',
    }
    data = http_get(IMAGE_SERVER + '/exportImage', params)
    if data[:4] not in (b'II*\x00', b'MM\x00*'):
        sys.exit('error: exportImage did not return a TIFF (%r...)' % data[:60])
    path = os.path.join(download_dir, 'export.tif')
    with open(path, 'wb') as fh:
        fh.write(data)
    url = IMAGE_SERVER + '/exportImage?' + urllib.parse.urlencode(params)
    return path, dict(epsg=epsg, zone=zone, bbox=bbox, centre_utm=(cx, cy), request_url=url)


def read_dem(path, px):
    with tifffile.TiffFile(path) as tif:
        if len(tif.pages) < 1:
            sys.exit('error: empty TIFF')
        page = tif.pages[0]
        if tuple(page.shape) != (px, px):
            sys.exit('error: DEM is %r, expected %dx%d' % (page.shape, px, px))
        arr = page.asarray().astype(np.float64)
    bad = ~np.isfinite(arr) | (arr < -500) | (arr > 9000)
    if bad.any():
        sys.exit('error: DEM has %d nodata/out-of-range cells' % int(bad.sum()))
    return arr


def write_geotiff(path, z, epsg, west, north, res):
    """Float32 GeoTIFF in NAD83 / UTM, PixelIsArea, origin at the north-west corner."""
    geokeys = (1, 1, 0, 4,
               1024, 0, 1, 1,        # GTModelTypeGeoKey = projected
               1025, 0, 1, 1,        # GTRasterTypeGeoKey = PixelIsArea
               3072, 0, 1, epsg,     # ProjectedCSTypeGeoKey
               3076, 0, 1, 9001)     # ProjLinearUnitsGeoKey = metre
    tifffile.imwrite(path, z.astype(np.float32), photometric='minisblack', extratags=[
        (33550, 'd', 3, (res, res, 0.0), False),
        (33922, 'd', 6, (0.0, 0.0, 0.0, west, north, 0.0), False),
        (34735, 'H', len(geokeys), geokeys, False),
    ])


# --------------------------------------------------------------------------- textures
def tileable_noise(n, rng, beta=2.0, lowcut=2.0):
    """Periodic fractal noise (FFT-filtered white noise), scaled to [0, 1]."""
    k = np.fft.fftfreq(n) * n
    kk = np.sqrt(k[None, :] ** 2 + k[:, None] ** 2)
    kk[0, 0] = 1.0
    spec = np.fft.fft2(rng.standard_normal((n, n))) / kk ** (beta / 2.0)
    spec[kk < lowcut] *= 0.3
    field = np.real(np.fft.ifft2(spec))
    field -= field.min()
    return field / field.max()


def make_textures(kind, out_dir, seed, n=512):
    rng = np.random.default_rng(seed)
    base = tileable_noise(n, rng, 2.2)
    fine = tileable_noise(n, rng, 0.8, 0.0)
    if kind == 'desert':
        # Tan silt with red-brown patches and darker pebbles.
        c0, c1 = np.array([196, 160, 116.0]), np.array([168, 98, 64.0])
        t = np.clip((base - 0.35) * 2.2, 0, 1)[..., None]
        rgb = c0 * (1 - t) + c1 * t
        rgb *= (0.88 + 0.24 * fine)[..., None]
        pebbles = rng.random((n, n)) > 0.996
        rgb[pebbles] *= 0.55
        bump = 0.6 * base + 0.4 * fine
    else:
        # Pale beach sand with faint wind ripples.
        c0, c1 = np.array([226, 206, 164.0]), np.array([201, 178, 134.0])
        y = np.arange(n)[:, None] * np.ones((1, n))
        ripple = 0.5 + 0.5 * np.sin(2 * np.pi * (y * 24 / n + 3.0 * base))
        t = np.clip(0.6 * base + 0.25 * ripple, 0, 1)[..., None]
        rgb = c0 * (1 - t) + c1 * t
        rgb *= (0.92 + 0.14 * fine)[..., None]
        bump = 0.5 * ripple + 0.5 * fine
    diffuse = Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8), 'RGB')
    # Tangent-space normal map from the bump field (periodic gradients keep it tileable).
    m = 256
    b = np.asarray(Image.fromarray((bump * 255).astype(np.uint8)).resize((m, m), Image.BILINEAR),
                   dtype=np.float64) / 255.0
    gx = (np.roll(b, -1, 1) - np.roll(b, 1, 1)) * 2.0
    gy = (np.roll(b, -1, 0) - np.roll(b, 1, 0)) * 2.0
    nz = 1.0 / np.sqrt(gx * gx + gy * gy + 1.0)
    nrm = np.stack([-gx * nz, gy * nz, nz], -1)
    normal = Image.fromarray(((nrm * 0.5 + 0.5) * 255).astype(np.uint8), 'RGB')
    os.makedirs(out_dir, exist_ok=True)
    diffuse.save(os.path.join(out_dir, '%s_diffuse.png' % kind), optimize=True)
    normal.save(os.path.join(out_dir, '%s_normal.png' % kind), optimize=True)


# --------------------------------------------------------------------------- placement
class Terrain:
    def __init__(self, z, size):
        self.z, self.size, self.n = z, size, z.shape[0]
        self.step = size / (self.n - 1)
        gy, gx = np.gradient(z, self.step)
        self.slope = np.degrees(np.arctan(np.hypot(gx, gy)))

    def rc(self, x, y):
        c = (x + self.size / 2) / self.step
        r = (self.size / 2 - y) / self.step
        return r, c

    def height(self, x, y):
        r, c = self.rc(x, y)
        r0, c0 = int(np.clip(np.floor(r), 0, self.n - 2)), int(np.clip(np.floor(c), 0, self.n - 2))
        fr, fc = r - r0, c - c0
        z = self.z
        return float((z[r0, c0] * (1 - fc) + z[r0, c0 + 1] * fc) * (1 - fr)
                     + (z[r0 + 1, c0] * (1 - fc) + z[r0 + 1, c0 + 1] * fc) * fr)

    def window(self, x, y, radius):
        r, c = self.rc(x, y)
        k = int(math.ceil(radius / self.step))
        r, c = int(round(r)), int(round(c))
        sl = (slice(max(r - k, 0), r + k + 1), slice(max(c - k, 0), c + k + 1))
        return self.z[sl], self.slope[sl]


def pick_spawn(t, search=40.0, radius=2.0):
    best = None
    for x in np.arange(-search, search + 0.1, 1.0):
        for y in np.arange(-search, search + 0.1, 1.0):
            zw, sw = t.window(x, y, radius)
            score = float(sw.max()) + 0.5 * float(zw.max() - zw.min()) + 0.1 * math.hypot(x, y)
            if best is None or score < best[0]:
                best = (score, float(x), float(y), float(sw.max()))
    _, x, y, smax = best
    zw, _ = t.window(x, y, 0.6)
    return x, y, float(zw.max()) + 0.1, smax


def scatter(t, kind, count, seed, spawn, flat_limit):
    rng = np.random.default_rng(seed)
    placed, items = [], []
    tries = 0
    while len(items) < count and tries < 5000:
        tries += 1
        x, y = rng.uniform(-75, 75, 2)
        if math.hypot(x - spawn[0], y - spawn[1]) < 6.0:
            continue
        if any(math.hypot(x - px, y - py) < 8.0 for px, py in placed):
            continue
        zw, sw = t.window(x, y, 1.0)
        if sw.max() > flat_limit:
            continue
        yaw = rng.uniform(-math.pi, math.pi)
        if kind == 'rocks':
            sx, sy, sz = rng.uniform(0.35, 1.3), rng.uniform(0.3, 1.0), rng.uniform(0.25, 0.8)
            roll, pitch = rng.uniform(-0.25, 0.25, 2)
            z = float(zw.min()) + sz * 0.3  # sink ~20 % below the lowest ground under it
            shade = rng.uniform(0.75, 1.1)
            rgb = tuple(round(min(1.0, v * shade), 3) for v in (0.55, 0.36, 0.27))
            items.append(('rock_%d' % len(items), (x, y, z, roll, pitch, yaw), ('box', (sx, sy, sz)), rgb))
        else:
            r, length = rng.uniform(0.07, 0.15), rng.uniform(1.0, 2.6)
            z = float(zw.min()) + r * 0.6
            rgb = (0.52, 0.47, 0.40)
            items.append(('log_%d' % len(items), (x, y, z, math.pi / 2, 0.0, yaw), ('cylinder', (r, length)), rgb))
        placed.append((x, y))
    return items


# --------------------------------------------------------------------------- SDF
def fmt(v):
    s = ('%.4f' % round(v, 4)).rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def vec(vals):
    return ' '.join(fmt(v) for v in vals)


def mat(rgb):
    s = vec(rgb)
    return ('<material><ambient>%s 1</ambient><diffuse>%s 1</diffuse>'
            '<specular>0.05 0.05 0.05 1</specular></material>' % (s, s))


def terrain_model_sdf(site, cfg, size, relief):
    m = cfg['model']
    tex = cfg['texture']
    hm = ('<uri>model://%s/dem/%s_dem.tif</uri>\n'
          '            <size>%s %s %s</size>\n'
          '            <pos>0 0 0</pos>' % (m, site, fmt(size), fmt(size), fmt(relief)))
    return '''<?xml version="1.0"?>
<!-- GENERATED by scripts/fetch_dem.py (site %(site)s). %(title)s.
     %(size)s x %(size)s m USGS 3DEP DEM heightmap, heights shifted so the lowest point is z = 0
     (relief %(relief)s m). See README.md for the source and elevations. -->
<sdf version="1.8">
  <model name="%(m)s">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <heightmap>
            %(hm)s
          </heightmap>
        </geometry>
        <surface><friction><ode><mu>%(mu)s</mu><mu2>%(mu)s</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry>
          <heightmap>
            <use_terrain_paging>false</use_terrain_paging>
            <texture>
              <diffuse>model://%(m)s/materials/textures/%(tex)s_diffuse.png</diffuse>
              <normal>model://%(m)s/materials/textures/%(tex)s_normal.png</normal>
              <size>8</size>
            </texture>
            %(hm)s
          </heightmap>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
''' % dict(site=site, title=cfg['title'], size=fmt(size), relief=fmt(relief), m=m, hm=hm,
           mu=fmt(cfg['friction']), tex=tex)


def world_sdf(site, cfg, items, spawn, relief, step):
    out = []
    w = out.append
    w('<?xml version="1.0"?>')
    w('<!--')
    w('  %s: %s.' % (site, cfg['title']))
    w('  GENERATED by scripts/fetch_dem.py (site %s) - edit the script, not this file.' % site)
    w('  Terrain: model://%s (USGS 3DEP DEM as a triangle mesh, relief %s m, lowest point z = 0).'
      % (cfg['model'], fmt(relief)))
    w('  +x is east, +y is north, the origin is the patch centre (%.6f, %.6f).' % (cfg['lat'], cfg['lon']))
    w('  Spawn (config/worlds.yaml): x=%s y=%s z=%s, on the flattest spot near the centre.'
      % (fmt(spawn[0]), fmt(spawn[1]), fmt(spawn[2])))
    w('  Physics: %s ms steps (%d Hz) = 5 steps per 50 Hz ros2_control update, like school_loop.'
      % (fmt(step * 1000), int(round(1 / step))))
    w('  DART uses the ODE collision detector, which handles the wheel cylinders and the terrain trimesh well.')
    w('  Rendering: one directional light, no shadows, no sky; obstacles are primitives.')
    w('-->')
    w('<sdf version="1.8">')
    w('  <world name="%s">' % site)
    w('    <physics name="terrain_physics" type="ignored">')
    w('      <max_step_size>%s</max_step_size>' % fmt(step))
    w('      <real_time_factor>1.0</real_time_factor>')
    w('      <real_time_update_rate>%d</real_time_update_rate>' % int(round(1 / step)))
    w('      <dart><collision_detector>ode</collision_detector></dart>')
    w('    </physics>')
    w('')
    w('    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>')
    w('    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>')
    w('    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>')
    w('    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">')
    w('      <render_engine>ogre2</render_engine>')
    w('    </plugin>')
    w('')
    w('    <gravity>0 0 -9.81</gravity>')
    w('    <spherical_coordinates>')
    w('      <surface_model>EARTH_WGS84</surface_model>')
    w('      <world_frame_orientation>ENU</world_frame_orientation>')
    w('      <latitude_deg>%.8f</latitude_deg>' % cfg['lat'])
    w('      <longitude_deg>%.8f</longitude_deg>' % cfg['lon'])
    w('      <elevation>%s</elevation>' % fmt(cfg['_min_elev']))
    w('      <heading_deg>0</heading_deg>')
    w('    </spherical_coordinates>')
    w('    <scene>')
    w('      <ambient>0.6 0.6 0.6 1</ambient>')
    w('      <background>0.62 0.75 0.9 1</background>')
    w('      <shadows>false</shadows>')
    w('      <grid>false</grid>')
    w('    </scene>')
    w('')
    w('    <light type="directional" name="sun">')
    w('      <cast_shadows>false</cast_shadows>')
    w('      <pose>0 0 50 0 0 0</pose>')
    w('      <diffuse>0.9 0.88 0.82 1</diffuse>')
    w('      <specular>0.1 0.1 0.1 1</specular>')
    w('      <direction>-0.5 0.3 -0.8</direction>')
    w('    </light>')
    w('')
    w('    <include>')
    w('      <uri>model://%s</uri>' % cfg['model'])
    w('      <name>terrain</name>')
    w('      <pose>0 0 0 0 0 0</pose>')
    w('    </include>')
    w('')
    w('    <model name="%s">' % cfg['scatter'])
    w('      <static>true</static>')
    w('      <link name="link">')
    for name, pose, (gtype, dims), rgb in items:
        if gtype == 'box':
            geom = '<box><size>%s</size></box>' % vec(dims)
        else:
            geom = '<cylinder><radius>%s</radius><length>%s</length></cylinder>' % (fmt(dims[0]), fmt(dims[1]))
        for tag in ('collision', 'visual'):
            w('        <%s name="%s_%s">' % (tag, name, tag[:3]))
            w('          <pose>%s</pose>' % vec(pose))
            w('          <geometry>%s</geometry>' % geom)
            if tag == 'visual':
                w('          %s' % mat(rgb))
            w('        </%s>' % tag)
    w('      </link>')
    w('    </model>')
    w('  </world>')
    w('</sdf>')
    return '\n'.join(out) + '\n'


def readme(site, cfg, meta, src, size, px, stats, spawn, files):
    lines = [
        '# %s' % cfg['model'],
        '',
        'Generated by `scripts/fetch_dem.py --site %s`; do not edit by hand.' % site,
        '',
        '%s.' % cfg['title'],
        '',
        '| | |',
        '|---|---|',
        '| Centre | %.8f, %.8f (WGS84) |' % (cfg['lat'], cfg['lon']),
        '| Patch | %s x %s m, %d x %d samples (%.3f m spacing), NAD83 / UTM zone %dN (EPSG:%d) |'
        % (fmt(size), fmt(size), px, px, size / (px - 1), meta['zone'], meta['epsg']),
        '| Source dataset | USGS 3DEP `%s` (native %s m), acquired %s |'
        % (src['dataset'], src.get('native_resolution_m'), src.get('acquired')),
        '| TNM product | %s |' % src.get('tnm_product', 'n/a'),
        '| Product URL | %s |' % src.get('tnm_download_url', 'n/a'),
        '| Vertical datum | %s |' % src['vertical_datum'],
        '| Elevation min / max | %.2f m / %.2f m (relief %.2f m) |' % (stats['min'], stats['max'], stats['relief']),
        '| Slope | median %.1f deg, 90th percentile %.1f deg |' % (stats['slope_p50'], stats['slope_p90']),
        '| Retrieved | %s |' % stats['retrieved'],
        '',
        'Fetched from the 3DEP ImageServer `exportImage` endpoint, which mosaics the best',
        'available 3DEP source (here the 1 m lidar DEM above) for the requested pixel size:',
        '',
        '```',
        meta['request_url'],
        '```',
        '',
        'The heightmap `dem/%s_dem.tif` is float32 metres with the minimum subtracted, so z = 0' % site,
        'in Gazebo is %.2f m NAVD88. Gazebo Fortress loads `.tif` heightmaps through GDAL as a DEM.' % stats['min'],
        'The model sets `<size>` explicitly (x, y = patch size, z = relief), so the result is right',
        'whether Gazebo scales by size or uses the raw heights. The textures are procedurally',
        'generated by the script (no imagery).',
        '',
        'USGS 3DEP data is public domain (U.S. Government work). Credit: U.S. Geological Survey,',
        '3D Elevation Program.',
        '',
        'Files: %s' % ', '.join('`%s`' % f for f in files),
        '',
    ]
    return '\n'.join(lines)


# --------------------------------------------------------------------------- main
def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--site', required=True, choices=sorted(SITES))
    p.add_argument('--download-dir', required=True, help='new or empty directory for the raw download')
    p.add_argument('--size', type=float, default=200.0, help='patch edge, m')
    p.add_argument('--px', type=int, default=257, help='samples per edge, must be 2^n+1')
    p.add_argument('--step', type=float, default=0.004, help='physics max_step_size, s')
    p.add_argument('--pkg-dir', default=PKG, help='gerbil_gazebo package dir to write into')
    args = p.parse_args()
    if args.px < 3 or (args.px - 1) & (args.px - 2):
        sys.exit('error: --px must be (2^n)+1, e.g. 129, 257, 513')
    cfg = dict(SITES[args.site])

    os.makedirs(args.download_dir, exist_ok=True)
    if os.listdir(args.download_dir):
        sys.exit('error: --download-dir %s is not empty' % args.download_dir)

    print('querying 3DEP source at %.6f, %.6f ...' % (cfg['lat'], cfg['lon']))
    src = source_info(cfg['lat'], cfg['lon'])
    print('  top source: %s (native %s m)' % (src['dataset'], src['native_resolution_m']))
    print('exporting %g m patch at %d px ...' % (args.size, args.px))
    path, meta = export_dem(cfg['lat'], cfg['lon'], args.size, args.px, args.download_dir)
    elev = read_dem(path, args.px)

    zmin, zmax = float(elev.min()), float(elev.max())
    rel = elev - zmin
    relief = round(zmax - zmin, 3)
    cfg['_min_elev'] = zmin
    t = Terrain(rel, args.size)
    stats = dict(min=zmin, max=zmax, relief=relief,
                 slope_p50=float(np.percentile(t.slope, 50)), slope_p90=float(np.percentile(t.slope, 90)),
                 retrieved=datetime.date.today().isoformat())
    print('  elevation %.2f .. %.2f m, relief %.2f m' % (zmin, zmax, relief))

    model_dir = os.path.join(args.pkg_dir, 'models', cfg['model'])
    os.makedirs(os.path.join(model_dir, 'dem'), exist_ok=True)
    res = args.size / args.px
    west, north = meta['bbox'][0], meta['bbox'][3]
    write_geotiff(os.path.join(model_dir, 'dem', '%s_dem.tif' % args.site), rel, meta['epsg'], west, north, res)
    make_textures(cfg['texture'], os.path.join(model_dir, 'materials', 'textures'), cfg['seed'])

    sx, sy, sz, smax = pick_spawn(t)
    spawn = (sx, sy, sz)
    print('  spawn x=%s y=%s z=%s (max slope %.1f deg within 2 m)' % (fmt(sx), fmt(sy), fmt(sz), smax))
    flat_limit = 20.0 if cfg['scatter'] == 'rocks' else 8.0
    items = scatter(t, cfg['scatter'], cfg['count'], cfg['seed'], spawn, flat_limit)

    with open(os.path.join(model_dir, 'model.sdf'), 'w') as fh:
        fh.write(terrain_model_sdf(args.site, cfg, args.size, relief))
    # Fortress cannot use a float DEM as a physics heightmap, so the shipped
    # model is a mesh. This overwrites the heightmap model.sdf written above.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import dem_to_mesh
    dem_to_mesh.convert(model_dir, args.size)
    with open(os.path.join(model_dir, 'model.config'), 'w') as fh:
        fh.write('<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n'
                 '  <sdf version="1.8">model.sdf</sdf>\n  <author><name>Gerbil team</name></author>\n'
                 '  <description>%s: USGS 3DEP DEM terrain mesh, %s x %s m.</description>\n</model>\n'
                 % (cfg['model'], cfg['title'], fmt(args.size), fmt(args.size)))
    files = sorted(os.path.relpath(os.path.join(dp, f), model_dir)
                   for dp, _, fs in os.walk(model_dir) for f in fs if f != 'README.md')
    with open(os.path.join(model_dir, 'README.md'), 'w') as fh:
        fh.write(readme(args.site, cfg, meta, src, args.size, args.px, stats, spawn, files))
    world_path = os.path.join(args.pkg_dir, 'worlds', '%s.sdf' % args.site)
    with open(world_path, 'w') as fh:
        fh.write(world_sdf(args.site, cfg, items, spawn, relief, args.step))
    print('wrote %s and %s (%d %s)' % (model_dir, world_path, len(items), cfg['scatter']))
    print('worlds.yaml:\n%s:\n  file: %s.sdf\n  spawn: {x: %s, y: %s, z: %s, yaw: 0.0}'
          % (args.site, args.site, fmt(sx), fmt(sy), fmt(sz)))


if __name__ == '__main__':
    main()
