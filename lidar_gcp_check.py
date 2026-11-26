#!/usr/bin/env python3
"""
lidar_gcp_check.py - check a LiDAR point cloud against surveyed ground points.

For every point in an Emlid Flow CSV export (or any CSV with Name / Easting /
Northing / Elevation columns) the script collects the LiDAR points in a small
square around it, fits a plane through them (outliers rejected), reads the
plane's height at the surveyed position and reports

    dz = LiDAR height - surveyed height      (positive = LiDAR is high)

Outputs, written to the --out folder:
    gcp_check.gpkg          layers  gcp_check     one point per GCP, all results
                                    patch_points  the LiDAR points used (PointZ,
                                                  with residual to the plane)
                                    summary       statistics table
    gcp_check.csv           every column, one row per GCP
    gcp_check_report.csv    short table for the survey report
    gcp_check_summary.txt   statistics (mean, std dev, RMSE, 95 %)
    gcp_check.png           map of the GCPs coloured by dz (needs matplotlib;
                            --ortho adds a background, needs rasterio)

Needs:  pip install laspy[lazrs] numpy
Nice to have:  pip install matplotlib pyproj rasterio

Example (Windows, one line):
    python lidar_gcp_check.py --las "D:\\job\\lidar\\*.las" --gcp gcps.csv
        --epsg 7854 --classes 2 --out gcp_check

The LiDAR and the CSV must be in the SAME coordinate system and the SAME height
datum. Emlid's "Elevation" column is the height in the project's vertical datum
(AHD when the project was set up with AUSGeoid); use --z-field "Ellipsoidal
height" for a cloud that is still in ellipsoidal heights.
"""
import argparse
import csv
import glob
import math
import os
import re
import sqlite3
import struct
import sys
import time

import numpy as np

try:
    import laspy
except ImportError:
    sys.exit("laspy is not installed.  Run:  pip install laspy[lazrs] numpy")


# ----------------------------------------------------------------- GCP file

ALIASES = {
    "name": ["name", "label", "point", "point id", "id", "pt"],
    "e": ["easting", "east", "x", "e"],
    "n": ["northing", "north", "y", "n"],
    "z": ["elevation", "height", "rl", "z", "h", "ortho height"],
    "lon": ["longitude", "lon", "long"],
    "lat": ["latitude", "lat"],
}
# Emlid columns copied through to the outputs when present
CARRY = [
    ("code", "Code"), ("description", "Description"),
    ("solution", "Solution status"), ("e_rms", "Easting RMS"),
    ("n_rms", "Northing RMS"), ("z_rms", "Elevation RMS"),
    ("lateral_rms", "Lateral RMS"), ("antenna_h", "Antenna height"),
    ("samples", "Samples"), ("pdop", "PDOP"), ("baseline", "Baseline"),
    ("measured", "Averaging start"),
]


def find_col(header, wanted, explicit=None):
    low = {h.strip().lower(): h for h in header}
    if explicit:
        if explicit.strip().lower() in low:
            return low[explicit.strip().lower()]
        sys.exit('Column "%s" is not in the CSV. Columns are: %s'
                 % (explicit, ", ".join(header)))
    for a in wanted:
        if a in low:
            return low[a]
    return None


def fnum(v):
    try:
        return float(str(v).strip())
    except (TypeError, ValueError):
        return None


def read_gcps(path, args):
    with open(path, newline="", encoding="utf-8-sig") as f:
        sample = f.read(4096)
        f.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
        except csv.Error:
            dialect = csv.excel
        rows = list(csv.DictReader(f, dialect=dialect))
    if not rows:
        sys.exit("No rows in " + path)
    header = list(rows[0].keys())
    c_name = find_col(header, ALIASES["name"], args.name_field)
    c_e = find_col(header, ALIASES["e"], args.e_field)
    c_n = find_col(header, ALIASES["n"], args.n_field)
    c_z = find_col(header, ALIASES["z"], args.z_field)
    if not c_z:
        sys.exit("No height column found. Use --z-field. Columns: " + ", ".join(header))
    transformer = None
    if not (c_e and c_n):
        c_lon = find_col(header, ALIASES["lon"])
        c_lat = find_col(header, ALIASES["lat"])
        if not (c_lon and c_lat):
            sys.exit("No Easting/Northing (or Longitude/Latitude) columns found. "
                     "Columns: " + ", ".join(header))
        try:
            from pyproj import Transformer
        except ImportError:
            sys.exit("The CSV only has longitude/latitude. Install pyproj "
                     "(pip install pyproj) or export eastings/northings.")
        if not args.epsg:
            sys.exit("--epsg is needed to convert longitude/latitude.")
        transformer = Transformer.from_crs(args.gcp_geographic_epsg, args.epsg, always_xy=True)
        c_e, c_n = c_lon, c_lat
    low = {h.strip().lower(): h for h in header}
    out, skipped = [], 0
    for i, r in enumerate(rows):
        e, n, z = fnum(r.get(c_e)), fnum(r.get(c_n)), fnum(r.get(c_z))
        if e is None or n is None or z is None:
            skipped += 1
            continue
        if transformer:
            e, n = transformer.transform(e, n)
        g = {"name": (r.get(c_name) or "").strip() if c_name else "", "e": e, "n": n, "z": z}
        if not g["name"]:
            g["name"] = "row%d" % (i + 2)
        for key, col in CARRY:
            src = low.get(col.lower())
            g[key] = (r.get(src) or "").strip() if src else ""
        out.append(g)
    if args.only:
        pats = [p.strip().lower() for p in args.only.split(",") if p.strip()]
        out = [g for g in out if any(p in g["name"].lower() or p in g["code"].lower() for p in pats)]
    if args.exclude:
        pats = [p.strip().lower() for p in args.exclude.split(",") if p.strip()]
        out = [g for g in out if not any(p in g["name"].lower() for p in pats)]
    seen = {}
    for g in out:                      # the same name is often used more than once in the field
        k = g["name"].lower()
        seen[k] = seen.get(k, 0) + 1
        if seen[k] > 1:
            g["name"] = "%s (%d)" % (g["name"], seen[k])
    print("GCP file: %d points read (%d rows skipped), height column \"%s\""
          % (len(out), skipped, c_z))
    return out


# ------------------------------------------------------------ point search

def collect(las_paths, gcps, half, classes, chunk):
    """Return {gcp index: (x, y, z, cls)} of LiDAR points inside each patch."""
    gx = np.array([g["e"] for g in gcps])
    gy = np.array([g["n"] for g in gcps])
    cell = 2.0 * half
    ox, oy = gx.min() - 2 * cell, gy.min() - 2 * cell
    width = int((gy.max() - oy) / cell) + 4          # cells per column
    cellmap = {}
    for i in range(len(gcps)):
        for ix in {int((gx[i] - half - ox) // cell), int((gx[i] + half - ox) // cell)}:
            for iy in {int((gy[i] - half - oy) // cell), int((gy[i] + half - oy) // cell)}:
                cellmap.setdefault(ix * width + iy, []).append(i)
    keys = np.array(sorted(cellmap), dtype=np.int64)
    x0, x1 = gx.min() - half, gx.max() + half
    y0, y1 = gy.min() - half, gy.max() + half
    got = {i: [] for i in range(len(gcps))}
    crs_epsg, total_read = None, 0
    for path in las_paths:
        with laspy.open(path) as rd:
            h = rd.header
            if crs_epsg is None:
                try:
                    crs = h.parse_crs()
                    crs_epsg = crs.to_epsg() if crs is not None else None
                except Exception:
                    crs_epsg = None
            if h.maxs[0] < x0 or h.mins[0] > x1 or h.maxs[1] < y0 or h.mins[1] > y1:
                print("  %s: no GCPs inside its extent, skipped" % os.path.basename(path))
                continue
            n_total, done, t0 = h.point_count, 0, time.time()
            print("  %s: %s points" % (os.path.basename(path), format(n_total, ",")))
            for pts in rd.chunk_iterator(chunk):
                done += len(pts)
                x = np.asarray(pts.x)
                y = np.asarray(pts.y)
                m = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
                if classes is not None:
                    m &= np.isin(np.asarray(pts.classification), classes)
                idx = np.nonzero(m)[0]
                if len(idx):
                    xs, ys = x[idx], y[idx]
                    k = ((xs - ox) // cell).astype(np.int64) * width + ((ys - oy) // cell).astype(np.int64)
                    hit = np.isin(k, keys)
                    if hit.any():
                        idx, xs, ys, k = idx[hit], xs[hit], ys[hit], k[hit]
                        zs = np.asarray(pts.z)[idx]
                        cs = np.asarray(pts.classification)[idx]
                        for key in np.unique(k):
                            sel = k == key
                            for gi in cellmap[int(key)]:
                                mm = sel & (np.abs(xs - gx[gi]) <= half) & (np.abs(ys - gy[gi]) <= half)
                                if mm.any():
                                    got[gi].append((xs[mm], ys[mm], zs[mm], cs[mm]))
                sys.stdout.write("\r    %5.1f %%  (%.0f s)" % (100.0 * done / max(n_total, 1), time.time() - t0))
                sys.stdout.flush()
            total_read += done
            print()
    out = {}
    for gi, parts in got.items():
        if parts:
            out[gi] = tuple(np.concatenate([p[j] for p in parts]) for j in range(4))
    return out, crs_epsg, total_read


# ------------------------------------------------------------------ plane

def fit_plane(dx, dy, z, reject=3.0, floor=0.02):
    """Least-squares plane z = a*dx + b*dy + c with iterative outlier rejection.
    Returns (a, b, c, used mask, rms of used residuals)."""
    used = np.ones(len(z), dtype=bool)
    coef = None
    for _ in range(8):
        A = np.column_stack([dx[used], dy[used], np.ones(used.sum())])
        coef, *_ = np.linalg.lstsq(A, z[used], rcond=None)
        res = z - (coef[0] * dx + coef[1] * dy + coef[2])
        r = res[used]
        sigma = 1.4826 * np.median(np.abs(r - np.median(r)))
        lim = max(reject * sigma, floor)
        new = np.abs(res - np.median(r)) <= lim
        if new.sum() < 3 or np.array_equal(new, used):
            break
        used = new
    res = z - (coef[0] * dx + coef[1] * dy + coef[2])
    rms = float(np.sqrt(np.mean(res[used] ** 2)))
    return float(coef[0]), float(coef[1]), float(coef[2]), used, rms, res


def analyse(gcps, patches, args):
    for i, g in enumerate(gcps):
        g.update(n_pts=0, n_used=0, lidar_z=None, dz=None, dz_median=None,
                 plane_rms=None, slope_deg=None, dz_per_10cm=None, flags=[])
        p = patches.get(i)
        if g["solution"] and g["solution"].upper() != "FIX":
            g["flags"].append("NOT_FIX")
        if p is None or len(p[0]) < 3:
            g["n_pts"] = 0 if p is None else len(p[0])
            g["flags"].append("NO_DATA")
            continue
        x, y, z, _ = p
        g["n_pts"] = len(z)
        g["dz_median"] = float(np.median(z) - g["z"])
        if len(z) < args.min_points:
            g["flags"].append("FEW_POINTS")
            continue
        dx, dy = x - g["e"], y - g["n"]
        a, b, c, used, rms, res = fit_plane(dx, dy, z)
        g["_res"], g["_used"] = res, used
        g["n_used"] = int(used.sum())
        g["lidar_z"] = c                       # plane height at the GCP itself
        g["dz"] = c - g["z"]
        g["plane_rms"] = rms
        slope = math.hypot(a, b)
        g["slope_deg"] = math.degrees(math.atan(slope))
        g["dz_per_10cm"] = slope * 0.10        # height change for 10 cm of horizontal error
        # the GCP should sit inside the cloud of points, not at its edge
        q = [((dx[used] > 0) & (dy[used] > 0)).sum(), ((dx[used] < 0) & (dy[used] > 0)).sum(),
             ((dx[used] < 0) & (dy[used] < 0)).sum(), ((dx[used] > 0) & (dy[used] < 0)).sum()]
        if sum(1 for v in q if v > 0) < 3:
            g["flags"].append("ONE_SIDED")
        if g["slope_deg"] > args.max_slope:
            g["flags"].append("STEEP")
        if rms > args.max_rms:
            g["flags"].append("ROUGH")
    fitted = [g for g in gcps if g["dz"] is not None]
    if len(fitted) >= 5:
        d = np.array([g["dz"] for g in fitted])
        med = np.median(d)
        sigma = 1.4826 * np.median(np.abs(d - med))
        for g in fitted:
            if abs(g["dz"] - med) > max(3 * sigma, args.outlier_min):
                g["flags"].append("OUTLIER")
    return fitted


def stats(values):
    d = np.asarray(values, dtype=float)
    if len(d) == 0:
        return None
    rmse = float(np.sqrt(np.mean(d ** 2)))
    return {
        "count": int(len(d)), "mean": float(d.mean()), "median": float(np.median(d)),
        "std": float(d.std(ddof=1)) if len(d) > 1 else 0.0, "rmse": rmse,
        "min": float(d.min()), "max": float(d.max()),
        "abs_p95": float(np.percentile(np.abs(d), 95)),
        "acc95": 1.96 * rmse,
    }


# -------------------------------------------------------------- GeoPackage

def mga_wkt(epsg):
    if 7846 <= epsg <= 7859:
        zone, datum, gcs, auth = epsg - 7800, "Geocentric_Datum_of_Australia_2020", "GDA2020", 7844
    elif 28348 <= epsg <= 28358:
        zone, datum, gcs, auth = epsg - 28300, "Geocentric_Datum_of_Australia_1994", "GDA94", 4283
    else:
        return None
    return ('PROJCS["%s / MGA zone %d",GEOGCS["%s",DATUM["%s",SPHEROID["GRS 1980",6378137,298.257222101]],'
            'PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433],AUTHORITY["EPSG","%d"]],'
            'PROJECTION["Transverse_Mercator"],PARAMETER["latitude_of_origin",0],'
            'PARAMETER["central_meridian",%d],PARAMETER["scale_factor",0.9996],'
            'PARAMETER["false_easting",500000],PARAMETER["false_northing",10000000],'
            'UNIT["metre",1],AXIS["Easting",EAST],AXIS["Northing",NORTH],AUTHORITY["EPSG","%d"]]'
            % (gcs, zone, gcs, datum, auth, 6 * zone - 183, epsg))


def srs_wkt(epsg):
    try:
        from pyproj import CRS
        c = CRS.from_epsg(epsg)
        return c.name, c.to_wkt("WKT1_GDAL")
    except Exception:
        w = mga_wkt(epsg)
        if w:
            return "EPSG:%d" % epsg, w
        print("! No definition for EPSG:%d without pyproj - set the layer CRS by hand in QGIS." % epsg)
        return "EPSG:%d" % epsg, "undefined"


def gp_point(srs, x, y, z=None):
    head = b"GP" + struct.pack("<BBi", 0, 1, srs)
    if z is None:
        return head + struct.pack("<BIdd", 1, 1, x, y)
    return head + struct.pack("<BIddd", 1, 1001, x, y, z)


def write_gpkg(path, epsg, gcps, summary_rows, half, max_patch_points):
    if os.path.exists(path):
        os.remove(path)
    db = sqlite3.connect(path)
    c = db.cursor()
    c.executescript("""
    PRAGMA application_id = 1196444487; PRAGMA user_version = 10300;
    CREATE TABLE gpkg_spatial_ref_sys (srs_name TEXT NOT NULL, srs_id INTEGER PRIMARY KEY,
      organization TEXT NOT NULL, organization_coordsys_id INTEGER NOT NULL,
      definition TEXT NOT NULL, description TEXT);
    CREATE TABLE gpkg_contents (table_name TEXT PRIMARY KEY, data_type TEXT NOT NULL,
      identifier TEXT UNIQUE, description TEXT DEFAULT '',
      last_change DATETIME NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now')),
      min_x DOUBLE, min_y DOUBLE, max_x DOUBLE, max_y DOUBLE, srs_id INTEGER);
    CREATE TABLE gpkg_geometry_columns (table_name TEXT NOT NULL, column_name TEXT NOT NULL,
      geometry_type_name TEXT NOT NULL, srs_id INTEGER NOT NULL, z TINYINT NOT NULL,
      m TINYINT NOT NULL, PRIMARY KEY (table_name, column_name));
    """)
    name, wkt = srs_wkt(epsg)
    c.executemany("INSERT INTO gpkg_spatial_ref_sys VALUES (?,?,?,?,?,?)", [
        ("Undefined Cartesian SRS", -1, "NONE", -1, "undefined", None),
        ("Undefined geographic SRS", 0, "NONE", 0, "undefined", None),
        ("WGS 84 geodetic", 4326, "EPSG", 4326,
         'GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563]],'
         'PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433],AUTHORITY["EPSG","4326"]]', None),
        (name, epsg, "EPSG", epsg, wkt, None)])
    xs = [g["e"] for g in gcps]
    ys = [g["n"] for g in gcps]
    box = (min(xs) - half, min(ys) - half, max(xs) + half, max(ys) + half)

    c.execute("""CREATE TABLE gcp_check (fid INTEGER PRIMARY KEY AUTOINCREMENT, geom BLOB,
      name TEXT, code TEXT, description TEXT, easting REAL, northing REAL, gcp_z REAL,
      lidar_z REAL, dz REAL, dz_cm REAL, abs_dz_cm REAL, dz_median REAL, n_pts INTEGER,
      n_used INTEGER, plane_rms REAL, slope_deg REAL, dz_per_10cm_shift REAL, flags TEXT,
      ok INTEGER, solution TEXT, gcp_z_rms REAL, gcp_lateral_rms REAL, samples TEXT,
      baseline TEXT, measured TEXT)""")
    for g in gcps:
        dz = g["dz"]
        c.execute("INSERT INTO gcp_check VALUES (NULL,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", (
            gp_point(epsg, g["e"], g["n"]), g["name"], g["code"], g["description"], g["e"], g["n"],
            g["z"], g["lidar_z"], dz, None if dz is None else dz * 100,
            None if dz is None else abs(dz) * 100, g["dz_median"], g["n_pts"], g["n_used"],
            g["plane_rms"], g["slope_deg"], g["dz_per_10cm"], ",".join(g["flags"]),
            1 if (dz is not None and not g["flags"]) else 0, g["solution"], fnum(g["z_rms"]),
            fnum(g["lateral_rms"]), g["samples"], g["baseline"], g["measured"]))
    c.execute("INSERT INTO gpkg_contents (table_name,data_type,identifier,min_x,min_y,max_x,max_y,srs_id) "
              "VALUES ('gcp_check','features','gcp_check',?,?,?,?,?)", box + (epsg,))
    c.execute("INSERT INTO gpkg_geometry_columns VALUES ('gcp_check','geom','POINT',?,0,0)", (epsg,))

    c.execute("""CREATE TABLE patch_points (fid INTEGER PRIMARY KEY AUTOINCREMENT, geom BLOB,
      gcp TEXT, z REAL, residual REAL, used INTEGER, z_minus_gcp REAL, classification INTEGER)""")
    for g in gcps:
        p = g.get("_patch")
        if p is None:
            continue
        x, y, z, cls = p
        res, used = g.get("_res"), g.get("_used")
        step = max(1, int(math.ceil(len(z) / float(max_patch_points))))
        for j in range(0, len(z), step):
            c.execute("INSERT INTO patch_points VALUES (NULL,?,?,?,?,?,?,?)", (
                gp_point(epsg, float(x[j]), float(y[j]), float(z[j])), g["name"], float(z[j]),
                None if res is None else float(res[j]), None if used is None else int(used[j]),
                float(z[j] - g["z"]), int(cls[j])))
    c.execute("INSERT INTO gpkg_contents (table_name,data_type,identifier,min_x,min_y,max_x,max_y,srs_id) "
              "VALUES ('patch_points','features','patch_points',?,?,?,?,?)", box + (epsg,))
    c.execute("INSERT INTO gpkg_geometry_columns VALUES ('patch_points','geom','POINT',?,1,0)", (epsg,))

    c.execute("CREATE TABLE summary (fid INTEGER PRIMARY KEY AUTOINCREMENT, item TEXT, all_points TEXT, unflagged_points TEXT)")
    c.executemany("INSERT INTO summary (item, all_points, unflagged_points) VALUES (?,?,?)", summary_rows)
    c.execute("INSERT INTO gpkg_contents (table_name,data_type,identifier) VALUES ('summary','attributes','summary')")
    db.commit()
    db.close()


# ------------------------------------------------------------------ outputs

def f(v, nd=3):
    return "" if v is None else ("%." + str(nd) + "f") % v


def write_csvs(out, gcps):
    cols = ["name", "code", "description", "easting", "northing", "gcp_z", "lidar_z", "dz",
            "dz_median", "n_pts", "n_used", "plane_rms", "slope_deg", "dz_per_10cm_shift",
            "flags", "solution", "gcp_z_rms", "gcp_lateral_rms", "samples", "baseline", "measured"]
    with open(os.path.join(out, "gcp_check.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for g in gcps:
            w.writerow([g["name"], g["code"], g["description"], f(g["e"]), f(g["n"]), f(g["z"]),
                        f(g["lidar_z"]), f(g["dz"]), f(g["dz_median"]), g["n_pts"], g["n_used"],
                        f(g["plane_rms"]), f(g["slope_deg"], 1), f(g["dz_per_10cm"]),
                        " ".join(g["flags"]), g["solution"], g["z_rms"], g["lateral_rms"],
                        g["samples"], g["baseline"], g["measured"]])
    with open(os.path.join(out, "gcp_check_report.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["Point", "Easting (m)", "Northing (m)", "Surveyed height (m)",
                    "LiDAR height (m)", "Difference (m)", "LiDAR points", "Slope (deg)", "Note"])
        for g in gcps:
            w.writerow([g["name"], f(g["e"]), f(g["n"]), f(g["z"]), f(g["lidar_z"]), f(g["dz"]),
                        g["n_used"] or g["n_pts"], f(g["slope_deg"], 1), " ".join(g["flags"])])


def summary_rows(s_all, s_ok, gcps, args, epsg, n_las):
    def col(s, k, unit=True):
        if not s:
            return ""
        return str(s[k]) if k == "count" else "%+.3f" % s[k] if k in ("mean", "median", "min", "max") else "%.3f" % s[k]
    rows = [
        ("Points in the GCP file", str(len(gcps)), ""),
        ("Points compared", col(s_all, "count"), col(s_ok, "count")),
        ("Mean difference, LiDAR - surveyed (m)", col(s_all, "mean"), col(s_ok, "mean")),
        ("Median difference (m)", col(s_all, "median"), col(s_ok, "median")),
        ("Standard deviation (m)", col(s_all, "std"), col(s_ok, "std")),
        ("RMSE (m)", col(s_all, "rmse"), col(s_ok, "rmse")),
        ("Vertical accuracy at 95 % = 1.96 x RMSE (m)", col(s_all, "acc95"), col(s_ok, "acc95")),
        ("95th percentile of absolute differences (m)", col(s_all, "abs_p95"), col(s_ok, "abs_p95")),
        ("Lowest difference (m)", col(s_all, "min"), col(s_ok, "min")),
        ("Highest difference (m)", col(s_all, "max"), col(s_ok, "max")),
        ("Patch size (m square)", "%.2f" % args.patch, ""),
        ("LiDAR classes used", "all" if args.classes is None else args.classes, ""),
        ("Coordinate system", "EPSG:%d" % epsg, ""),
        ("LiDAR points read", format(n_las, ","), ""),
    ]
    return rows


def write_summary(out, rows, gcps):
    lines = ["LiDAR check against surveyed ground points", "",
             "%-48s %14s %18s" % ("", "All compared", "Unflagged only")]
    for item, a, b in rows:
        lines.append("%-48s %14s %18s" % (item, a, b))
    flagged = [g for g in gcps if g["flags"]]
    if flagged:
        lines += ["", "Flagged points:"]
        for g in flagged:
            lines.append("  %-22s dz %8s m   %s" % (g["name"], f(g["dz"]) or "-", " ".join(g["flags"])))
    lines += ["",
              "Flags: NO_DATA = no LiDAR points in the patch; FEW_POINTS = too few for a plane;",
              "STEEP / ROUGH = plane steeper or rougher than the limits, so the height there is",
              "sensitive to horizontal error or to what the laser hit; ONE_SIDED = LiDAR points",
              "on one side of the GCP only; OUTLIER = more than 3 robust standard deviations from",
              "the median difference; NOT_FIX = the rover solution was not a fixed solution.",
              "RMSE and the 95 % figure assume the differences are roughly normally distributed.",
              "A mean well away from zero is a systematic offset (base height, antenna height,",
              "geoid model or pole height), not noise."]
    text = "\n".join(lines) + "\n"
    with open(os.path.join(out, "gcp_check_summary.txt"), "w", encoding="utf-8") as fh:
        fh.write(text)
    return text


def write_plot(out, gcps, epsg, s_all, args):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed - no picture written (pip install matplotlib).")
        return
    fitted = [g for g in gcps if g["dz"] is not None]
    if not fitted:
        return
    xs = np.array([g["e"] for g in gcps])
    ys = np.array([g["n"] for g in gcps])
    pad = max(xs.max() - xs.min(), ys.max() - ys.min(), 50) * 0.06
    ext = (xs.min() - pad, xs.max() + pad, ys.min() - pad, ys.max() + pad)
    w, h = ext[1] - ext[0], ext[3] - ext[2]
    fig, ax = plt.subplots(figsize=(11, max(5, min(15, 11 * h / w))), dpi=200)
    if args.ortho:
        try:
            import rasterio
            from rasterio.windows import from_bounds
            with rasterio.open(args.ortho) as src:
                win = from_bounds(ext[0], ext[2], ext[1], ext[3], src.transform)
                scale = max(win.width, win.height) / 3000.0
                shape = (max(1, int(win.height / max(scale, 1))), max(1, int(win.width / max(scale, 1))))
                bands = [1, 2, 3] if src.count >= 3 else [1]
                img = src.read(bands, window=win, out_shape=(len(bands),) + shape, boundless=True, fill_value=0)
            img = np.moveaxis(img, 0, -1)
            if img.shape[2] == 1:
                ax.imshow(img[:, :, 0], extent=ext, cmap="gray", alpha=0.75)
            else:
                ax.imshow(img, extent=ext, alpha=0.75)
        except Exception as e:  # picture is a nice-to-have
            print("Ortho background skipped: %s" % e)
    cm_ = np.array([g["dz"] for g in fitted]) * 100
    lim = args.plot_range or max(1.0, float(np.ceil(np.percentile(np.abs(cm_), 95))))
    sc = ax.scatter([g["e"] for g in fitted], [g["n"] for g in fitted], c=cm_, cmap="RdBu_r",
                    vmin=-lim, vmax=lim, s=70, edgecolors="black", linewidths=0.8, zorder=3)
    bad = [g for g in fitted if g["flags"]]
    if bad:
        ax.scatter([g["e"] for g in bad], [g["n"] for g in bad], s=190, facecolors="none",
                   edgecolors="#d08700", linewidths=1.4, zorder=4, label="flagged")
    none = [g for g in gcps if g["dz"] is None]
    if none:
        ax.scatter([g["e"] for g in none], [g["n"] for g in none], marker="x", c="black", s=50,
                   zorder=3, label="no LiDAR result")
    if not args.no_labels:
        for g in gcps:
            ax.annotate(g["name"] if g["dz"] is None else "%s  %+.1f" % (g["name"], g["dz"] * 100),
                        (g["e"], g["n"]), xytext=(6, 5), textcoords="offset points", fontsize=6,
                        bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.75), zorder=5)
    cb = fig.colorbar(sc, ax=ax, shrink=0.75, pad=0.02)
    cb.set_label("LiDAR minus surveyed height (cm)")
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect("equal")
    ax.ticklabel_format(useOffset=False, style="plain")
    ax.tick_params(labelsize=7)
    ax.set_xlabel("Easting (m)")
    ax.set_ylabel("Northing (m)")
    ax.set_title("LiDAR check points: n = %d, mean %+.3f m, RMSE %.3f m  (EPSG:%d)"
                 % (s_all["count"], s_all["mean"], s_all["rmse"], epsg), fontsize=10)
    if bad or none:
        ax.legend(loc="lower right", fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "gcp_check.png"))
    plt.close(fig)


# --------------------------------------------------------------------- main

def build_parser():
    ap = argparse.ArgumentParser(description="Check a LiDAR point cloud against surveyed ground points.")
    ap.add_argument("--las", nargs="+", required=True, help="LAS / LAZ file(s); wildcards allowed")
    ap.add_argument("--gcp", required=True, help="CSV of surveyed points (Emlid Flow export)")
    ap.add_argument("--out", default="gcp_check", help="output folder (default gcp_check)")
    ap.add_argument("--epsg", type=int, help="EPSG code of the LiDAR and the CSV eastings/northings "
                                             "(read from the LAS header when it has one)")
    ap.add_argument("--patch", type=float, default=1.5, help="side of the square around each point, m (default 1.5)")
    ap.add_argument("--classes", help="LiDAR classes to use, e.g. 2 for ground or 2,8 (default all)")
    ap.add_argument("--min-points", type=int, default=8, help="fewest points for a plane (default 8)")
    ap.add_argument("--max-slope", type=float, default=10.0, help="flag STEEP above this slope, degrees (default 10)")
    ap.add_argument("--max-rms", type=float, default=0.05, help="flag ROUGH above this plane RMS, m (default 0.05)")
    ap.add_argument("--outlier-min", type=float, default=0.05,
                    help="never flag OUTLIER closer than this to the median, m (default 0.05)")
    ap.add_argument("--z-field", help='height column in the CSV (default "Elevation")')
    ap.add_argument("--name-field")
    ap.add_argument("--e-field")
    ap.add_argument("--n-field")
    ap.add_argument("--only", help="keep only points whose name or code contains one of these (comma separated)")
    ap.add_argument("--exclude", help="drop points whose name contains one of these (comma separated)")
    ap.add_argument("--gcp-geographic-epsg", type=int, default=7844,
                    help="EPSG of longitude/latitude when the CSV has no eastings (default 7844, GDA2020)")
    ap.add_argument("--ortho", help="GeoTIFF in the same CRS to draw behind the picture (needs rasterio)")
    ap.add_argument("--plot-range", type=float, help="colour scale limit for the picture, cm")
    ap.add_argument("--no-labels", action="store_true", help="no point labels on the picture")
    ap.add_argument("--max-patch-points", type=int, default=2000,
                    help="most LiDAR points written per GCP to the patch_points layer (default 2000)")
    ap.add_argument("--chunk", type=int, default=2_000_000, help="points read at a time (default 2,000,000)")
    return ap


def run(args):
    las_paths = []
    for p in args.las:
        hits = sorted(glob.glob(p))
        las_paths += hits if hits else [p]
    missing = [p for p in las_paths if not os.path.isfile(p)]
    if missing:
        sys.exit("File not found: " + ", ".join(missing))
    classes = None
    if args.classes:
        classes = np.array([int(v) for v in args.classes.replace(" ", "").split(",") if v != ""], dtype=np.uint8)

    gcps = read_gcps(args.gcp, args)
    if not gcps:
        sys.exit("No usable points in the GCP file.")
    print("Reading LiDAR (%d file%s), patch %.2f m square%s" % (
        len(las_paths), "" if len(las_paths) == 1 else "s", args.patch,
        "" if classes is None else ", classes " + args.classes))
    patches, las_epsg, n_las = collect(las_paths, gcps, args.patch / 2.0, classes, args.chunk)
    epsg = args.epsg or las_epsg
    if not epsg:
        sys.exit("The LAS header has no coordinate system. Add --epsg (e.g. --epsg 7854).")
    if args.epsg and las_epsg and args.epsg != las_epsg:
        print("! The LAS header says EPSG:%d but --epsg %d was given. Using %d." % (las_epsg, args.epsg, args.epsg))
    for i, g in enumerate(gcps):
        g["_patch"] = patches.get(i)

    fitted = analyse(gcps, patches, args)
    s_all = stats([g["dz"] for g in fitted])
    s_ok = stats([g["dz"] for g in fitted if not g["flags"]])
    os.makedirs(args.out, exist_ok=True)
    rows = summary_rows(s_all, s_ok, gcps, args, epsg, n_las)
    write_csvs(args.out, gcps)
    print()
    print(write_summary(args.out, rows, gcps))
    write_gpkg(os.path.join(args.out, "gcp_check.gpkg"), epsg, gcps, rows, args.patch / 2.0, args.max_patch_points)
    if s_all:
        write_plot(args.out, gcps, epsg, s_all, args)
        if abs(s_all["median"]) > 3.0:
            print("! The median difference is %.1f m. That is a height-datum mismatch, not an accuracy\n"
                  "  result: most likely one side is ellipsoidal and the other AHD. Try --z-field\n"
                  "  \"Ellipsoidal height\" or check which heights the cloud was exported in." % s_all["median"])
    else:
        print("! No GCP had enough LiDAR points. Check the coordinate system (--epsg), that the CSV\n"
              "  eastings/northings match the cloud, and the --classes filter.")
    print("Written to " + os.path.abspath(args.out))


# ---------------------------------------------------------------------- GUI

def gui():
    import queue
    import threading
    import tkinter as tk
    from tkinter import filedialog, ttk

    root = tk.Tk()
    root.title("Brolga Tech - LiDAR check against surveyed points")
    root.minsize(760, 560)
    frm = ttk.Frame(root, padding=12)
    frm.pack(fill="both", expand=True)
    frm.columnconfigure(1, weight=1)
    las_files = []
    v = {k: tk.StringVar(value=d) for k, d in [
        ("las", ""), ("gcp", ""), ("out", ""), ("ortho", ""), ("epsg", "7854"), ("patch", "1.5"),
        ("classes", "2"), ("zfield", "Elevation"), ("only", ""), ("exclude", "")]}
    ground = tk.BooleanVar(value=True)

    def row(r, label, key, browse=None, hint=None):
        ttk.Label(frm, text=label).grid(row=r, column=0, sticky="w", pady=3)
        e = ttk.Entry(frm, textvariable=v[key])
        e.grid(row=r, column=1, sticky="ew", padx=6)
        if browse:
            ttk.Button(frm, text="Browse...", command=browse).grid(row=r, column=2)
        if hint:
            ttk.Label(frm, text=hint, foreground="#666").grid(row=r, column=3, sticky="w", padx=6)
        return e

    def pick_las():
        fs = filedialog.askopenfilenames(title="LiDAR point cloud(s)",
                                         filetypes=[("LAS / LAZ", "*.las *.laz"), ("All files", "*.*")])
        if fs:
            las_files[:] = list(fs)
            v["las"].set(fs[0] if len(fs) == 1 else "%d files: %s ..." % (len(fs), os.path.basename(fs[0])))

    def pick_gcp():
        fn = filedialog.askopenfilename(title="Surveyed points (Emlid Flow CSV)",
                                        filetypes=[("CSV", "*.csv *.txt"), ("All files", "*.*")])
        if fn:
            v["gcp"].set(fn)
            if not v["out"].get():
                v["out"].set(os.path.join(os.path.dirname(fn), "gcp_check"))

    def pick_out():
        d = filedialog.askdirectory(title="Output folder")
        if d:
            v["out"].set(d)

    def pick_ortho():
        fn = filedialog.askopenfilename(title="Ortho for the picture (optional)",
                                        filetypes=[("GeoTIFF", "*.tif *.tiff"), ("All files", "*.*")])
        if fn:
            v["ortho"].set(fn)

    las_entry = row(0, "LiDAR cloud (LAS / LAZ)", "las", pick_las)
    las_entry.configure(state="readonly")
    row(1, "Surveyed points (CSV)", "gcp", pick_gcp)
    row(2, "Output folder", "out", pick_out)
    row(3, "Ortho behind the picture", "ortho", pick_ortho, "optional")
    opts = ttk.Frame(frm)
    opts.grid(row=4, column=0, columnspan=4, sticky="w", pady=(10, 2))

    def opt(label, key, width, col):
        ttk.Label(opts, text=label).grid(row=0, column=col, sticky="w", padx=(0, 4))
        ttk.Entry(opts, textvariable=v[key], width=width).grid(row=0, column=col + 1, padx=(0, 16))

    opt("EPSG", "epsg", 7, 0)
    opt("Patch size (m)", "patch", 5, 2)
    opt("Height column", "zfield", 18, 4)
    ttk.Checkbutton(opts, text="Only LiDAR classes:", variable=ground).grid(row=0, column=6)
    ttk.Entry(opts, textvariable=v["classes"], width=6).grid(row=0, column=7, padx=4)
    flt = ttk.Frame(frm)
    flt.grid(row=5, column=0, columnspan=4, sticky="ew", pady=2)
    ttk.Label(flt, text="Only points named like").grid(row=0, column=0, padx=(0, 4))
    ttk.Entry(flt, textvariable=v["only"], width=26).grid(row=0, column=1, padx=(0, 16))
    ttk.Label(flt, text="Leave out points named like").grid(row=0, column=2, padx=(0, 4))
    ttk.Entry(flt, textvariable=v["exclude"], width=26).grid(row=0, column=3)
    ttk.Label(flt, text="(comma separated)", foreground="#666").grid(row=0, column=4, padx=6)

    btns = ttk.Frame(frm)
    btns.grid(row=6, column=0, columnspan=4, sticky="w", pady=8)
    go = ttk.Button(btns, text="Run check")
    go.grid(row=0, column=0)
    openb = ttk.Button(btns, text="Open output folder", state="disabled")
    openb.grid(row=0, column=1, padx=8)
    log = tk.Text(frm, height=20, wrap="none", font=("Consolas", 9))
    log.grid(row=7, column=0, columnspan=4, sticky="nsew")
    frm.rowconfigure(7, weight=1)
    sb = ttk.Scrollbar(frm, command=log.yview)
    sb.grid(row=7, column=4, sticky="ns")
    log.configure(yscrollcommand=sb.set)
    q = queue.Queue()

    class Out:
        def write(self, s):
            q.put(s)

        def flush(self):
            pass

    def pump():
        try:
            while True:
                s = q.get_nowait()
                if s is None:
                    go.configure(state="normal")
                    if os.path.isdir(v["out"].get()):
                        openb.configure(state="normal")
                    continue
                for tok in re.split(r"(\r|\n)", s):
                    if tok == "\r":                    # progress: overwrite the current line
                        log.delete("end-1c linestart", "end-1c")
                    elif tok:
                        log.insert("end", tok)
                log.see("end")
        except queue.Empty:
            pass
        root.after(80, pump)

    def work(argv):
        old = sys.stdout, sys.stderr
        sys.stdout = sys.stderr = Out()
        try:
            run(build_parser().parse_args(argv))
        except SystemExit as e:
            if e.code not in (None, 0):
                print("\nSTOPPED: %s" % e.code)
        except Exception:
            import traceback
            print("\nSomething went wrong:\n" + traceback.format_exc())
        finally:
            sys.stdout, sys.stderr = old
            q.put(None)

    def start():
        log.delete("1.0", "end")
        if not las_files or not v["gcp"].get():
            log.insert("end", "Choose the LiDAR cloud and the CSV of surveyed points first.\n")
            return
        out = v["out"].get() or os.path.join(os.path.dirname(v["gcp"].get()), "gcp_check")
        v["out"].set(out)
        argv = ["--las"] + las_files + ["--gcp", v["gcp"].get(), "--out", out, "--patch", v["patch"].get() or "1.5"]
        if v["epsg"].get().strip():
            argv += ["--epsg", v["epsg"].get().strip()]
        if ground.get() and v["classes"].get().strip():
            argv += ["--classes", v["classes"].get().strip()]
        if v["zfield"].get().strip():
            argv += ["--z-field", v["zfield"].get().strip()]
        for key, flag in (("ortho", "--ortho"), ("only", "--only"), ("exclude", "--exclude")):
            if v[key].get().strip():
                argv += [flag, v[key].get().strip()]
        go.configure(state="disabled")
        openb.configure(state="disabled")
        threading.Thread(target=work, args=(argv,), daemon=True).start()

    def open_out():
        d = v["out"].get()
        if hasattr(os, "startfile"):
            os.startfile(d)
        else:
            import subprocess
            subprocess.Popen(["xdg-open", d])

    go.configure(command=start)
    openb.configure(command=open_out)
    pump()
    root.mainloop()


def main():
    if len(sys.argv) == 1:          # double-clicked, or run with nothing after the name
        gui()
    else:
        run(build_parser().parse_args())


if __name__ == "__main__":
    main()
