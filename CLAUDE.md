# brolgatech-tools-LiDAR-QA - notes for Claude

Read this first. Keep it updated (newest at the top of the changelog).

## What this is
Desktop Python tools for checking the accuracy of Brolga Tech LiDAR point clouds.
Run on Hamish's Windows processing PC. Not part of the website; nothing here is deployed.

## Workflow (Hamish)
- Local folder `J:\00_AI_Working_Folder\brolgatech-tools-LiDAR-QA` is the source of truth.
- Claude edits files in place in that folder and writes `commit-message.txt` (plain ASCII; first line = subject).
- Hamish double-clicks `push-to-git.bat`: pulls, commits with that message (and deletes it), pushes.
- GitHub: `nitrofruit-flyer/brolgatech-tools-LiDAR-QA` (private). On the first run the bat runs `git init` and links the folder to that repo, which must already exist on GitHub.

## Files
- `lidar_gcp_check.py` - LiDAR height check against surveyed ground points (Emlid Flow CSV).
  `build_parser()` / `run(args)` / `gui()`; no arguments = tkinter window.
- `lidar_gcp_check.bat` - double-click launcher for the window.

## lidar_gcp_check.py
- Needs `pip install laspy[lazrs] numpy`; optional matplotlib, pyproj, rasterio.
- CLI: `python lidar_gcp_check.py --las <las/laz...> --gcp <csv> --epsg 7854 --classes 2 --out gcp_check`
- For each CSV point: LiDAR points in a square (default 1.5 m), plane fit with 3 x MAD outlier
  rejection (2 cm floor), dz = plane height at the point minus surveyed height (+ = LiDAR high).
- LAS read in 2 M point chunks with a grid hash (165 M points in one pass).
- Outputs: gcp_check.gpkg (gcp_check, patch_points PointZ, summary; written with sqlite3, no GDAL),
  gcp_check.csv, gcp_check_report.csv, gcp_check_summary.txt (mean, std, RMSE, 1.96 x RMSE,
  all and unflagged), gcp_check.png (dz colours, optional --ortho background).
- Flags: NO_DATA, FEW_POINTS, STEEP (>10 deg), ROUGH (plane RMS >5 cm), ONE_SIDED, OUTLIER, NOT_FIX.
- Height only: horizontal accuracy cannot be read off flat ground.

## Changelog
### 2026-11-26 - moved into its own repo
Built earlier the same day in a Claude Code session in the website repo
(`brolgatech-website/tools/`, commit d01e19c on main), then copied here as its own repo.
The copy in the website repo can be removed once this repo is pushed.

### 2026-11-26 - first version (Capricorn Copper, 780 ha)
- Tested on a synthetic 3 M point LAS with a known 3 cm offset (recovered 2.7-2.8 cm; planted bad
  point and vegetation flagged); GeoPackage read back with GDAL.
- Worker run with Hamish's real Capricorn Emlid CSV (57 points; Name / Easting / Northing /
  Elevation read as-is) against a synthetic cloud.
- NOT yet run on real LiDAR. The tkinter window is untested.
- Repeated point names get " (2)", " (3)".

## Not built / ideas
- Write results into the survey report (`rp_reports.lidar_accuracy` in the Books Supabase).
- Error ellipses like the Metashape report.
- The field procedure for collecting check points as a controlled document.
- One desktop launcher for all tools (Bathy Studio, raster publisher, LAS tiler, strip alignment,
  this check, plus Metashape / RTKLib / Emlid Studio shortcuts), each kept current with git pull.
