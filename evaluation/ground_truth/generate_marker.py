#!/usr/bin/env python3
"""Generate a known-good ArUco marker PNG for the overhead ground-truth rig.

Use this script whenever the physical marker needs to be reprinted so the
exact dict+id in `aruco_detector.py` DEFAULTS match the print. It saves
the marker with a descriptive filename and a sidecar `.txt` that records
the dict name, id, pixel size, and git commit — so future reprints are
trivially reproducible.

Example:
    ./generate_marker.py                              # default: DICT_4X4_50 id 0
    ./generate_marker.py --dict 4X4_50 --id 0 --px 1200
"""
import argparse, subprocess, sys, os
import cv2

VALID_DICTS = {
    "4X4_50":   cv2.aruco.DICT_4X4_50,
    "4X4_100":  cv2.aruco.DICT_4X4_100,
    "4X4_250":  cv2.aruco.DICT_4X4_250,
    "4X4_1000": cv2.aruco.DICT_4X4_1000,
    "5X5_50":   cv2.aruco.DICT_5X5_50,
    "6X6_50":   cv2.aruco.DICT_6X6_50,
}

# Max id + 1 per dict, taken from the trailing number in the name.
DICT_SIZE = {name: int(name.split("_")[-1]) for name in VALID_DICTS}

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dict", default="4X4_50", choices=list(VALID_DICTS),
                    help="ArUco dictionary (default: 4X4_50 — smallest = best error correction)")
    ap.add_argument("--id", type=int, default=0, help="marker id (default: 0)")
    ap.add_argument("--px", type=int, default=1200,
                    help="output pixel side (default 1200 = crisp at 10cm printed)")
    ap.add_argument("--out", default="",
                    help="output PNG path (default: aruco_<dict>_id<id>_<px>px.png next to this script)")
    args = ap.parse_args()

    max_id = DICT_SIZE[args.dict] - 1
    if args.id < 0 or args.id > max_id:
        ap.error(f"--id {args.id} out of range for DICT_{args.dict} (valid 0..{max_id})")

    d = cv2.aruco.getPredefinedDictionary(VALID_DICTS[args.dict])
    img = cv2.aruco.generateImageMarker(d, args.id, args.px, borderBits=1)

    here = os.path.dirname(os.path.abspath(__file__))
    out = args.out or os.path.join(here, f"aruco_{args.dict}_id{args.id}_{args.px}px.png")
    cv2.imwrite(out, img)

    try:
        commit = subprocess.check_output(
            ["git", "-C", here, "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        commit = "unknown"

    sidecar = out + ".txt"
    with open(sidecar, "w") as fh:
        fh.write(
            f"aruco marker print spec\n"
            f"-----------------------\n"
            f"dict:     {args.dict} (cv2.aruco.DICT_{args.dict})\n"
            f"id:       {args.id}\n"
            f"side_px:  {args.px}\n"
            f"border:   1 module (OpenCV default; detector relies on this)\n"
            f"polarity: black modules on white background (do NOT invert)\n"
            f"commit:   {commit}\n\n"
            f"Print at 10 cm physical side on matte paper. Keep >= 5 mm white\n"
            f"quiet zone around the marker. Do not laminate (specular glare\n"
            f"kills detection).\n\n"
            f"Detector must match: aruco_detector.py DEFAULTS['marker_id'] = {args.id}\n"
            f"and build_detector() DICT_{args.dict}.\n"
        )
    print(f"wrote {out}")
    print(f"wrote {sidecar}")
    print(f"\nprint spec: dict=DICT_{args.dict}, id={args.id}, black-on-white, 10 cm side.")

if __name__ == "__main__":
    sys.exit(main())
