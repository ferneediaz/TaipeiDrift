"""Temporary debug script for SIFT+LightGlue zero-match issue (delete after use)."""

import os
import sys
import warnings

os.environ.setdefault(
    "TORCH_HOME",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data/raw/models/torch_hub"),
)
warnings.filterwarnings("ignore")

import kornia.feature as KF
import rasterio
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import r_matchers as R  # noqa: E402

q = rasterio.open(R.QUERY_TIF)
qc = R._read_gray_window(q, 217000.0, 2661000.0, 160)
gq = R.to_gray(qc)
m = R._kornia_feature("sift")
l, rr, d = m(R._chw_gray(gq))
k = (rr[0] > 0).nonzero().flatten()
laf, desc = l[:, k], d[:, k]

params = {"depth_confidence": -1, "width_confidence": -1}
lg_neg = KF.LightGlueMatcher("sift", dict(params, filter_threshold=-1.0)).eval()
lg_def = KF.LightGlueMatcher("sift", params).eval()
for tag, lg in (("th=-1", lg_neg), ("default", lg_def)):
    with torch.inference_mode():
        dist, idx = lg(desc, desc, laf, laf, (160, 160), (160, 160))
    print(tag, "matches", idx.shape[0], "dists[:4]", dist.flatten()[:4].tolist())

# inspect raw scores inside forward path
kp = KF.get_laf_center(laf)
img = {
    "keypoints": kp,
    "scales": KF.get_laf_scale(laf).reshape(1, -1),
    "oris": KF.get_laf_orientation(laf).reshape(1, -1),
    "lafs": laf,
    "descriptors": desc,
    "image_size": torch.tensor([[160, 160]]),
}
with torch.inference_mode():
    out = lg_def.matcher({"image0": dict(img), "image1": dict(img)})
print("scores max", out["matching_scores0"].max().item(), "matches0>-1", int((out["matches0"] > -1).sum()))
