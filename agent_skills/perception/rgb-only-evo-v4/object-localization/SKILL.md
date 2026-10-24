---
name: object-localization
description: Select and evaluate RGB perception tools for Evo head and wrist camera trials without assuming depth, towel size, or a table model.
---
# Evo RGB perception

Separate object identity, image boundary measurement, and metric localization.
SAM can provide the object mask; local gradient fitting can refine a visible line;
forward/backward optical flow can check temporal feature correspondence. None alone
provides metric depth or proves cross-camera physical point identity.

For towel trials, inspect the lower visible image edge separately from side-corner
intersections. At a shallow angle, the intersections can vary while the edge is stable.
Do not infer that the lowest image edge is the nearest edge in robot coordinates.

Apply each camera's lens correction once. Keep ROI coordinates in corrected-image
pixels and preserve capture identity. A crop that removes gripper-pad distractors in
one wrist may reduce support in the other. Test regions separately and freeze settings
before collecting a new trial. Report failures as well as variation on accepted fits.

Known local evidence: at one fixed raised pose, the left wrist benefited from excluding
the bottom quarter; the right wrist did better without that crop. This is not a universal
workspace region. Gradient refinement failed on the left wrist and must not be forced.

Metric reconstruction requires validated optical transforms and measured state at each
capture time. Never substitute transferred lens calibration, nominal URDF geometry,
assumed towel size, or a manually guessed table height for that validation.
