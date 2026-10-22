# One panel region found by RF-DETR Nano and read by crop OCR.
# This box is not a verified physical button boundary or press point.

# Axis-aligned bounding box in pixels, in the source image frame.
int32 x_min
int32 y_min
int32 x_max
int32 y_max

# Detector confidence in [0, 1] for this panel region.
float32 score

# Decoded panel label, e.g. "3", "12", "LG", "B2", "G".
# Also carries the non-numeric panel glyphs the model knows:
#   "<>" door-open, "><" door-close, "alarm", "star" (lobby), "-", "call".
# Empty when the OCR head returned only padding.
string text

# False when the OCR head reported a non-glyph token ("blur", "hazy", "special"),
# meaning a button was located but its label could not be read. Consumers doing
# floor grounding must treat this as unreadable and refuse to guess, rather than
# acting on `text`.
bool text_readable
