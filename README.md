# Half-wave plate data collector: version history

The .ino files are flat inside each category folder. The Arduino IDE wants every sketch in a folder with
its own name, so the first time you open one it will offer to create that folder. Say yes.

The upload removed the original file dates, so the order below follows the file contents and names,
not the timestamps. Treat the order inside a folder as approximate.

## Folders

01_early_prototypes_28BYJ48
  First sketches. 28BYJ-48 stepper through Stepper.h (pins D7-D4), Vector library, photodiodes on A4/A5
  (draftTESTING used A6 and laser pin 0). No CSV output.
  PHYS2210_draftTESTING                          earliest, smallest draft
  PHYS2210_copy_20261026143850                   Oct 26 copy
  motornotworkinglab                             variant of the Oct 26 sketch
  PHYS2210_2026_11_07_workingwithoutbitchooser   Nov 7, thetaRange 40, dark-level variables
  rotate_half_wave_plate                         rotates the plate (wire colours noted in the comments)
  rotate_laser                                   same idea, rotates the laser (laserPosition)

02_split_collectors_28BYJ48
  Five separate collectors on the 28BYJ-48. 01-04 are identical except for two switches
  (USE_AVERAGING and USE_ANGULAR_NOISE). 05 is the manual-plate version.
  01 single / no noise, 02 averaging / no noise, 03 single / noise, 04 averaging / noise,
  05 manual plate / averaging / no noise

03_hardware_tests
  sketch_nov20b   bare DRV8825 test: STEP 6, DIR 5, EN 7, microstep pins M1/M2/M3 = 4/3/2 (1/16 mode)

04_rev3_single_run_NEMA17
  "rev 3" single-file collector, NEMA17 + DRV8825, auto 360-point sweep, one run per reset.
  ZekeCodePlease        STEP 6, DIR 5, EN 7 (a second copy named ZekeCodePleaseV3 was identical and was removed)
  ZekeCodePleaseV2      same as ZekeCodePlease but STEP 5, DIR 4, EN 6
  ZekeCodePleaseNoCalV3 adds M1/M2/M3 pins; calibration values typed over Serial instead of a sweep

05_batch_22runs
  HalfWave_Data_Collector_long_batch_autocal   22 runs (0,10,20,30,40,45 deg noise), one auto sweep
                                               (an identical copy named _1 was removed)

06_V4_per_set_batch
  HalfWave_Data_Collector_V4   edit the SETS table at the top: one row per experiment set
                               (noise SD, runs, message, reps, calibration, averaging, timings)

## Pin summary
  28BYJ-48 era : stepper D7 D6 D5 D4, photodiodes A4 / A5, laser D8
  NEMA17 era   : STEP 6, DIR 5, EN 7 (V2: 5 / 4 / 6), M1/M2/M3 = 4 / 3 / 2, laser D8, photodiodes A4 / A5

## Libraries needed
  Folders 01 and 02 need Stepper.h; folder 01 also needs the Vector library.
  Folders 04 to 06 need nothing extra.
