# F-22 model assets

Source: [FGMEMBERS/F-22](https://github.com/FGMEMBERS/F-22), commit
`da046fba441d9fb513b981905bcee80fd61e6706`.

Original aircraft model by Fabrizio Fracaroli / AeroFab (2010), with
FlightGear / FGUK contributions. The upstream `ReadMe.txt` retains the
original credits and conditions; `COPYING` contains the GPL version 2.
The model and textures are GPL-2.0 assets, separately from the application
code's MIT license. Keep these notices, license and editable source assets
when redistributing the model. `source.json` lists the original files.

`Models/F-22.ac`, the PNG textures and the upstream XML are preserved
unmodified. The AC3D file is the editable mesh source. The application
reads it directly; no opaque converted mesh replaces it. Runtime preparation
triangulates surfaces, transforms axes to X forward / Y right / Z down,
and hides landing gear, internal bays and cockpit instruments for the
flight exterior view. The canopy receives a translucent amber material.

This is a flight simulator visual model, not an engineering CAD model or
a source of aircraft dimensions. It is provided without warranty, as
described in the original license.
