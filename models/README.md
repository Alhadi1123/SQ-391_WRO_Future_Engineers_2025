# Models

Mechanical design files for the **WRO Future Engineers 2025** robot.  
This directory contains laser-cut plates (DXF), 3D-printed parts (STL), and small assembly helpers.

> ▶️ **Building Instructions (YouTube):**  
> [Watch the full assembly video](https://youtu.be/wnCx0N_8Aeo)  

---

## Folder map

```
models/
├── 3d-printed-parts/
│   ├── cameraholder.STL
│   ├── cameraholder2.STL
│   ├── dc holder.STL
│   ├── dc holder2.STL
│   ├── urm_holder_hor.STL
│   ├── urm_holder_ver.STL
│   └── wall.STL
├── CNC-cut-parts/
│   ├── additional.DXF
│   ├── chassis.DXF
│   ├── cooplerlego (6).DXF
│   ├── cooplermotor (6).DXF
│   ├── hexaforrasprebrry (8).DXF
│   ├── layer2.DXF
│   ├── powerboardholder2.DXF
│   └── powerboardholderr.DXF
└── assembly-parts/
```

> **Units:** all drawings are in millimeters (mm).  
> **Scale:** export/print at 1:1 scale.  
> **Default quantity:** `1×` each unless otherwise noted in the filename.

---

## 3D-printed parts

| File | Purpose | Notes / Print hints |
|---|---|---|
| `cameraholder.STL` | Primary camera mount. | Print with the flat face on the bed; consider 3–4 perimeters for screw stiffness. |
| `cameraholder2.STL` | Alternate/extended camera mount. | Use if you need different tilt/offset; same print settings as above. |
| `dc holder.STL` | DC motor holder (variant A). | PETG recommended near motor; 35–40% infill. |
| `dc holder2.STL` | DC motor holder (variant B). | Choose the one that matches your motor body/holes. |
| `urm_holder_hor.STL` | Ultrasonic sensor holder – horizontal. | Mounts URM sensor in a horizontal orientation. |
| `urm_holder_ver.STL` | Ultrasonic sensor holder – vertical. |  Mounts URM sensor in a vertial orientation. |
| `wall.STL` | Spacer/brace between plates. | Print upright; increase walls/perimeters for stiffness. |

**General print recommendations**
- **Material:** PETG (preferred) or PLA.  
- **Layer height:** 0.2 mm.  
- **Perimeters & infill:** 3–4 perimeters, 25–40% infill (Gyroid/Cubic).  
- **Fasteners:** designed around typical **M3** screws + heat-set inserts where applicable.

---

## CNC / Laser-cut parts (DXF)

| File | Purpose | Notes |
|---|---|---|
| `chassis.DXF` | Lower/main plate of the two-layer chassis. | 
| `layer2.DXF` | Upper plate of the chassis. |
| `hexaforrasprebrry (8).DXF` | Hex spacer/adapter for Raspberry Pi mounting (×8 implied). |
| `powerboardholder2.DXF` | Power board mount – v2. |
| `powerboardholderr.DXF` | Power board mount – alt. | 
| `cooplerlego (6).DXF` | LEGO axle coupler outline (×6 implied). | 
| `cooplermotor (6).DXF` | Motor shaft coupler outline (×6 implied). | 
| `additional.DXF` | Extra small brackets/spacers. | 

**Cutting recommendations**
- **Material:** 3–4 mm clear acrylic (PMMA) for rigidity/weight balance.
- **Kerf:** compensate ~0.10–0.20 mm (depends on your cutter).  
- **Holes:** pilot undersized if threads will be tapped; otherwise size for M3 clearance.  

---

## Quick build flow

1. **Cut** `CNC-cut-parts/*.DXF` with 3–4 mm sheet (see kerf note).  
2. **Print** required `3d-printed-parts/*.STL` 
3. **Dry fit** plates + `wall.STL` spacers with M3 hardware.  
4. **Install** drivetrain (DC motor in `dc holder*`, coupler(s), axle/wheels).  
5. **Mount** electronics (Raspberry Pi/ESP32, power board using the holder DXFs).  
6. **Attach** sensors (URM holders; camera holder variant that fits your lens/angle).  
7. **Cable manage** and verify clearances through full steering/suspension travel.

> Detailed steps are demonstrated in the **YouTube build video** linked above.

---
