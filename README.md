# gltf2md5

A standalone CLI tool that converts glTF 2.0 skeletal (or static) meshes and
animations to Doom 3 / idTech4 MD5 format. Supports the original MD5v10 spec
and the MD5v12 extension (per-vertex normals + MikkTSpace tangents) used by
StormEngine2.

Companion to `fbx2md5`, intended for pipelines that route through Cascadeur,
Akeytsu, or other animation tools where the FBX path causes too much grief
(node-transform scale shenanigans, tangent inconsistencies, axis convention
drift, take-name parsing). glTF avoids all of those.

## Why glTF over FBX

| Issue with FBX path | glTF behaviour |
| --- | --- |
| Blender's exporter bakes a 100× scale into root node transform | No equivalent — units are explicit |
| Tangents may or may not be present; algorithm unspecified | Spec §3.7.2.1 mandates MikkTSpace if `TANGENT` present |
| `ArmatureName\|ActionName` stack-name pipe pattern | Plain animation names |
| Multiple takes packed into stacks | One animation = one named clip |
| Axis convention varies per exporter | Y-up by spec |

If your DCC tool strips tangents on re-export (Cascadeur, Akeytsu, several
glTF round-trippers), the converter recomputes them with the canonical
MikkTSpace reference implementation. This is the **same algorithm** that
Substance Painter, Blender, Unity, and Unreal all use for tangent space, so
bake-time and runtime tangents are guaranteed to agree by construction.

## Build

### Windows / Visual Studio 2022

Run `generate_vs2022.bat` from the project root. It produces
`build\gltf2md5.sln`, opens it, and on Release build drops the binary at
`bin\Release\gltf2md5.exe`.

Prerequisites: CMake 3.15+ on PATH, Visual Studio 2022 with the
"Desktop development with C++" workload.

### Command line (any platform with CMake)

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### Direct compile (no CMake)

```
g++ -std=c++17 -O2 src/gltf2md5.cpp src/mikktspace.c -o gltf2md5 -lm
```

MSVC equivalent:

```
cl /std:c++17 /O2 /EHsc src/gltf2md5.cpp src/mikktspace.c
```

## Usage

```
gltf2md5 INPUT.{gltf,glb} OUTPUT_STEM [options]
```

Options:

| Flag | Effect |
| --- | --- |
| `-scale F` | Multiply all positions and translations by F (default 1.0). glTF is unitless-meters; if your engine works in inches, try `-scale 39.37`. |
| `-fps N` | Animation framerate (default 24). Resamples sampler keyframes at this rate. |
| `-v12` | Write MD5v12 with per-vertex normals and tangents (MikkTSpace). Required for the StormEngine2 v12 interaction shader path. |
| `-noaxes` | Skip the default Y-up -> Z-up conversion. Use when your engine expects glTF-native Y-up. |
| `-noAnimPrefix` | Don't prepend the output stem to per-animation filenames. |
| `-shader NAME` | Override the shader name written into every mesh block. |

### Example

```
gltf2md5 boot.glb out/boot -v12 -scale 39.37 -fps 24
```

Produces:

```
out/boot.md5mesh
out/boot_idle.md5anim     (one per animation in the glTF)
```

## Workflow note

The pipeline this tool is built around:

1. Model + rig in Blender, export glTF with `+Tangents` enabled.
2. Import to your animation tool (Cascadeur, Akeytsu, etc.), animate.
3. Export the rigged + animated mesh back out as glTF.
4. Bake normal maps in Substance Painter using **the same glTF file** you'll
   convert in step 5. Project tangent space must be set to **MikkTSpace**
   ("Compute tangent space per fragment" must be **off**).
5. `gltf2md5 file.glb output -v12 [options]`.

The bake-mesh and convert-mesh being the same file is the critical invariant.
If you bake against File A and convert File B, MikkTSpace will produce
different sign conventions and UV-seam shading will not match.

## License

MIT (see `LICENSE`). Bundles cgltf (MIT) and MikkTSpace (zlib). Third-party
notices reproduced in `LICENSE`.
