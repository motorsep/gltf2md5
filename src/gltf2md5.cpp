// gltf2md5.cpp -- glTF 2.0 (.gltf/.glb) -> idTech4 / StormEngine2 MD5 converter
//
// Mirrors fbx2md5's CLI and output exactly, but takes glTF as input instead of FBX.
// Drop next to cgltf.h, build with any modern C++17 compiler.
//
//   g++ -std=c++17 -O2 gltf2md5.cpp -o gltf2md5 -lm
//   gltf2md5.exe input.glb output_stem [-scale N] [-fps N] [-v12] [-noaxes] [-noAnimPrefix]
//
// Why glTF: tangents are spec-mandated MikkTSpace, no FBX node-transform games,
// unit-clean. The blender FBX exporter's 100x scale embed and tangent
// inconsistencies that caused the v12 seam bug just don't exist here.

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

// MikkTSpace (canonical reference impl by Morten Mikkelsen, zlib license).
// Included so we can recompute tangents when the source glTF lacks them —
// e.g. Cascadeur, Akeytsu, and other animation tools tend to strip TANGENT
// on re-export. Since Substance Painter also bakes against MikkTSpace, the
// computed tangents WILL match the bake basis by construction.
#include "mikktspace.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

// ---------------------------------------------------------------- math types

struct Vec2 { float x, y; };
struct Vec3 { float x, y, z; };
struct Vec4 { float x, y, z, w; };
struct Quat { float x, y, z, w; };   // (x,y,z) is imaginary, w is real
struct Mat4 { float m[16]; };        // column-major (matches glTF + cgltf)

static Vec3 V3(float x, float y, float z) { return Vec3{x,y,z}; }
static Vec3 NormalizeV3(Vec3 v) {
    float L = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (L > 1e-12f) { v.x/=L; v.y/=L; v.z/=L; }
    return v;
}
static float DotV3(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
[[maybe_unused]] static Vec3 CrossV3(Vec3 a, Vec3 b) {
    return V3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static Vec3 SubV3(Vec3 a, Vec3 b) { return V3(a.x-b.x, a.y-b.y, a.z-b.z); }
static Vec3 ScaleV3(Vec3 v, float s) { return V3(v.x*s, v.y*s, v.z*s); }

// Identity matrix
static Mat4 Mat4Identity() {
    Mat4 m{};
    m.m[0]=1; m.m[5]=1; m.m[10]=1; m.m[15]=1;
    return m;
}

// Mat4 * Mat4 (column-major)
static Mat4 MatMul(const Mat4 &A, const Mat4 &B) {
    Mat4 r{};
    for (int c = 0; c < 4; c++)
    for (int rr = 0; rr < 4; rr++) {
        float s = 0;
        for (int k = 0; k < 4; k++) s += A.m[k*4+rr] * B.m[c*4+k];
        r.m[c*4+rr] = s;
    }
    return r;
}

// Transform a direction (no translation) by Mat4's 3x3 rotation+scale part
static Vec3 MatTransformDir(const Mat4 &M, Vec3 v) {
    return V3(M.m[0]*v.x + M.m[4]*v.y + M.m[8]*v.z,
              M.m[1]*v.x + M.m[5]*v.y + M.m[9]*v.z,
              M.m[2]*v.x + M.m[6]*v.y + M.m[10]*v.z);
}

// Transform a point (with translation) by Mat4
static Vec3 MatTransformPoint(const Mat4 &M, Vec3 v) {
    return V3(M.m[0]*v.x + M.m[4]*v.y + M.m[8]*v.z + M.m[12],
              M.m[1]*v.x + M.m[5]*v.y + M.m[9]*v.z + M.m[13],
              M.m[2]*v.x + M.m[6]*v.y + M.m[10]*v.z + M.m[14]);
}

// Invert an affine matrix (rotation + translation, no perspective)
static Mat4 InvertAffine(const Mat4 &M) {
    Mat4 r = M;
    // Transpose 3x3 rotation block
    std::swap(r.m[1], r.m[4]);
    std::swap(r.m[2], r.m[8]);
    std::swap(r.m[6], r.m[9]);
    // Re-translate: t' = -R^T * t
    float tx = M.m[12], ty = M.m[13], tz = M.m[14];
    r.m[12] = -(r.m[0]*tx + r.m[4]*ty + r.m[8]*tz);
    r.m[13] = -(r.m[1]*tx + r.m[5]*ty + r.m[9]*tz);
    r.m[14] = -(r.m[2]*tx + r.m[6]*ty + r.m[10]*tz);
    return r;
}

// Quaternion -> rotation matrix (column-major), Hamilton convention
static Mat4 MatFromQuat(Quat q) {
    Mat4 m = Mat4Identity();
    float xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
    float xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
    float wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;
    m.m[0]  = 1 - 2*(yy + zz);
    m.m[1]  = 2*(xy + wz);
    m.m[2]  = 2*(xz - wy);
    m.m[4]  = 2*(xy - wz);
    m.m[5]  = 1 - 2*(xx + zz);
    m.m[6]  = 2*(yz + wx);
    m.m[8]  = 2*(xz + wy);
    m.m[9]  = 2*(yz - wx);
    m.m[10] = 1 - 2*(xx + yy);
    return m;
}

static Mat4 MatFromTRS(Vec3 t, Quat q, Vec3 s) {
    Mat4 R = MatFromQuat(q);
    // Apply scale to rotation columns
    R.m[0]*=s.x; R.m[1]*=s.x; R.m[2]*=s.x;
    R.m[4]*=s.y; R.m[5]*=s.y; R.m[6]*=s.y;
    R.m[8]*=s.z; R.m[9]*=s.z; R.m[10]*=s.z;
    R.m[12]=t.x; R.m[13]=t.y; R.m[14]=t.z;
    return R;
}

// Slerp two quats. Used for animation sampling at inter-keyframe times.
static Quat SlerpQuat(Quat a, Quat b, float t) {
    float cosO = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
    if (cosO < 0) { b.x=-b.x; b.y=-b.y; b.z=-b.z; b.w=-b.w; cosO=-cosO; }
    float sA, sB;
    if (cosO > 0.9995f) { sA = 1-t; sB = t; }
    else {
        float O = std::acos(cosO), sO = std::sin(O);
        sA = std::sin((1-t)*O) / sO; sB = std::sin(t*O) / sO;
    }
    return Quat{ a.x*sA + b.x*sB, a.y*sA + b.y*sB, a.z*sA + b.z*sB, a.w*sA + b.w*sB };
}
static Quat NormalizeQuat(Quat q) {
    float L = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (L > 1e-12f) { q.x/=L; q.y/=L; q.z/=L; q.w/=L; }
    return q;
}

// ---------------------------------------------------------------- MD5 helpers

struct CQuat { float x, y, z; };  // MD5-compressed quaternion (w reconstructed)

// Compress a quat to MD5 stored form. idTech4 uses the convention where
// w >= 0 (stored quat is non-negative-real-part). The MD5 ToMat3() routine
// applies the same Hamilton-rotation formula but operating on row vectors,
// which is equivalent to using the conjugate vs the column-vector formula.
// Hence we *conjugate* on write and decompress.
static CQuat CompressQuat(Quat q) {
    q = NormalizeQuat(q);
    // ensure w >= 0
    if (q.w < 0) { q.x=-q.x; q.y=-q.y; q.z=-q.z; q.w=-q.w; }
    // conjugate (column->row vector convention) - same as fbx2md5
    return CQuat{ -q.x, -q.y, -q.z };
}
[[maybe_unused]] static Quat DecompressQuat(CQuat c) {
    float t = 1.0f - c.x*c.x - c.y*c.y - c.z*c.z;
    float w = (t < 0) ? 0.0f : std::sqrt(t);
    return Quat{ -c.x, -c.y, -c.z, w };  // undo conjugate
}

// ---------------------------------------------------------------- glTF accessors helpers

// Read a single vec3 from a cgltf accessor.
static Vec3 ReadVec3(const cgltf_accessor *acc, size_t i) {
    float v[3] = {0,0,0};
    cgltf_accessor_read_float(acc, i, v, 3);
    return V3(v[0], v[1], v[2]);
}
static Vec4 ReadVec4(const cgltf_accessor *acc, size_t i) {
    float v[4] = {0,0,0,0};
    cgltf_accessor_read_float(acc, i, v, 4);
    return Vec4{v[0], v[1], v[2], v[3]};
}
static Vec2 ReadVec2(const cgltf_accessor *acc, size_t i) {
    float v[2] = {0,0};
    cgltf_accessor_read_float(acc, i, v, 2);
    return Vec2{v[0], v[1]};
}
static Mat4 ReadMat4(const cgltf_accessor *acc, size_t i) {
    float v[16]; cgltf_accessor_read_float(acc, i, v, 16);
    Mat4 m; std::memcpy(m.m, v, sizeof(v));
    return m;
}

// Find an attribute by type within a primitive.
static const cgltf_attribute *FindAttr(const cgltf_primitive *prim, cgltf_attribute_type t, int idx=0) {
    for (size_t i = 0; i < prim->attributes_count; i++) {
        if (prim->attributes[i].type == t && prim->attributes[i].index == idx)
            return &prim->attributes[i];
    }
    return nullptr;
}

// ---------------------------------------------------------------- joint table

struct ExportJoint {
    std::string name;
    int         parent;        // index into joints[] of parent (-1 for root)
    Vec3        local_t;       // local translation (relative to parent)
    Quat        local_r;       // local rotation
    Vec3        local_s;       // local scale (almost always (1,1,1))
    Mat4        bind_to_world; // joint's world transform at bind pose
    Mat4        inv_bind;      // == inverse(bind_to_world); from glTF IBM
};

// Resolve a joint's node-to-world by walking parent links, since glTF stores
// local TRS only and we want the world bind matrix. We *could* read IBM and
// invert it (and we do — see BuildJoints), but we also want the parent chain
// for emitting hierarchical joint indices.
static Mat4 NodeLocalMatrix(const cgltf_node *n) {
    if (n->has_matrix) {
        Mat4 m; std::memcpy(m.m, n->matrix, sizeof(n->matrix));
        return m;
    }
    Vec3 t = n->has_translation ? V3(n->translation[0], n->translation[1], n->translation[2]) : V3(0,0,0);
    Quat r = n->has_rotation ? Quat{n->rotation[0], n->rotation[1], n->rotation[2], n->rotation[3]} : Quat{0,0,0,1};
    Vec3 s = n->has_scale ? V3(n->scale[0], n->scale[1], n->scale[2]) : V3(1,1,1);
    return MatFromTRS(t, r, s);
}

// Build the joints[] list from a skin. cgltf orders joints by skin->joints[]
// (the canonical order), which is what we write into the MD5.
static std::vector<ExportJoint> BuildJoints(const cgltf_skin *skin) {
    std::vector<ExportJoint> out;
    out.reserve(skin->joints_count);
    // First, build a node* -> joint-index map so we can resolve parents.
    auto find_joint_index = [&](const cgltf_node *n) -> int {
        for (size_t i = 0; i < skin->joints_count; i++)
            if (skin->joints[i] == n) return (int)i;
        return -1;
    };
    for (size_t i = 0; i < skin->joints_count; i++) {
        cgltf_node *jn = skin->joints[i];
        ExportJoint j;
        j.name = jn->name ? jn->name : ("joint_" + std::to_string(i));
        j.parent = jn->parent ? find_joint_index(jn->parent) : -1;
        j.local_t = jn->has_translation ? V3(jn->translation[0], jn->translation[1], jn->translation[2]) : V3(0,0,0);
        j.local_r = jn->has_rotation ? Quat{jn->rotation[0], jn->rotation[1], jn->rotation[2], jn->rotation[3]} : Quat{0,0,0,1};
        j.local_s = jn->has_scale ? V3(jn->scale[0], jn->scale[1], jn->scale[2]) : V3(1,1,1);
        // Read inverse bind matrix (or fall back to walking parents)
        if (skin->inverse_bind_matrices) {
            j.inv_bind = ReadMat4(skin->inverse_bind_matrices, i);
            j.bind_to_world = InvertAffine(j.inv_bind);
        } else {
            // Walk parent chain to compute bind_to_world
            j.bind_to_world = Mat4Identity();
            for (cgltf_node *cur = jn; cur; cur = cur->parent) {
                j.bind_to_world = MatMul(NodeLocalMatrix(cur), j.bind_to_world);
            }
            j.inv_bind = InvertAffine(j.bind_to_world);
        }
        out.push_back(std::move(j));
    }
    return out;
}

// ---------------------------------------------------------------- axis conversion

// glTF is Y-up by spec; idTech4 is Z-up. Apply the rotation:
//   (x, y, z)_gltf -> (x, -z, y)_idtech
// = rotate -90° about X
// Returns the rotation as a Mat4 we can pre-multiply into joint/world transforms.
static Mat4 AxesGltfToIdtech() {
    Mat4 m = Mat4Identity();
    m.m[0]=1; m.m[1]=0;  m.m[2]=0;
    m.m[4]=0; m.m[5]=0;  m.m[6]=1;
    m.m[8]=0; m.m[9]=-1; m.m[10]=0;
    return m;
}

// ---------------------------------------------------------------- export mesh

struct VertWeight { int joint_index; float weight; };  // up to 4 per vert

struct ExportVert {
    Vec2 uv;
    int  first_weight;
    int  num_weights;
    Vec3 stored_normal;   // bone-local (relative to dominant joint)
    Vec4 stored_tangent;  // xyz=bone-local tangent, w=bitangent sign
};

struct ExportTri { int v0, v1, v2; };

struct ExportWeight {
    int   joint_index;
    float weight;
    Vec3  offset;     // position in dominant joint's local space
};

struct ExportMesh {
    std::string shader;
    std::vector<ExportVert>   verts;
    std::vector<ExportTri>    tris;
    std::vector<ExportWeight> weights;
    bool has_colors = false;  // (not used yet — glTF COLOR_0 if you want)
};

// ---------------------------------------------------------------- MikkTSpace fallback

// Per-corner MikkT context: reads raw glTF accessors, writes per-vertex tangents.
// glTF data is already split at UV seams (one vertex per N/UV pair), so when
// multiple corners reference the same vertex, MikkT computes the same tangent
// at each — last-write-wins is safe.
struct MikkContext {
    const cgltf_accessor *pos;
    const cgltf_accessor *nrm;
    const cgltf_accessor *uv;
    const cgltf_accessor *idx;       // may be null for non-indexed
    size_t num_verts;
    std::vector<Vec4> *out_tangents; // filled by setTSpaceBasic
};

static uint32_t mk_corner_to_vert(MikkContext *c, int iFace, int iVert) {
    size_t corner = (size_t)iFace * 3 + (size_t)iVert;
    if (c->idx) {
        uint32_t v = 0;
        cgltf_accessor_read_uint(c->idx, corner, &v, 1);
        return v;
    }
    return (uint32_t)corner;
}
static int mk_getNumFaces(const SMikkTSpaceContext *ctx) {
    MikkContext *c = (MikkContext*)ctx->m_pUserData;
    return c->idx ? (int)(c->idx->count / 3) : (int)(c->num_verts / 3);
}
static int mk_getNumVerticesOfFace(const SMikkTSpaceContext *ctx, const int iFace) {
    (void)ctx; (void)iFace; return 3;
}
static void mk_getPosition(const SMikkTSpaceContext *ctx, float out[], const int iFace, const int iVert) {
    MikkContext *c = (MikkContext*)ctx->m_pUserData;
    cgltf_accessor_read_float(c->pos, mk_corner_to_vert(c, iFace, iVert), out, 3);
}
static void mk_getNormal(const SMikkTSpaceContext *ctx, float out[], const int iFace, const int iVert) {
    MikkContext *c = (MikkContext*)ctx->m_pUserData;
    cgltf_accessor_read_float(c->nrm, mk_corner_to_vert(c, iFace, iVert), out, 3);
}
static void mk_getTexCoord(const SMikkTSpaceContext *ctx, float out[], const int iFace, const int iVert) {
    MikkContext *c = (MikkContext*)ctx->m_pUserData;
    cgltf_accessor_read_float(c->uv, mk_corner_to_vert(c, iFace, iVert), out, 2);
}
static void mk_setTSpaceBasic(const SMikkTSpaceContext *ctx, const float t[], const float sign, const int iFace, const int iVert) {
    MikkContext *c = (MikkContext*)ctx->m_pUserData;
    uint32_t v = mk_corner_to_vert(c, iFace, iVert);
    if (v < c->out_tangents->size()) {
        (*c->out_tangents)[v] = Vec4{ t[0], t[1], t[2], sign };
    }
}

// Compute MikkTSpace tangents for a primitive whose source has no TANGENT.
// Operates on raw mesh-local positions/normals/UVs from glTF accessors — no
// transforms applied. The caller (BuildExportMesh) will then transform these
// tangents through the same g2w + bone matrices as the source-provided ones.
static std::vector<Vec4> ComputeMikkTangents(const cgltf_primitive *prim) {
    const cgltf_attribute *aPos = FindAttr(prim, cgltf_attribute_type_position);
    const cgltf_attribute *aNrm = FindAttr(prim, cgltf_attribute_type_normal);
    const cgltf_attribute *aUV  = FindAttr(prim, cgltf_attribute_type_texcoord, 0);
    if (!aPos || !aNrm || !aUV) {
        std::fprintf(stderr, "  ERROR: cannot compute MikkTSpace without POSITION+NORMAL+TEXCOORD_0\n");
        return {};
    }
    size_t n = aPos->data->count;
    std::vector<Vec4> tangents(n, Vec4{1, 0, 0, 1});

    MikkContext mctx;
    mctx.pos = aPos->data;
    mctx.nrm = aNrm->data;
    mctx.uv  = aUV->data;
    mctx.idx = prim->indices;
    mctx.num_verts = n;
    mctx.out_tangents = &tangents;

    SMikkTSpaceInterface iface = {};
    iface.m_getNumFaces          = mk_getNumFaces;
    iface.m_getNumVerticesOfFace = mk_getNumVerticesOfFace;
    iface.m_getPosition          = mk_getPosition;
    iface.m_getNormal            = mk_getNormal;
    iface.m_getTexCoord          = mk_getTexCoord;
    iface.m_setTSpaceBasic       = mk_setTSpaceBasic;

    SMikkTSpaceContext ctx;
    ctx.m_pInterface = &iface;
    ctx.m_pUserData = &mctx;

    if (!genTangSpaceDefault(&ctx)) {
        std::fprintf(stderr, "  ERROR: MikkTSpace genTangSpaceDefault failed\n");
        return {};
    }
    return tangents;
}

// Build the ExportMesh from one primitive. This is the core conversion.
// Per-vertex: pick dominant joint, transform N/T to that joint's local space,
// Gram-Schmidt, emit. Weights are emitted per (vertex, joint) pair.
static bool BuildExportMesh(const cgltf_primitive *prim,
                            const std::vector<ExportJoint> &joints,
                            const Mat4 &mesh_world,           // mesh node's world tx
                            const Mat4 &axes,                 // identity or axis-conv
                            float scale,
                            bool flip_v,
                            ExportMesh &out)
{
    const cgltf_attribute *aPos = FindAttr(prim, cgltf_attribute_type_position);
    const cgltf_attribute *aNrm = FindAttr(prim, cgltf_attribute_type_normal);
    const cgltf_attribute *aTan = FindAttr(prim, cgltf_attribute_type_tangent);
    const cgltf_attribute *aUV  = FindAttr(prim, cgltf_attribute_type_texcoord, 0);
    const cgltf_attribute *aJnt = FindAttr(prim, cgltf_attribute_type_joints,   0);
    const cgltf_attribute *aWgt = FindAttr(prim, cgltf_attribute_type_weights,  0);
    if (!aPos) { std::fprintf(stderr, "  ERROR: no POSITION attribute\n"); return false; }
    if (!aUV)  { std::fprintf(stderr, "  ERROR: no TEXCOORD_0 attribute\n"); return false; }
    if (!aNrm) { std::fprintf(stderr, "  ERROR: no NORMAL attribute (run Blender 'Compute normals' on export)\n"); return false; }

    // If TANGENT is missing, compute MikkTSpace ourselves. Result matches what
    // Substance Painter / Blender / glTF spec all use for bake tangents.
    std::vector<Vec4> computed_tangents;
    bool tangents_are_computed = false;
    if (!aTan) {
        std::fprintf(stderr, "  INFO: source has no TANGENT — computing MikkTSpace from positions/normals/UVs\n");
        computed_tangents = ComputeMikkTangents(prim);
        if (computed_tangents.empty()) {
            std::fprintf(stderr, "  ERROR: failed to compute tangents\n");
            return false;
        }
        tangents_are_computed = true;
        std::fprintf(stderr, "  INFO: MikkTSpace produced %zu tangents\n", computed_tangents.size());
    }

    size_t n = aPos->data->count;
    out.verts.resize(n);

    // Pre-compute the geometry-to-world transform (axes * mesh_world).
    Mat4 g2w = MatMul(axes, mesh_world);

    bool have_skin = (aJnt && aWgt && !joints.empty());

    int total_weights_emitted = 0;
    for (size_t v = 0; v < n; v++) {
        // ---- Read raw per-vertex data
        Vec3 pos_geom = ReadVec3(aPos->data, v);
        Vec3 nrm_geom = aNrm ? ReadVec3(aNrm->data, v) : V3(0, 0, 1);
        Vec4 tan_geom = tangents_are_computed
            ? computed_tangents[v]
            : (aTan ? ReadVec4(aTan->data, v) : Vec4{1, 0, 0, 1});
        Vec2 uv       = ReadVec2(aUV->data, v);
        if (flip_v) uv.y = 1.0f - uv.y;

        // ---- Joint/weight reading (glTF JOINTS_0 is uvec4, WEIGHTS_0 is vec4)
        int   jidx[4] = {0,0,0,0};
        float jwgt[4] = {0,0,0,0};
        if (have_skin) {
            uint32_t jraw[4] = {0,0,0,0};
            cgltf_accessor_read_uint(aJnt->data, v, jraw, 4);
            for (int k = 0; k < 4; k++) jidx[k] = (int)jraw[k];
            float wraw[4] = {0,0,0,0};
            cgltf_accessor_read_float(aWgt->data, v, wraw, 4);
            for (int k = 0; k < 4; k++) jwgt[k] = wraw[k];
        } else {
            // No skin: bind everything to joint 0 with weight 1
            jidx[0] = 0; jwgt[0] = 1.0f;
        }

        // Normalize weights (glTF allows un-normalized, but Blender exports normalized)
        float wsum = jwgt[0]+jwgt[1]+jwgt[2]+jwgt[3];
        if (wsum > 1e-9f) for (int k=0; k<4; k++) jwgt[k] /= wsum;

        // ---- Dominant joint = max weight
        int dom = 0;
        for (int k = 1; k < 4; k++) if (jwgt[k] > jwgt[dom]) dom = k;
        int dom_joint = jidx[dom];
        if (dom_joint < 0 || dom_joint >= (int)joints.size()) dom_joint = 0;

        const ExportJoint &dj = (have_skin && dom_joint < (int)joints.size())
            ? joints[dom_joint]
            : (joints.empty() ? ExportJoint{} : joints[0]);

        // ---- World-space attributes
        Vec3 pos_w = MatTransformPoint(g2w, ScaleV3(pos_geom, scale));
        Vec3 nrm_w = NormalizeV3(MatTransformDir(g2w, nrm_geom));
        Vec3 tan_w = NormalizeV3(MatTransformDir(g2w, V3(tan_geom.x, tan_geom.y, tan_geom.z)));

        // ---- Transform to dominant joint's local space.
        // For positions: pos_bone = inv_bind * pos_world
        // For directions (normal, tangent): same matrix, no translation.
        Mat4 ib = dj.inv_bind;
        Vec3 pos_bone = MatTransformPoint(ib, pos_w);
        Vec3 nrm_bone = NormalizeV3(MatTransformDir(ib, nrm_w));
        Vec3 tan_bone = NormalizeV3(MatTransformDir(ib, tan_w));

        // ---- Gram-Schmidt: re-orthogonalize T against N in bone-local space
        // (handles minor non-orthogonality from float drift / bone scale)
        float n_dot_t = DotV3(nrm_bone, tan_bone);
        tan_bone = NormalizeV3(SubV3(tan_bone, ScaleV3(nrm_bone, n_dot_t)));

        out.verts[v].uv = uv;
        out.verts[v].stored_normal = nrm_bone;
        out.verts[v].stored_tangent = Vec4{tan_bone.x, tan_bone.y, tan_bone.z, tan_geom.w};  // preserve bitangent sign
        out.verts[v].first_weight = total_weights_emitted;

        // ---- Emit one weight per non-zero joint influence
        int emitted = 0;
        for (int k = 0; k < 4; k++) {
            if (jwgt[k] < 1e-4f) continue;
            int jx = jidx[k];
            if (jx < 0 || jx >= (int)joints.size()) continue;

            // weight offset = inv_bind_jx * pos_world
            Mat4 ibk = joints[jx].inv_bind;
            Vec3 off = MatTransformPoint(ibk, pos_w);

            ExportWeight w;
            w.joint_index = jx;
            w.weight = jwgt[k];
            w.offset = off;
            out.weights.push_back(w);
            emitted++;
        }
        if (emitted == 0) {
            // Safety: emit dummy weight to joint 0 so the MD5 isn't malformed
            ExportWeight w;
            w.joint_index = 0;
            w.weight = 1.0f;
            w.offset = pos_bone;
            out.weights.push_back(w);
            emitted = 1;
        }
        out.verts[v].num_weights = emitted;
        total_weights_emitted += emitted;
    }

    // ---- Triangle indices
    if (prim->indices) {
        size_t ni = prim->indices->count;
        if (ni % 3) { std::fprintf(stderr, "  ERROR: index count %zu not divisible by 3\n", ni); return false; }
        out.tris.reserve(ni / 3);
        for (size_t t = 0; t < ni; t += 3) {
            uint32_t i0, i1, i2;
            cgltf_accessor_read_uint(prim->indices, t,   &i0, 1);
            cgltf_accessor_read_uint(prim->indices, t+1, &i1, 1);
            cgltf_accessor_read_uint(prim->indices, t+2, &i2, 1);
            // MD5 winding matches glTF (CCW front-facing). If you see inverted
            // faces in-engine, swap the (i1, i2) order here.
            out.tris.push_back({(int)i0, (int)i1, (int)i2});
        }
    } else {
        // Non-indexed: 3 verts per tri
        for (size_t t = 0; t < n; t += 3) {
            out.tris.push_back({(int)t, (int)(t+1), (int)(t+2)});
        }
    }

    return true;
}

// ---------------------------------------------------------------- md5 writers

static std::string SanitizeShaderName(const char *src) {
    if (!src) return std::string("default");
    std::string s(src);
    for (auto &c : s) {
        if (c == ' ' || c == '\t' || c == '\\') c = '/';
    }
    return s;
}

static bool WriteMD5Mesh(const char *path,
                         const std::vector<ExportJoint> &joints,
                         const std::vector<ExportMesh> &meshes,
                         bool v12)
{
    FILE *fp = std::fopen(path, "w");
    if (!fp) { std::fprintf(stderr, "ERROR: can't open %s for write\n", path); return false; }

    std::fprintf(fp, "MD5Version %d\n", v12 ? 12 : 10);
    std::fprintf(fp, "commandline \"gltf2md5\"\n\n");
    std::fprintf(fp, "numJoints %zu\n", joints.size());
    std::fprintf(fp, "numMeshes %zu\n\n", meshes.size());

    // Joints
    std::fprintf(fp, "joints {\n");
    for (size_t i = 0; i < joints.size(); i++) {
        const ExportJoint &j = joints[i];
        // Joint position = bind_to_world translation
        float tx = j.bind_to_world.m[12];
        float ty = j.bind_to_world.m[13];
        float tz = j.bind_to_world.m[14];
        // Joint rotation = bind_to_world rotation as quat, then compressed
        // Extract quat from 3x3 rotation part of bind_to_world
        const float *m = j.bind_to_world.m;
        float trace = m[0] + m[5] + m[10];
        Quat q;
        if (trace > 0) {
            float s = 0.5f / std::sqrt(trace + 1.0f);
            q.w = 0.25f / s;
            q.x = (m[6] - m[9]) * s;
            q.y = (m[8] - m[2]) * s;
            q.z = (m[1] - m[4]) * s;
        } else if (m[0] > m[5] && m[0] > m[10]) {
            float s = 2.0f * std::sqrt(1.0f + m[0] - m[5] - m[10]);
            q.w = (m[6] - m[9]) / s;
            q.x = 0.25f * s;
            q.y = (m[4] + m[1]) / s;
            q.z = (m[8] + m[2]) / s;
        } else if (m[5] > m[10]) {
            float s = 2.0f * std::sqrt(1.0f + m[5] - m[0] - m[10]);
            q.w = (m[8] - m[2]) / s;
            q.x = (m[4] + m[1]) / s;
            q.y = 0.25f * s;
            q.z = (m[9] + m[6]) / s;
        } else {
            float s = 2.0f * std::sqrt(1.0f + m[10] - m[0] - m[5]);
            q.w = (m[1] - m[4]) / s;
            q.x = (m[8] + m[2]) / s;
            q.y = (m[9] + m[6]) / s;
            q.z = 0.25f * s;
        }
        CQuat cq = CompressQuat(q);
        const char *parent_name = j.parent >= 0 ? joints[j.parent].name.c_str() : "";
        std::fprintf(fp, "\t\"%s\"\t%d ( %f %f %f ) ( %f %f %f )\t\t// %s\n",
            j.name.c_str(), j.parent, tx, ty, tz, cq.x, cq.y, cq.z, parent_name);
    }
    std::fprintf(fp, "}\n\n");

    // Meshes
    for (const auto &m : meshes) {
        std::fprintf(fp, "mesh {\n");
        std::fprintf(fp, "\tshader \"%s\"\n\n", m.shader.c_str());
        std::fprintf(fp, "\tnumverts %zu\n", m.verts.size());
        for (size_t i = 0; i < m.verts.size(); i++) {
            const auto &v = m.verts[i];
            if (v12) {
                std::fprintf(fp, "\tvert %zu ( %f %f ) %d %d ( %f %f %f ) ( %f %f %f %f )\n",
                    i, v.uv.x, v.uv.y, v.first_weight, v.num_weights,
                    v.stored_normal.x, v.stored_normal.y, v.stored_normal.z,
                    v.stored_tangent.x, v.stored_tangent.y, v.stored_tangent.z, v.stored_tangent.w);
            } else {
                std::fprintf(fp, "\tvert %zu ( %f %f ) %d %d\n",
                    i, v.uv.x, v.uv.y, v.first_weight, v.num_weights);
            }
        }
        std::fprintf(fp, "\n\tnumtris %zu\n", m.tris.size());
        for (size_t i = 0; i < m.tris.size(); i++) {
            std::fprintf(fp, "\ttri %zu %d %d %d\n", i, m.tris[i].v0, m.tris[i].v1, m.tris[i].v2);
        }
        std::fprintf(fp, "\n\tnumweights %zu\n", m.weights.size());
        for (size_t i = 0; i < m.weights.size(); i++) {
            const auto &w = m.weights[i];
            std::fprintf(fp, "\tweight %zu %d %f ( %f %f %f )\n",
                i, w.joint_index, w.weight, w.offset.x, w.offset.y, w.offset.z);
        }
        std::fprintf(fp, "}\n\n");
    }
    std::fclose(fp);
    return true;
}

// ---------------------------------------------------------------- anim sampling

// Sample one animation channel at time t.
// Returns the interpolated value, or `def` if the sampler has no keys covering t.
struct SamplerEval {
    const cgltf_animation_sampler *s;
    int target_path;  // cgltf_animation_path_type
};

static void SampleChannelV3(const cgltf_animation_sampler *s, float t, Vec3 *out) {
    if (!s || s->input->count == 0) return;
    size_t n = s->input->count;
    // Find keyframe indices
    if (t <= 0) { float v[3]; cgltf_accessor_read_float(s->output, 0, v, 3); *out = V3(v[0],v[1],v[2]); return; }
    float t_last; cgltf_accessor_read_float(s->input, n-1, &t_last, 1);
    if (t >= t_last) {
        float v[3]; cgltf_accessor_read_float(s->output, n-1, v, 3);
        *out = V3(v[0],v[1],v[2]); return;
    }
    for (size_t i = 0; i+1 < n; i++) {
        float t0 = 0, t1 = 0;
        cgltf_accessor_read_float(s->input, i, &t0, 1);
        cgltf_accessor_read_float(s->input, i+1, &t1, 1);
        if (t >= t0 && t <= t1) {
            float u = (t1 > t0) ? (t - t0) / (t1 - t0) : 0;
            float a[3], b[3];
            cgltf_accessor_read_float(s->output, i,   a, 3);
            cgltf_accessor_read_float(s->output, i+1, b, 3);
            if (s->interpolation == cgltf_interpolation_type_step) {
                *out = V3(a[0], a[1], a[2]);
            } else {
                *out = V3(a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]), a[2]+u*(b[2]-a[2]));
            }
            return;
        }
    }
}

static void SampleChannelQuat(const cgltf_animation_sampler *s, float t, Quat *out) {
    if (!s || s->input->count == 0) return;
    size_t n = s->input->count;
    if (t <= 0) { float v[4]; cgltf_accessor_read_float(s->output, 0, v, 4); *out = Quat{v[0],v[1],v[2],v[3]}; return; }
    float t_last; cgltf_accessor_read_float(s->input, n-1, &t_last, 1);
    if (t >= t_last) { float v[4]; cgltf_accessor_read_float(s->output, n-1, v, 4); *out = Quat{v[0],v[1],v[2],v[3]}; return; }
    for (size_t i = 0; i+1 < n; i++) {
        float t0 = 0, t1 = 0;
        cgltf_accessor_read_float(s->input, i, &t0, 1);
        cgltf_accessor_read_float(s->input, i+1, &t1, 1);
        if (t >= t0 && t <= t1) {
            float u = (t1 > t0) ? (t - t0) / (t1 - t0) : 0;
            float a[4], b[4];
            cgltf_accessor_read_float(s->output, i,   a, 4);
            cgltf_accessor_read_float(s->output, i+1, b, 4);
            if (s->interpolation == cgltf_interpolation_type_step) {
                *out = Quat{a[0], a[1], a[2], a[3]};
            } else {
                *out = SlerpQuat(Quat{a[0],a[1],a[2],a[3]}, Quat{b[0],b[1],b[2],b[3]}, u);
            }
            return;
        }
    }
}

// Walk the joint chain from root to leaf, accumulating world transforms at time t.
static std::vector<Mat4> EvaluateJointWorldAtTime(
    const cgltf_skin *skin,
    const std::vector<ExportJoint> &joints,
    const cgltf_animation *anim,
    float t,
    const Mat4 &axes)
{
    std::vector<Mat4> world(joints.size());
    for (size_t i = 0; i < joints.size(); i++) {
        // Resolve the local transform at time t for this joint
        Vec3 T = joints[i].local_t;
        Quat R = joints[i].local_r;
        Vec3 S = joints[i].local_s;
        if (anim) {
            cgltf_node *jn = skin->joints[i];
            for (size_t c = 0; c < anim->channels_count; c++) {
                const cgltf_animation_channel *ch = &anim->channels[c];
                if (ch->target_node != jn) continue;
                if (ch->target_path == cgltf_animation_path_type_translation) {
                    SampleChannelV3(ch->sampler, t, &T);
                } else if (ch->target_path == cgltf_animation_path_type_rotation) {
                    SampleChannelQuat(ch->sampler, t, &R);
                    R = NormalizeQuat(R);
                } else if (ch->target_path == cgltf_animation_path_type_scale) {
                    SampleChannelV3(ch->sampler, t, &S);
                }
            }
        }
        Mat4 local = MatFromTRS(T, R, S);
        if (joints[i].parent >= 0) {
            world[i] = MatMul(world[joints[i].parent], local);
        } else {
            world[i] = MatMul(axes, local);  // axes only on the root joints
        }
    }
    return world;
}

// Compute which animation components are NON-CONSTANT across the timeline.
// (6 components per joint: Tx Ty Tz Qx Qy Qz)  The MD5anim hierarchy block
// describes which of these are streamed per frame vs. baseframe-only.
static std::vector<uint32_t> ComputeAnimBits(
    const cgltf_skin *skin,
    const std::vector<ExportJoint> &joints,
    const cgltf_animation *anim,
    int num_frames, float fps,
    const Mat4 &axes)
{
    std::vector<uint32_t> bits(joints.size(), 0);
    if (num_frames < 2) return bits;
    // Get baseline frame 0 transforms
    auto base = EvaluateJointWorldAtTime(skin, joints, anim, 0.0f, axes);
    for (int f = 1; f < num_frames; f++) {
        float t = (float)f / fps;
        auto cur = EvaluateJointWorldAtTime(skin, joints, anim, t, axes);
        for (size_t i = 0; i < joints.size(); i++) {
            // Per-joint deltas vs base. We measure in *local-to-parent* space
            // (= what the MD5anim emits).
            Mat4 base_local, cur_local;
            if (joints[i].parent >= 0) {
                base_local = MatMul(InvertAffine(base[joints[i].parent]), base[i]);
                cur_local  = MatMul(InvertAffine(cur[joints[i].parent]),  cur[i]);
            } else {
                base_local = base[i];
                cur_local  = cur[i];
            }
            float dtx = std::fabs(cur_local.m[12] - base_local.m[12]);
            float dty = std::fabs(cur_local.m[13] - base_local.m[13]);
            float dtz = std::fabs(cur_local.m[14] - base_local.m[14]);
            // For rotation deltas, compare 3x3 element differences (coarse but fine)
            float drx = std::fabs(cur_local.m[1] - base_local.m[1]) + std::fabs(cur_local.m[2] - base_local.m[2]);
            float dry = std::fabs(cur_local.m[4] - base_local.m[4]) + std::fabs(cur_local.m[6] - base_local.m[6]);
            float drz = std::fabs(cur_local.m[8] - base_local.m[8]) + std::fabs(cur_local.m[9] - base_local.m[9]);
            const float EPS = 1e-5f;
            if (dtx > EPS) bits[i] |= 1;
            if (dty > EPS) bits[i] |= 2;
            if (dtz > EPS) bits[i] |= 4;
            if (drx > EPS) bits[i] |= 8;
            if (dry > EPS) bits[i] |= 16;
            if (drz > EPS) bits[i] |= 32;
        }
    }
    return bits;
}

// Helper to extract local TRS from a joint's local-to-parent matrix.
static void DecomposeLocal(const Mat4 &m, Vec3 *T, Quat *R) {
    T->x = m.m[12]; T->y = m.m[13]; T->z = m.m[14];
    // Quat from rotation part (re-use the trace logic from above)
    float trace = m.m[0] + m.m[5] + m.m[10];
    if (trace > 0) {
        float s = 0.5f / std::sqrt(trace + 1.0f);
        R->w = 0.25f / s;
        R->x = (m.m[6] - m.m[9]) * s;
        R->y = (m.m[8] - m.m[2]) * s;
        R->z = (m.m[1] - m.m[4]) * s;
    } else if (m.m[0] > m.m[5] && m.m[0] > m.m[10]) {
        float s = 2.0f * std::sqrt(1.0f + m.m[0] - m.m[5] - m.m[10]);
        R->w = (m.m[6] - m.m[9]) / s;
        R->x = 0.25f * s;
        R->y = (m.m[4] + m.m[1]) / s;
        R->z = (m.m[8] + m.m[2]) / s;
    } else if (m.m[5] > m.m[10]) {
        float s = 2.0f * std::sqrt(1.0f + m.m[5] - m.m[0] - m.m[10]);
        R->w = (m.m[8] - m.m[2]) / s;
        R->x = (m.m[4] + m.m[1]) / s;
        R->y = 0.25f * s;
        R->z = (m.m[9] + m.m[6]) / s;
    } else {
        float s = 2.0f * std::sqrt(1.0f + m.m[10] - m.m[0] - m.m[5]);
        R->w = (m.m[1] - m.m[4]) / s;
        R->x = (m.m[8] + m.m[2]) / s;
        R->y = (m.m[9] + m.m[6]) / s;
        R->z = 0.25f * s;
    }
}

static bool WriteMD5Anim(const char *path,
                         const cgltf_skin *skin,
                         const std::vector<ExportJoint> &joints,
                         const cgltf_animation *anim,
                         int num_frames, float fps,
                         const Mat4 &axes,
                         float scale)
{
    FILE *fp = std::fopen(path, "w");
    if (!fp) { std::fprintf(stderr, "ERROR: can't open %s for write\n", path); return false; }

    auto bits = ComputeAnimBits(skin, joints, anim, num_frames, fps, axes);
    int total_anim_components = 0;
    for (auto b : bits) for (int k = 0; k < 6; k++) if (b & (1u<<k)) total_anim_components++;

    std::fprintf(fp, "MD5Version 10\n");
    std::fprintf(fp, "commandline \"gltf2md5\"\n\n");
    std::fprintf(fp, "numFrames %d\n", num_frames);
    std::fprintf(fp, "numJoints %zu\n", joints.size());
    std::fprintf(fp, "frameRate %d\n", (int)fps);
    std::fprintf(fp, "numAnimatedComponents %d\n\n", total_anim_components);

    // Hierarchy
    std::fprintf(fp, "hierarchy {\n");
    int idx = 0;
    for (size_t i = 0; i < joints.size(); i++) {
        std::fprintf(fp, "\t\"%s\"\t%d %u %d\t//\n",
            joints[i].name.c_str(), joints[i].parent, bits[i], idx);
        for (int k = 0; k < 6; k++) if (bits[i] & (1u<<k)) idx++;
    }
    std::fprintf(fp, "}\n\n");

    // Bounds — compute by min/maxing joint world positions per frame, scaled.
    std::fprintf(fp, "bounds {\n");
    for (int f = 0; f < num_frames; f++) {
        float t = (float)f / fps;
        auto world = EvaluateJointWorldAtTime(skin, joints, anim, t, axes);
        float bx_min=1e30, bx_max=-1e30, by_min=1e30, by_max=-1e30, bz_min=1e30, bz_max=-1e30;
        for (size_t i = 0; i < joints.size(); i++) {
            float x = world[i].m[12] * scale;
            float y = world[i].m[13] * scale;
            float z = world[i].m[14] * scale;
            if (x < bx_min) bx_min = x;
            if (x > bx_max) bx_max = x;
            if (y < by_min) by_min = y;
            if (y > by_max) by_max = y;
            if (z < bz_min) bz_min = z;
            if (z > bz_max) bz_max = z;
        }
        // Inflate bounds a little since joints are just points
        const float pad = 16.0f;
        std::fprintf(fp, "\t( %f %f %f ) ( %f %f %f )\n",
            bx_min-pad, by_min-pad, bz_min-pad, bx_max+pad, by_max+pad, bz_max+pad);
    }
    std::fprintf(fp, "}\n\n");

    // Baseframe (frame 0 local transforms)
    {
        auto world = EvaluateJointWorldAtTime(skin, joints, anim, 0.0f, axes);
        std::fprintf(fp, "baseframe {\n");
        for (size_t i = 0; i < joints.size(); i++) {
            Mat4 local;
            if (joints[i].parent >= 0) {
                local = MatMul(InvertAffine(world[joints[i].parent]), world[i]);
            } else {
                local = world[i];
            }
            Vec3 T; Quat R; DecomposeLocal(local, &T, &R);
            CQuat cq = CompressQuat(R);
            std::fprintf(fp, "\t( %f %f %f ) ( %f %f %f )\n",
                T.x*scale, T.y*scale, T.z*scale, cq.x, cq.y, cq.z);
        }
        std::fprintf(fp, "}\n\n");
    }

    // Frames
    for (int f = 0; f < num_frames; f++) {
        float t = (float)f / fps;
        auto world = EvaluateJointWorldAtTime(skin, joints, anim, t, axes);
        std::fprintf(fp, "frame %d {\n", f);
        for (size_t i = 0; i < joints.size(); i++) {
            uint32_t b = bits[i];
            if (!b) continue;
            Mat4 local;
            if (joints[i].parent >= 0) {
                local = MatMul(InvertAffine(world[joints[i].parent]), world[i]);
            } else {
                local = world[i];
            }
            Vec3 T; Quat R; DecomposeLocal(local, &T, &R);
            CQuat cq = CompressQuat(R);
            std::fprintf(fp, "\t");
            if (b & 1)  std::fprintf(fp, "%f ", T.x*scale);
            if (b & 2)  std::fprintf(fp, "%f ", T.y*scale);
            if (b & 4)  std::fprintf(fp, "%f ", T.z*scale);
            if (b & 8)  std::fprintf(fp, "%f ", cq.x);
            if (b & 16) std::fprintf(fp, "%f ", cq.y);
            if (b & 32) std::fprintf(fp, "%f ", cq.z);
            std::fprintf(fp, "\n");
        }
        std::fprintf(fp, "}\n");
    }

    std::fclose(fp);
    return true;
}

// ---------------------------------------------------------------- main

static void PrintUsage() {
    std::fprintf(stderr,
        "Usage: gltf2md5 INPUT.{gltf,glb} OUTPUT_STEM [options]\n"
        "  -scale F           Multiply all positions and translations by F (default 1.0)\n"
        "                     glTF is unitless-meters; idTech4 wants inches. Try 39.37\n"
        "                     for a 1m-real-size character if your engine uses inches.\n"
        "  -fps N             Animation framerate (default 24)\n"
        "  -v12               Write MD5v12 with per-vertex normals + tangents\n"
        "  -noaxes            Skip glTF Y-up -> idTech4 Z-up conversion\n"
        "  -noAnimPrefix      Don't prepend OUTPUT_STEM to anim filenames\n"
        "  -shader NAME       Override shader name on all meshes\n"
    );
}

int main(int argc, char **argv) {
    if (argc < 3) { PrintUsage(); return 1; }
    const char *in_path  = argv[1];
    const char *out_stem = argv[2];

    float scale = 1.0f;
    float fps   = 24.0f;
    bool  v12   = false;
    bool  noaxes = false;
    bool  no_anim_prefix = false;
    const char *shader_override = nullptr;

    for (int i = 3; i < argc; i++) {
        if      (!std::strcmp(argv[i], "-scale") && i+1 < argc) scale = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "-fps")   && i+1 < argc) fps   = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "-v12"))                  v12 = true;
        else if (!std::strcmp(argv[i], "-noaxes"))               noaxes = true;
        else if (!std::strcmp(argv[i], "-noAnimPrefix"))         no_anim_prefix = true;
        else if (!std::strcmp(argv[i], "-shader") && i+1 < argc) shader_override = argv[++i];
        else { std::fprintf(stderr, "Unknown arg: %s\n", argv[i]); PrintUsage(); return 1; }
    }

    cgltf_options opts = {};
    cgltf_data *data = nullptr;
    cgltf_result r = cgltf_parse_file(&opts, in_path, &data);
    if (r != cgltf_result_success) { std::fprintf(stderr, "ERROR parsing glTF: %d\n", (int)r); return 1; }
    r = cgltf_load_buffers(&opts, data, in_path);
    if (r != cgltf_result_success) { std::fprintf(stderr, "ERROR loading buffers: %d\n", (int)r); cgltf_free(data); return 1; }
    r = cgltf_validate(data);
    if (r != cgltf_result_success) { std::fprintf(stderr, "WARN: validation %d\n", (int)r); }

    std::printf("Loaded: scenes=%zu meshes=%zu skins=%zu anims=%zu\n",
        data->scenes_count, data->meshes_count, data->skins_count, data->animations_count);
    std::printf("Options: scale=%g fps=%g v12=%d noaxes=%d\n", scale, fps, v12, noaxes);

    Mat4 axes = noaxes ? Mat4Identity() : AxesGltfToIdtech();

    // Pick first skin (most rigs have one).
    cgltf_skin *skin = data->skins_count > 0 ? &data->skins[0] : nullptr;
    std::vector<ExportJoint> joints;
    if (skin) {
        joints = BuildJoints(skin);
        // Apply axis conversion to bind_to_world so root joints land in idTech4 space
        for (auto &j : joints) {
            if (j.parent < 0) {
                j.bind_to_world = MatMul(axes, j.bind_to_world);
                j.inv_bind = InvertAffine(j.bind_to_world);
                // also scale the translation
                j.bind_to_world.m[12] *= scale;
                j.bind_to_world.m[13] *= scale;
                j.bind_to_world.m[14] *= scale;
                j.inv_bind = InvertAffine(j.bind_to_world);
            }
        }
        // Propagate to children: rebuild bind_to_world hierarchically
        for (size_t i = 0; i < joints.size(); i++) {
            if (joints[i].parent >= 0) {
                Mat4 local = MatFromTRS(
                    V3(joints[i].local_t.x*scale, joints[i].local_t.y*scale, joints[i].local_t.z*scale),
                    joints[i].local_r, joints[i].local_s);
                joints[i].bind_to_world = MatMul(joints[joints[i].parent].bind_to_world, local);
                joints[i].inv_bind = InvertAffine(joints[i].bind_to_world);
            }
        }
        std::printf("Joints: %zu\n", joints.size());
        for (size_t i = 0; i < joints.size(); i++)
            std::printf("  [%zu] '%s' parent=%d\n", i, joints[i].name.c_str(), joints[i].parent);
    } else {
        // No skin: synthesize a single root joint at origin
        ExportJoint j;
        j.name = "origin";
        j.parent = -1;
        j.local_t = V3(0,0,0); j.local_r = Quat{0,0,0,1}; j.local_s = V3(1,1,1);
        j.bind_to_world = Mat4Identity();
        j.inv_bind = Mat4Identity();
        joints.push_back(j);
    }

    // Build meshes from primitives. Per-primitive becomes a "mesh {}" block.
    std::vector<ExportMesh> meshes;
    for (size_t mi = 0; mi < data->meshes_count; mi++) {
        cgltf_mesh *gm = &data->meshes[mi];
        // Find the node that references this mesh, to get its world transform.
        Mat4 mesh_world = Mat4Identity();
        for (size_t ni = 0; ni < data->nodes_count; ni++) {
            if (data->nodes[ni].mesh == gm) {
                // Walk parent chain to compute world matrix
                Mat4 acc = Mat4Identity();
                for (cgltf_node *cur = &data->nodes[ni]; cur; cur = cur->parent) {
                    acc = MatMul(NodeLocalMatrix(cur), acc);
                }
                mesh_world = acc;
                break;
            }
        }
        for (size_t pi = 0; pi < gm->primitives_count; pi++) {
            cgltf_primitive *prim = &gm->primitives[pi];
            ExportMesh em;
            // Shader name: prefer material name, else mesh name, else "default"
            const char *shader = nullptr;
            if (shader_override) shader = shader_override;
            else if (prim->material && prim->material->name) shader = prim->material->name;
            else if (gm->name) shader = gm->name;
            em.shader = SanitizeShaderName(shader);
            if (!BuildExportMesh(prim, joints, mesh_world, axes, scale, /*flip_v=*/true, em)) {
                std::fprintf(stderr, "ERROR building mesh %zu prim %zu\n", mi, pi);
                cgltf_free(data); return 1;
            }
            std::printf("Mesh '%s' prim %zu: %zu verts, %zu tris, %zu weights\n",
                gm->name ? gm->name : "(no-name)", pi,
                em.verts.size(), em.tris.size(), em.weights.size());
            meshes.push_back(std::move(em));
        }
    }

    if (meshes.empty()) { std::fprintf(stderr, "ERROR: no meshes found\n"); cgltf_free(data); return 1; }

    // Write .md5mesh
    {
        std::string out_path = std::string(out_stem) + ".md5mesh";
        if (!WriteMD5Mesh(out_path.c_str(), joints, meshes, v12)) { cgltf_free(data); return 1; }
        std::printf("Wrote %s\n", out_path.c_str());
    }

    // Write .md5anim for each animation
    for (size_t ai = 0; ai < data->animations_count; ai++) {
        cgltf_animation *an = &data->animations[ai];
        // Determine time range from input samplers
        float t_min = 0, t_max = 0;
        for (size_t s = 0; s < an->samplers_count; s++) {
            cgltf_accessor *in = an->samplers[s].input;
            if (!in) continue;
            float a, b;
            cgltf_accessor_read_float(in, 0, &a, 1);
            cgltf_accessor_read_float(in, in->count-1, &b, 1);
            if (s == 0 || a < t_min) t_min = a;
            if (s == 0 || b > t_max) t_max = b;
        }
        int num_frames = std::max(1, (int)std::round((t_max - t_min) * fps) + 1);
        std::printf("Anim '%s' t=[%.3f, %.3f] -> %d frames at %g fps\n",
            an->name ? an->name : "(no-name)", t_min, t_max, num_frames, fps);

        std::string anim_name = an->name ? an->name : ("anim_" + std::to_string(ai));
        std::string out_path = no_anim_prefix
            ? (anim_name + ".md5anim")
            : (std::string(out_stem) + "_" + anim_name + ".md5anim");
        if (skin) {
            if (!WriteMD5Anim(out_path.c_str(), skin, joints, an, num_frames, fps, axes, scale)) {
                std::fprintf(stderr, "ERROR writing anim %s\n", out_path.c_str());
            } else {
                std::printf("Wrote %s\n", out_path.c_str());
            }
        }
    }

    cgltf_free(data);
    return 0;
}
