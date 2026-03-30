// dda_raycast.wgsl
// Grid-tree (voxel octree) DDA raycast.
// Included/imported by the main pipeline shader.
//
// Binding (group 2):
//   @binding(0)  grid_tree_buf : array<u32>   (packed, all trees back-to-back)
//
// Tree layout at byte offset `tree_base` (== BVHItem::item_index):
//   [+0 .. +8)  : i16 root_x, i16 root_y, i16 root_z, i16 pad
//   [+8 ..)     : GpuGridTreeNode[]
//
// GpuGridTreeNode = 132 bytes:
//   u16 parent_offset | u8 used_cell_count | u8 depth | u16[64] contents
//
// Cell encoding:
//   0xFFFF             → NONE
//   bit15 == 0         → DATA   (value = raw & 0x7FFF)
//   bit15 == 1, ≠ MAX  → NODE   (child offset = raw & 0x7FFF)
//
// LOG_SIZE=2, SIZE=4, SIZE³=64 cells per node.

const DDA_LOG_SIZE:          u32 = 2u;
const DDA_SIZE:              u32 = 4u;
const DDA_NODE_STRIDE_BYTES: u32 = 132u;
const DDA_NODE_HEADER_BYTES: u32 = 4u;
const DDA_MAX_STEPS:         u32 = 4096u;
const DDA_ROOT_BYTES:        u32 = 8u;
const DDA_PALETTE_LEN_BYTES: u32 = 4u;
const DDA_COLOR_BYTES:       u32 = 4u; // rgba u8

@group(2) @binding(0) var<storage, read> grid_tree_buf: array<u32>;

// ── Raw buffer accessors ──────────────────────────────────────────────────────

fn dda_u16(byte_off: u32) -> u32 {
    return (grid_tree_buf[byte_off / 4u] >> ((byte_off % 4u) * 8u)) & 0xFFFFu;
}
fn dda_u8(byte_off: u32) -> u32 {
    return (grid_tree_buf[byte_off / 4u] >> ((byte_off % 4u) * 8u)) & 0xFFu;
}

// ── Node accessors ────────────────────────────────────────────────────────────

fn dda_node_off(tree_base: u32, node_index: u32) -> u32 {
    return dda_nodes_base(tree_base) + node_index * DDA_NODE_STRIDE_BYTES;
}
fn dda_node_depth(tree_base: u32, n: u32) -> u32 {
    return dda_u8(dda_node_off(tree_base, n) + 3u);
}
fn dda_node_parent_offset(tree_base: u32, n: u32) -> u32 {
    return dda_u16(dda_node_off(tree_base, n) + 0u);
}
fn dda_node_sz(depth: u32)  -> u32 { return 1u << (DDA_LOG_SIZE * (depth + 1u)); }
fn dda_child_sz(depth: u32) -> u32 { return 1u << (DDA_LOG_SIZE * depth); }
fn dda_cell_raw(tree_base: u32, n: u32, ci: u32) -> u32 {
    return dda_u16(dda_node_off(tree_base, n) + DDA_NODE_HEADER_BYTES + ci * 2u);
}
fn dda_cell_type(raw: u32) -> u32 {
    if raw == 0xFFFFu { return 0u; }
    return 1u + (raw >> 15u);
}
fn dda_cell_val(raw: u32) -> u32 { return raw & 0x7FFFu; }
fn dda_ci3(p: vec3<u32>) -> u32 {
    return p.x + p.y * DDA_SIZE + p.z * DDA_SIZE * DDA_SIZE;
}

fn dda_read_root_pos(tree_base: u32) -> vec3<i32> {
    let w0 = grid_tree_buf[tree_base / 4u];
    let w1 = grid_tree_buf[(tree_base + 4u) / 4u];
    var rx = i32(w0 & 0xFFFFu);          if (rx & 0x8000) != 0 { rx -= 0x10000; }
    var ry = i32((w0 >> 16u) & 0xFFFFu); if (ry & 0x8000) != 0 { ry -= 0x10000; }
    var rz = i32(w1 & 0xFFFFu);          if (rz & 0x8000) != 0 { rz -= 0x10000; }
    return vec3<i32>(rx, ry, rz);
}
fn dda_palette_len(tree_base: u32) -> u32 {
    return dda_u8(tree_base + DDA_ROOT_BYTES);
}
fn dda_nodes_base(tree_base: u32) -> u32 {
    let len = dda_palette_len(tree_base);
    return tree_base
        + DDA_ROOT_BYTES
        + DDA_PALETTE_LEN_BYTES
        + len * DDA_COLOR_BYTES;
}
fn dda_palette_color(tree_base: u32, index: u32) -> vec4<f32> {
    let palette_base = tree_base + DDA_ROOT_BYTES + DDA_PALETTE_LEN_BYTES;

    let byte_off = palette_base + index * DDA_COLOR_BYTES;

    let r = f32(dda_u8(byte_off + 0u)) / 255.0;
    let g = f32(dda_u8(byte_off + 1u)) / 255.0;
    let b = f32(dda_u8(byte_off + 2u)) / 255.0;
    let a = f32(dda_u8(byte_off + 3u)) / 255.0;

    return vec4<f32>(r, g, b, a);
}

// ── DDA result ────────────────────────────────────────────────────────────────

struct DDAHit {
    hit:    bool,
    value:  u32,
    normal: vec3<f32>,
    dist:   f32,
}

// ── Grid-tree DDA raycast ─────────────────────────────────────────────────────
//
// Direct port of GridTree::raycast() in Rust.
// Variable names match the Rust source exactly.
// The main loop mirrors the Rust structure: a single flat loop where
// NODE cells descend inline, DATA returns immediately, and NONE cells
// perform a per-axis DDA step with inline tree walk-up — matching the
// three separate match arms (axis 0, 1, 2) in the Rust NONE branch.

fn dda_raycast(
    ray_pos:   vec3<f32>,   // Rust: origin
    ray_dir:   vec3<f32>,   // Rust: dir
    max_dist:  f32,         // Rust: max_length
    tree_base: u32,
) -> DDAHit {
    var no_hit: DDAHit;
    no_hit.hit = false;

    let root_i    = dda_read_root_pos(tree_base);
    let root_f    = vec3<f32>(root_i);
    let d0        = dda_node_depth(tree_base, 0u);
    let root_sz   = dda_node_sz(d0);
    let root_sz_f = f32(root_sz);

    // ── Ray vs root AABB ──────────────────────────────────────────────────────
    // Rust: ray_aabb_intersection — slab test, returns distance_to_aabb.
    let inv   = vec3<f32>(1.0) / ray_dir;
    let t1    = (root_f             - ray_pos) * inv;
    let t2    = (root_f + root_sz_f - ray_pos) * inv;
    let tmin  = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
    let tmax  = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
    if tmin > tmax || tmax < 0.0 { return no_hit; }

    let distance_to_aabb = max(tmin, 0.0);
    if distance_to_aabb > max_dist { return no_hit; }

    // ── post_aabb_origin: entry point clamped into root, then nudged ──────────
    // Rust:
    //   let post_aabb_origin = (origin + dir * distance_to_aabb)
    //       .min(root_pos + root.size() - 0.00001).max(root_pos);
    //   let post_aabb_origin = post_aabb_origin
    //       .move_towards(post_aabb_origin.floor() + 0.5, 0.001);
    let post_aabb_origin_unclamped = ray_pos + ray_dir * distance_to_aabb;
    let post_aabb_origin_clamped   = clamp(
        post_aabb_origin_unclamped,
        root_f,
        root_f + vec3<f32>(root_sz_f - 0.00001),
    );
    // move_towards(target, max_delta): move each component toward floor+0.5 by
    // at most 0.001, without overshooting. In practice the nudge is always the
    // full 0.001 because we're at a slab boundary far from the cell centre.
    let post_aabb_origin_nudge_target = floor(post_aabb_origin_clamped) + vec3<f32>(0.5);
    let post_aabb_origin_nudge_delta  = post_aabb_origin_nudge_target - post_aabb_origin_clamped;
    let post_aabb_origin_nudge_len    = length(post_aabb_origin_nudge_delta);
    let post_aabb_origin = select(
        post_aabb_origin_clamped + normalize(post_aabb_origin_nudge_delta) * min(post_aabb_origin_nudge_len, 0.001),
        post_aabb_origin_clamped,                 // fallback when delta == 0 (already centred)
        post_aabb_origin_nudge_len < 0.00001,
    );

    // ── root_relative_post_aabb_origin and derived DDA locals ─────────────────
    // Rust: let root_relative_post_aabb_origin = post_aabb_origin - root_pos.as_vec3();
    let root_relative_post_aabb_origin = post_aabb_origin - root_f;

    // Rust: let delta = dir.recip().abs();
    let delta = abs(inv);

    // Rust: let step = dir.signum().as_i8vec3();
    let step = vec3<i32>(
        select(-1, 1, ray_dir.x >= 0.0),
        select(-1, 1, ray_dir.y >= 0.0),
        select(-1, 1, ray_dir.z >= 0.0),
    );

    // Rust: axis_distances = dir.recip() * (ceil_or_floor(rr) - rr)
    // where ceil is used for positive step, floor for negative.
    let axis_distances_first_boundary = vec3<f32>(
        select(floor(root_relative_post_aabb_origin.x), ceil(root_relative_post_aabb_origin.x), step.x > 0),
        select(floor(root_relative_post_aabb_origin.y), ceil(root_relative_post_aabb_origin.y), step.y > 0),
        select(floor(root_relative_post_aabb_origin.z), ceil(root_relative_post_aabb_origin.z), step.z > 0),
    );
    var axis_distances = inv * (axis_distances_first_boundary - root_relative_post_aabb_origin);

    // Rust: let mut root_relative_grid_pos = root_relative_post_aabb_origin.as_u16vec3();
    // (floor-truncate to integer voxel index)
    var root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_post_aabb_origin));

    // Rust: let mut last_step_axis = dir.abs().max_position() as u8;
    let abs_dir = abs(ray_dir);
    var last_step_axis: u32;
    if abs_dir.x >= abs_dir.y && abs_dir.x >= abs_dir.z { last_step_axis = 0u; }
    else if abs_dir.y >= abs_dir.z                       { last_step_axis = 1u; }
    else                                                  { last_step_axis = 2u; }

    var current_node_index: u32 = 0u;
    var last_distance = distance_to_aabb;

    // ── Main loop ─────────────────────────────────────────────────────────────
    // Mirrors the Rust loop structure exactly:
    //   - Compute node_relative_grid_pos and contents_pos inline each iteration.
    //   - Match on cell type:
    //       NODE (2) → current_node_index += cell.1   (inline descent, no sub-loop)
    //       DATA (1) → return hit
    //       NONE (0) → per-axis match arm, walk up tree, advance grid pos
    for (var _step: u32 = 0u; _step < DDA_MAX_STEPS; _step++) {

        let current_node_depth    = dda_node_depth(tree_base, current_node_index);
        let current_node_size     = dda_node_sz(current_node_depth);
        let current_node_child_sz = dda_child_sz(current_node_depth);

        // Rust: let node_relative_grid_pos = root_relative_grid_pos % current_node.size();
        // node_size is always a power of two, so modulo == bitwise AND.
        let current_node_size_mask   = current_node_size - 1u;
        let node_relative_grid_pos   = root_relative_grid_pos & vec3<u32>(current_node_size_mask);

        // Rust: let contents_pos = (node_relative_grid_pos / current_node.child_size()).as_u8vec3();
        let contents_pos = node_relative_grid_pos / vec3<u32>(current_node_child_sz);
        let ci           = dda_ci3(contents_pos);
        let raw          = dda_cell_raw(tree_base, current_node_index, ci);
        let cell_type    = dda_cell_type(raw);
        let cell_value   = dda_cell_val(raw);

        if cell_type == 2u {
            // NODE → descend inline (Rust: current_node_index += cell.1 as u32)
            current_node_index += cell_value;

        } else if cell_type == 1u {
            // DATA → hit
            // Rust: return Some((pos, -step[last_step_axis] * I8Vec3::AXES[last_step_axis], last_distance))
            var normal = vec3<f32>(0.0);
            normal[last_step_axis] = -f32(step[last_step_axis]);
            var result: DDAHit;
            result.hit    = true;
            result.value  = cell_value;
            result.normal = normal;
            result.dist   = last_distance;
            return result;

        } else {
            // NONE → DDA step.
            // Rust has three separate match arms, one per axis, each with an
            // inline tree walk-up loop. We replicate that structure here.

            // Find the minimum axis (Rust: axis_distances.min_position()).
            var min_axis: u32;
            if axis_distances.x <= axis_distances.y && axis_distances.x <= axis_distances.z { min_axis = 0u; }
            else if axis_distances.y <= axis_distances.z                                     { min_axis = 1u; }
            else                                                                              { min_axis = 2u; }

            // All three arms share the same structure — we branch on min_axis
            // to keep the variable reads/writes identical to the Rust match arms.

            if min_axis == 0u {
                // ── Axis 0 (X) ────────────────────────────────────────────────
                last_distance = distance_to_aabb + axis_distances.x;
                if last_distance > max_dist { return no_hit; }

                let step_amount = step.x;
                let root_relative_grid_pos_x_signed = i32(root_relative_grid_pos.x) + step_amount;
                if root_relative_grid_pos_x_signed < 0 || u32(root_relative_grid_pos_x_signed) >= root_sz { return no_hit; }
                let root_relative_grid_pos_x_new = u32(root_relative_grid_pos_x_signed);

                // Walk up until current_node's size covers both old and new X.
                // Rust: loop { if old_x / node.size() == new_x / node.size() { break; }
                //              current_node_index -= parent_offset; }
                loop {
                    let walk_up_node_size = dda_node_sz(dda_node_depth(tree_base, current_node_index));
                    if root_relative_grid_pos.x / walk_up_node_size == root_relative_grid_pos_x_new / walk_up_node_size { break; }
                    let walk_up_parent_offset = dda_node_parent_offset(tree_base, current_node_index);
                    if walk_up_parent_offset == 0u { break; }
                    current_node_index -= walk_up_parent_offset;
                }

                root_relative_grid_pos.x = root_relative_grid_pos_x_new;
                axis_distances.x += delta.x;
                last_step_axis = 0u;

            } else if min_axis == 1u {
                // ── Axis 1 (Y) ────────────────────────────────────────────────
                last_distance = distance_to_aabb + axis_distances.y;
                if last_distance > max_dist { return no_hit; }

                let step_amount = step.y;
                let root_relative_grid_pos_y_signed = i32(root_relative_grid_pos.y) + step_amount;
                if root_relative_grid_pos_y_signed < 0 || u32(root_relative_grid_pos_y_signed) >= root_sz { return no_hit; }
                let root_relative_grid_pos_y_new = u32(root_relative_grid_pos_y_signed);

                loop {
                    let walk_up_node_size = dda_node_sz(dda_node_depth(tree_base, current_node_index));
                    if root_relative_grid_pos.y / walk_up_node_size == root_relative_grid_pos_y_new / walk_up_node_size { break; }
                    let walk_up_parent_offset = dda_node_parent_offset(tree_base, current_node_index);
                    if walk_up_parent_offset == 0u { break; }
                    current_node_index -= walk_up_parent_offset;
                }

                root_relative_grid_pos.y = root_relative_grid_pos_y_new;
                axis_distances.y += delta.y;
                last_step_axis = 1u;

            } else {
                // ── Axis 2 (Z) ────────────────────────────────────────────────
                last_distance = distance_to_aabb + axis_distances.z;
                if last_distance > max_dist { return no_hit; }

                let step_amount = step.z;
                let root_relative_grid_pos_z_signed = i32(root_relative_grid_pos.z) + step_amount;
                if root_relative_grid_pos_z_signed < 0 || u32(root_relative_grid_pos_z_signed) >= root_sz { return no_hit; }
                let root_relative_grid_pos_z_new = u32(root_relative_grid_pos_z_signed);

                loop {
                    let walk_up_node_size = dda_node_sz(dda_node_depth(tree_base, current_node_index));
                    if root_relative_grid_pos.z / walk_up_node_size == root_relative_grid_pos_z_new / walk_up_node_size { break; }
                    let walk_up_parent_offset = dda_node_parent_offset(tree_base, current_node_index);
                    if walk_up_parent_offset == 0u { break; }
                    current_node_index -= walk_up_parent_offset;
                }

                root_relative_grid_pos.z = root_relative_grid_pos_z_new;
                axis_distances.z += delta.z;
                last_step_axis = 2u;
            }
        }
    }

    return no_hit;
}
