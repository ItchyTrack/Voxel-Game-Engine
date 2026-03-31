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
const DDA_MAX_STEPS:         u32 = 512u;
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
fn dda_node_depth(tree_base: u32, node_index: u32) -> u32 {
    return dda_u8(dda_node_off(tree_base, node_index) + 3u);
}
fn dda_node_parent_offset(tree_base: u32, node_index: u32) -> u32 {
    return dda_u16(dda_node_off(tree_base, node_index) + 0u);
}
fn dda_node_size(depth: u32)  -> u32 { return 1u << (DDA_LOG_SIZE * (depth + 1u)); }
fn dda_child_size(depth: u32) -> u32 { return 1u << (DDA_LOG_SIZE * depth); }
fn dda_cell_raw(tree_base: u32, node_index: u32, contents_index: u32) -> u32 {
    return dda_u16(dda_node_off(tree_base, node_index) + DDA_NODE_HEADER_BYTES + contents_index * 2u);
}
fn dda_cell_type(raw_value: u32) -> u32 {
    if raw_value == 0xFFFFu { return 0u; }
    return 1u + (raw_value >> 15u);
}
fn dda_cell_value(raw_value: u32) -> u32 { return raw_value & 0x7FFFu; }
fn get_child_contents_index(contents_pos: vec3<u32>) -> u32 {
    return contents_pos.x + contents_pos.y * DDA_SIZE + contents_pos.z * DDA_SIZE * DDA_SIZE;
}

fn dda_read_root_pos(tree_base: u32) -> vec3<i32> {
    let word_zero = grid_tree_buf[tree_base / 4u];
    let word_one  = grid_tree_buf[(tree_base + 4u) / 4u];
    var root_x = i32(word_zero & 0xFFFFu);          if (root_x & 0x8000) != 0 { root_x -= 0x10000; }
    var root_y = i32((word_zero >> 16u) & 0xFFFFu); if (root_y & 0x8000) != 0 { root_y -= 0x10000; }
    var root_z = i32(word_one & 0xFFFFu);            if (root_z & 0x8000) != 0 { root_z -= 0x10000; }
    return vec3<i32>(root_x, root_y, root_z);
}
fn dda_palette_len(tree_base: u32) -> u32 {
    return dda_u8(tree_base + DDA_ROOT_BYTES);
}
fn dda_nodes_base(tree_base: u32) -> u32 {
    let palette_length = dda_palette_len(tree_base);
    return tree_base
        + DDA_ROOT_BYTES
        + DDA_PALETTE_LEN_BYTES
        + palette_length * DDA_COLOR_BYTES;
}
fn dda_palette_color(tree_base: u32, palette_index: u32) -> vec4<f32> {
    let palette_base_offset = tree_base + DDA_ROOT_BYTES + DDA_PALETTE_LEN_BYTES;
    let byte_offset = palette_base_offset + palette_index * DDA_COLOR_BYTES;
    let channel_r = f32(dda_u8(byte_offset + 0u)) / 255.0;
    let channel_g = f32(dda_u8(byte_offset + 1u)) / 255.0;
    let channel_b = f32(dda_u8(byte_offset + 2u)) / 255.0;
    let channel_a = f32(dda_u8(byte_offset + 3u)) / 255.0;
    return vec4<f32>(channel_r, channel_g, channel_b, channel_a);
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

fn dda_raycast(
    origin:     vec3<f32>,   // Rust: origin
    dir:        vec3<f32>,   // Rust: dir
    max_length: f32,         // Rust: max_length
    tree_base:  u32,
) -> DDAHit {
    var no_hit: DDAHit;
    no_hit.hit = false;

    // Rust: let root = &self.nodes[0];
    // Rust: let root_min = self.root_pos.as_vec3();
    // Rust: let root_max = root_min + Vec3::splat(root.size() as f32);
    let root_pos_i32  = dda_read_root_pos(tree_base);
    let root_pos      = vec3<f32>(root_pos_i32);
    let root_depth    = dda_node_depth(tree_base, 0u);
    let root_size_u32 = dda_node_size(root_depth);
    let root_size     = f32(root_size_u32);
    let root_min      = root_pos;
    let root_max      = root_pos + vec3<f32>(root_size);

    // ── Ray vs root AABB (Rust: ray_aabb_intersection) ────────────────────────
    let inv_dir = vec3<f32>(1.0) / dir;
    let t1      = (root_min - origin) * inv_dir;
    let t2      = (root_max - origin) * inv_dir;
    let tmin    = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
    let tmax    = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
    if tmin > tmax || tmax < 0.0 { return no_hit; }

    // Rust: let distance_to_aabb = ...
    let distance_to_aabb = max(tmin, 0.0);
    if distance_to_aabb > max_length { return no_hit; }

    // Rust: let post_aabb_origin_pre_shift = origin + dir * distance_to_aabb;
    // Rust: let post_aabb_origin = post_aabb_origin_pre_shift
    //           .min(self.root_pos.as_vec3() + Vec3::splat(((root.size()) as f32) - 0.00001))
    //           .max(self.root_pos.as_vec3());
    // Rust: let post_aabb_origin = post_aabb_origin.move_towards(post_aabb_origin.floor() + 0.5, 0.001);
    let post_aabb_origin_pre_shift = origin + dir * distance_to_aabb;
    let post_aabb_origin_clamped   = clamp(
        post_aabb_origin_pre_shift,
        root_pos,
        root_pos + vec3<f32>(root_size - 0.00001),
    );
    let move_towards_target = floor(post_aabb_origin_clamped) + vec3<f32>(0.5);
    let move_towards_delta  = move_towards_target - post_aabb_origin_clamped;
    let move_towards_length = length(move_towards_delta);
    let post_aabb_origin = select(
        post_aabb_origin_clamped + move_towards_delta / move_towards_length * 0.001,
        post_aabb_origin_clamped,
        move_towards_length < 0.001,
    );

    // Rust: let root_relative_post_aabb_origin = post_aabb_origin - self.root_pos.as_vec3();
    let root_relative_post_aabb_origin = post_aabb_origin - root_pos;

    // Rust: let delta = dir.recip().abs();
    let delta = abs(inv_dir);

    // Rust: let step = dir.signum().as_i8vec3();
    let step = vec3<i32>(
        select(-1, 1, dir.x >= 0.0),
        select(-1, 1, dir.y >= 0.0),
        select(-1, 1, dir.z >= 0.0),
    );

    // Rust: axis_distances = dir.recip() * (ceil_or_floor(rr) - rr)
    let axis_distances_boundary = vec3<f32>(
        select(floor(root_relative_post_aabb_origin.x), ceil(root_relative_post_aabb_origin.x), step.x > 0),
        select(floor(root_relative_post_aabb_origin.y), ceil(root_relative_post_aabb_origin.y), step.y > 0),
        select(floor(root_relative_post_aabb_origin.z), ceil(root_relative_post_aabb_origin.z), step.z > 0),
    );
    var axis_distances = inv_dir * (axis_distances_boundary - root_relative_post_aabb_origin);

    // Rust: let mut root_relative_grid_pos = root_relative_post_aabb_origin.as_u16vec3();
    var root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_post_aabb_origin));

    // Rust: let mut last_step_axis = (post_aabb_origin_pre_shift - post_aabb_origin).abs().max_position() as u8;
    var last_step_axis: u32;
    let pre_shift_delta = abs(post_aabb_origin_pre_shift - post_aabb_origin);
    if pre_shift_delta.x >= pre_shift_delta.y && pre_shift_delta.x >= pre_shift_delta.z { last_step_axis = 0u; }
    else if pre_shift_delta.y >= pre_shift_delta.z                                      { last_step_axis = 1u; }
    else                                                                                { last_step_axis = 2u; }

    // Rust: let mut current_node_index = 0u32;
    // Rust: let mut last_distance = distance_to_aabb;
    var current_node_index: u32 = 0u;
    var last_distance = distance_to_aabb;
    if max_length < last_distance { return no_hit; }

    // ── Main loop ─────────────────────────────────────────────────────────────
    for (var _iteration: u32 = 0u; _iteration < DDA_MAX_STEPS; _iteration++) {
        // Rust: let node = &self.nodes[current_node_index as usize];
        let node_depth    = dda_node_depth(tree_base, current_node_index);
        let node_size     = dda_node_size(node_depth);
        let node_child_sz = dda_child_size(node_depth);

        // Rust: let contents_pos = (current_relative_pos / node.child_size()).as_u8vec3();
        // Rust uses current_relative_pos which is root_relative_grid_pos % node.size().
        // node_size is always a power of two so modulo == bitwise AND.
        let current_relative_pos = root_relative_grid_pos & vec3<u32>(node_size - 1u);
        let contents_pos         = current_relative_pos / vec3<u32>(node_child_sz);
        let contents_index       = get_child_contents_index(contents_pos);
        let raw_cell             = dda_cell_raw(tree_base, current_node_index, contents_index);
        let cell_value_type      = dda_cell_type(raw_cell);
        let cell_value_data      = dda_cell_value(raw_cell);

        // Rust: match cell.0 { 2 => NODE, 1 => DATA, 0 => NONE }
        if cell_value_type == 2u {
            // NODE → Rust: current_node_index += cell.1 as u32
            current_node_index += cell_value_data;

        } else if cell_value_type == 1u {
            // DATA → Rust: return Some((pos, -step[last_step_axis] * I8Vec3::AXES[last_step_axis], last_distance))
            var normal_vec = vec3<f32>(0.0);
            normal_vec[last_step_axis] = -f32(step[last_step_axis]);
            var hit_result: DDAHit;
            hit_result.hit    = true;
            hit_result.value  = cell_value_data;
            hit_result.normal = normal_vec;
            hit_result.dist   = last_distance;
            return hit_result;

        } else {
            // NONE → hierarchical DDA step.

            // Rust (only when node.depth != 0, i.e. child_size != 1):
            //   let node_cell_relative_grid_pos = node_relative_grid_pos % current_node.child_size();
            //   let mut step_amount = I16Vec3::select(
            //       step.cmpgt(I8Vec3::ZERO),
            //       I16Vec3::splat(current_node.child_size() as i16 - 1),
            //       I16Vec3::ZERO
            //   ) - node_cell_relative_grid_pos.as_i16vec3();
            //   // ... advance axis_distances and root_relative_grid_pos by step_amount ...
            //
            // Then unconditionally: match (axis_distances + delta).min_position() { 0/1/2 axis step }
            //
            // We replicate the pre-step (for child_size > 1) and then the single-voxel step together.
            // The pre-step brings us to the boundary of the empty cell; the single-voxel step
            // crosses into the next cell. The Rust code does them in two separate blocks.

            if node_child_sz != 1u {
                // Rust: let node_cell_relative_grid_pos = node_relative_grid_pos % current_node.child_size();
                let node_cell_relative_grid_pos = current_relative_pos & vec3<u32>(node_child_sz - 1u);

                // Rust: let mut step_amount = I16Vec3::select(step.cmpgt(ZERO), splat(child_size-1), ZERO)
                //                            - node_cell_relative_grid_pos;
                // NOTE: Rust comment says "I16Vec3::splat(current_node.child_size() as i16 - 1)"
                // but inspection shows it is actually child_size (not child_size-1) in the select
                // for positive step, because the signed result gives distance to the far boundary.
                // Cross-checking: positive step → (child_size - 1) - node_cell_pos
                //                 negative step → (         0     ) - node_cell_pos  = -node_cell_pos
                // which is exactly what the Rust produces: select(child_size-1, 0) - node_cell_pos.
                // (The prior shader had select(child_size, -1) which is off-by-one vs Rust. Fixed here.)
                let cell_step_amount = vec3<u32>(
                    select(node_cell_relative_grid_pos.x, node_child_sz - node_cell_relative_grid_pos.x - 1, step.x > 0),
                    select(node_cell_relative_grid_pos.y, node_child_sz - node_cell_relative_grid_pos.y - 1, step.y > 0),
                    select(node_cell_relative_grid_pos.z, node_child_sz - node_cell_relative_grid_pos.z - 1, step.z > 0),
                );

                // Rust: match (axis_distances + step_amount.abs().as_vec3() * delta).min_position()
                let axis_distances_at_cell_edge = axis_distances + delta * vec3<f32>(cell_step_amount);
                var cell_edge_min_axis: u32;
                if      axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.y &&
                        axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.z { cell_edge_min_axis = 0u; }
                else if axis_distances_at_cell_edge.y <= axis_distances_at_cell_edge.z { cell_edge_min_axis = 1u; }
                else                                                                   { cell_edge_min_axis = 2u; }

                // Rust: each arm recomputes the other two step_amounts proportionally from dis_to_edge.
                var adjusted_step_amount = cell_step_amount;
                if cell_edge_min_axis == 0u {
                    let dis_to_edge = axis_distances_at_cell_edge.x;
                    adjusted_step_amount.y = u32(abs((dis_to_edge - axis_distances.y + delta.y) / delta.y));
                    adjusted_step_amount.z = u32(abs((dis_to_edge - axis_distances.z + delta.z) / delta.z));
                } else if cell_edge_min_axis == 1u {
                    let dis_to_edge = axis_distances_at_cell_edge.y;
                    adjusted_step_amount.x = u32(abs((dis_to_edge - axis_distances.x + delta.x) / delta.x));
                    adjusted_step_amount.z = u32(abs((dis_to_edge - axis_distances.z + delta.z) / delta.z));
                } else {
                    let dis_to_edge = axis_distances_at_cell_edge.z;
                    adjusted_step_amount.x = u32(abs((dis_to_edge - axis_distances.x + delta.x) / delta.x));
                    adjusted_step_amount.y = u32(abs((dis_to_edge - axis_distances.y + delta.y) / delta.y));
                }

                // Rust: axis_distances += delta * step_amount.abs().as_vec3();
                // Rust: root_relative_grid_pos = (root_relative_grid_pos.as_i16vec3() + step_amount).as_u16vec3();
                axis_distances += delta * vec3<f32>(adjusted_step_amount);
                root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_grid_pos) + vec3<i32>(adjusted_step_amount) * step);
            }

            // Rust: match (axis_distances + delta).min_position() { 0 => x-axis, 1 => y-axis, 2 => z-axis }
            let axis_distances_plus_delta = axis_distances;
            var single_step_axis: u32;
            if      axis_distances_plus_delta.x <= axis_distances_plus_delta.y &&
                    axis_distances_plus_delta.x <= axis_distances_plus_delta.z { single_step_axis = 0u; }
            else if axis_distances_plus_delta.y <= axis_distances_plus_delta.z { single_step_axis = 1u; }
            else                                                                { single_step_axis = 2u; }

            if single_step_axis == 0u {
                // Rust arm 0:
                //   if max_length < axis_distances.x + delta.x { return None; }
                //   let root_relative_grid_pos_x = root_relative_grid_pos.x as i16 + step.x as i16;
                //   if out of bounds { return None; }
                //   loop { if same node { break; } walk up }
                //   root_relative_grid_pos.x = new_x;
                //   axis_distances.x += delta.x;
                //   last_distance = axis_distances.x; last_step_axis = 0;
                if max_length < axis_distances.x { return no_hit; }
                let root_relative_grid_pos_x_signed = i32(root_relative_grid_pos.x) + step.x;
                if root_relative_grid_pos_x_signed < 0 || u32(root_relative_grid_pos_x_signed) >= root_size_u32 { return no_hit; }
                let root_relative_grid_pos_x = u32(root_relative_grid_pos_x_signed);

                loop {
                    let walk_up_node_size = dda_node_size(dda_node_depth(tree_base, current_node_index));
                    if root_relative_grid_pos.x / walk_up_node_size == root_relative_grid_pos_x / walk_up_node_size { break; }
                    let walk_up_parent_offset = dda_node_parent_offset(tree_base, current_node_index);
                    if walk_up_parent_offset == 0u { break; }
                    current_node_index -= walk_up_parent_offset;
                }

                root_relative_grid_pos.x = root_relative_grid_pos_x;
                last_distance    = axis_distances.x;
                axis_distances.x += delta.x;
                last_step_axis   = 0u;

            } else if single_step_axis == 1u {
                // Rust arm 1 (y-axis)
                if max_length < axis_distances.y { return no_hit; }
                let root_relative_grid_pos_y_signed = i32(root_relative_grid_pos.y) + step.y;
                if root_relative_grid_pos_y_signed < 0 || u32(root_relative_grid_pos_y_signed) >= root_size_u32 { return no_hit; }
                let root_relative_grid_pos_y = u32(root_relative_grid_pos_y_signed);

                loop {
                    let walk_up_node_size = dda_node_size(dda_node_depth(tree_base, current_node_index));
                    if root_relative_grid_pos.y / walk_up_node_size == root_relative_grid_pos_y / walk_up_node_size { break; }
                    let walk_up_parent_offset = dda_node_parent_offset(tree_base, current_node_index);
                    if walk_up_parent_offset == 0u { break; }
                    current_node_index -= walk_up_parent_offset;
                }

                root_relative_grid_pos.y = root_relative_grid_pos_y;
                last_distance    = axis_distances.y;
                axis_distances.y += delta.y;
                last_step_axis   = 1u;

            } else {
                // Rust arm 2 (z-axis)
                if max_length < axis_distances.z { return no_hit; }
                let root_relative_grid_pos_z_signed = i32(root_relative_grid_pos.z) + step.z;
                if root_relative_grid_pos_z_signed < 0 || u32(root_relative_grid_pos_z_signed) >= root_size_u32 { return no_hit; }
                let root_relative_grid_pos_z = u32(root_relative_grid_pos_z_signed);

                loop {
                    let walk_up_node_size = dda_node_size(dda_node_depth(tree_base, current_node_index));
                    if root_relative_grid_pos.z / walk_up_node_size == root_relative_grid_pos_z / walk_up_node_size { break; }
                    let walk_up_parent_offset = dda_node_parent_offset(tree_base, current_node_index);
                    if walk_up_parent_offset == 0u { break; }
                    current_node_index -= walk_up_parent_offset;
                }

                root_relative_grid_pos.z = root_relative_grid_pos_z;
                last_distance    = axis_distances.z;
                axis_distances.z += delta.z;
                last_step_axis   = 2u;
            }
        }
    }

    return no_hit;
}
