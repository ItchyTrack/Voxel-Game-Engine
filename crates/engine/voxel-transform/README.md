# Voxel Transform

`Transform` is the authoritative local pose: fixed-point translation, quaternion rotation, and positive uniform `Scale` (Q16.16). `Body` requires this transform, not Bevy's transform.

Use `TransformQuery::get_world(entity)` to compose the current `ChildOf` chain. It also reads `voxel_data::grid::GridLocalTransform` integer offsets. Missing local transforms inherit their parent. A chain with no spatial component, a missing entity, a cycle, or an invalid composed scale returns `None`. When both local components exist, the grid offset is applied in the custom transform's local axes.

`Scale::from_bits` and checked operations return `Option`. `from_num`, `from_fixed`, multiplication, and inverse panic on an unrepresentable result. Rounding uses nearest, ties to even. Inverse transforms can be approximate because reciprocal scales must fit Q16.16.

`VoxelTransformPlugin` prepares only camera render snapshots in `PostUpdate`. Order simulation before `TransformSystems::PrepareRender` and render consumers after it. The active custom `Camera3d` with the lowest entity bits supplies the shared `RenderOrigin`; no active camera resets it to zero. Bevy camera transforms are snapshots, never authoritative inputs. No voxel transform propagation or world-transform cache is used.

Use `world_transform.relative_to(origin.0)` at rendering boundaries. It subtracts the fixed origin before casting to floats. `compute_matrix` and `as_affine` are absolute, render-only conversions and can lose precision at large coordinates.

The `aabb` and `bvh` modules use fixed-point bounds and ray distances. Raycasts accept `voxel_math::Transform`, including `voxel_math::Ray`. Distances are parameters along the supplied direction; use a unit direction for world distances.
