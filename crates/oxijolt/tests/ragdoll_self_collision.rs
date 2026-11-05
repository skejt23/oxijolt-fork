//! Ragdoll self-collision: a folded chain of three spheres whose root and grandchild overlap.
//! By default no two parts collide; `RagdollSelfCollision::ExceptParentChild` lets every pair
//! collide except parent/child pairs and the listed ones; invalid pairs are refused.
//!
//! Self-contained (no shared test helpers), so the file also runs on its own against the crate.

use oxijolt::*;

/// Fixed tick, seconds.
const DT: f32 = 1.0 / 60.0;

/// Radius of each sphere, metres.
const RADIUS: f32 = 0.25;
/// Bind centres: the child beside the root, the grandchild folded back over the root. All three
/// pairs overlap: 0.45 m, 0.455 m and 0.429 m apart against a contact distance of 0.5 m.
const CENTRES: [[f32; 3]; 3] = [[0.0, 0.0, 0.0], [0.45, 0.0, 0.0], [0.2, 0.0, 0.38]];

fn skeleton() -> Skeleton {
    Skeleton::new(&[
        SkeletonJoint {
            name: "root",
            parent: None,
        },
        SkeletonJoint {
            name: "child",
            parent: Some(0),
        },
        SkeletonJoint {
            name: "grandchild",
            parent: Some(1),
        },
    ])
    .unwrap()
}

fn centre(part: usize) -> RVec3 {
    let [x, y, z] = CENTRES[part];
    RVec3::new(Real::from(x), Real::from(y), Real::from(z))
}

/// A loose swing-twist joint between `parent` and `part`, anchored halfway between their centres.
fn joint(parent: usize, part: usize) -> RagdollJoint {
    let [a, b] = [CENTRES[parent], CENTRES[part]];
    let mid = [
        0.5 * (a[0] + b[0]),
        0.5 * (a[1] + b[1]),
        0.5 * (a[2] + b[2]),
    ];
    let axis = [b[0] - a[0], 0.0, b[2] - a[2]];
    let length = (axis[0] * axis[0] + axis[2] * axis[2]).sqrt();
    RagdollJoint::SwingTwist(
        SwingTwistConstraintSettings::new(
            RVec3::new(Real::from(mid[0]), Real::from(mid[1]), Real::from(mid[2])),
            Vec3::new(axis[0] / length, 0.0, axis[2] / length),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .half_cone_angles(1.0, 1.0)
        .twist_limits(-1.0, 1.0),
    )
}

fn parts(sphere: &Shape) -> Vec<RagdollPart<'_>> {
    (0..3)
        .map(|part| RagdollPart {
            shape: sphere,
            body: BodySettings::new_dynamic().position(centre(part)).mass(1.0),
            joint: (part > 0).then(|| joint(part - 1, part)),
        })
        .collect()
}

fn settings(self_collision: &RagdollSelfCollision) -> Result<RagdollSettings, RagdollError> {
    let sphere = Shape::new_sphere(RADIUS).unwrap();
    RagdollSettings::with_self_collision(&skeleton(), &parts(&sphere), false, self_collision)
}

// `Real` is already `f64` with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn v3(p: RVec3) -> [f64; 3] {
    [f64::from(p.x), f64::from(p.y), f64::from(p.z)]
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Every pair of parts of `ragdoll` that touched in the last step.
fn part_pairs_in_contact(world: &PhysicsWorld, ragdoll: RagdollId) -> Vec<(usize, usize)> {
    let parts = world.ragdoll(ragdoll).unwrap().body_ids().to_vec();
    let mut pairs = Vec::new();
    for a in 0..parts.len() {
        for b in a + 1..parts.len() {
            if world.were_bodies_in_contact(parts[a], parts[b]).unwrap() {
                pairs.push((a, b));
            }
        }
    }
    pairs
}

/// Distance between the root's and the grandchild's centres.
fn root_to_grandchild(world: &PhysicsWorld, ragdoll: RagdollId) -> f64 {
    let ids = world.ragdoll(ragdoll).unwrap().body_ids().to_vec();
    let at = |part: usize| v3(world.body(ids[part]).unwrap().position());
    distance(at(0), at(2))
}

/// Creates the chain from `settings` in a world without gravity, steps it `ticks` times and
/// returns the world, the ragdoll and the pairs in contact after the first step.
fn run(settings: &RagdollSettings, ticks: usize) -> (PhysicsWorld, RagdollId, Vec<(usize, usize)>) {
    let mut world = PhysicsWorld::new(
        WorldSettings::default()
            .gravity(Vec3::ZERO)
            .worker_threads(1),
    )
    .unwrap();
    let ragdoll = world
        .create_ragdoll(settings, None, Activation::Activate)
        .unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    let first = part_pairs_in_contact(&world, ragdoll);
    for _ in 1..ticks {
        assert!(world.step(DT).unwrap().is_complete());
    }
    (world, ragdoll, first)
}

/// The bind distance between the root and the grandchild, below the two radii.
fn bind_overlap_distance() -> f64 {
    let bind = distance(v3(centre(0)), v3(centre(2)));
    assert!(bind < 2.0 * f64::from(RADIUS) - 0.05, "{bind}");
    bind
}

#[test]
fn default_settings_let_no_parts_collide_and_none_builds_the_same() {
    let skeleton = skeleton();
    let sphere = Shape::new_sphere(RADIUS).unwrap();
    let plain = RagdollSettings::new(&skeleton, &parts(&sphere)).unwrap();
    let none = settings(&RagdollSelfCollision::None).unwrap();
    assert_eq!(RagdollSelfCollision::default(), RagdollSelfCollision::None);

    let (plain_world, plain_ragdoll, plain_pairs) = run(&plain, 60);
    let (none_world, none_ragdoll, none_pairs) = run(&none, 60);
    assert!(plain_pairs.is_empty(), "{plain_pairs:?}");
    assert!(none_pairs.is_empty(), "{none_pairs:?}");
    // The folded chain keeps its overlap: nothing pushes the root and grandchild apart.
    let bind = bind_overlap_distance();
    let after = root_to_grandchild(&plain_world, plain_ragdoll);
    assert!((after - bind).abs() < 1e-3, "{after} vs {bind}");
    // Same settings, same steps: the same bits.
    let bits = |world: &PhysicsWorld, ragdoll: RagdollId| -> Vec<[u64; 3]> {
        world
            .ragdoll(ragdoll)
            .unwrap()
            .body_ids()
            .iter()
            .map(|&id| v3(world.body(id).unwrap().position()).map(f64::to_bits))
            .collect()
    };
    assert_eq!(
        bits(&plain_world, plain_ragdoll),
        bits(&none_world, none_ragdoll)
    );
}

#[test]
fn except_parent_child_lets_the_other_pairs_collide() {
    let settings = settings(&RagdollSelfCollision::ExceptParentChild {
        also_disabled: Vec::new(),
    })
    .unwrap();
    let (world, ragdoll, first) = run(&settings, 60);
    // Every pair overlaps in the bind pose; only the root and the grandchild are not parent and
    // child.
    assert_eq!(first, vec![(0, 2)]);
    // The contact pushes the grandchild off the root.
    let bind = bind_overlap_distance();
    let after = root_to_grandchild(&world, ragdoll);
    assert!(
        after > 2.0 * f64::from(RADIUS) - 0.03 && after > bind + 0.03,
        "{after} vs bind {bind}"
    );
}

#[test]
fn listed_pairs_never_collide_in_either_order() {
    for pair in [(0, 2), (2, 0)] {
        let settings = settings(&RagdollSelfCollision::ExceptParentChild {
            also_disabled: vec![pair],
        })
        .unwrap();
        let (world, ragdoll, first) = run(&settings, 60);
        assert!(first.is_empty(), "{pair:?}: {first:?}");
        let bind = bind_overlap_distance();
        let after = root_to_grandchild(&world, ragdoll);
        assert!((after - bind).abs() < 1e-3, "{pair:?}: {after} vs {bind}");
    }
}

#[test]
fn invalid_pairs_are_refused_before_anything_is_built() {
    for pair in [(1, 1), (0, 3), (3, 0), (0, u32::MAX)] {
        let result = settings(&RagdollSelfCollision::ExceptParentChild {
            also_disabled: vec![(0, 2), pair],
        });
        assert!(
            matches!(result, Err(RagdollError::InvalidValue(_))),
            "{pair:?} accepted"
        );
    }
    // A listed parent/child pair is redundant, not invalid; stabilizing works as with
    // `new_stabilized`.
    let sphere = Shape::new_sphere(RADIUS).unwrap();
    assert!(RagdollSettings::with_self_collision(
        &skeleton(),
        &parts(&sphere),
        true,
        &RagdollSelfCollision::ExceptParentChild {
            also_disabled: vec![(1, 0)],
        },
    )
    .is_ok());
}
