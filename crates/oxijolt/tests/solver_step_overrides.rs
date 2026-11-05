//! Per-body solver step overrides (`BodySettings::velocity_steps_override`,
//! `position_steps_override`): they act through the body's contacts, so a stack that a world
//! with the fewest steps cannot hold stands when its boxes carry overrides, while joints without
//! overrides of their own keep the world's count in free flight until a part touches something.
//!
//! Self-contained (no shared test helpers), so the file also runs on its own against the crate.

use oxijolt::*;

/// Fixed tick, seconds.
const DT: f32 = 1.0 / 60.0;

/// A world with Jolt's fewest velocity steps and no position steps.
fn sparse_world() -> PhysicsWorld {
    PhysicsWorld::new(WorldSettings::default().velocity_steps(2).position_steps(0)).unwrap()
}

/// A static 20 m floor whose top is at y = 0.
fn add_floor(world: &mut PhysicsWorld) {
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0)).unwrap();
    world
        .create_body(
            &floor,
            &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap();
}

// `Real` is already `f64` with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn height(world: &PhysicsWorld, id: BodyId) -> f64 {
    f64::from(world.body(id).unwrap().position().y)
}

/// How far the top of a stack of three 0.2 m boxes, a 10 kg one on two 1 kg ones, ends below
/// its resting height after three seconds.
fn stack_drop(velocity: u32, position: u32) -> f64 {
    let mut world = sparse_world();
    add_floor(&mut world);
    let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let mut top = None;
    for level in 0..3u8 {
        let mass = if level == 2 { 10.0 } else { 1.0 };
        let settings = BodySettings::new_dynamic()
            .position(RVec3::new(0.0, 0.1 + 0.2 * Real::from(level), 0.0))
            .mass(mass)
            .velocity_steps_override(velocity)
            .position_steps_override(position);
        top = Some(world.create_body(&shape, &settings).unwrap());
    }
    for _ in 0..180 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    0.5 - height(&world, top.unwrap())
}

#[test]
fn overrides_stiffen_the_contacts_of_their_bodies() {
    // Two velocity steps and no position steps cannot hold the heavy box up; the overrides,
    // through the boxes' contacts, can.
    let sparse = stack_drop(0, 0);
    let overridden = stack_drop(20, 4);
    assert!(sparse > 0.1, "the sparse stack sank only {sparse} m");
    assert!(
        (0.0..0.01).contains(&overridden),
        "the stack with overrides sank {overridden} m"
    );
}

/// A two-part ragdoll: two capsules joined end to end by a loose swing-twist joint.
fn two_part_ragdoll(velocity: u32, position: u32) -> RagdollSettings {
    let skeleton = Skeleton::new(&[
        SkeletonJoint {
            name: "upper",
            parent: None,
        },
        SkeletonJoint {
            name: "lower",
            parent: Some(0),
        },
    ])
    .unwrap();
    let capsule = Shape::new_capsule(0.2, 0.05).unwrap();
    let body = |y: Real| {
        BodySettings::new_dynamic()
            .position(RVec3::new(0.0, y, 0.0))
            .mass(2.0)
            .velocity_steps_override(velocity)
            .position_steps_override(position)
    };
    let joint = RagdollJoint::SwingTwist(
        SwingTwistConstraintSettings::new(
            RVec3::new(0.0, 2.25, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .half_cone_angles(0.3, 0.3)
        .twist_limits(-0.2, 0.2),
    );
    let parts = [
        RagdollPart {
            shape: &capsule,
            body: body(2.5),
            joint: None,
        },
        RagdollPart {
            shape: &capsule,
            body: body(2.0),
            joint: Some(joint),
        },
    ];
    RagdollSettings::new(&skeleton, &parts).unwrap()
}

/// Drops the two-part ragdoll spinning against its joint limits and returns the pose of both
/// parts after every tick.
fn drop_ragdoll(settings: &RagdollSettings, ticks: usize) -> Vec<[[Real; 3]; 2]> {
    let mut world = sparse_world();
    add_floor(&mut world);
    let ragdoll = world
        .create_ragdoll(settings, None, Activation::Activate)
        .unwrap();
    let ids = world.ragdoll(ragdoll).unwrap().body_ids().to_vec();
    world
        .body_mut(ids[1])
        .unwrap()
        .set_angular_velocity(Vec3::new(0.0, 6.0, 9.0))
        .unwrap();
    let mut poses = Vec::with_capacity(ticks);
    for _ in 0..ticks {
        assert!(world.step(DT).unwrap().is_complete());
        poses.push([0, 1].map(|part| world.body(ids[part]).unwrap().position().into()));
    }
    poses
}

#[test]
fn joints_in_free_flight_keep_the_worlds_count_until_a_part_touches() {
    let plain = drop_ragdoll(&two_part_ragdoll(0, 0), 60);
    let overridden = drop_ragdoll(&two_part_ragdoll(20, 4), 60);
    // About 0.6 s of free fall from 1.75 m above the floor: no contact, so nothing differs.
    assert_eq!(plain[..30], overridden[..30]);
    // On the floor the parts' contacts carry the overrides into the whole island.
    assert_ne!(plain[59], overridden[59]);
}
