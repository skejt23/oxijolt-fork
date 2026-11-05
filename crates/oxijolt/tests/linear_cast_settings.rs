//! The world's linear-cast settings: the threshold decides whether a fast thin plate is cast or
//! skips a thin wall.
//!
//! Self-contained (no shared test helpers), so the file also runs on its own against the crate.

use oxijolt::*;

/// Fixed tick, seconds.
const DT: f32 = 1.0 / 60.0;

/// Whether a thin fast plate, with `MotionQuality::LinearCast`, ends on the far side of a thin
/// wall.
fn tunnels(settings: WorldSettings) -> bool {
    let mut world = PhysicsWorld::new(settings.gravity(Vec3::ZERO)).unwrap();
    let wall = Shape::new_box(Vec3::new(0.01, 2.0, 2.0)).unwrap();
    world
        .create_body(&wall, &BodySettings::new_static())
        .unwrap();
    // A 4 cm plate moving 0.3 m per step: its inner radius is 2 cm, so the default threshold
    // (0.75 of it, 1.5 cm) casts it, and a threshold above 15 lets it skip the wall.
    let plate = Shape::new_box(Vec3::new(0.02, 0.5, 0.5)).unwrap();
    let id = world
        .create_body(
            &plate,
            &BodySettings::new_dynamic()
                .position(RVec3::new(-1.0, 0.0, 0.0))
                .linear_velocity(Vec3::new(18.0, 0.0, 0.0))
                .motion_quality(MotionQuality::LinearCast),
        )
        .unwrap();
    for _ in 0..30 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    let position: [Real; 3] = world.body(id).unwrap().position().into();
    position[0] > 0.0
}

#[test]
fn the_linear_cast_threshold_decides_which_moves_are_cast() {
    assert!(!tunnels(WorldSettings::default()));
    assert!(!tunnels(
        WorldSettings::default().linear_cast_threshold(0.0)
    ));
    assert!(tunnels(
        WorldSettings::default().linear_cast_threshold(20.0)
    ));
}

#[test]
fn linear_cast_settings_outside_their_range_are_refused() {
    type Setter = fn(WorldSettings, f32) -> WorldSettings;
    let setters: [(&str, Setter); 2] = [
        (
            "linear_cast_threshold",
            WorldSettings::linear_cast_threshold,
        ),
        (
            "linear_cast_max_penetration",
            WorldSettings::linear_cast_max_penetration,
        ),
    ];
    for (name, set) in setters {
        for value in [-0.001, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert!(
                matches!(
                    PhysicsWorld::new(set(WorldSettings::default(), value)),
                    Err(WorldError::InvalidValue(_))
                ),
                "{name}({value:?}) was accepted"
            );
        }
    }
}
