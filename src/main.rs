use rand::prelude::*;
use std::num;
use std::sync::{Arc, Mutex};
use SmolECS::{component::*, entity::*, rayon::*, system::*, world::*};
use clap::{Arg, App};

#[derive(Copy, Clone)]
pub struct Body;

#[derive(Copy, Clone)]
pub struct Mass {
    mass: f32,
}

#[derive(Copy, Clone)]
pub struct Acceleration {
    x: f32,
    y: f32,
}

#[derive(Copy, Clone)]
pub struct Velocity {
    x: f32,
    y: f32,
}

#[derive(Copy, Clone)]
pub struct Position {
    x: f32,
    y: f32,
}

#[derive(Copy, Clone)]
pub struct WorldBounds {
    x: f32,
    y: f32,
}

#[derive(Copy, Clone)]
pub struct Time {
    beginning: std::time::Instant,
    last: std::time::Instant,
    total: f64,
    delta: f64,
}

// SYSTEMS
pub struct UpdateTime;
impl<'d, 'w: 'd> System<'d, 'w, World> for UpdateTime {
    type SystemData = (Write<'d, Time>);

    fn run(&self, (mut time): Self::SystemData) {
        let current = std::time::Instant::now();
        time.delta = current.duration_since(time.last).as_secs_f64();
        time.total = current.duration_since(time.beginning).as_secs_f64();
        time.last = current;
    }
}

fn overlapping(pos_one: &Position, pos_two: &Position) -> bool {
    let epsilon: f32 = 0.05;

    if ((pos_one.x - pos_two.x).abs() <= epsilon) && ((pos_one.y - pos_two.y).abs() <= epsilon) {
        return true;
    } else {
        return false;
    }
}

fn distance(pos_one: &Position, pos_two: &Position) -> f32 {
    let delta_x = pos_one.x - pos_two.x;
    let delta_y = pos_one.y - pos_two.y;

    let d = delta_x.powf(2.0) + delta_y.powf(2.0);
    return d.sqrt();
}

use std::ops::Deref;
pub struct ApplyGravity;
impl<'d, 'w: 'd> System<'d, 'w, World> for ApplyGravity {
    type SystemData = (
        ReadComp<'d, Mass>,
        ReadComp<'d, Position>,
        WriteComp<'d, Acceleration>,
        Read<'d, EntityStorage>,
    );

    fn run(&self, (masses, positions, mut accels, ents): Self::SystemData) {
        const G: f32 = 6.67430e-11_f32;

        for (mass_one, pos_one, accel, ent_one) in
            (&masses, &positions, &mut accels, ents.deref()).join()
        {
            accel.x = 0.0;
            accel.y = 0.0;
            for (mass_two, pos_two, ent_two) in (&masses, &positions, ents.deref()).join() {
                if ent_one == ent_two {
                    continue;
                }

                if overlapping(pos_one, pos_two) {
                    continue;
                }

                let dist_x = (pos_one.x - pos_two.x).abs();
                let dist_y = (pos_one.y - pos_two.y).abs();

                // BREAK THIS INTO X AND Y COMPONENTS
                let force_x = G * (mass_one.mass * mass_two.mass) / dist_x.powf(2.0);
                let force_y = G * (mass_one.mass * mass_two.mass) / dist_y.powf(2.0);

                accel.x += force_x / mass_one.mass;
                accel.y += force_y / mass_one.mass;
            }
        }
    }
}

pub struct ApplyAccelerations;
impl<'d, 'w: 'd> System<'d, 'w, World> for ApplyAccelerations {
    type SystemData = (
        ReadComp<'d, Acceleration>,
        Read<'d, Time>,
        WriteComp<'d, Velocity>,
    );

    fn run(&self, (accels, time, mut vels): Self::SystemData) {
        for (accel, vel) in (&accels, &mut vels).join() {
            vel.x += accel.x * time.delta as f32;
            vel.y += accel.y * time.delta as f32;
        }
    }
}

pub struct ApplyVelocities;
impl<'d, 'w: 'd> System<'d, 'w, World> for ApplyVelocities {
    type SystemData = (
        ReadComp<'d, Velocity>,
        Read<'d, Time>,
        WriteComp<'d, Position>,
    );

    fn run(&self, (vels, time, mut positions): Self::SystemData) {
        for (vel, position) in (&vels, &mut positions).join() {
            position.x += vel.x * time.delta as f32;
            position.y += vel.y * time.delta as f32;
        }
    }
}

fn main() {
	let app = App::new("nBody")
		.version("1.0")
		.about("runs an n-body simulation")
		.author("SmolECS")
		.arg(Arg::with_name("count")
			.short("n")
			.long("count")
			.help("the amount of bodies to be simulated")
			.takes_value(true)
			.required(true))
		.get_matches();

	let count = app.value_of("count").unwrap_or("100");
	let n: u32 = count.parse().unwrap();

    let mut world = World::new();
    world.register_comp::<Body>();
    world.register_comp::<Mass>();
    world.register_comp::<Acceleration>();
    world.register_comp::<Velocity>();
    world.register_comp::<Position>();

    world.insert(WorldBounds { x: 10.0, y: 10.0 });
    world.insert(Time {
        beginning: std::time::Instant::now(),
        last: std::time::Instant::now(),
        total: 0.0,
        delta: 0.0,
    });
    world.insert(EntityStorage::new());

    let mut ents = Write::<EntityStorage>::get_data(&world);
    let mut bodies = WriteComp::<Body>::get_data(&world);
    let mut masses = WriteComp::<Mass>::get_data(&world);
    let mut accels = WriteComp::<Acceleration>::get_data(&world);
    let mut vels = WriteComp::<Velocity>::get_data(&world);
    let mut positions = WriteComp::<Position>::get_data(&world);

    let mut rng = rand::thread_rng();
    for _ in 0..n {
        ents.create_entity()
            .add(&mut bodies, Body {})
            .add(
                &mut masses,
                Mass {
                    mass: rng.gen_range(1.0, 5.0),
                },
            )
            .add(&mut accels, Acceleration { x: 0.0, y: 0.0 })
            .add(
                &mut vels,
                Velocity {
                    x: rng.gen_range(-1.0, 1.0),
                    y: rng.gen_range(-1.0, 1.0),
                },
            )
            .add(
                &mut positions,
                Position {
                    x: rng.gen_range(0.0, 10.0),
                    y: rng.gen_range(0.0, 10.0),
                },
            );
    }

    let mut scheduler = SystemScheduler::new(Arc::new(
        ThreadPoolBuilder::new().num_threads(4).build().unwrap(),
    ));
    scheduler.add(UpdateTime {}, "update_time", vec![]);
    scheduler.add(ApplyGravity {}, "apply_gravity", vec!["update_time"]);
    scheduler.add(
        ApplyAccelerations {},
        "update_vels",
        vec!["update_time", "apply_gravity"],
    );
    scheduler.add(
        ApplyVelocities {},
        "update_positions",
        vec!["update_time", "update_vels"],
    );

    drop(ents);
    drop(bodies);
    drop(masses);
    drop(accels);
    drop(vels);
    drop(positions);

    for _ in 0..100_000 {
        scheduler.run(&world);
    }
}
