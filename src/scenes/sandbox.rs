use crate::sim::{forces::Gravity, world::World};

pub fn build() -> World {
    let mut world = World::new();
    world.add_force(Gravity::new(9.81));
    world
}
