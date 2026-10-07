use glam::{DVec2, Vec2};

#[derive(Clone, Debug)]
pub enum BodyShape {
    Disk { radius: f32 },
    Rod { half_len: f32, half_width: f32 },
    Point,
}

/// Dynamic state is f64: with many small steps per frame, f32 positions lose
/// a sizeable fraction of every `v·dt` increment to rounding, which shows up
/// as energy drift that gets *worse* the more steps you take. Geometry
/// (shapes, attachment points) stays f32; it is only ever read.
#[derive(Clone, Debug)]
pub struct Body {
    pub pos: DVec2,
    pub angle: f64,
    pub vel: DVec2,
    pub ang_vel: f64,
    pub mass: f32,
    pub inertia: f32,
    pub force_accum: DVec2,
    pub torque_accum: f64,
    pub shape: BodyShape,
    pub fixed: bool,
    /// Takes part in collisions (`contact.rs`).
    pub collide: bool,
}

/// A body's rotation, `(cos θ, sin θ)`, computed once and reused for every
/// attachment point of that body.
#[derive(Clone, Copy, Debug)]
pub struct Rot { pub c: f64, pub s: f64 }

impl Rot {
    #[inline]
    pub fn new(angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        Self { c, s }
    }

    /// R·v
    #[inline]
    pub fn apply(self, v: DVec2) -> DVec2 {
        DVec2::new(self.c * v.x - self.s * v.y, self.s * v.x + self.c * v.y)
    }
}

impl Body {
    pub fn new(pos: Vec2, angle: f32, mass: f32, inertia: f32, shape: BodyShape) -> Self {
        Self {
            pos: pos.as_dvec2(),
            angle: angle as f64,
            vel: DVec2::ZERO,
            ang_vel: 0.0,
            mass,
            inertia,
            force_accum: DVec2::ZERO,
            torque_accum: 0.0,
            shape,
            fixed: false,
            collide: false,
        }
    }

    pub fn fixed(mut self) -> Self {
        self.fixed = true;
        self
    }

    pub fn colliding(mut self) -> Self {
        self.collide = true;
        self
    }

    /// Position in f32, for drawing and UI.
    #[inline]
    pub fn pos32(&self) -> Vec2 { self.pos.as_vec2() }

    /// Angle in f32, for drawing and UI.
    #[inline]
    pub fn angle32(&self) -> f32 { self.angle as f32 }

    #[inline]
    pub fn rot(&self) -> Rot { Rot::new(self.angle) }

    /// World-space position of a local attachment point (f32, for drawing and UI).
    pub fn world_point(&self, local: Vec2) -> Vec2 {
        self.world_point_d(local.as_dvec2()).as_vec2()
    }

    /// World-space position of a local attachment point.
    pub fn world_point_d(&self, local: DVec2) -> DVec2 {
        self.pos + self.rot().apply(local)
    }

    /// Velocity of the point at world offset `r` from the centre: v + ω × r.
    #[inline]
    pub fn point_vel(&self, r: DVec2) -> DVec2 {
        self.vel + self.ang_vel * r.perp()
    }

    pub fn clear_accumulators(&mut self) {
        self.force_accum = DVec2::ZERO;
        self.torque_accum = 0.0;
    }

    pub fn apply_force_at_world_point(&mut self, force: DVec2, world_point: DVec2) {
        self.force_accum += force;
        self.torque_accum += (world_point - self.pos).perp_dot(force);
    }

    #[inline]
    pub fn inv_mass(&self) -> f64 {
        if self.fixed { 0.0 } else { 1.0 / self.mass as f64 }
    }

    #[inline]
    pub fn inv_inertia(&self) -> f64 {
        if self.fixed { 0.0 } else { 1.0 / self.inertia as f64 }
    }
}

/// Moment of inertia for a uniform rod about its center.
pub fn rod_inertia(mass: f32, half_len: f32) -> f32 {
    mass * (2.0 * half_len).powi(2) / 12.0
}

/// Moment of inertia for a uniform disk about its center.
pub fn disk_inertia(mass: f32, radius: f32) -> f32 {
    0.5 * mass * radius * radius
}
