use glam::{Mat4, Vec2};

#[derive(Clone)]
pub struct Camera {
    pub center: Vec2,
    pub scale: f32,  // world units per pixel
}

impl Camera {
    pub fn new() -> Self {
        Self {
            center: Vec2::new(0.0, 0.0),
            scale: 0.006,  // ~167 pixels per world unit
        }
    }

    pub fn view_proj(&self, width: f32, height: f32) -> Mat4 {
        // Orthographic: map world coords to NDC
        // world_x in [center.x - w/2*scale, center.x + w/2*scale] -> [-1, 1]
        let half_w = width * 0.5 * self.scale;
        let half_h = height * 0.5 * self.scale;
        Mat4::orthographic_rh(
            self.center.x - half_w,
            self.center.x + half_w,
            self.center.y - half_h,
            self.center.y + half_h,
            -1.0,
            1.0,
        )
    }

    pub fn pan(&mut self, screen_delta: Vec2) {
        // Screen Y is down-positive; world Y is up-positive, so flip Y.
        self.center -= Vec2::new(screen_delta.x, -screen_delta.y) * self.scale;
    }

    pub fn zoom(&mut self, factor: f32, screen_pos: Vec2, screen_size: Vec2) {
        // Zoom towards the cursor position
        let world_before = self.screen_to_world(screen_pos, screen_size);
        self.scale *= factor;
        self.scale = self.scale.clamp(0.001, 1.0);
        let world_after = self.screen_to_world(screen_pos, screen_size);
        self.center += world_before - world_after;
    }

    pub fn screen_to_world(&self, screen: Vec2, screen_size: Vec2) -> Vec2 {
        let ndc = (screen / screen_size) * 2.0 - Vec2::ONE;
        let half_w = screen_size.x * 0.5 * self.scale;
        let half_h = screen_size.y * 0.5 * self.scale;
        Vec2::new(
            self.center.x + ndc.x * half_w,
            self.center.y - ndc.y * half_h,
        )
    }
}
