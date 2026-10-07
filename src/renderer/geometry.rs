use glam::Vec2;
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct Vertex {
    pub position: [f32; 2],
    pub color: [f32; 4],
}

impl Vertex {
    pub fn new(pos: Vec2, color: [f32; 4]) -> Self {
        Self { position: [pos.x, pos.y], color }
    }
}

pub struct GeometryBuilder {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

impl GeometryBuilder {
    pub fn new() -> Self {
        Self { vertices: Vec::new(), indices: Vec::new() }
    }

    pub fn clear(&mut self) {
        self.vertices.clear();
        self.indices.clear();
    }

    fn push_tri(&mut self, a: Vertex, b: Vertex, c: Vertex) {
        let base = self.vertices.len() as u32;
        self.vertices.push(a);
        self.vertices.push(b);
        self.vertices.push(c);
        self.indices.push(base);
        self.indices.push(base + 1);
        self.indices.push(base + 2);
    }

    fn push_quad(&mut self, a: Vertex, b: Vertex, c: Vertex, d: Vertex) {
        let base = self.vertices.len() as u32;
        self.vertices.extend_from_slice(&[a, b, c, d]);
        self.indices.extend_from_slice(&[base, base+1, base+2, base, base+2, base+3]);
    }

    /// Filled diamond: tips at `c ± along` and `c ± across`.
    pub fn draw_diamond(&mut self, c: Vec2, along: Vec2, across: Vec2, color: [f32; 4]) {
        self.push_quad(
            Vertex::new(c + along, color),
            Vertex::new(c + across, color),
            Vertex::new(c - along, color),
            Vertex::new(c - across, color),
        );
    }

    /// Thick line segment with squared ends.
    pub fn draw_line(&mut self, a: Vec2, b: Vec2, width: f32, color: [f32; 4]) {
        let dir = (b - a).normalize_or_zero();
        let perp = Vec2::new(-dir.y, dir.x) * (width * 0.5);
        self.push_quad(
            Vertex::new(a - perp, color),
            Vertex::new(a + perp, color),
            Vertex::new(b + perp, color),
            Vertex::new(b - perp, color),
        );
    }

    /// Dashed line: `dash`-long segments separated by `gap`.
    pub fn draw_dashed(&mut self, a: Vec2, b: Vec2, width: f32, dash: f32, gap: f32, color: [f32; 4]) {
        let len = (b - a).length();
        if len < 1e-6 { return; }
        let dir = (b - a) / len;
        let mut t = 0.0;
        while t < len {
            let e = (t + dash).min(len);
            self.draw_line(a + dir * t, a + dir * e, width, color);
            t += dash + gap;
        }
    }

    /// Dashed circle outline.
    pub fn draw_dashed_ring(&mut self, center: Vec2, radius: f32, dashes: u32, width: f32, color: [f32; 4]) {
        let step = std::f32::consts::TAU / dashes as f32;
        for i in 0..dashes {
            let a0 = i as f32 * step;
            self.draw_arc(center, radius, a0, a0 + step * 0.55, 4, width, color);
        }
    }

    /// Thick rod with smooth (16-gon) end caps.
    pub fn draw_rod(&mut self, a: Vec2, b: Vec2, width: f32, color: [f32; 4]) {
        self.draw_line(a, b, width, color);
        self.draw_circle(a, width * 0.5, 16, color);
        self.draw_circle(b, width * 0.5, 16, color);
    }

    /// Filled circle using a triangle fan.
    pub fn draw_circle(&mut self, center: Vec2, radius: f32, segments: u32, color: [f32; 4]) {
        let center_v = Vertex::new(center, color);
        for i in 0..segments {
            let a0 = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let a1 = ((i + 1) as f32 / segments as f32) * std::f32::consts::TAU;
            let p0 = center + Vec2::new(a0.cos(), a0.sin()) * radius;
            let p1 = center + Vec2::new(a1.cos(), a1.sin()) * radius;
            self.push_tri(center_v, Vertex::new(p0, color), Vertex::new(p1, color));
        }
    }

    /// Thin arc (partial ring) from start to end angle.
    pub fn draw_arc(&mut self, center: Vec2, radius: f32, start: f32, end: f32, segments: u32, width: f32, color: [f32; 4]) {
        if segments == 0 { return; }
        let mut prev = center + Vec2::new(start.cos(), start.sin()) * radius;
        for i in 1..=segments {
            let t = start + (end - start) * (i as f32 / segments as f32);
            let next = center + Vec2::new(t.cos(), t.sin()) * radius;
            self.draw_line(prev, next, width, color);
            prev = next;
        }
    }


    /// Mechanical spring: short straight leads + zigzag coils between them.
    pub fn draw_spring(&mut self, a: Vec2, b: Vec2, coils: u32, amplitude: f32, width: f32, color: [f32; 4]) {
        let dir = b - a;
        let len = dir.length();
        if len < 1e-6 { return; }
        let tang = dir / len;
        let perp = Vec2::new(-tang.y, tang.x);

        let stub = (len * 0.12).min(0.18);
        let a1 = a + tang * stub;
        let b1 = b - tang * stub;
        self.draw_line(a, a1, width, color);
        self.draw_line(b1, b, width, color);

        // Zigzag coils between a1 and b1
        let coil_len = (b1 - a1).length();
        if coil_len < 1e-6 { return; }
        let total_segs = coils * 2;
        let seg_len = coil_len / total_segs as f32;

        let mut prev = a1;
        for i in 1..=total_segs {
            let t = i as f32 * seg_len;
            let side = if i % 2 == 0 { 1.0f32 } else { -1.0 };
            let next = if i == total_segs {
                b1
            } else {
                a1 + tang * t + perp * side * amplitude
            };
            self.draw_line(prev, next, width, color);
            self.draw_circle(next, width * 0.5, 8, color); // round join
            prev = next;
        }
    }

}
