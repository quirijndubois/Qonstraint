struct Uniforms {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    viewport_size: vec2<f32>,
    _pad: vec2<f32>,
}

@group(0) @binding(0) var<uniform> u: Uniforms;

struct VertOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) ndc: vec2<f32>,
}

@vertex
fn vs_main(@builtin(vertex_index) vi: u32) -> VertOut {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 3.0, -1.0),
        vec2<f32>(-1.0,  3.0),
    );
    var out: VertOut;
    out.clip_pos = vec4<f32>(positions[vi], 0.0, 1.0);
    out.ndc = positions[vi];
    return out;
}

@fragment
fn fs_main(in: VertOut) -> @location(0) vec4<f32> {
    let bg = vec4<f32>(0.018, 0.019, 0.023, 1.0);

    // Unproject to world space
    let world_h = u.inv_view_proj * vec4<f32>(in.ndc, 0.0, 1.0);
    let world = world_h.xy / world_h.w;

    // Pixel size in world units (for antialiased line width)
    let ndc_dx = vec2<f32>(2.0 / u.viewport_size.x, 0.0);
    let w0 = (u.inv_view_proj * vec4<f32>(in.ndc,          0.0, 1.0)).xy;
    let w1 = (u.inv_view_proj * vec4<f32>(in.ndc + ndc_dx, 0.0, 1.0)).xy;
    let px_world = length(w1 - w0);
    let line_w = px_world * 1.2;

    // Fine grid (0.5 unit cells) — drawn first, more transparent
    let fine_cell = fract(world / 0.5);
    let fine_dist = min(min(fine_cell.x, 1.0 - fine_cell.x), min(fine_cell.y, 1.0 - fine_cell.y));
    let fine_alpha = (1.0 - smoothstep(0.0, line_w, fine_dist)) * 0.28;

    // Coarse grid (1.0 unit cells) — on top, stronger
    let coarse_cell = fract(world / 1.0);
    let coarse_dist = min(min(coarse_cell.x, 1.0 - coarse_cell.x), min(coarse_cell.y, 1.0 - coarse_cell.y));
    let coarse_alpha = (1.0 - smoothstep(0.0, line_w, coarse_dist)) * 0.55;

    let grid_col = vec4<f32>(0.20, 0.21, 0.25, 1.0);
    var color = mix(bg, grid_col, fine_alpha);
    color = mix(color, grid_col, coarse_alpha);
    return color;
}
