use bytemuck::{Pod, Zeroable};
use winit::window::Window;

use super::geometry::{GeometryBuilder, Vertex};
use super::camera::Camera;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
    inv_view_proj: [[f32; 4]; 4],
    viewport_size: [f32; 2],
    _pad: [f32; 2],
}

const MSAA_SAMPLES: u32 = 4;

pub struct RenderState {
    pub surface: wgpu::Surface<'static>,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub config: wgpu::SurfaceConfiguration,
    pub size: winit::dpi::PhysicalSize<u32>,
    /// Format everything renders in. Equal to `config.format` except on the
    /// web, where WebGPU canvases are linear-only and we render through an
    /// sRGB view of them so colours match native.
    pub view_format: wgpu::TextureFormat,

    grid_pipeline: wgpu::RenderPipeline,
    geo_pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    vertex_capacity: usize,
    index_capacity: usize,
    msaa_view: wgpu::TextureView,
}

const INITIAL_VERTS: usize = 4096;
const INITIAL_IDXS:  usize = 8192;

fn create_msaa_view(device: &wgpu::Device, config: &wgpu::SurfaceConfiguration, format: wgpu::TextureFormat) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("msaa"),
            size: wgpu::Extent3d {
                width:                 config.width.max(1),
                height:                config.height.max(1),
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count:    MSAA_SAMPLES,
            dimension:       wgpu::TextureDimension::D2,
            format,
            usage:           wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats:    &[],
        })
        .create_view(&Default::default())
}

impl RenderState {
    pub async fn new(window: &'static Window) -> Self {
        let size = window.inner_size();
        #[cfg(not(target_arch = "wasm32"))]
        let backends = wgpu::Backends::all();
        // `?webgl` in the page URL forces WebGL2 even where WebGPU exists.
        #[cfg(target_arch = "wasm32")]
        let backends = match web_sys::window().and_then(|w| w.location().search().ok()) {
            Some(q) if q.contains("webgl") => wgpu::Backends::GL,
            _ => wgpu::Backends::all(),
        };
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends,
            ..Default::default()
        });
        let surface = instance.create_surface(window).unwrap();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .unwrap();
        #[cfg(target_arch = "wasm32")]
        {
            let info = adapter.get_info();
            log::warn!("wgpu backend: {:?} ({})", info.backend, info.name);
        }
        #[cfg(not(target_arch = "wasm32"))]
        let device_desc = wgpu::DeviceDescriptor::default();
        // Desktop default limits exceed what WebGL2 offers.
        #[cfg(target_arch = "wasm32")]
        let device_desc = wgpu::DeviceDescriptor {
            required_limits: wgpu::Limits::downlevel_webgl2_defaults().using_resolution(adapter.limits()),
            ..Default::default()
        };
        let (device, queue) = adapter
            .request_device(&device_desc, None)
            .await
            .unwrap();

        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .find(|f| f.is_srgb())
            .copied()
            .unwrap_or(surface_caps.formats[0]);
        #[cfg(not(target_arch = "wasm32"))]
        let view_format = surface_format;
        #[cfg(target_arch = "wasm32")]
        let view_format = surface_format.add_srgb_suffix();

        let config = wgpu::SurfaceConfiguration {
            usage:                          wgpu::TextureUsages::RENDER_ATTACHMENT,
            format:                         surface_format,
            width:                          size.width.max(1),
            height:                         size.height.max(1),
            // No vsync: in target-fps mode the physics budget sets the frame rate.
            present_mode:                   wgpu::PresentMode::AutoNoVsync,
            alpha_mode:                     surface_caps.alpha_modes[0],
            view_formats:                   if view_format != surface_format { vec![view_format] } else { vec![] },
            desired_maximum_frame_latency:  2,
        };
        surface.configure(&device, &config);

        let msaa_view = create_msaa_view(&device, &config, view_format);

        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label:              Some("uniforms"),
            size:               std::mem::size_of::<Uniforms>() as u64,
            usage:              wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding:    0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty:                 wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size:   None,
                },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label:   Some("bg"),
            layout:  &bgl,
            entries: &[wgpu::BindGroupEntry {
                binding:  0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label:                None,
            bind_group_layouts:   &[&bgl],
            push_constant_ranges: &[],
        });

        let msaa_state = wgpu::MultisampleState {
            count:                     MSAA_SAMPLES,
            mask:                      !0,
            alpha_to_coverage_enabled: false,
        };

        let grid_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label:  Some("grid"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/grid.wgsl").into()),
        });
        let geo_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label:  Some("geometry"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/geometry.wgsl").into()),
        });

        let grid_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label:  Some("grid"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module:               &grid_shader,
                entry_point:          Some("vs_main"),
                buffers:              &[],
                compilation_options:  Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module:              &grid_shader,
                entry_point:         Some("fs_main"),
                targets:             &[Some(wgpu::ColorTargetState {
                    format:     view_format,
                    blend:      Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive:    wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample:  msaa_state,
            multiview:    None,
            cache:        None,
        });

        let vertex_attrs = wgpu::vertex_attr_array![0 => Float32x2, 1 => Float32x4];
        let geo_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label:  Some("geometry"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module:      &geo_shader,
                entry_point: Some("vs_main"),
                buffers:     &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode:    wgpu::VertexStepMode::Vertex,
                    attributes:   &vertex_attrs,
                }],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module:      &geo_shader,
                entry_point: Some("fs_main"),
                targets:     &[Some(wgpu::ColorTargetState {
                    format:     view_format,
                    blend:      Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive:     wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample:   msaa_state,
            multiview:     None,
            cache:         None,
        });

        let vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label:              Some("vb"),
            size:               (std::mem::size_of::<Vertex>() * INITIAL_VERTS) as u64,
            usage:              wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let index_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label:              Some("ib"),
            size:               (std::mem::size_of::<u32>() * INITIAL_IDXS) as u64,
            usage:              wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            surface, device, queue, config, size, view_format,
            grid_pipeline, geo_pipeline,
            uniform_buffer, bind_group,
            vertex_buffer, index_buffer,
            vertex_capacity: INITIAL_VERTS,
            index_capacity:  INITIAL_IDXS,
            msaa_view,
        }
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width  = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
            self.msaa_view = create_msaa_view(&self.device, &self.config, self.view_format);
        }
    }

    /// Record the geometry render pass and hand back the surface texture + encoder
    /// so the caller can append more passes (e.g. egui) before presenting.
    pub fn render_geometry(
        &mut self,
        camera: &Camera,
        geo: &GeometryBuilder,
    ) -> Result<(wgpu::SurfaceTexture, wgpu::TextureView, wgpu::CommandEncoder), wgpu::SurfaceError> {
        self.upload(camera, geo, self.size.width as f32, self.size.height as f32);

        let output = self.surface.get_current_texture()?;
        let surface_view = output.texture.create_view(&wgpu::TextureViewDescriptor {
            format: Some(self.view_format),
            ..Default::default()
        });
        let mut encoder  = self.device.create_command_encoder(&Default::default());
        self.encode_scene(&mut encoder, &self.msaa_view, &surface_view, geo.indices.len() as u32);

        Ok((output, surface_view, encoder))
    }

    /// Draw `geo` seen through `camera` into a new `width` × `height`
    /// texture (for a thumbnail), submitted at once. Call it outside a
    /// frame: uniforms and buffers are shared with `render_geometry`.
    pub fn render_to_texture(&mut self, camera: &Camera, geo: &GeometryBuilder, width: u32, height: u32) -> wgpu::TextureView {
        self.upload(camera, geo, width as f32, height as f32);
        let config = wgpu::SurfaceConfiguration { width, height, ..self.config.clone() };
        let msaa = create_msaa_view(&self.device, &config, self.view_format);
        let target = self.device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("thumbnail"),
                size: wgpu::Extent3d { width: width.max(1), height: height.max(1), depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count:    1,
                dimension:       wgpu::TextureDimension::D2,
                format:          self.view_format,
                usage:           wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats:    &[],
            })
            .create_view(&Default::default());
        let mut encoder = self.device.create_command_encoder(&Default::default());
        self.encode_scene(&mut encoder, &msaa, &target, geo.indices.len() as u32);
        self.queue.submit(std::iter::once(encoder.finish()));
        target
    }

    /// Uniforms for a `w` × `h` target, and the geometry buffers.
    fn upload(&mut self, camera: &Camera, geo: &GeometryBuilder, w: f32, h: f32) {
        let vp     = camera.view_proj(w, h);
        let inv_vp = vp.inverse();
        let uniforms = Uniforms {
            view_proj:     vp.to_cols_array_2d(),
            inv_view_proj: inv_vp.to_cols_array_2d(),
            viewport_size: [w, h],
            _pad:          [0.0; 2],
        };
        self.queue.write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(&uniforms));

        if !geo.vertices.is_empty() {
            let nv = geo.vertices.len();
            let ni = geo.indices.len();
            if nv > self.vertex_capacity || ni > self.index_capacity {
                let new_vc = (nv * 2).max(self.vertex_capacity);
                let new_ic = (ni * 2).max(self.index_capacity);
                self.vertex_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
                    label:              Some("vb"),
                    size:               (std::mem::size_of::<Vertex>() * new_vc) as u64,
                    usage:              wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                self.index_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
                    label:              Some("ib"),
                    size:               (std::mem::size_of::<u32>() * new_ic) as u64,
                    usage:              wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                self.vertex_capacity = new_vc;
                self.index_capacity  = new_ic;
            }
            self.queue.write_buffer(&self.vertex_buffer, 0, bytemuck::cast_slice(&geo.vertices));
            self.queue.write_buffer(&self.index_buffer,  0, bytemuck::cast_slice(&geo.indices));
        }
    }

    /// Grid, then the uploaded geometry, into `msaa` resolved to `target`.
    fn encode_scene(&self, encoder: &mut wgpu::CommandEncoder, msaa: &wgpu::TextureView, target: &wgpu::TextureView, n_indices: u32) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: None,
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view:           msaa,           // MSAA render target
                resolve_target: Some(target),   // resolved to swapchain / texture
                ops: wgpu::Operations {
                    load:  wgpu::LoadOp::Clear(wgpu::Color { r: 0.018, g: 0.019, b: 0.023, a: 1.0 }),
                    store: wgpu::StoreOp::Discard,
                },
            })],
            depth_stencil_attachment: None,
            timestamp_writes:         None,
            occlusion_query_set:      None,
        });

        pass.set_pipeline(&self.grid_pipeline);
        pass.set_bind_group(0, &self.bind_group, &[]);
        pass.draw(0..3, 0..1);

        if n_indices > 0 {
            pass.set_pipeline(&self.geo_pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..n_indices, 0, 0..1);
        }
    }
}
