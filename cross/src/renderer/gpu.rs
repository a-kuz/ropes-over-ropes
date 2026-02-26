use std::borrow::Cow;
use std::sync::Arc;
use bytemuck;
use wgpu;
use wgpu::util::DeviceExt;

use super::camera::Camera;
use super::frame_types::*;
use super::hole_mesh;
use super::passes::build_frame_uniforms;

const SHADOW_MAP_SIZE: u32 = 1024;
const BLOOM_DIVISOR: u32 = 4;

pub struct FrameInFlight {
    pub encoder: wgpu::CommandEncoder,
    pub screen_view: wgpu::TextureView,
    output: wgpu::SurfaceTexture,
}

// ─────────────────────────── GpuRenderer ───────────────────────────

pub struct GpuRenderer {
    device: wgpu::Device,
    queue: wgpu::Queue,
    surface: wgpu::Surface<'static>,
    surface_config: wgpu::SurfaceConfiguration,

    table_pipeline: wgpu::RenderPipeline,
    hole_pipeline: wgpu::RenderPipeline,
    rope_pipeline: wgpu::RenderPipeline,
    post_pipeline: wgpu::RenderPipeline,
    shadow_rope_pipeline: wgpu::RenderPipeline,
    shadow_hole_pipeline: wgpu::RenderPipeline,
    bloom_threshold_pipeline: wgpu::ComputePipeline,
    bloom_blur_h_pipeline: wgpu::ComputePipeline,
    bloom_blur_v_pipeline: wgpu::ComputePipeline,

    shadow_depth_view: wgpu::TextureView,
    hdr_texture: wgpu::Texture,
    hdr_view: wgpu::TextureView,
    bloom_a_texture: wgpu::Texture,
    bloom_a_view: wgpu::TextureView,
    bloom_b_texture: wgpu::Texture,
    bloom_b_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,

    frame_uniforms_buffer: wgpu::Buffer,
    hole_instance_buffer: Option<wgpu::Buffer>,
    hole_vertex_buffer: Option<wgpu::Buffer>,
    hole_index_buffer: Option<wgpu::Buffer>,
    rope_vertex_buffer: Option<wgpu::Buffer>,
    rope_index_buffer: Option<wgpu::Buffer>,
    rope_vertex_capacity: usize,
    rope_index_capacity: usize,
    shadow_segment_buffer: wgpu::Buffer,

    frame_bind_group_layout: wgpu::BindGroupLayout,
    frame_bind_group: wgpu::BindGroup,
    shadow_bind_group_layout: wgpu::BindGroupLayout,
    shadow_bind_group: wgpu::BindGroup,
    hole_bind_group_layout: wgpu::BindGroupLayout,
    hole_bind_group: Option<wgpu::BindGroup>,
    post_bind_group_layout: wgpu::BindGroupLayout,
    post_bind_group: wgpu::BindGroup,
    bloom_bind_group_layout: wgpu::BindGroupLayout,
    bloom_threshold_bind_group: wgpu::BindGroup,
    bloom_blur_h_bind_group: wgpu::BindGroup,
    bloom_blur_v_bind_group: wgpu::BindGroup,

    shadow_sampler: wgpu::Sampler,
    linear_sampler: wgpu::Sampler,
    empty_bind_group: wgpu::BindGroup,

    hole_index_count: u32,
    hole_instance_count: u32,
    rope_index_count: u32,

    pub camera: Camera,
    pub highlight_hole: i32,
    width: u32,
    height: u32,
}

// ─────────────────────────── vertex layouts ───────────────────────────

fn rope_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<RopeVertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 0, shader_location: 0 },
            wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 12, shader_location: 1 },
            wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 24, shader_location: 2 },
            wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x2, offset: 36, shader_location: 3 },
            wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x4, offset: 44, shader_location: 4 },
        ],
    }
}

fn hole_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<HoleVertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 0, shader_location: 0 },
            wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 12, shader_location: 1 },
        ],
    }
}

// ─────────────────────────── texture helpers ───────────────────────────

fn create_depth_texture(device: &wgpu::Device, w: u32, h: u32, label: &str) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        })
        .create_view(&wgpu::TextureViewDescriptor::default())
}

fn create_hdr_texture(device: &wgpu::Device, w: u32, h: u32, label: &str) -> (wgpu::Texture, wgpu::TextureView) {
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba16Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::STORAGE_BINDING,
        view_formats: &[],
    });
    let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
    (tex, view)
}

// ─────────────────────────── bind group layouts ───────────────────────────

fn create_frame_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("frame_bind_group_layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Depth,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Comparison),
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
    })
}

fn create_hole_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("hole_bind_group_layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    })
}

fn create_post_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("post_bind_group_layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
    })
}

fn create_bloom_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("bloom_bind_group_layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::StorageTexture {
                    access: wgpu::StorageTextureAccess::WriteOnly,
                    format: wgpu::TextureFormat::Rgba16Float,
                    view_dimension: wgpu::TextureViewDimension::D2,
                },
                count: None,
            },
        ],
    })
}

// ─────────────────────────── bind group builders ───────────────────────────

fn build_frame_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    uniforms_buf: &wgpu::Buffer,
    shadow_view: &wgpu::TextureView,
    shadow_sampler: &wgpu::Sampler,
    linear_sampler: &wgpu::Sampler,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("frame_bind_group"),
        layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: uniforms_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::TextureView(shadow_view) },
            wgpu::BindGroupEntry { binding: 2, resource: wgpu::BindingResource::Sampler(shadow_sampler) },
            wgpu::BindGroupEntry { binding: 3, resource: wgpu::BindingResource::Sampler(linear_sampler) },
        ],
    })
}

fn build_post_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    hdr_view: &wgpu::TextureView,
    bloom_view: &wgpu::TextureView,
    sampler: &wgpu::Sampler,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("post_bind_group"),
        layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(hdr_view) },
            wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::TextureView(bloom_view) },
            wgpu::BindGroupEntry { binding: 2, resource: wgpu::BindingResource::Sampler(sampler) },
        ],
    })
}

fn build_bloom_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    src: &wgpu::TextureView,
    dst: &wgpu::TextureView,
    label: &str,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(src) },
            wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::TextureView(dst) },
        ],
    })
}

// ─────────────────────────── pipeline builders ───────────────────────────

fn build_render_pipeline(
    device: &wgpu::Device,
    label: &str,
    layout: &wgpu::PipelineLayout,
    shader: &wgpu::ShaderModule,
    vs_entry: &str,
    fs_entry: &str,
    vertex_layouts: &[wgpu::VertexBufferLayout<'_>],
    color_format: wgpu::TextureFormat,
    depth_format: Option<wgpu::TextureFormat>,
    depth_write: bool,
    cull_mode: Option<wgpu::Face>,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some(vs_entry),
            buffers: vertex_layouts,
            compilation_options: Default::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some(fs_entry),
            targets: &[Some(wgpu::ColorTargetState {
                format: color_format,
                blend: Some(wgpu::BlendState::REPLACE),
                write_mask: wgpu::ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode,
            unclipped_depth: false,
            polygon_mode: wgpu::PolygonMode::Fill,
            conservative: false,
        },
        depth_stencil: depth_format.map(|fmt| wgpu::DepthStencilState {
            format: fmt,
            depth_write_enabled: depth_write,
            depth_compare: if depth_write { wgpu::CompareFunction::LessEqual } else { wgpu::CompareFunction::Always },
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        multiview: None,
        cache: None,
    })
}

fn build_shadow_pipeline(
    device: &wgpu::Device,
    label: &str,
    layout: &wgpu::PipelineLayout,
    shader: &wgpu::ShaderModule,
    vs_entry: &str,
    vertex_layouts: &[wgpu::VertexBufferLayout<'_>],
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some(vs_entry),
            buffers: vertex_layouts,
            compilation_options: Default::default(),
        },
        fragment: None,
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: Some(wgpu::Face::Back),
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::Less,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState {
                constant: 1,
                slope_scale: 1.0,
                clamp: 0.0,
            },
        }),
        multisample: wgpu::MultisampleState::default(),
        multiview: None,
        cache: None,
    })
}

fn build_compute_pipeline(
    device: &wgpu::Device,
    label: &str,
    layout: &wgpu::PipelineLayout,
    shader: &wgpu::ShaderModule,
    entry: &str,
) -> wgpu::ComputePipeline {
    device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(label),
        layout: Some(layout),
        module: shader,
        entry_point: Some(entry),
        compilation_options: Default::default(),
        cache: None,
    })
}

// ─────────────────────────── impl GpuRenderer ───────────────────────────

impl GpuRenderer {
    pub async fn new(window: Arc<winit::window::Window>) -> Self {
        let size = window.inner_size();
        let width = size.width.max(1);
        let height = size.height.max(1);

        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor::default());
        let surface = instance.create_surface(window).unwrap();

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .expect("no suitable GPU adapter");

        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("uzls_device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::Performance,
            }, None)
            .await
            .expect("failed to create device");

        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .find(|f| !f.is_srgb())
            .copied()
            .unwrap_or(surface_caps.formats[0]);

        let surface_config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width,
            height,
            present_mode: wgpu::PresentMode::Fifo,
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &surface_config);

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("all.wgsl"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!("../shaders/all.wgsl"))),
        });

        // ── samplers ──

        let shadow_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("shadow_sampler"),
            compare: Some(wgpu::CompareFunction::LessEqual),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        let linear_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("linear_sampler"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        // ── textures ──

        let shadow_depth_view = create_depth_texture(&device, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, "shadow_depth");
        let (hdr_texture, hdr_view) = create_hdr_texture(&device, width, height, "hdr");
        let depth_view = create_depth_texture(&device, width, height, "scene_depth");

        let bloom_w = (width / BLOOM_DIVISOR).max(1);
        let bloom_h = (height / BLOOM_DIVISOR).max(1);
        let (bloom_a_texture, bloom_a_view) = create_hdr_texture(&device, bloom_w, bloom_h, "bloom_a");
        let (bloom_b_texture, bloom_b_view) = create_hdr_texture(&device, bloom_w, bloom_h, "bloom_b");

        // ── buffers ──

        let frame_uniforms_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("frame_uniforms"),
            size: std::mem::size_of::<FrameUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let empty_seg = ShadowSegment { ax: 0.0, ay: 0.0, az: 0.0, radius: 0.0, bx: 0.0, by: 0.0, bz: 0.0, rope_id: 0.0 };
        let mut initial_data: Vec<u8> = Vec::new();
        initial_data.extend_from_slice(bytemuck::bytes_of(&ShadowSegmentCount { count: 0, _pad: [0; 3] }));
        initial_data.extend_from_slice(bytemuck::bytes_of(&empty_seg));
        let shadow_segment_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("shadow_segments"),
            contents: &initial_data,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });

        // ── bind group layouts ──

        let frame_bind_group_layout = create_frame_bind_group_layout(&device);
        let hole_bind_group_layout = create_hole_bind_group_layout(&device);
        let post_bind_group_layout = create_post_bind_group_layout(&device);
        let bloom_bind_group_layout = create_bloom_bind_group_layout(&device);

        let shadow_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("shadow_bind_group_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let shadow_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("shadow_bind_group"),
            layout: &shadow_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: frame_uniforms_buffer.as_entire_binding(),
            }],
        });

        let frame_bind_group = build_frame_bind_group(
            &device,
            &frame_bind_group_layout,
            &frame_uniforms_buffer,
            &shadow_depth_view,
            &shadow_sampler,
            &linear_sampler,
        );

        let post_bind_group = build_post_bind_group(
            &device,
            &post_bind_group_layout,
            &hdr_view,
            &bloom_a_view,
            &linear_sampler,
        );

        let bloom_threshold_bind_group = build_bloom_bind_group(
            &device, &bloom_bind_group_layout, &hdr_view, &bloom_a_view, "bloom_threshold_bg",
        );
        let bloom_blur_h_bind_group = build_bloom_bind_group(
            &device, &bloom_bind_group_layout, &bloom_a_view, &bloom_b_view, "bloom_blur_h_bg",
        );
        let bloom_blur_v_bind_group = build_bloom_bind_group(
            &device, &bloom_bind_group_layout, &bloom_b_view, &bloom_a_view, "bloom_blur_v_bg",
        );

        // ── pipeline layouts ──

        let frame_only_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("frame_only_layout"),
            bind_group_layouts: &[&frame_bind_group_layout],
            push_constant_ranges: &[],
        });

        let frame_hole_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("frame_hole_layout"),
            bind_group_layouts: &[&frame_bind_group_layout, &hole_bind_group_layout],
            push_constant_ranges: &[],
        });

        let empty_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("empty"),
            entries: &[],
        });
        let empty_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("empty_bg"),
            layout: &empty_bind_group_layout,
            entries: &[],
        });
        let post_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("post_layout"),
            bind_group_layouts: &[&frame_bind_group_layout, &empty_bind_group_layout, &post_bind_group_layout],
            push_constant_ranges: &[],
        });

        let bloom_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("bloom_layout"),
            bind_group_layouts: &[&empty_bind_group_layout, &empty_bind_group_layout, &empty_bind_group_layout, &bloom_bind_group_layout],
            push_constant_ranges: &[],
        });

        // ── render pipelines ──

        let hdr_format = wgpu::TextureFormat::Rgba16Float;
        let depth_fmt = wgpu::TextureFormat::Depth32Float;

        let table_pipeline = build_render_pipeline(
            &device, "table_pipeline", &frame_hole_layout, &shader,
            "table_vertex", "table_fragment",
            &[], hdr_format, Some(depth_fmt), true, None,
        );

        let hole_pipeline = build_render_pipeline(
            &device, "hole_pipeline", &frame_hole_layout, &shader,
            "hole_vertex", "hole_fragment",
            &[hole_vertex_layout()], hdr_format, Some(depth_fmt), true,
            None,
        );

        let rope_pipeline = build_render_pipeline(
            &device, "rope_pipeline", &frame_hole_layout, &shader,
            "rope_vertex", "rope_fragment",
            &[rope_vertex_layout()], hdr_format, Some(depth_fmt), true,
            None,
        );

        let post_pipeline = build_render_pipeline(
            &device, "post_pipeline", &post_layout, &shader,
            "fullscreen_vertex", "post_fragment",
            &[], surface_format, None, false, None,
        );

        let shadow_only_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("shadow_only_layout"),
            bind_group_layouts: &[&shadow_bind_group_layout],
            push_constant_ranges: &[],
        });

        let shadow_hole_pl_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("shadow_hole_pl_layout"),
            bind_group_layouts: &[&shadow_bind_group_layout, &hole_bind_group_layout],
            push_constant_ranges: &[],
        });

        let shadow_rope_pipeline = build_shadow_pipeline(
            &device, "shadow_rope", &shadow_only_layout, &shader,
            "rope_shadow_vertex", &[rope_vertex_layout()],
        );

        let shadow_hole_pipeline = build_shadow_pipeline(
            &device, "shadow_hole", &shadow_hole_pl_layout, &shader,
            "hole_shadow_vertex", &[hole_vertex_layout()],
        );

        let bloom_threshold_pipeline = build_compute_pipeline(
            &device, "bloom_threshold", &bloom_layout, &shader, "bloom_threshold",
        );
        let bloom_blur_h_pipeline = build_compute_pipeline(
            &device, "bloom_blur_h", &bloom_layout, &shader, "bloom_blur_h",
        );
        let bloom_blur_v_pipeline = build_compute_pipeline(
            &device, "bloom_blur_v", &bloom_layout, &shader, "bloom_blur_v",
        );

        Self {
            device,
            queue,
            surface,
            surface_config,

            table_pipeline,
            hole_pipeline,
            rope_pipeline,
            post_pipeline,
            shadow_rope_pipeline,
            shadow_hole_pipeline,
            bloom_threshold_pipeline,
            bloom_blur_h_pipeline,
            bloom_blur_v_pipeline,

            shadow_depth_view,
            hdr_texture,
            hdr_view,
            bloom_a_texture,
            bloom_a_view,
            bloom_b_texture,
            bloom_b_view,
            depth_view,

            frame_uniforms_buffer,
            hole_instance_buffer: None,
            hole_vertex_buffer: None,
            hole_index_buffer: None,
            rope_vertex_buffer: None,
            rope_index_buffer: None,
            rope_vertex_capacity: 0,
            rope_index_capacity: 0,
            shadow_segment_buffer,

            frame_bind_group_layout,
            frame_bind_group,
            shadow_bind_group_layout,
            shadow_bind_group,
            hole_bind_group_layout,
            hole_bind_group: None,
            post_bind_group_layout,
            post_bind_group,
            bloom_bind_group_layout,
            bloom_threshold_bind_group,
            bloom_blur_h_bind_group,
            bloom_blur_v_bind_group,

            shadow_sampler,
            linear_sampler,
            empty_bind_group,

            hole_index_count: 0,
            hole_instance_count: 0,
            rope_index_count: 0,

            camera: Camera::default(),
            highlight_hole: -1,
            width,
            height,
        }
    }

    pub fn surface_size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    pub fn device(&self) -> &wgpu::Device { &self.device }
    pub fn queue(&self) -> &wgpu::Queue { &self.queue }
    pub fn surface_format(&self) -> wgpu::TextureFormat { self.surface_config.format }

    pub fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        self.width = width;
        self.height = height;
        self.surface_config.width = width;
        self.surface_config.height = height;
        self.surface.configure(&self.device, &self.surface_config);

        let (hdr_texture, hdr_view) = create_hdr_texture(&self.device, width, height, "hdr");
        self.hdr_texture = hdr_texture;
        self.hdr_view = hdr_view;
        self.depth_view = create_depth_texture(&self.device, width, height, "scene_depth");

        let bloom_w = (width / BLOOM_DIVISOR).max(1);
        let bloom_h = (height / BLOOM_DIVISOR).max(1);
        let (ba_tex, ba_view) = create_hdr_texture(&self.device, bloom_w, bloom_h, "bloom_a");
        let (bb_tex, bb_view) = create_hdr_texture(&self.device, bloom_w, bloom_h, "bloom_b");
        self.bloom_a_texture = ba_tex;
        self.bloom_a_view = ba_view;
        self.bloom_b_texture = bb_tex;
        self.bloom_b_view = bb_view;

        self.rebuild_texture_bind_groups();
    }

    fn rebuild_texture_bind_groups(&mut self) {
        self.frame_bind_group = build_frame_bind_group(
            &self.device,
            &self.frame_bind_group_layout,
            &self.frame_uniforms_buffer,
            &self.shadow_depth_view,
            &self.shadow_sampler,
            &self.linear_sampler,
        );

        self.post_bind_group = build_post_bind_group(
            &self.device,
            &self.post_bind_group_layout,
            &self.hdr_view,
            &self.bloom_a_view,
            &self.linear_sampler,
        );

        self.bloom_threshold_bind_group = build_bloom_bind_group(
            &self.device, &self.bloom_bind_group_layout,
            &self.hdr_view, &self.bloom_a_view, "bloom_threshold_bg",
        );
        self.bloom_blur_h_bind_group = build_bloom_bind_group(
            &self.device, &self.bloom_bind_group_layout,
            &self.bloom_a_view, &self.bloom_b_view, "bloom_blur_h_bg",
        );
        self.bloom_blur_v_bind_group = build_bloom_bind_group(
            &self.device, &self.bloom_bind_group_layout,
            &self.bloom_b_view, &self.bloom_a_view, "bloom_blur_v_bg",
        );
    }

    // ─────────────────────────── data uploads ───────────────────────────

    pub fn update_hole_instances(&mut self, positions: &[glam::Vec2], radius: f32) {
        let mesh = hole_mesh::build(48, 0.76, 1.0, 1.25);

        let gpu_verts: Vec<HoleVertex> = mesh
            .vertices
            .iter()
            .map(|v| HoleVertex {
                position: [v.position.x, v.position.y, v.position.z],
                normal: [v.normal.x, v.normal.y, v.normal.z],
            })
            .collect();
        let gpu_indices: Vec<u32> = mesh.indices.iter().map(|&i| i as u32).collect();

        self.hole_vertex_buffer = Some(self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("hole_verts"),
            contents: bytemuck::cast_slice(&gpu_verts),
            usage: wgpu::BufferUsages::VERTEX,
        }));
        self.hole_index_buffer = Some(self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("hole_indices"),
            contents: bytemuck::cast_slice(&gpu_indices),
            usage: wgpu::BufferUsages::INDEX,
        }));
        self.hole_index_count = gpu_indices.len() as u32;

        let instances: Vec<HoleInstance> = positions
            .iter()
            .map(|p| HoleInstance {
                position_radius: [p.x, p.y, 0.0, radius],
            })
            .collect();
        self.hole_instance_count = instances.len() as u32;

        let instance_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("hole_instances"),
            contents: bytemuck::cast_slice(&instances),
            usage: wgpu::BufferUsages::STORAGE,
        });

        self.hole_bind_group = Some(self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("hole_bind_group"),
            layout: &self.hole_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: instance_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: self.shadow_segment_buffer.as_entire_binding(),
                },
            ],
        }));
        self.hole_instance_buffer = Some(instance_buf);
    }

    pub fn update_shadow_segments(&mut self, segments: &[ShadowSegment]) {
        let count = ShadowSegmentCount { count: segments.len() as u32, _pad: [0; 3] };
        let count_bytes = bytemuck::bytes_of(&count);
        let seg_bytes: &[u8] = if segments.is_empty() {
            bytemuck::bytes_of(&ShadowSegment { ax: 0.0, ay: 0.0, az: 0.0, radius: 0.0, bx: 0.0, by: 0.0, bz: 0.0, rope_id: 0.0 })
        } else {
            bytemuck::cast_slice(segments)
        };

        let mut data = Vec::with_capacity(count_bytes.len() + seg_bytes.len());
        data.extend_from_slice(count_bytes);
        data.extend_from_slice(seg_bytes);

        self.shadow_segment_buffer = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("shadow_segments"),
            contents: &data,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });

        if let Some(inst_buf) = &self.hole_instance_buffer {
            self.hole_bind_group = Some(self.device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("hole_bind_group"),
                layout: &self.hole_bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: inst_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: self.shadow_segment_buffer.as_entire_binding(),
                    },
                ],
            }));
        }
    }

    pub fn update_rope_mesh(&mut self, vertices: &[RopeVertex], indices: &[u32]) {
        if vertices.is_empty() || indices.is_empty() {
            self.rope_index_count = 0;
            return;
        }

        let vert_bytes: &[u8] = bytemuck::cast_slice(vertices);
        let idx_bytes: &[u8] = bytemuck::cast_slice(indices);

        if vertices.len() > self.rope_vertex_capacity {
            let new_cap = (vertices.len() * 3 / 2).max(vertices.len());
            self.rope_vertex_buffer = Some(self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("rope_verts"),
                size: (new_cap * std::mem::size_of::<RopeVertex>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
            self.rope_vertex_capacity = new_cap;
        }

        if indices.len() > self.rope_index_capacity {
            let new_cap = (indices.len() * 3 / 2).max(indices.len());
            self.rope_index_buffer = Some(self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("rope_indices"),
                size: (new_cap * std::mem::size_of::<u32>()) as u64,
                usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
            self.rope_index_capacity = new_cap;
        }

        if let Some(buf) = &self.rope_vertex_buffer {
            self.queue.write_buffer(buf, 0, vert_bytes);
        }
        if let Some(buf) = &self.rope_index_buffer {
            self.queue.write_buffer(buf, 0, idx_bytes);
        }

        self.rope_index_count = indices.len() as u32;
    }

    // ─────────────────────────── render ───────────────────────────

    pub fn begin_frame(&mut self, time: f32, drag_active: bool, victory_time: f32, level_seed: f32, table_mode: u32) -> Option<FrameInFlight> {
        let aspect = self.width as f32 / self.height.max(1) as f32;
        let uniforms = build_frame_uniforms(&self.camera, aspect, SHADOW_MAP_SIZE, time, drag_active, victory_time, level_seed, self.highlight_hole, table_mode);
        self.queue.write_buffer(&self.frame_uniforms_buffer, 0, bytemuck::bytes_of(&uniforms));

        let output = match self.surface.get_current_texture() {
            Ok(t) => t,
            Err(wgpu::SurfaceError::Lost) => {
                self.resize(self.width, self.height);
                return None;
            }
            Err(wgpu::SurfaceError::OutOfMemory) => return None,
            Err(_) => return None,
        };
        let screen_view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("frame_encoder"),
        });

        self.encode_shadow_pass(&mut encoder);
        self.encode_hdr_pass(&mut encoder);
        self.encode_bloom_pass(&mut encoder);
        self.encode_composite_pass(&mut encoder, &screen_view);

        Some(FrameInFlight { encoder, screen_view, output })
    }

    pub fn end_frame(&self, frame: FrameInFlight) {
        self.queue.submit(std::iter::once(frame.encoder.finish()));
        frame.output.present();
    }

    // ─────────────────────────── shadow pass ───────────────────────────

    fn encode_shadow_pass(&self, encoder: &mut wgpu::CommandEncoder) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("shadow_pass"),
            color_attachments: &[],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.shadow_depth_view,
                depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Store }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });

        pass.set_bind_group(0, &self.shadow_bind_group, &[]);

        if let (Some(vb), Some(ib), Some(bg)) = (&self.hole_vertex_buffer, &self.hole_index_buffer, &self.hole_bind_group) {
            pass.set_pipeline(&self.shadow_hole_pipeline);
            pass.set_bind_group(1, bg, &[]);
            pass.set_vertex_buffer(0, vb.slice(..));
            pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.hole_index_count, 0, 0..self.hole_instance_count);
        }

    }

    // ─────────────────────────── HDR pass ───────────────────────────

    fn encode_hdr_pass(&self, encoder: &mut wgpu::CommandEncoder) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("hdr_pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &self.hdr_view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.0, g: 0.0, b: 0.0, a: 1.0 }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.depth_view,
                depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Store }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });

        pass.set_bind_group(0, &self.frame_bind_group, &[]);

        pass.set_pipeline(&self.table_pipeline);
        if let Some(bg) = &self.hole_bind_group {
            pass.set_bind_group(1, bg, &[]);
        }
        pass.draw(0..6, 0..1);

        if let (Some(vb), Some(ib), Some(bg)) = (&self.hole_vertex_buffer, &self.hole_index_buffer, &self.hole_bind_group) {
            pass.set_pipeline(&self.hole_pipeline);
            pass.set_bind_group(1, bg, &[]);
            pass.set_vertex_buffer(0, vb.slice(..));
            pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.hole_index_count, 0, 0..self.hole_instance_count);
        }

        if let (Some(vb), Some(ib)) = (&self.rope_vertex_buffer, &self.rope_index_buffer) {
            pass.set_pipeline(&self.rope_pipeline);
            pass.set_bind_group(0, &self.frame_bind_group, &[]);
            if let Some(bg) = &self.hole_bind_group {
                pass.set_bind_group(1, bg, &[]);
            }
            pass.set_vertex_buffer(0, vb.slice(..));
            pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.rope_index_count, 0, 0..1);
        }
    }

    // ─────────────────────────── bloom pass ───────────────────────────

    fn encode_bloom_pass(&self, encoder: &mut wgpu::CommandEncoder) {
        let bw = (self.width / BLOOM_DIVISOR).max(1);
        let bh = (self.height / BLOOM_DIVISOR).max(1);
        let wg_x = (bw + 7) / 8;
        let wg_y = (bh + 7) / 8;

        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bloom_threshold"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.bloom_threshold_pipeline);
            pass.set_bind_group(0, &self.empty_bind_group, &[]);
            pass.set_bind_group(1, &self.empty_bind_group, &[]);
            pass.set_bind_group(2, &self.empty_bind_group, &[]);
            pass.set_bind_group(3, &self.bloom_threshold_bind_group, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }

        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bloom_blur_h"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.bloom_blur_h_pipeline);
            pass.set_bind_group(0, &self.empty_bind_group, &[]);
            pass.set_bind_group(1, &self.empty_bind_group, &[]);
            pass.set_bind_group(2, &self.empty_bind_group, &[]);
            pass.set_bind_group(3, &self.bloom_blur_h_bind_group, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }

        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bloom_blur_v"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.bloom_blur_v_pipeline);
            pass.set_bind_group(0, &self.empty_bind_group, &[]);
            pass.set_bind_group(1, &self.empty_bind_group, &[]);
            pass.set_bind_group(2, &self.empty_bind_group, &[]);
            pass.set_bind_group(3, &self.bloom_blur_v_bind_group, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }
    }

    // ─────────────────────────── composite pass ───────────────────────────

    fn encode_composite_pass(&self, encoder: &mut wgpu::CommandEncoder, screen_view: &wgpu::TextureView) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("composite_pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: screen_view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
        });

        pass.set_pipeline(&self.post_pipeline);
        pass.set_bind_group(0, &self.frame_bind_group, &[]);
        pass.set_bind_group(1, &self.empty_bind_group, &[]);
        pass.set_bind_group(2, &self.post_bind_group, &[]);
        pass.draw(0..3, 0..1);
    }
}
