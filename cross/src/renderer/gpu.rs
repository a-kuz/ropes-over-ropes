use bytemuck;
use std::borrow::Cow;
use std::sync::Arc;
use wgpu;
use wgpu::util::DeviceExt;

use super::camera::Camera;
use super::frame_types::*;
use super::hole_mesh;
use super::passes::build_frame_uniforms;

const SHADOW_MAP_SIZE: u32 = 2048;
const BLOOM_DIVISOR: u32 = 16;

pub struct FrameInFlight {
    pub encoder: wgpu::CommandEncoder,
    pub screen_view: wgpu::TextureView,
    output: wgpu::SurfaceTexture,
}

#[derive(Clone, Debug, Default)]
pub struct GpuTimings {
    pub shadow_ms: f32,
    pub hdr_ms: f32,
    pub bloom_ms: f32,
    pub total_ms: f32,
}

#[derive(Clone, Copy, Default)]
pub struct DrawFlags {
    pub skip_table: bool,
    pub skip_holes: bool,
    pub skip_ropes: bool,
    pub table_shadow_mode: u8,
}

const TIMESTAMP_QUERY_COUNT: u32 = 10;

const TS_SHADOW_BEGIN: u32 = 0;
const TS_SHADOW_END: u32 = 1;
const TS_HDR_BEGIN: u32 = 2;
const TS_HDR_END: u32 = 3;
const TS_BLOOM_TH_BEGIN: u32 = 4;
const TS_BLOOM_TH_END: u32 = 5;
const TS_BLOOM_H_BEGIN: u32 = 6;
const TS_BLOOM_H_END: u32 = 7;
const TS_BLOOM_V_BEGIN: u32 = 8;
const TS_BLOOM_V_END: u32 = 9;

// ─────────────────────────── GpuRenderer ───────────────────────────

pub struct GpuRenderer {
    device: wgpu::Device,
    queue: wgpu::Queue,
    surface: wgpu::Surface<'static>,
    surface_config: wgpu::SurfaceConfiguration,

    table_pipeline: wgpu::RenderPipeline,
    board_pipeline: wgpu::RenderPipeline,
    hole_pipeline: wgpu::RenderPipeline,
    rope_pipeline: wgpu::RenderPipeline,
    post_pipeline: wgpu::RenderPipeline,
    shadow_board_pipeline: wgpu::RenderPipeline,
    shadow_rope_pipeline: wgpu::RenderPipeline,
    shadow_hole_pipeline: wgpu::RenderPipeline,
    planar_mask_pipeline: wgpu::ComputePipeline,
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
    planar_mask_texture: wgpu::Texture,
    planar_mask_view: wgpu::TextureView,
    planar_mask_w: u32,
    planar_mask_h: u32,
    depth_view: wgpu::TextureView,

    frame_uniforms_buffer: wgpu::Buffer,
    hole_instance_buffer: Option<wgpu::Buffer>,
    hole_vertex_buffer: Option<wgpu::Buffer>,
    hole_index_buffer: Option<wgpu::Buffer>,
    board_vertex_buffer: Option<wgpu::Buffer>,
    board_index_buffer: Option<wgpu::Buffer>,
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
    planar_mask_bind_group: wgpu::BindGroup,
    bloom_threshold_bind_group: wgpu::BindGroup,
    bloom_blur_h_bind_group: wgpu::BindGroup,
    bloom_blur_v_bind_group: wgpu::BindGroup,

    shadow_sampler: wgpu::Sampler,
    linear_sampler: wgpu::Sampler,
    noise_view: wgpu::TextureView,
    noise_sampler: wgpu::Sampler,
    empty_bind_group: wgpu::BindGroup,

    wood_baked_texture: wgpu::Texture,
    wood_baked_view: wgpu::TextureView,
    wood_baked_storage_view: wgpu::TextureView,
    bake_wood_pipeline: wgpu::ComputePipeline,
    bake_wood_bind_group_layout: wgpu::BindGroupLayout,
    bake_wood_bind_group: wgpu::BindGroup,
    pub needs_wood_bake: bool,

    hole_index_count: u32,
    hole_instance_count: u32,
    board_index_count: u32,
    rope_index_count: u32,

    hole_mask_view: wgpu::TextureView,
    hole_mask_bounds: [f32; 4],

    pub camera: Camera,
    pub highlight_hole: i32,
    pub render_scale: f32,
    pub rope_material: RopeMaterialSettings,
    pub lighting: LightingSettings,
    pub visual: VisualSettings,
    pub table: TableSettings,
    pub cartoon: CartoonSettings,
    pub cap: CapSettings,
    pub worm: WormSettings,
    pub ssr: SsrSettings,
    width: u32,
    height: u32,

    timestamp_supported: bool,
    timestamp_query_set: Option<wgpu::QuerySet>,
    timestamp_resolve_buffer: Option<wgpu::Buffer>,
    timestamp_readback_buffer: Option<wgpu::Buffer>,
    timestamp_period: f32,
    pub gpu_timings: GpuTimings,
    timestamp_pending: bool,
    frame_index: u64,
    surface_error_streak: u32,
    pub draw_flags: DrawFlags,
}

// ─────────────────────────── vertex layouts ───────────────────────────

fn rope_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<RopeVertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x3,
                offset: 0,
                shader_location: 0,
            },
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x3,
                offset: 12,
                shader_location: 1,
            },
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x3,
                offset: 24,
                shader_location: 2,
            },
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x2,
                offset: 36,
                shader_location: 3,
            },
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x4,
                offset: 44,
                shader_location: 4,
            },
        ],
    }
}

fn hole_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<HoleVertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x3,
                offset: 0,
                shader_location: 0,
            },
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x3,
                offset: 12,
                shader_location: 1,
            },
        ],
    }
}

fn board_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<BoardVertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x3,
                offset: 0,
                shader_location: 0,
            },
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x3,
                offset: 12,
                shader_location: 1,
            },
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x2,
                offset: 24,
                shader_location: 2,
            },
        ],
    }
}

// ─────────────────────────── texture helpers ───────────────────────────

fn create_depth_texture(device: &wgpu::Device, w: u32, h: u32, label: &str) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        })
        .create_view(&wgpu::TextureViewDescriptor::default())
}

fn create_hdr_texture(
    device: &wgpu::Device,
    w: u32,
    h: u32,
    label: &str,
) -> (wgpu::Texture, wgpu::TextureView) {
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
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

fn create_planar_mask_texture(
    device: &wgpu::Device,
    w: u32,
    h: u32,
    label: &str,
) -> (wgpu::Texture, wgpu::TextureView) {
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba16Float,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::STORAGE_BINDING,
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
                visibility: wgpu::ShaderStages::VERTEX
                    | wgpu::ShaderStages::FRAGMENT
                    | wgpu::ShaderStages::COMPUTE,
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
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 5,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 6,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 7,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
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
                visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
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
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Depth,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
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
    planar_mask_view: &wgpu::TextureView,
    noise_view: &wgpu::TextureView,
    noise_sampler: &wgpu::Sampler,
    wood_baked_view: &wgpu::TextureView,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("frame_bind_group"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniforms_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(shadow_view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(shadow_sampler),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::Sampler(linear_sampler),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::TextureView(planar_mask_view),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: wgpu::BindingResource::TextureView(noise_view),
            },
            wgpu::BindGroupEntry {
                binding: 6,
                resource: wgpu::BindingResource::Sampler(noise_sampler),
            },
            wgpu::BindGroupEntry {
                binding: 7,
                resource: wgpu::BindingResource::TextureView(wood_baked_view),
            },
        ],
    })
}

fn build_post_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    hdr_view: &wgpu::TextureView,
    bloom_view: &wgpu::TextureView,
    sampler: &wgpu::Sampler,
    depth_view: &wgpu::TextureView,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("post_bind_group"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(hdr_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(bloom_view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(sampler),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::TextureView(depth_view),
            },
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
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(src),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(dst),
            },
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
            depth_compare: if depth_write {
                wgpu::CompareFunction::LessEqual
            } else {
                wgpu::CompareFunction::Always
            },
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
            cull_mode: None,
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::Less,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
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

        let timestamp_supported = adapter.features().contains(wgpu::Features::TIMESTAMP_QUERY);
        let mut required_features = wgpu::Features::empty();
        if timestamp_supported {
            required_features |= wgpu::Features::TIMESTAMP_QUERY;
        }

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("uzls_device"),
                    required_features,
                    required_limits: wgpu::Limits::default(),
                    memory_hints: wgpu::MemoryHints::Performance,
                },
                None,
            )
            .await
            .expect("failed to create device");

        let timestamp_period = if timestamp_supported {
            queue.get_timestamp_period()
        } else {
            0.0
        };

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

        let noise_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("noise_sampler"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::Repeat,
            ..Default::default()
        });

        let noise_view = {
            const NOISE_SIZE: u32 = 2048;
            const GRID: u32 = 128;
            let tex = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("noise_texture"),
                size: wgpu::Extent3d {
                    width: NOISE_SIZE,
                    height: NOISE_SIZE,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rg8Unorm,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            });

            fn lcg(s: &mut u32) -> f32 {
                *s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (*s >> 8) as f32 / 16_777_216.0
            }
            fn grid_val(grid: &[f32], gx: i32, gy: i32, sz: u32, ch: usize) -> f32 {
                let wx = ((gx % sz as i32) + sz as i32) as u32 % sz;
                let wy = ((gy % sz as i32) + sz as i32) as u32 % sz;
                grid[((wy * sz + wx) * 2 + ch as u32) as usize]
            }
            fn smoothstep_f(t: f32) -> f32 {
                t * t * (3.0 - 2.0 * t)
            }

            fn value_noise(grid: &[f32], x: f32, y: f32, freq: u32, sz: u32, ch: usize) -> f32 {
                let fx = x * freq as f32;
                let fy = y * freq as f32;
                let ix = fx.floor() as i32;
                let iy = fy.floor() as i32;
                let tx = smoothstep_f(fx - fx.floor());
                let ty = smoothstep_f(fy - fy.floor());
                let c00 = grid_val(grid, ix, iy, sz, ch);
                let c10 = grid_val(grid, ix + 1, iy, sz, ch);
                let c01 = grid_val(grid, ix, iy + 1, sz, ch);
                let c11 = grid_val(grid, ix + 1, iy + 1, sz, ch);
                let a = c00 + (c10 - c00) * tx;
                let b = c01 + (c11 - c01) * tx;
                a + (b - a) * ty
            }

            let mut rng_state: u32 = 0xDEAD_BEEF;
            let mut grid_data = vec![0.0f32; (GRID * GRID * 2) as usize];
            for v in grid_data.iter_mut() {
                *v = lcg(&mut rng_state);
            }

            let mut pixels = vec![0u8; (NOISE_SIZE * NOISE_SIZE * 2) as usize];
            for y in 0..NOISE_SIZE {
                for x in 0..NOISE_SIZE {
                    let u = x as f32 / NOISE_SIZE as f32;
                    let v = y as f32 / NOISE_SIZE as f32;
                    let idx = ((y * NOISE_SIZE + x) * 2) as usize;
                    for ch in 0..2usize {
                        let o1 = value_noise(&grid_data, u, v, 16, GRID, ch);
                        let o2 = value_noise(&grid_data, u, v, 37, GRID, ch);
                        let o3 = value_noise(&grid_data, u, v, 79, GRID, ch);
                        let o4 = value_noise(&grid_data, u, v, 128, GRID, ch);
                        let fbm = o1 * 0.15 + o2 * 0.30 + o3 * 0.30 + o4 * 0.15;
                        let white = lcg(&mut rng_state);
                        let val = fbm * 0.85 + white * 0.15;
                        pixels[idx + ch] = (val.clamp(0.0, 1.0) * 255.0) as u8;
                    }
                }
            }
            queue.write_texture(
                wgpu::TexelCopyTextureInfo {
                    texture: &tex,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                &pixels,
                wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(NOISE_SIZE * 2),
                    rows_per_image: None,
                },
                wgpu::Extent3d {
                    width: NOISE_SIZE,
                    height: NOISE_SIZE,
                    depth_or_array_layers: 1,
                },
            );
            tex.create_view(&wgpu::TextureViewDescriptor::default())
        };

        // ── textures ──

        let render_scale: f32 = 0.5;
        let rw = ((width as f32 * render_scale) as u32).clamp(1, 8192);
        let rh = ((height as f32 * render_scale) as u32).clamp(1, 8192);

        let shadow_depth_view =
            create_depth_texture(&device, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, "shadow_depth");
        let (hdr_texture, hdr_view) = create_hdr_texture(&device, rw, rh, "hdr");
        let depth_view = create_depth_texture(&device, rw, rh, "scene_depth");

        let bloom_w = (rw / BLOOM_DIVISOR).max(1);
        let bloom_h = (rh / BLOOM_DIVISOR).max(1);
        let (bloom_a_texture, bloom_a_view) =
            create_hdr_texture(&device, bloom_w, bloom_h, "bloom_a");
        let (bloom_b_texture, bloom_b_view) =
            create_hdr_texture(&device, bloom_w, bloom_h, "bloom_b");
        let planar_mask_w = (rw / 4).clamp(128, 1024);
        let planar_mask_h = (rh / 4).clamp(128, 1024);
        let (planar_mask_texture, planar_mask_view) =
            create_planar_mask_texture(&device, planar_mask_w, planar_mask_h, "planar_mask");

        // ── buffers ──

        let frame_uniforms_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("frame_uniforms"),
            size: std::mem::size_of::<FrameUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let empty_seg = ShadowSegment {
            ax: 0.0,
            ay: 0.0,
            az: 0.0,
            radius: 0.0,
            bx: 0.0,
            by: 0.0,
            bz: 0.0,
            rope_id: 0.0,
        };
        let mut initial_data: Vec<u8> = Vec::new();
        initial_data.extend_from_slice(bytemuck::bytes_of(&ShadowSegmentCount {
            count: 0,
            _pad: [0; 3],
        }));
        initial_data.extend_from_slice(bytemuck::bytes_of(&empty_seg));
        let shadow_segment_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("shadow_segments"),
            contents: &initial_data,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });

        let hole_mask_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("hole_mask_placeholder"),
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &hole_mask_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &[255u8],
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(1),
                rows_per_image: None,
            },
            wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
        );
        let hole_mask_view = hole_mask_tex.create_view(&wgpu::TextureViewDescriptor::default());

        // ── bind group layouts ──

        let frame_bind_group_layout = create_frame_bind_group_layout(&device);
        let hole_bind_group_layout = create_hole_bind_group_layout(&device);
        let post_bind_group_layout = create_post_bind_group_layout(&device);
        let bloom_bind_group_layout = create_bloom_bind_group_layout(&device);

        let shadow_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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

        // ── Wood baked texture ──
        const WOOD_TEX_SIZE: u32 = 8192;
        let wood_baked_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("wood_baked"),
            size: wgpu::Extent3d { width: WOOD_TEX_SIZE, height: WOOD_TEX_SIZE, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::STORAGE_BINDING,
            view_formats: &[],
        });
        let wood_baked_view = wood_baked_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let wood_baked_storage_view = wood_baked_texture.create_view(&wgpu::TextureViewDescriptor {
            format: Some(wgpu::TextureFormat::Rgba8Unorm),
            ..Default::default()
        });

        // ── Bake wood pipeline ──
        let bake_wood_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("bake_wood_bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 8,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::StorageTexture {
                        access: wgpu::StorageTextureAccess::WriteOnly,
                        format: wgpu::TextureFormat::Rgba8Unorm,
                        view_dimension: wgpu::TextureViewDimension::D2,
                    },
                    count: None,
                },
            ],
        });
        let bake_wood_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("bake_wood_bg"),
            layout: &bake_wood_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: frame_uniforms_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 8, resource: wgpu::BindingResource::TextureView(&wood_baked_storage_view) },
            ],
        });
        let bake_wood_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("bake_wood_layout"),
            bind_group_layouts: &[&bake_wood_bind_group_layout],
            push_constant_ranges: &[],
        });
        let bake_wood_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("bake_wood"),
            layout: Some(&bake_wood_pipeline_layout),
            module: &shader,
            entry_point: Some("bake_wood_kernel"),
            compilation_options: Default::default(),
            cache: None,
        });

        let frame_bind_group = build_frame_bind_group(
            &device,
            &frame_bind_group_layout,
            &frame_uniforms_buffer,
            &shadow_depth_view,
            &shadow_sampler,
            &linear_sampler,
            &planar_mask_view,
            &noise_view,
            &noise_sampler,
            &wood_baked_view,
        );

        let post_bind_group = build_post_bind_group(
            &device,
            &post_bind_group_layout,
            &hdr_view,
            &bloom_a_view,
            &linear_sampler,
            &depth_view,
        );

        let bloom_threshold_bind_group = build_bloom_bind_group(
            &device,
            &bloom_bind_group_layout,
            &hdr_view,
            &bloom_a_view,
            "bloom_threshold_bg",
        );
        let bloom_blur_h_bind_group = build_bloom_bind_group(
            &device,
            &bloom_bind_group_layout,
            &bloom_a_view,
            &bloom_b_view,
            "bloom_blur_h_bg",
        );
        let bloom_blur_v_bind_group = build_bloom_bind_group(
            &device,
            &bloom_bind_group_layout,
            &bloom_b_view,
            &bloom_a_view,
            "bloom_blur_v_bg",
        );
        let planar_mask_bind_group = build_bloom_bind_group(
            &device,
            &bloom_bind_group_layout,
            &hdr_view,
            &planar_mask_view,
            "planar_mask_bg",
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

        let empty_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
            bind_group_layouts: &[
                &frame_bind_group_layout,
                &empty_bind_group_layout,
                &post_bind_group_layout,
            ],
            push_constant_ranges: &[],
        });

        let bloom_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("bloom_layout"),
            bind_group_layouts: &[
                &empty_bind_group_layout,
                &empty_bind_group_layout,
                &empty_bind_group_layout,
                &bloom_bind_group_layout,
            ],
            push_constant_ranges: &[],
        });
        let planar_mask_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("planar_mask_layout"),
            bind_group_layouts: &[
                &frame_bind_group_layout,
                &hole_bind_group_layout,
                &empty_bind_group_layout,
                &bloom_bind_group_layout,
            ],
            push_constant_ranges: &[],
        });

        // ── render pipelines ──

        let hdr_format = wgpu::TextureFormat::Rgba16Float;
        let depth_fmt = wgpu::TextureFormat::Depth32Float;

        let table_pipeline = build_render_pipeline(
            &device,
            "table_pipeline",
            &frame_hole_layout,
            &shader,
            "table_vertex",
            "table_fragment",
            &[],
            hdr_format,
            Some(depth_fmt),
            true,
            None,
        );

        let board_pipeline = build_render_pipeline(
            &device,
            "board_pipeline",
            &frame_hole_layout,
            &shader,
            "board_vertex",
            "board_fragment",
            &[board_vertex_layout()],
            hdr_format,
            Some(depth_fmt),
            true,
            Some(wgpu::Face::Back),
        );

        let hole_pipeline = build_render_pipeline(
            &device,
            "hole_pipeline",
            &frame_hole_layout,
            &shader,
            "hole_vertex",
            "hole_fragment",
            &[hole_vertex_layout()],
            hdr_format,
            Some(depth_fmt),
            true,
            Some(wgpu::Face::Back),
        );

        let rope_pipeline = build_render_pipeline(
            &device,
            "rope_pipeline",
            &frame_hole_layout,
            &shader,
            "rope_vertex",
            "rope_fragment",
            &[rope_vertex_layout()],
            hdr_format,
            Some(depth_fmt),
            true,
            None,
        );

        let post_pipeline = build_render_pipeline(
            &device,
            "post_pipeline",
            &post_layout,
            &shader,
            "fullscreen_vertex",
            "post_fragment",
            &[],
            surface_format,
            None,
            false,
            None,
        );

        let shadow_only_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("shadow_only_layout"),
            bind_group_layouts: &[&shadow_bind_group_layout],
            push_constant_ranges: &[],
        });

        let shadow_hole_pl_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("shadow_hole_pl_layout"),
                bind_group_layouts: &[&shadow_bind_group_layout, &hole_bind_group_layout],
                push_constant_ranges: &[],
            });

        let shadow_rope_pipeline = build_shadow_pipeline(
            &device,
            "shadow_rope",
            &shadow_only_layout,
            &shader,
            "rope_shadow_vertex",
            &[rope_vertex_layout()],
        );

        let shadow_board_pipeline = build_shadow_pipeline(
            &device,
            "shadow_board",
            &shadow_only_layout,
            &shader,
            "board_shadow_vertex",
            &[board_vertex_layout()],
        );

        let shadow_hole_pipeline = build_shadow_pipeline(
            &device,
            "shadow_hole",
            &shadow_hole_pl_layout,
            &shader,
            "hole_shadow_vertex",
            &[hole_vertex_layout()],
        );

        let bloom_threshold_pipeline = build_compute_pipeline(
            &device,
            "bloom_threshold",
            &bloom_layout,
            &shader,
            "bloom_threshold",
        );
        let bloom_blur_h_pipeline = build_compute_pipeline(
            &device,
            "bloom_blur_h",
            &bloom_layout,
            &shader,
            "bloom_blur_h",
        );
        let bloom_blur_v_pipeline = build_compute_pipeline(
            &device,
            "bloom_blur_v",
            &bloom_layout,
            &shader,
            "bloom_blur_v",
        );
        let planar_mask_pipeline = build_compute_pipeline(
            &device,
            "planar_mask",
            &planar_mask_layout,
            &shader,
            "planar_shadow_mask_cs",
        );

        Self {
            device,
            queue,
            surface,
            surface_config,

            table_pipeline,
            board_pipeline,
            hole_pipeline,
            rope_pipeline,
            post_pipeline,
            shadow_board_pipeline,
            shadow_rope_pipeline,
            shadow_hole_pipeline,
            planar_mask_pipeline,
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
            planar_mask_texture,
            planar_mask_view,
            planar_mask_w,
            planar_mask_h,
            depth_view,

            frame_uniforms_buffer,
            hole_instance_buffer: None,
            hole_vertex_buffer: None,
            hole_index_buffer: None,
            board_vertex_buffer: None,
            board_index_buffer: None,
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
            planar_mask_bind_group,
            bloom_threshold_bind_group,
            bloom_blur_h_bind_group,
            bloom_blur_v_bind_group,

            shadow_sampler,
            linear_sampler,
            noise_view,
            noise_sampler,
            empty_bind_group,

            wood_baked_texture,
            wood_baked_view,
            wood_baked_storage_view,
            bake_wood_pipeline,
            bake_wood_bind_group_layout,
            bake_wood_bind_group,
            needs_wood_bake: true,

            hole_index_count: 0,
            hole_instance_count: 0,
            board_index_count: 0,
            rope_index_count: 0,

            hole_mask_view,
            hole_mask_bounds: [0.0; 4],

            camera: Camera::default(),
            highlight_hole: -1,
            render_scale,
            rope_material: RopeMaterialSettings::default(),
            lighting: LightingSettings::default(),
            visual: VisualSettings::default(),
            table: TableSettings::default(),
            cartoon: CartoonSettings::default(),
            cap: CapSettings::default(),
            worm: WormSettings::default(),
            ssr: SsrSettings::default(),
            width,
            height,

            timestamp_supported,
            timestamp_query_set: None,
            timestamp_resolve_buffer: None,
            timestamp_readback_buffer: None,
            timestamp_period,
            gpu_timings: GpuTimings::default(),
            timestamp_pending: false,
            frame_index: 0,
            surface_error_streak: 0,
            draw_flags: DrawFlags::default(),
        }
    }

    fn ensure_timestamp_resources(&mut self) {
        if !self.timestamp_supported || self.timestamp_query_set.is_some() {
            return;
        }
        self.timestamp_query_set = Some(self.device.create_query_set(&wgpu::QuerySetDescriptor {
            label: Some("timestamp_queries"),
            ty: wgpu::QueryType::Timestamp,
            count: TIMESTAMP_QUERY_COUNT,
        }));
        let resolve_size = (TIMESTAMP_QUERY_COUNT as u64) * 8;
        self.timestamp_resolve_buffer = Some(self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("timestamp_resolve"),
            size: resolve_size,
            usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        }));
        self.timestamp_readback_buffer = Some(self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("timestamp_readback"),
            size: resolve_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }));
    }

    pub fn surface_size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    pub fn device(&self) -> &wgpu::Device {
        &self.device
    }
    pub fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }
    pub fn surface_format(&self) -> wgpu::TextureFormat {
        self.surface_config.format
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        self.width = width;
        self.height = height;
        self.surface_config.width = width;
        self.surface_config.height = height;
        self.surface.configure(&self.device, &self.surface_config);
        self.rebuild_render_textures();
    }

    pub fn set_render_scale(&mut self, scale: f32) {
        let scale = scale.clamp(0.25, 2.0);
        if (scale - self.render_scale).abs() < 0.001 {
            return;
        }
        self.render_scale = scale;
        self.rebuild_render_textures();
    }

    fn rebuild_render_textures(&mut self) {
        let rw = ((self.width as f32 * self.render_scale) as u32).clamp(1, 8192);
        let rh = ((self.height as f32 * self.render_scale) as u32).clamp(1, 8192);

        let (hdr_texture, hdr_view) = create_hdr_texture(&self.device, rw, rh, "hdr");
        self.hdr_texture = hdr_texture;
        self.hdr_view = hdr_view;
        self.depth_view = create_depth_texture(&self.device, rw, rh, "scene_depth");

        let bloom_w = (rw / BLOOM_DIVISOR).max(1);
        let bloom_h = (rh / BLOOM_DIVISOR).max(1);
        let (ba_tex, ba_view) = create_hdr_texture(&self.device, bloom_w, bloom_h, "bloom_a");
        let (bb_tex, bb_view) = create_hdr_texture(&self.device, bloom_w, bloom_h, "bloom_b");
        self.bloom_a_texture = ba_tex;
        self.bloom_a_view = ba_view;
        self.bloom_b_texture = bb_tex;
        self.bloom_b_view = bb_view;
        self.planar_mask_w = (rw / 4).clamp(128, 1024);
        self.planar_mask_h = (rh / 4).clamp(128, 1024);
        let (pm_tex, pm_view) = create_planar_mask_texture(
            &self.device,
            self.planar_mask_w,
            self.planar_mask_h,
            "planar_mask",
        );
        self.planar_mask_texture = pm_tex;
        self.planar_mask_view = pm_view;

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
            &self.planar_mask_view,
            &self.noise_view,
            &self.noise_sampler,
            &self.wood_baked_view,
        );

        self.post_bind_group = build_post_bind_group(
            &self.device,
            &self.post_bind_group_layout,
            &self.hdr_view,
            &self.bloom_a_view,
            &self.linear_sampler,
            &self.depth_view,
        );

        self.bloom_threshold_bind_group = build_bloom_bind_group(
            &self.device,
            &self.bloom_bind_group_layout,
            &self.hdr_view,
            &self.bloom_a_view,
            "bloom_threshold_bg",
        );
        self.bloom_blur_h_bind_group = build_bloom_bind_group(
            &self.device,
            &self.bloom_bind_group_layout,
            &self.bloom_a_view,
            &self.bloom_b_view,
            "bloom_blur_h_bg",
        );
        self.bloom_blur_v_bind_group = build_bloom_bind_group(
            &self.device,
            &self.bloom_bind_group_layout,
            &self.bloom_b_view,
            &self.bloom_a_view,
            "bloom_blur_v_bg",
        );
        self.planar_mask_bind_group = build_bloom_bind_group(
            &self.device,
            &self.bloom_bind_group_layout,
            &self.hdr_view,
            &self.planar_mask_view,
            "planar_mask_bg",
        );
    }

    // ─────────────────────────── data uploads ───────────────────────────

    pub fn update_hole_instances(
        &mut self,
        positions: &[glam::Vec2],
        elevations: &[f32],
        radius: f32,
    ) {
        self.update_hole_instances_with_shape(positions, elevations, radius, false);
    }

    pub fn update_hole_instances_square(
        &mut self,
        positions: &[glam::Vec2],
        elevations: &[f32],
        radius: f32,
    ) {
        self.update_hole_instances_with_shape(positions, elevations, radius, true);
    }

    fn update_hole_instances_with_shape(
        &mut self,
        positions: &[glam::Vec2],
        elevations: &[f32],
        radius: f32,
        square: bool,
    ) {
        let mesh = if square {
            hole_mesh::build_square(0.76, 1.0, 1.25, 4)
        } else {
            hole_mesh::build(48, 0.76, 1.0, 1.25)
        };

        let gpu_verts: Vec<HoleVertex> = mesh
            .vertices
            .iter()
            .map(|v| HoleVertex {
                position: [v.position.x, v.position.y, v.position.z],
                normal: [v.normal.x, v.normal.y, v.normal.z],
            })
            .collect();
        let gpu_indices: Vec<u32> = mesh.indices.iter().map(|&i| i as u32).collect();

        self.hole_vertex_buffer = Some(self.device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("hole_verts"),
                contents: bytemuck::cast_slice(&gpu_verts),
                usage: wgpu::BufferUsages::VERTEX,
            },
        ));
        self.hole_index_buffer = Some(self.device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("hole_indices"),
                contents: bytemuck::cast_slice(&gpu_indices),
                usage: wgpu::BufferUsages::INDEX,
            },
        ));
        self.hole_index_count = gpu_indices.len() as u32;

        let instances: Vec<HoleInstance> = positions
            .iter()
            .enumerate()
            .map(|(i, p)| HoleInstance {
                position_radius: [p.x, p.y, elevations.get(i).copied().unwrap_or(0.0), radius],
            })
            .collect();
        self.hole_instance_count = instances.len() as u32;

        let instance_buf = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("hole_instances"),
                contents: bytemuck::cast_slice(&instances),
                usage: wgpu::BufferUsages::STORAGE,
            });

        if square {
            self.bake_hole_mask_square(positions, radius);
        } else {
            self.bake_hole_mask(positions, radius);
        }

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
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::TextureView(&self.hole_mask_view),
                },
            ],
        }));
        self.hole_instance_buffer = Some(instance_buf);
    }

    fn bake_hole_mask(&mut self, positions: &[glam::Vec2], radius: f32) {
        self.bake_hole_mask_impl(positions, radius, false);
    }

    fn bake_hole_mask_square(&mut self, positions: &[glam::Vec2], radius: f32) {
        self.bake_hole_mask_impl(positions, radius, true);
    }

    fn bake_hole_mask_impl(&mut self, positions: &[glam::Vec2], radius: f32, square: bool) {
        if positions.is_empty() {
            return;
        }
        let inner_r = radius * 0.76;
        let margin = radius * 2.0;

        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;
        for p in positions {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
        min_x -= margin;
        min_y -= margin;
        max_x += margin;
        max_y += margin;

        let world_w = max_x - min_x;
        let world_h = max_y - min_y;
        let pixels_per_unit = 64.0;
        let tw = ((world_w * pixels_per_unit) as u32).clamp(4, 2048);
        let th = ((world_h * pixels_per_unit) as u32).clamp(4, 2048);

        let sdf_range = inner_r * 1.5;
        let mut pixels = vec![255u8; (tw * th) as usize];
        for y in 0..th {
            for x in 0..tw {
                let wx = (x as f32 + 0.5) / tw as f32 * world_w + min_x;
                let wy = (y as f32 + 0.5) / th as f32 * world_h + min_y;
                let mut min_dist = f32::MAX;
                for p in positions {
                    let dx = (wx - p.x).abs();
                    let dy = (wy - p.y).abs();
                    let dist = if square {
                        dx.max(dy) - inner_r
                    } else {
                        (dx * dx + dy * dy).sqrt() - inner_r
                    };
                    if dist < min_dist {
                        min_dist = dist;
                    }
                }
                let norm = ((min_dist / sdf_range) * 0.5 + 0.5).clamp(0.0, 1.0);
                pixels[(y * tw + x) as usize] = (norm * 255.0) as u8;
            }
        }

        let bytes_per_row = tw;
        let aligned_bpr = (bytes_per_row + 255) & !255;
        let mut aligned_pixels = vec![255u8; (aligned_bpr * th) as usize];
        for y in 0..th {
            let src_start = (y * tw) as usize;
            let dst_start = (y * aligned_bpr) as usize;
            aligned_pixels[dst_start..dst_start + tw as usize]
                .copy_from_slice(&pixels[src_start..src_start + tw as usize]);
        }

        let tex = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("hole_mask"),
            size: wgpu::Extent3d {
                width: tw,
                height: th,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        self.queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &aligned_pixels,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(aligned_bpr),
                rows_per_image: None,
            },
            wgpu::Extent3d {
                width: tw,
                height: th,
                depth_or_array_layers: 1,
            },
        );
        self.hole_mask_view = tex.create_view(&wgpu::TextureViewDescriptor::default());
        self.hole_mask_bounds = [min_x, min_y, max_x, max_y];
    }

    pub fn update_board_mesh(&mut self, boards: &[(glam::Vec2, glam::Vec2, f32)]) {
        if boards.is_empty() {
            self.board_vertex_buffer = None;
            self.board_index_buffer = None;
            self.board_index_count = 0;
            return;
        }

        let mut vertices: Vec<BoardVertex> = Vec::with_capacity(boards.len() * 20);
        let mut indices: Vec<u32> = Vec::with_capacity(boards.len() * 30);

        for &(center, size, elevation) in boards {
            let hw = size.x * 0.5;
            let hh = size.y * 0.5;
            let cx = center.x;
            let cy = center.y;
            let base = vertices.len() as u32;
            vertices.push(BoardVertex {
                position: [cx - hw, cy - hh, elevation],
                normal: [0.0, 0.0, 1.0],
                world_xy: [cx - hw, cy - hh],
            });
            vertices.push(BoardVertex {
                position: [cx + hw, cy - hh, elevation],
                normal: [0.0, 0.0, 1.0],
                world_xy: [cx + hw, cy - hh],
            });
            vertices.push(BoardVertex {
                position: [cx + hw, cy + hh, elevation],
                normal: [0.0, 0.0, 1.0],
                world_xy: [cx + hw, cy + hh],
            });
            vertices.push(BoardVertex {
                position: [cx - hw, cy + hh, elevation],
                normal: [0.0, 0.0, 1.0],
                world_xy: [cx - hw, cy + hh],
            });
            indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);

            let sides = [
                ((cx - hw, cy - hh), (cx + hw, cy - hh), [0.0, -1.0, 0.0]),
                ((cx + hw, cy - hh), (cx + hw, cy + hh), [1.0, 0.0, 0.0]),
                ((cx + hw, cy + hh), (cx - hw, cy + hh), [0.0, 1.0, 0.0]),
                ((cx - hw, cy + hh), (cx - hw, cy - hh), [-1.0, 0.0, 0.0]),
            ];
            for side in sides {
                let sb = vertices.len() as u32;
                let ((p0x, p0y), (p1x, p1y), [nx, ny, nz]) = side;
                vertices.push(BoardVertex {
                    position: [p0x, p0y, elevation],
                    normal: [nx, ny, nz],
                    world_xy: [p0x, p0y],
                });
                vertices.push(BoardVertex {
                    position: [p1x, p1y, elevation],
                    normal: [nx, ny, nz],
                    world_xy: [p1x, p1y],
                });
                vertices.push(BoardVertex {
                    position: [p1x, p1y, 0.0],
                    normal: [nx, ny, nz],
                    world_xy: [p1x, p1y],
                });
                vertices.push(BoardVertex {
                    position: [p0x, p0y, 0.0],
                    normal: [nx, ny, nz],
                    world_xy: [p0x, p0y],
                });
                indices.extend_from_slice(&[sb, sb + 2, sb + 1, sb, sb + 3, sb + 2]);
            }
        }

        self.board_vertex_buffer = Some(self.device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("board_verts"),
                contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsages::VERTEX,
            },
        ));
        self.board_index_buffer = Some(self.device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("board_indices"),
                contents: bytemuck::cast_slice(&indices),
                usage: wgpu::BufferUsages::INDEX,
            },
        ));
        self.board_index_count = indices.len() as u32;
    }

    pub fn update_shadow_segments(&mut self, segments: &[ShadowSegment]) {
        let count = ShadowSegmentCount {
            count: segments.len() as u32,
            _pad: [0; 3],
        };
        let count_bytes = bytemuck::bytes_of(&count);
        let seg_bytes: &[u8] = if segments.is_empty() {
            bytemuck::bytes_of(&ShadowSegment {
                ax: 0.0,
                ay: 0.0,
                az: 0.0,
                radius: 0.0,
                bx: 0.0,
                by: 0.0,
                bz: 0.0,
                rope_id: 0.0,
            })
        } else {
            bytemuck::cast_slice(segments)
        };

        let mut data = Vec::with_capacity(count_bytes.len() + seg_bytes.len());
        data.extend_from_slice(count_bytes);
        data.extend_from_slice(seg_bytes);

        self.shadow_segment_buffer =
            self.device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("shadow_segments"),
                    contents: &data,
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                });

        if let Some(inst_buf) = &self.hole_instance_buffer {
            self.hole_bind_group =
                Some(self.device.create_bind_group(&wgpu::BindGroupDescriptor {
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
                        wgpu::BindGroupEntry {
                            binding: 2,
                            resource: wgpu::BindingResource::TextureView(&self.hole_mask_view),
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

    pub fn begin_frame(
        &mut self,
        time: f32,
        drag_active: bool,
        victory_time: f32,
        level_seed: f32,
        render_mode: u32,
        cel_mode: bool,
    ) -> Option<FrameInFlight> {
        self.ensure_timestamp_resources();
        self.poll_timestamp_readback();

        let aspect = self.width as f32 / self.height.max(1) as f32;
        let uniforms = build_frame_uniforms(
            &self.camera,
            aspect,
            SHADOW_MAP_SIZE,
            time,
            drag_active,
            victory_time,
            level_seed,
            self.highlight_hole,
            render_mode,
            cel_mode,
            self.hole_mask_bounds,
            self.draw_flags.table_shadow_mode,
            &self.rope_material,
            &self.lighting,
            &self.visual,
            &self.table,
            &self.cartoon,
            &self.worm,
            &self.ssr,
        );
        self.queue.write_buffer(
            &self.frame_uniforms_buffer,
            0,
            bytemuck::bytes_of(&uniforms),
        );

        if self.needs_wood_bake {
            self.needs_wood_bake = false;
            self.encode_bake_wood_pass();
        }

        let output = match self.surface.get_current_texture() {
            Ok(t) => {
                if self.surface_error_streak > 0 {
                    log::warn!(
                        "surface recovered after {} dropped frame(s); size={}x{} scale={:.2}",
                        self.surface_error_streak,
                        self.width,
                        self.height,
                        self.render_scale
                    );
                    self.surface_error_streak = 0;
                }
                t
            }
            Err(wgpu::SurfaceError::Lost) => {
                self.surface_error_streak += 1;
                if self.surface_error_streak <= 5 || self.surface_error_streak % 60 == 0 {
                    log::warn!(
                        "surface error Lost; streak={} size={}x{} scale={:.2} render_mode={} victory_time={:.3}",
                        self.surface_error_streak,
                        self.width,
                        self.height,
                        self.render_scale,
                        render_mode,
                        victory_time
                    );
                }
                self.resize(self.width, self.height);
                return None;
            }
            Err(wgpu::SurfaceError::OutOfMemory) => {
                self.surface_error_streak += 1;
                log::error!(
                    "surface error OutOfMemory; streak={} size={}x{} scale={:.2} render_mode={} victory_time={:.3}",
                    self.surface_error_streak,
                    self.width,
                    self.height,
                    self.render_scale,
                    render_mode,
                    victory_time
                );
                return None;
            }
            Err(err) => {
                self.surface_error_streak += 1;
                if self.surface_error_streak <= 5 || self.surface_error_streak % 60 == 0 {
                    log::warn!(
                        "surface error {:?}; streak={} size={}x{} scale={:.2} render_mode={} victory_time={:.3}",
                        err,
                        self.surface_error_streak,
                        self.width,
                        self.height,
                        self.render_scale,
                        render_mode,
                        victory_time
                    );
                }
                return None;
            }
        };
        let screen_view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("frame_encoder"),
            });

        let sd = render_mode == 0;
        #[cfg(target_arch = "wasm32")]
        let victory_simplify = victory_time > 0.0;
        #[cfg(not(target_arch = "wasm32"))]
        let victory_simplify = false;
        let do_shadow_pass =
            !cel_mode && !victory_simplify && (render_mode > 0 || self.draw_flags.table_shadow_mode == 0);
        let do_planar_mask_pass =
            !cel_mode && !victory_simplify && self.draw_flags.table_shadow_mode == 1;
        if do_shadow_pass {
            self.encode_shadow_pass(&mut encoder);
        }
        if do_planar_mask_pass {
            self.encode_planar_mask_pass(&mut encoder);
        }
        self.frame_index = self.frame_index.wrapping_add(1);
        self.encode_hdr_pass(&mut encoder);
        if !sd && !cel_mode && !victory_simplify {
            self.encode_bloom_pass(&mut encoder);
        }
        self.encode_composite_pass(&mut encoder, &screen_view);

        if self.timestamp_supported && !self.timestamp_pending {
            if let (Some(qs), Some(resolve), Some(readback)) = (
                &self.timestamp_query_set,
                &self.timestamp_resolve_buffer,
                &self.timestamp_readback_buffer,
            ) {
                let size = (TIMESTAMP_QUERY_COUNT as u64) * 8;
                encoder.resolve_query_set(qs, 0..TIMESTAMP_QUERY_COUNT, resolve, 0);
                encoder.copy_buffer_to_buffer(resolve, 0, readback, 0, size);
                self.timestamp_pending = true;
            }
        }

        Some(FrameInFlight {
            encoder,
            screen_view,
            output,
        })
    }

    fn poll_timestamp_readback(&mut self) {
        if !self.timestamp_pending {
            return;
        }
        #[cfg(target_arch = "wasm32")]
        {
            self.timestamp_pending = false;
            return;
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let readback = match &self.timestamp_readback_buffer {
                Some(b) => b,
                None => return,
            };
            let slice = readback.slice(..);
            let (tx, rx) = std::sync::mpsc::channel();
            slice.map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
            self.device.poll(wgpu::Maintain::Wait);
            if rx.recv().ok().and_then(|r| r.ok()).is_some() {
                let data = slice.get_mapped_range();
                let timestamps: &[u64] = bytemuck::cast_slice(&data);
                let ns_per_tick = self.timestamp_period as f64;
                let to_ms = |begin: u32, end: u32| -> f32 {
                    let b = timestamps.get(begin as usize).copied().unwrap_or(0);
                    let e = timestamps.get(end as usize).copied().unwrap_or(0);
                    if e > b {
                        ((e - b) as f64 * ns_per_tick / 1_000_000.0) as f32
                    } else {
                        0.0
                    }
                };
                self.gpu_timings.shadow_ms = to_ms(TS_SHADOW_BEGIN, TS_SHADOW_END);
                self.gpu_timings.hdr_ms = to_ms(TS_HDR_BEGIN, TS_HDR_END);
                let bloom_th = to_ms(TS_BLOOM_TH_BEGIN, TS_BLOOM_TH_END);
                let bloom_h = to_ms(TS_BLOOM_H_BEGIN, TS_BLOOM_H_END);
                let bloom_v = to_ms(TS_BLOOM_V_BEGIN, TS_BLOOM_V_END);
                self.gpu_timings.bloom_ms = bloom_th + bloom_h + bloom_v;
                self.gpu_timings.total_ms = self.gpu_timings.shadow_ms
                    + self.gpu_timings.hdr_ms
                    + self.gpu_timings.bloom_ms;
                drop(data);
            }
            readback.unmap();
            self.timestamp_pending = false;
        }
    }

    pub fn end_frame(&self, frame: FrameInFlight) {
        self.queue.submit(std::iter::once(frame.encoder.finish()));
        frame.output.present();
    }

    // ─────────────────────────── bake wood ───────────────────────────

    fn encode_bake_wood_pass(&self) {
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("bake_wood_encoder"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bake_wood_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.bake_wood_pipeline);
            pass.set_bind_group(0, &self.bake_wood_bind_group, &[]);
            let size = self.wood_baked_texture.size();
            let wg = 8u32;
            pass.dispatch_workgroups((size.width + wg - 1) / wg, (size.height + wg - 1) / wg, 1);
        }
        self.queue.submit(std::iter::once(encoder.finish()));
    }

    // ─────────────────────────── shadow pass ───────────────────────────

    fn encode_shadow_pass(&self, encoder: &mut wgpu::CommandEncoder) {
        let ts = self
            .timestamp_query_set
            .as_ref()
            .map(|qs| wgpu::RenderPassTimestampWrites {
                query_set: qs,
                beginning_of_pass_write_index: Some(TS_SHADOW_BEGIN),
                end_of_pass_write_index: Some(TS_SHADOW_END),
            });
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("shadow_pass"),
            color_attachments: &[],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.shadow_depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: ts,
            occlusion_query_set: None,
        });

        pass.set_bind_group(0, &self.shadow_bind_group, &[]);

        if let (Some(vb), Some(ib), Some(bg)) = (
            &self.hole_vertex_buffer,
            &self.hole_index_buffer,
            &self.hole_bind_group,
        ) {
            pass.set_pipeline(&self.shadow_hole_pipeline);
            pass.set_bind_group(1, bg, &[]);
            pass.set_vertex_buffer(0, vb.slice(..));
            pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.hole_index_count, 0, 0..self.hole_instance_count);
        }

        if let (Some(vb), Some(ib)) = (&self.board_vertex_buffer, &self.board_index_buffer) {
            pass.set_pipeline(&self.shadow_board_pipeline);
            pass.set_vertex_buffer(0, vb.slice(..));
            pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.board_index_count, 0, 0..1);
        }

        if let (Some(vb), Some(ib)) = (&self.rope_vertex_buffer, &self.rope_index_buffer) {
            pass.set_pipeline(&self.shadow_rope_pipeline);
            pass.set_vertex_buffer(0, vb.slice(..));
            pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.rope_index_count, 0, 0..1);
        }
    }

    // ─────────────────────────── HDR pass ───────────────────────────

    fn encode_hdr_pass(&self, encoder: &mut wgpu::CommandEncoder) {
        let ts = self
            .timestamp_query_set
            .as_ref()
            .map(|qs| wgpu::RenderPassTimestampWrites {
                query_set: qs,
                beginning_of_pass_write_index: Some(TS_HDR_BEGIN),
                end_of_pass_write_index: Some(TS_HDR_END),
            });
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("hdr_pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &self.hdr_view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.0,
                        g: 0.0,
                        b: 0.0,
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: ts,
            occlusion_query_set: None,
        });

        pass.set_bind_group(0, &self.frame_bind_group, &[]);

        if !self.draw_flags.skip_table {
            pass.set_pipeline(&self.table_pipeline);
            if let Some(bg) = &self.hole_bind_group {
                pass.set_bind_group(1, bg, &[]);
            }
            pass.draw(0..6, 0..1);
        }

        if let (Some(vb), Some(ib), Some(bg)) = (
            &self.board_vertex_buffer,
            &self.board_index_buffer,
            &self.hole_bind_group,
        ) {
            pass.set_pipeline(&self.board_pipeline);
            pass.set_bind_group(1, bg, &[]);
            pass.set_vertex_buffer(0, vb.slice(..));
            pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.board_index_count, 0, 0..1);
        }

        if !self.draw_flags.skip_holes {
            if let (Some(vb), Some(ib), Some(bg)) = (
                &self.hole_vertex_buffer,
                &self.hole_index_buffer,
                &self.hole_bind_group,
            ) {
                pass.set_pipeline(&self.hole_pipeline);
                pass.set_bind_group(1, bg, &[]);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..self.hole_index_count, 0, 0..self.hole_instance_count);
            }
        }

        if !self.draw_flags.skip_ropes {
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
    }

    // ─────────────────────────── bloom pass ───────────────────────────

    fn encode_planar_mask_pass(&self, encoder: &mut wgpu::CommandEncoder) {
        let Some(bg) = &self.hole_bind_group else {
            return;
        };
        let frame_bg_planar_write = build_frame_bind_group(
            &self.device,
            &self.frame_bind_group_layout,
            &self.frame_uniforms_buffer,
            &self.shadow_depth_view,
            &self.shadow_sampler,
            &self.linear_sampler,
            &self.hole_mask_view,
            &self.noise_view,
            &self.noise_sampler,
            &self.wood_baked_view,
        );
        let wg_x = (self.planar_mask_w + 7) / 8;
        let wg_y = (self.planar_mask_h + 7) / 8;
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("planar_mask"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&self.planar_mask_pipeline);
        pass.set_bind_group(0, &frame_bg_planar_write, &[]);
        pass.set_bind_group(1, bg, &[]);
        pass.set_bind_group(2, &self.empty_bind_group, &[]);
        pass.set_bind_group(3, &self.planar_mask_bind_group, &[]);
        pass.dispatch_workgroups(wg_x, wg_y, 1);
    }

    fn encode_bloom_pass(&self, encoder: &mut wgpu::CommandEncoder) {
        let bw = (self.width / BLOOM_DIVISOR).max(1);
        let bh = (self.height / BLOOM_DIVISOR).max(1);
        let wg_x = (bw + 7) / 8;
        let wg_y = (bh + 7) / 8;

        {
            let ts = self
                .timestamp_query_set
                .as_ref()
                .map(|qs| wgpu::ComputePassTimestampWrites {
                    query_set: qs,
                    beginning_of_pass_write_index: Some(TS_BLOOM_TH_BEGIN),
                    end_of_pass_write_index: Some(TS_BLOOM_TH_END),
                });
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bloom_threshold"),
                timestamp_writes: ts,
            });
            pass.set_pipeline(&self.bloom_threshold_pipeline);
            pass.set_bind_group(0, &self.empty_bind_group, &[]);
            pass.set_bind_group(1, &self.empty_bind_group, &[]);
            pass.set_bind_group(2, &self.empty_bind_group, &[]);
            pass.set_bind_group(3, &self.bloom_threshold_bind_group, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }

        {
            let ts = self
                .timestamp_query_set
                .as_ref()
                .map(|qs| wgpu::ComputePassTimestampWrites {
                    query_set: qs,
                    beginning_of_pass_write_index: Some(TS_BLOOM_H_BEGIN),
                    end_of_pass_write_index: Some(TS_BLOOM_H_END),
                });
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bloom_blur_h"),
                timestamp_writes: ts,
            });
            pass.set_pipeline(&self.bloom_blur_h_pipeline);
            pass.set_bind_group(0, &self.empty_bind_group, &[]);
            pass.set_bind_group(1, &self.empty_bind_group, &[]);
            pass.set_bind_group(2, &self.empty_bind_group, &[]);
            pass.set_bind_group(3, &self.bloom_blur_h_bind_group, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }

        {
            let ts = self
                .timestamp_query_set
                .as_ref()
                .map(|qs| wgpu::ComputePassTimestampWrites {
                    query_set: qs,
                    beginning_of_pass_write_index: Some(TS_BLOOM_V_BEGIN),
                    end_of_pass_write_index: Some(TS_BLOOM_V_END),
                });
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bloom_blur_v"),
                timestamp_writes: ts,
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

    fn encode_composite_pass(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        screen_view: &wgpu::TextureView,
    ) {
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
